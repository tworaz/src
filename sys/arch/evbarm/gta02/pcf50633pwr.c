/*-
 * Copyright (c) 2011, Peter Tworek
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/callout.h>

#include <dev/sysmon/sysmonvar.h>

#include <dev/i2c/pcf50633var.h>
#include <dev/i2c/pcf50633reg.h>

#ifndef PCF50633PWR_SHUTDOWN_TIMEOUT
#define PCF50633PWR_SHUTDOWN_TIMEOUT	5 /* sec */
#endif /* !PCF50633PWR_SHUTDOWN_TIMEOUT */

#ifndef PCF50633PWR_SLEEP_TIMEOUT
#define PCF50633PWR_SLEEP_TIMEOUT	2 /* sec */
#endif /* !PCF50633PWR_SLEEP_TIMEOUT */

struct pcf50633pwr_softc {
	device_t	sc_dev;
	callout_t	sc_callout;
	bool		sc_pressed;
	struct timeval	sc_pressed_tv;

	struct sysmon_pswitch	sc_smpsw;
	struct sysmon_pswitch	sc_smssw;
};

static int	pcf50633pwr_match(device_t, cfdata_t, void *);
static void	pcf50633pwr_attach(device_t, device_t, void *);
static void	pcf50633pwr_onkey_released(void *);
static void	pcf50633pwr_onkey_pressed(void *);
static void	pcf50633pwr_shutdown(void *);

CFATTACH_DECL_NEW(pcf50633pwr, sizeof(struct pcf50633pwr_softc),
                  pcf50633pwr_match, pcf50633pwr_attach, NULL, NULL);

static int
pcf50633pwr_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
pcf50633pwr_attach(device_t parent, device_t self, void *aux)
{
	struct pcf50633pwr_softc *sc = device_private(self);
	uint8_t ooctim2;
	int error;

	aprint_normal(": Power/Sleep Button\n");

	sc->sc_dev = self;
	sc->sc_smpsw.smpsw_name = "pcf50633";
	sc->sc_smpsw.smpsw_type = PSWITCH_TYPE_POWER;
	sc->sc_smssw.smpsw_name = "pcf50633";
	sc->sc_smssw.smpsw_type = PSWITCH_TYPE_SLEEP;

	if (pcf50633bus_read(PCF50633_OOCTIM2, &ooctim2, 1, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to read OOCTIM2!\n");
		return;
	}

	ooctim2 &= ~OOCTIM2_ONKEY_DEB_MASK;
	ooctim2 |= OOCTIM2_ONKEY_DEB_62;

	if (pcf50633bus_write(PCF50633_OOCTIM2, &ooctim2, 1, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write OOCTIM2!\n");
		return;
	}

	if (sysmon_pswitch_register(&sc->sc_smpsw) != 0) {
		aprint_error_dev(sc->sc_dev, "unable to register with sysmon\n");
		return;
	}

	if (sysmon_pswitch_register(&sc->sc_smssw) != 0) {
		aprint_error_dev(sc->sc_dev, "unable to  register with sysmon\n");
		goto fail_sysmon;
	}

	error = pcf50633bus_intr_establish(PCF50633_INT2, INT2_ONKEYR_BIT, 90,
	                                   pcf50633pwr_onkey_released, sc);
	if (error) {
		aprint_error_dev(self, "Failed to register ONKEYR interrupt handler!\n");
		goto fail_intr1;
	}

	error = pcf50633bus_intr_establish(PCF50633_INT2, INT2_ONKEYF_BIT, 90,
	                                   pcf50633pwr_onkey_pressed, sc);
	if (error) {
		aprint_error_dev(self, "Failed to register ONKEYF interrupt handler!\n");
		goto fail_intr2;
	}

	callout_init(&sc->sc_callout, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_callout, pcf50633pwr_shutdown, &sc->sc_smpsw);

	return;

fail_intr2:
	pcf50633bus_intr_disestablish(PCF50633_INT2, INT2_ONKEYR_BIT);
fail_intr1:
	sysmon_pswitch_unregister(&sc->sc_smssw);
fail_sysmon:
	sysmon_pswitch_unregister(&sc->sc_smpsw);
}

static void
pcf50633pwr_onkey_released(void *arg)
{
	struct pcf50633pwr_softc *sc = arg;
	struct timeval tv;
	int diff_sec, diff_usec, total_usec;

	if (sc->sc_pressed) {
		sc->sc_pressed = false;
		getmicrouptime(&tv);
		diff_sec = tv.tv_sec - sc->sc_pressed_tv.tv_sec;
		diff_usec = tv.tv_usec - sc->sc_pressed_tv.tv_usec;
		total_usec = (diff_sec * 1000000) + diff_usec;

		if (total_usec >= (PCF50633PWR_SHUTDOWN_TIMEOUT * 1000000)) {
			return;
		} else if (total_usec >= (PCF50633PWR_SLEEP_TIMEOUT * 1000000)) {
			sysmon_pswitch_event(&sc->sc_smssw, PSWITCH_EVENT_PRESSED);
		} else {
			printf("TODO: send keyboard event to userspace\n");
		}
	}

	callout_stop(&sc->sc_callout);
}

static void
pcf50633pwr_onkey_pressed(void *arg)
{
	struct pcf50633pwr_softc *sc = arg;

	getmicrouptime(&sc->sc_pressed_tv);

	callout_schedule(&sc->sc_callout, mstohz(PCF50633PWR_SHUTDOWN_TIMEOUT * 1000));
	sc->sc_pressed = true;
}

static void
pcf50633pwr_shutdown(void *arg)
{
	struct sysmon_pswitch *smpsw = arg;

	sysmon_pswitch_event(smpsw, PSWITCH_EVENT_PRESSED);
}
