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
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/bus.h>

#include <dev/sysmon/sysmonvar.h>

#include <arm/s3c2xx0/s3c2410reg.h>
#include <arm/s3c2xx0/s3c2410var.h>

#include "locators.h"

#ifndef SSWDOG_DEFAULT_PERIOD
#define SSWDOG_DEFAULT_PERIOD	10
#endif

#define PRESCALE	255
#define CLKSEL_VAL	128
#define CLKSEL		WTCON_CLKSEL_128

struct sswdog_softc {
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;

	struct sysmon_wdog	sc_smw;
	uint32_t		sc_period;
	uint32_t		sc_mult;
	uint16_t		sc_max;
	uint16_t		sc_reload;
};

static int  	sswdog_match(device_t, cfdata_t, void *);
static void 	sswdog_attach(device_t, device_t, void *);
static int  	sswdog_tickle(struct sysmon_wdog *);
static int  	sswdog_setmode(struct sysmon_wdog *);
static void	sswdog_enable(struct sswdog_softc *);
static void	sswdog_disable(struct sswdog_softc *);

CFATTACH_DECL_NEW(sswdog, sizeof(struct sswdog_softc),
	sswdog_match, sswdog_attach, NULL, NULL);

static int
sswdog_match(device_t parent, cfdata_t cf, void *aux)
{
	struct s3c2xx0_attach_args *sa = aux;

	if (sa->sa_addr == SSIOCF_ADDR_DEFAULT)
		sa->sa_addr = S3C2410_WDT_BASE;

	return 1;
}

static void
sswdog_attach(device_t parent, device_t self, void *aux)
{
	struct sswdog_softc *sc = device_private(self);
	struct s3c2xx0_attach_args *sa = aux;
	uint32_t pclk = s3c2xx0_softc->sc_pclk;
	uint32_t wtcon = 0;

	sc->sc_iot = sa->sa_iot;
	if (bus_space_map(sc->sc_iot, sa->sa_addr, PAGE_SIZE, 0, &sc->sc_ioh)) {
		aprint_error(": failed to map registers\n");
		return;
	}

	sc->sc_period = SSWDOG_DEFAULT_PERIOD;
	sc->sc_mult = (pclk / PRESCALE / CLKSEL_VAL);
	sc->sc_max = UINT16_MAX / sc->sc_mult;
	sc->sc_reload = sc->sc_period * sc->sc_mult;

	sc->sc_smw.smw_name = device_xname(self);
	sc->sc_smw.smw_cookie = sc;
	sc->sc_smw.smw_setmode = sswdog_setmode;
	sc->sc_smw.smw_tickle = sswdog_tickle;
	sc->sc_smw.smw_period = sc->sc_period;

	sswdog_disable(sc);
	wtcon |= (PRESCALE << WTCON_PRESCALE_SHIFT);
	wtcon |= CLKSEL;
	wtcon |= WTCON_ENRST;
	bus_space_write_2(sc->sc_iot, sc->sc_ioh, WDT_WTCON, wtcon);

	if (sysmon_wdog_register(&sc->sc_smw) != 0) {
		aprint_error(": unable to register with sysmon\n");
	}

	aprint_normal(": %ds period (%ds max), disarmed\n",
		sc->sc_period, sc->sc_max);
}

static int
sswdog_tickle(struct sysmon_wdog *smw)
{
	struct sswdog_softc *sc = smw->smw_cookie;

	sswdog_disable(sc);
	bus_space_write_2(sc->sc_iot, sc->sc_ioh, WDT_WTCNT, sc->sc_reload);
	sswdog_enable(sc);

	return 0;
}

static int
sswdog_setmode(struct sysmon_wdog *smw)
{
	struct sswdog_softc *sc = smw->smw_cookie;

	if ((smw->smw_mode & WDOG_MODE_MASK) == WDOG_MODE_DISARMED) {
		sswdog_disable(sc);
	} else {
		if (smw->smw_period != sc->sc_period) {
			if (smw->smw_period > sc->sc_max) {
				return EOPNOTSUPP;
			}
			sc->sc_period = smw->smw_period;
			sc->sc_reload = sc->sc_period * sc->sc_mult;
		}

		sswdog_disable(sc);
		bus_space_write_2(sc->sc_iot, sc->sc_ioh, WDT_WTCNT, sc->sc_reload);
		sswdog_enable(sc);
	}

	return 0;
}

static void
sswdog_enable(struct sswdog_softc *sc)
{
	uint32_t wtcon;

	wtcon = bus_space_read_2(sc->sc_iot, sc->sc_ioh, WDT_WTCON);
	wtcon |= WTCON_ENABLE;
	bus_space_write_2(sc->sc_iot, sc->sc_ioh, WDT_WTCON, wtcon);
}

static void
sswdog_disable(struct sswdog_softc *sc)
{
	uint32_t wtcon;

	wtcon = bus_space_read_2(sc->sc_iot, sc->sc_ioh, WDT_WTCON);
	wtcon &= ~WTCON_ENABLE;
	bus_space_write_2(sc->sc_iot, sc->sc_ioh, WDT_WTCON, wtcon);
}
