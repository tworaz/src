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

#include <dev/clock_subr.h>

#include <dev/i2c/pcf50633reg.h>
#include <dev/i2c/pcf50633var.h>

enum rtc_reg_type {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DAY,
	RTC_MONTH,
	RTC_YEAR
};

struct pcf50633rtc_softc {
	device_t	sc_dev;
	struct todr_chip_handle	sc_todr;
};

static int	pcf50633rtc_match(device_t, cfdata_t, void *);
static void	pcf50633rtc_attach(device_t, device_t, void *);

static int	pcf50633rtc_gettime(struct todr_chip_handle *,
                                    struct clock_ymdhms *);
static int	pcf50633rtc_settime(struct todr_chip_handle *,
                                    struct clock_ymdhms *);

CFATTACH_DECL_NEW(pcf50633rtc, sizeof(struct pcf50633rtc_softc),
                  pcf50633rtc_match, pcf50633rtc_attach, NULL, NULL);

static int
pcf50633rtc_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
pcf50633rtc_attach(device_t parent, device_t self, void *aux)
{
	struct pcf50633rtc_softc *sc = device_private(self);

	aprint_normal(": Real-time Clock\n");

	sc->sc_dev = self;
	sc->sc_todr.cookie = sc;
	sc->sc_todr.todr_gettime_ymdhms = pcf50633rtc_gettime;
	sc->sc_todr.todr_settime_ymdhms = pcf50633rtc_settime;
	sc->sc_todr.todr_setwen = NULL;

	todr_attach(&sc->sc_todr);
}

static int
pcf50633rtc_gettime(struct todr_chip_handle *ch, struct clock_ymdhms *dt)
{
	struct pcf50633rtc_softc *sc = ch->cookie;
	uint8_t reg = PCF50633_REG_RTCSC;
	uint8_t bcd[PCF50633_RTC_REGS];

	if (pcf50633bus_read(reg, bcd, PCF50633_RTC_REGS, I2C_F_POLL) != 0) {
		device_printf(sc->sc_dev, "failed to read rtc clock value\n");
		return -1;
	}

	dt->dt_sec = FROMBCD(bcd[RTC_SEC]);
	dt->dt_min = FROMBCD(bcd[RTC_MIN]);
	dt->dt_hour = FROMBCD(bcd[RTC_HOUR]);
	dt->dt_wday = FROMBCD(bcd[RTC_WEEKDAY]);
	dt->dt_day = FROMBCD(bcd[RTC_DAY]);
	dt->dt_mon = FROMBCD(bcd[RTC_MONTH]);
	dt->dt_year = FROMBCD(bcd[RTC_YEAR]);
	dt->dt_year += 2000;

	return 0;
}

static int
pcf50633rtc_settime(struct todr_chip_handle *ch, struct clock_ymdhms *dt)
{
	struct pcf50633rtc_softc *sc = ch->cookie;
	uint8_t reg = PCF50633_REG_RTCSC;
	uint8_t bcd[PCF50633_RTC_REGS];

	bcd[RTC_SEC] = TOBCD(dt->dt_sec);
	bcd[RTC_MIN] = TOBCD(dt->dt_min);
	bcd[RTC_HOUR] = TOBCD(dt->dt_hour);
	bcd[RTC_WEEKDAY] = TOBCD(dt->dt_wday);
	bcd[RTC_DAY] = TOBCD(dt->dt_day);
	bcd[RTC_MONTH] = TOBCD(dt->dt_mon);
	bcd[RTC_YEAR] = TOBCD(dt->dt_year % 100);

	if (pcf50633bus_write(reg, bcd, PCF50633_RTC_REGS, I2C_F_POLL) != 0) {
		device_printf(sc->sc_dev, "failed to write rtc clock value\n");
		return -1;
	}

	return 0;
}
