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
#include <sys/device.h>

#include <dev/sysmon/sysmonvar.h>

#include <dev/i2c/pcf50633var.h>
#include <dev/i2c/pcf50633reg.h>

struct gta02bat_softc {
	device_t	sc_dev;

	struct sysmon_envsys	*sc_sme;
};

static int	gta02bat_match(device_t, cfdata_t, void *);
static void	gta02bat_attach(device_t, device_t, void *);
static int	gta02bat_init_intrs(struct gta02bat_softc *);
static void	gta02bat_refresh(struct sysmon_envsys *, envsys_data_t *);
static void	gta02bat_get_limits(struct sysmon_envsys *, envsys_data_t *,
		                    sysmon_envsys_lim_t *, uint32_t *);

CFATTACH_DECL_NEW(gta02bat, sizeof (struct gta02bat_softc),
                  gta02bat_match, gta02bat_attach, NULL, NULL);

static void	gta02bat_adpins(void *);
static void	gta02bat_adprem(void *);
static void	gta02bat_usbins(void *);
static void	gta02bat_usbrem(void *);
static void	gta02bat_batfull(void *);
static void	gta02bat_chghalt(void *);
static void	gta02bat_thlimon(void *);
static void	gta02bat_thlimoff(void *);
static void	gta02bat_usblimon(void *);
static void	gta02bat_usblimoff(void *);
static void	gta02bat_lowsys(void *);
static void	gta02bat_lowbat(void *);

struct intr_def {
	uint8_t		reg;
	uint8_t 	bit;
	uint16_t	lvl;
	void		(* handler)(void *);
};

#define _ENT(l, r, b, h) { .reg = r, .bit = b, .handler = h, .lvl = l }
struct intr_def intr_defs[] = {
	_ENT(10, PCF50633_INT1, INT1_ADPINS_BIT, gta02bat_adpins),
	_ENT(10, PCF50633_INT1, INT1_ADPREM_BIT, gta02bat_adprem),
	_ENT(10, PCF50633_INT1, INT1_USBINS_BIT, gta02bat_usbins),
	_ENT(10, PCF50633_INT1, INT1_USBREM_BIT, gta02bat_usbrem),
	_ENT(5,  PCF50633_INT3, INT3_BATFULL_BIT, gta02bat_batfull),
	_ENT(5,  PCF50633_INT3, INT3_CHGHALT_BIT, gta02bat_chghalt),
	_ENT(30, PCF50633_INT3, INT3_THLIMON_BIT, gta02bat_thlimon),
	_ENT(30, PCF50633_INT3, INT3_THLIMOFF_BIT, gta02bat_thlimoff),
	_ENT(20, PCF50633_INT3, INT3_USBLIMON_BIT, gta02bat_usblimon),
	_ENT(20, PCF50633_INT3, INT3_USBLIMOFF_BIT, gta02bat_usblimoff),
	_ENT(50, PCF50633_INT4, INT4_LOWSYS_BIT, gta02bat_lowsys),
	_ENT(50, PCF50633_INT4, INT4_LOWBAT_BIT, gta02bat_lowbat)
};
#undef _ENT

static int
gta02bat_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
gta02bat_attach(device_t parent, device_t self, void *aux)
{
	struct gta02bat_softc *sc = device_private(self);

	aprint_normal(": OpenMoko Battery\n");

	sc->sc_dev = self;
	sc->sc_sme = sysmon_envsys_create();

	sc->sc_sme->sme_name = device_xname(self);
	sc->sc_sme->sme_cookie = sc;
	sc->sc_sme->sme_refresh = gta02bat_refresh;
	sc->sc_sme->sme_class = SME_CLASS_BATTERY;
	sc->sc_sme->sme_flags = SME_POLL_ONLY | SME_INIT_REFRESH;
	sc->sc_sme->sme_get_limits = gta02bat_get_limits;

	if (gta02bat_init_intrs(sc)) {
		aprint_error_dev(self, "Failed to register interrupt handlers!\n");
		return;
	}

	if (sysmon_envsys_register(sc->sc_sme)) {
		aprint_error_dev(self, "Failed to register witn envsys!\n");
		return;
	}
}

static int
gta02bat_init_intrs(struct gta02bat_softc *sc)
{
	struct intr_def *def;
	int i, error;

	for (i = 0; i < __arraycount(intr_defs); i++) {
		def = &intr_defs[i];
		error = pcf50633bus_intr_establish(def->reg, def->bit, def->lvl,
		                                   def->handler, sc);
		if (error)
			break;
	}

	if (error == 0) {
		return 0;
	}

	for (; i >= 0; i--) {
		def = &intr_defs[i];
		pcf50633bus_intr_disestablish(def->reg, def->bit);
	}

	return -1;
}

static void
gta02bat_refresh(struct sysmon_envsys *sme, envsys_data_t *edata)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_get_limits(struct sysmon_envsys *sme, envsys_data_t *edata,
                    sysmon_envsys_lim_t *limits, uint32_t *props)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_adpins(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_adprem(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_usbins(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_usbrem(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_batfull(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_chghalt(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_thlimon(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_thlimoff(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_usblimon(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_usblimoff(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_lowsys(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}

static void
gta02bat_lowbat(void *arg)
{

	printf("TODO: implement %s\n", __FUNCTION__);
}
