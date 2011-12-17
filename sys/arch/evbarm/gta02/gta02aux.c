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
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/callout.h>

#include <arm/s3c2xx0/s3c2410reg.h>
#include <arm/s3c2xx0/s3c2410var.h>

#define GTA02_EINTR_AUX	6
#define AUX_DEBOUNCE_MS	50

struct gta02aux_softc {
	device_t		sc_dev;
	callout_t		sc_debounce;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_gpioh;
	int			sc_intr;
};

static int	gta02aux_match(device_t, cfdata_t, void *);
static void	gta02aux_attach(device_t, device_t, void *);
static int	gta02aux_intr(void *);
static void	gta02aux_debounce(void *);

CFATTACH_DECL_NEW(gta02aux, sizeof(struct gta02aux_softc),
	gta02aux_match, gta02aux_attach, NULL, NULL);

static int
gta02aux_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
gta02aux_attach(device_t parent, device_t self, void *aux)
{
	struct s3c2xx0_attach_args 	*sa = aux;
	struct gta02aux_softc		*sc = device_private(self);
	uint32_t pfcon;

	aprint_normal(": AUX button\n");

	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_gpioh = s3c2xx0_softc->sc_gpio_ioh;
	sc->sc_intr = sa->sa_intr;

	pfcon = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PFCON);
	pfcon = GPIO_SET_FUNC(pfcon, 6, PCON_ALTFUN); /* EINT6 */
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PFCON, pfcon);

	callout_init(&sc->sc_debounce, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_debounce, gta02aux_debounce, sc);

	s3c2410_extint_establish(GTA02_EINTR_AUX, IPL_TTY, IST_EDGE_BOTH,
	                         gta02aux_intr, sc);
}

static int
gta02aux_intr(void *arg)
{
	struct gta02aux_softc *sc = arg;

	callout_stop(&sc->sc_debounce);
	callout_schedule(&sc->sc_debounce, mstohz(AUX_DEBOUNCE_MS));

	return 1;
}

static void
gta02aux_debounce(void *arg)
{
	struct gta02aux_softc *sc = arg;
	uint8_t pfdat;

	pfdat = bus_space_read_1(sc->sc_iot, sc->sc_gpioh, GPIO_PFDAT);

	if (pfdat & (1U << 6)) {
		printf("AUX button pressed\n");
	} else {
		printf("AUX button released\n");
	}
}
