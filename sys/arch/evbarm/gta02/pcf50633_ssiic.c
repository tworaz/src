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

#include <dev/i2c/i2cvar.h>

#include <arm/s3c2xx0/s3c2410reg.h>
#include <arm/s3c2xx0/s3c2410var.h>

#include <dev/i2c/pcf50633var.h>
#include <dev/i2c/pcf50633reg.h>

#define GTA02_EINTR_PCF50633	9

struct pcf50633_ssiic_softc {
	struct pcf50633bus_softc	sc_bus;
};

static int	pcf50633_ssiic_match(device_t, cfdata_t, void *);
static void	pcf50633_ssiic_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(pcf50633_ssiic, sizeof(struct pcf50633_ssiic_softc),
	pcf50633_ssiic_match, pcf50633_ssiic_attach, NULL, NULL);

static int
pcf50633_ssiic_match(device_t parent, cfdata_t cf, void *aux)
{
	struct i2c_attach_args *ia = aux;

	if (ia->ia_addr  == PCF50633_ADDR)
		return 1;

	return 0;
}

static void
pcf50633_ssiic_attach(device_t parent, device_t self, void *aux)
{
	struct pcf50633_ssiic_softc	*sc = device_private(self);
	struct i2c_attach_args		*ia = aux;
	bus_space_tag_t			iot = s3c2xx0_softc->sc_iot;
	bus_space_handle_t		gpioh = s3c2xx0_softc->sc_gpio_ioh;
	uint32_t			pgcon;

	aprint_normal("\n");
	aprint_naive("\n");

	sc->sc_bus.sc_dev = self;
	sc->sc_bus.sc_tag = ia->ia_tag;
	sc->sc_bus.sc_addr = ia->ia_addr;

	if (pcf50633bus_attach_sub(&sc->sc_bus)) {
		aprint_error_dev(self, "unable to attach PCF50633 Device Bus\n");
		return;
	}

	pgcon = bus_space_read_4(iot, gpioh, GPIO_PGCON);
	pgcon = GPIO_SET_FUNC(pgcon, 1, PCON_ALTFUN); /* EINT9 */
	bus_space_write_4(iot, gpioh, GPIO_PGCON, pgcon);

	s3c2410_extint_establish(GTA02_EINTR_PCF50633, IPL_NONE, IST_EDGE,
		pcf50633bus_intr, &sc->sc_bus);
}
