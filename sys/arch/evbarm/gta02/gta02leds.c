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
#include <sys/sysctl.h>

#include <arm/s3c2xx0/s3c2410reg.h>
#include <arm/s3c2xx0/s3c2410var.h>

#define ORANGE_BIT	0
#define BLUE_BIT	1
#define RED_BIT		2

struct gta02leds_softc {
	u_int32_t		sc_red;
	u_int32_t		sc_orange;
	u_int32_t		sc_blue;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_gpioh;
};

struct gta02leds_softc *leds_sc = NULL;

static int 	gta02leds_match(device_t, cfdata_t, void *);
static void	gta02leds_attach(device_t, device_t, void *);
static int	gta02leds_sysctl_red(SYSCTLFN_ARGS);
static int	gta02leds_sysctl_orange(SYSCTLFN_ARGS);
static int	gta02leds_sysctl_blue(SYSCTLFN_ARGS);

CFATTACH_DECL_NEW(gta02leds, sizeof(struct gta02leds_softc),
	gta02leds_match, gta02leds_attach, NULL, NULL);

static int
gta02leds_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
gta02leds_attach(device_t parent, device_t self, void *aux)
{
	struct gta02leds_softc		*sc = device_private(self);
	struct s3c2xx0_attach_args	*sa = aux;
	const struct sysctlnode		*node, *datanode;
	uint32_t pbcon, pbdat;
	int error;

	aprint_normal(": OpenMoko leds\n");

	sc->sc_iot = sa->sa_iot;
	sc->sc_gpioh = s3c2xx0_softc->sc_gpio_ioh;

	error = sysctl_createv(NULL, 0, NULL, NULL, CTLFLAG_PERMANENT, CTLTYPE_NODE,
	                       "hw", NULL, NULL, 0, NULL, 0, CTL_HW, CTL_EOL);
	if (error) {
		aprint_error_dev(self, "failed to create sysctl\n");
		return;
	}

	error = sysctl_createv(NULL, 0, NULL, &node, 0, CTLTYPE_NODE, device_xname(self),
	                       NULL, NULL, 0, NULL, 0, CTL_HW, CTL_CREATE, CTL_EOL);
	if (error) {
		aprint_error_dev(self, "failed to create sysctl\n");
		return;
	}

#define CREATE_LED_SYSCTL(color) \
	error = sysctl_createv(NULL, 0, NULL, &datanode, CTLFLAG_READWRITE, CTLTYPE_INT, \
		               #color, SYSCTL_DESCR( #color " led on/off control"), \
			       gta02leds_sysctl_ ## color ,0, &sc->sc_ ## color, 0, CTL_HW, \
	                       node->sysctl_num, CTL_CREATE, CTL_EOL); \
	if (error) { \
		aprint_error_dev(self, "failed to create sysctl\n"); \
		return; \
	}

	CREATE_LED_SYSCTL(red);
	CREATE_LED_SYSCTL(orange);
	CREATE_LED_SYSCTL(blue);
#undef CREATE_LED_SYSCTL

	/* Configure GPIO pins as output */
	pbcon = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBCON);
	pbcon = GPIO_SET_FUNC(pbcon, ORANGE_BIT, PCON_OUTPUT);
	pbcon = GPIO_SET_FUNC(pbcon, BLUE_BIT, PCON_OUTPUT);
	pbcon = GPIO_SET_FUNC(pbcon, RED_BIT, PCON_OUTPUT);
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBCON, pbcon);

	/* Make sure that leds are disabled */
	pbdat = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT);
	pbdat &= ~(1U << ORANGE_BIT);
	pbdat &= ~(1U << BLUE_BIT);
	pbdat &= ~(1U << RED_BIT);
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT, pbdat);

	leds_sc = sc;
}

#define DECLARE_LED_SYSCTL_FN(color, pin) \
	static int \
	gta02leds_sysctl_ ## color (SYSCTLFN_ARGS) \
	{ \
		struct gta02leds_softc *sc = leds_sc; \
		struct sysctlnode node; \
		uint32_t pbdat; \
		int error, t; \
		\
		node = *rnode; \
		t = *(int *)rnode->sysctl_data; \
		node.sysctl_data = &t; \
		error = sysctl_lookup(SYSCTLFN_CALL(&node)); \
		if (error || newp == NULL) \
			return error; \
		\
		if (t != 0 && t != 1) \
			return EINVAL; \
		\
		*(int *)rnode->sysctl_data = t;\
		\
		pbdat = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT); \
		if (t) { \
			pbdat |= (1U << pin); \
		} else { \
			pbdat &= ~(1U << pin); \
		} \
		bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT, pbdat); \
		\
		return 0; \
	}

DECLARE_LED_SYSCTL_FN(orange, ORANGE_BIT)
DECLARE_LED_SYSCTL_FN(blue, BLUE_BIT)
DECLARE_LED_SYSCTL_FN(red, RED_BIT)
#undef DECLARE_LED_SYSCTL_FN
