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

#define VIBRATOR_BIT	3

struct gta02vib_softc {
	u_int32_t		sc_vibra;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_gpioh;
};

struct gta02vib_softc *vib_sc = NULL;

static int	gta02vib_match(device_t, cfdata_t, void *);
static void	gta02vib_attach(device_t, device_t, void *);
static int	gta02vib_sysctl(SYSCTLFN_ARGS);

CFATTACH_DECL_NEW(gta02vib, sizeof(struct gta02vib_softc),
	gta02vib_match, gta02vib_attach, NULL, NULL);

static int
gta02vib_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
gta02vib_attach(device_t parent, device_t self, void *aux)
{
	struct gta02vib_softc		*sc = device_private(self);
	struct s3c2xx0_attach_args 	*sa = aux;
	const struct sysctlnode		*node, *datanode;
	uint32_t pbcon, pbdat;
	int error;

	aprint_normal(": OpenMoko vibrator\n");

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

	error = sysctl_createv(NULL, 0, NULL, &datanode, CTLFLAG_READWRITE, CTLTYPE_INT,
	                       "enable", SYSCTL_DESCR("Vibrator on/off control"),
			       gta02vib_sysctl, 0, &sc->sc_vibra, 0, CTL_HW,
	                       node->sysctl_num, CTL_CREATE, CTL_EOL);
	if (error) {
		aprint_error_dev(self, "failed to create sysctl\n");
		return;
	}

	/* Configure GPIO as output */
	pbcon = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBCON);
	pbcon = GPIO_SET_FUNC(pbcon, VIBRATOR_BIT, PCON_OUTPUT);
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBCON, pbcon);

	/* Make sure vibrator is disabled */
	pbdat = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT);
	pbdat &= ~(1U << VIBRATOR_BIT);
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT, pbdat);

	vib_sc = sc;
}

static int
gta02vib_sysctl(SYSCTLFN_ARGS)
{
	struct gta02vib_softc *sc = vib_sc;
	struct sysctlnode node;
	uint32_t pbdat;
	int error, t;

	node = *rnode;
	t = *(int *)rnode->sysctl_data;
	node.sysctl_data = &t;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;

	if (t != 0 && t != 1)
		return EINVAL;

	*(int *)rnode->sysctl_data = t;

	pbdat = bus_space_read_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT);
	if (t) {
		pbdat |= (1U << VIBRATOR_BIT);
	} else {
		pbdat &= ~(1U << VIBRATOR_BIT);
	}
	bus_space_write_4(sc->sc_iot, sc->sc_gpioh, GPIO_PBDAT, pbdat);

	return 0;
}
