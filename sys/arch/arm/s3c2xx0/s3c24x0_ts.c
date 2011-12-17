/*
 * Copyright (c) 2009 Paul Fleischer
 * All rights reserved.
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * TODO: Right now this implementation only works on S3C2440.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/bus.h>

#include <dev/wscons/wsconsio.h>
#include <dev/wscons/wsmousevar.h>
#include <dev/wscons/tpcalibvar.h>
#include <dev/hpc/hpcfbio.h>

#include <lib/libsa/qsort.c>

#include <arm/s3c2xx0/s3c24x0var.h>
#ifdef S3C2440
#include <arm/s3c2xx0/s3c2440reg.h>
#else
#include <arm/s3c2xx0/s3c2410reg.h>
#endif /* S3C2440 */

#include "locators.h"

#ifdef ssts_DEBUG
#define	DPRINTF(s)	printf s
#else
#define	DPRINTF(s)	do { } while (/*CONSTCOND*/0)
#endif /* ssts_DEBUG */

#define MAX_SAMPLES 20

struct ssts_softc {
	device_t		sc_dev;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;

	uint32_t		sc_next_stylus_intr;

	device_t		sc_wsmousedev;

	struct tpcalib_softc	sc_tpcalib;

	int			sc_sample_count;
	int			sc_samples_x[MAX_SAMPLES];
	int			sc_samples_y[MAX_SAMPLES];

	callout_t		sc_callout;
};

/* Basic Driver Stuff */
static int	ssts_match(device_t, cfdata_t, void *);
static void	ssts_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(ssts, sizeof(struct ssts_softc), ssts_match,
                  ssts_attach, NULL, NULL);

/* wsmousedev */
int	ssts_enable(void *);
int	ssts_ioctl(void *, u_long, void *, int, struct lwp *);
void	ssts_disable(void *);

const struct wsmouse_accessops ssts_accessops = {
	ssts_enable,
	ssts_ioctl,
	ssts_disable
};

/* Interrupt Handlers */
int	ssts_tc_intr(void *);
int	ssts_adc_intr(void *);

void 	ssts_callout(void *);
int	ssts_filter_values(int *, int);
void	ssts_initialize(struct ssts_softc *);

#define STYLUS_DOWN	0
#define STYLUS_UP	ADCTSC_UD_SEN

static struct wsmouse_calibcoords default_calib = {
	.minx = 0,
	.miny = 0,
	.maxx = 0,
	.maxy = 0,
	.samplelen = WSMOUSE_CALIBCOORDS_RESET
};

static int
ssts_match(device_t parent, cfdata_t cf, void *aux)
{
	struct s3c2xx0_attach_args *sa = aux;

	if (sa->sa_size == SSIOCF_SIZE_DEFAULT)
#ifdef S3C2440
		sa->sa_size = S3C2440_ADC_SIZE;
#else
		sa->sa_size = S3C2410_ADC_SIZE;
#endif /* S3C2440 */

	return 1;
}

static void
ssts_attach(device_t parent, device_t self, void *aux)
{
	struct ssts_softc		*sc = device_private(self);
	struct s3c2xx0_attach_args	*sa = aux;
	struct wsmousedev_attach_args	mas;

	sc->sc_iot = sa->sa_iot;
	sc->sc_next_stylus_intr = STYLUS_DOWN;
	sc->sc_sample_count = 0;

	if (bus_space_map(sc->sc_iot, sa->sa_addr, sa->sa_size, 0, &sc->sc_ioh)) {
		aprint_error(": failed to map registers\n");
		return;
	}

	/* XXX: Is IPL correct? */
	s3c24x0_intr_establish(S3C2410_INT_TC, IPL_TTY, IST_EDGE_RISING,
			       ssts_tc_intr, sc);
	s3c24x0_intr_establish(S3C2410_INT_ADC, IPL_TTY, IST_EDGE_RISING,
			       ssts_adc_intr, sc);

	aprint_normal("\n");

	mas.accessops = &ssts_accessops;
	mas.accesscookie = sc;

	sc->sc_wsmousedev = config_found_ia(self, "wsmousedev", &mas, wsmousedevprint);

	tpcalib_init(&sc->sc_tpcalib);
	tpcalib_ioctl(&sc->sc_tpcalib, WSMOUSEIO_SCALIBCOORDS,
	              (void*)&default_calib, 0, 0);

	/* Add CALLOUT_MPSAFE to avoid holding the global kernel lock */
	callout_init(&sc->sc_callout, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_callout, ssts_callout, sc);

	/* Actual initialization is performed by ssts_initialize(),
	   which is called by ssts_enable() */
}

/*
 * ssts_tc_intr is the TC interrupt handler.
 * The TC interrupt is generated when the stylus changes up->down,
 * or down->up state (depending on configuration of ADC_ADCTSC).
 */
int
ssts_tc_intr(void *arg)
{
	struct ssts_softc *sc = arg;
	uint32_t reg;

	DPRINTF(("%s\n", __func__));

	/*
	 * Figure out if the stylus was lifted or lowered
	 */
	reg = bus_space_read_4(sc->sc_iot, sc->sc_ioh, ADC_ADCUPDN);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCUPDN, 0x0);
	if (sc->sc_next_stylus_intr == STYLUS_DOWN && (reg & ADCUPDN_TSC_DN) ) {
		sc->sc_next_stylus_intr = STYLUS_UP;
		ssts_callout(sc);

	} else if (sc->sc_next_stylus_intr == STYLUS_UP && (reg & ADCUPDN_TSC_UP)) {
		uint32_t adctsc = 0;
		sc->sc_next_stylus_intr = STYLUS_DOWN;

		wsmouse_input(sc->sc_wsmousedev, 0x0, 0, 0, 0, 0, 0);

		sc->sc_sample_count = 0;

		adctsc |= ADCTSC_YM_SEN | ADCTSC_YP_SEN | ADCTSC_XP_SEN |
			sc->sc_next_stylus_intr |
			3; /* 3 selects "Waiting for Interrupt Mode" */
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCTSC, adctsc);
	}

	return 1;
}

/*
 * ssts_adc_intr is ADC interrupt handler.ADC interrupt is triggered
 * when the ADC controller has a measurement ready.
 */
int
ssts_adc_intr(void *arg)
{
	struct ssts_softc *sc = arg;
	uint32_t reg;
	uint32_t adctsc = 0;
	int x, y;

	DPRINTF(("%s\n", __func__));

	reg = bus_space_read_4(sc->sc_iot, sc->sc_ioh, ADC_ADCDAT0);
	y = reg & ADCDAT_DATAMASK;

	reg = bus_space_read_4(sc->sc_iot, sc->sc_ioh, ADC_ADCDAT1);
	x = reg & ADCDAT_DATAMASK;


	sc->sc_samples_x[sc->sc_sample_count] = x;
	sc->sc_samples_y[sc->sc_sample_count] = y;

	sc->sc_sample_count++;

	x = ssts_filter_values(sc->sc_samples_x, sc->sc_sample_count);
	y = ssts_filter_values(sc->sc_samples_y, sc->sc_sample_count);

	if (x == -1 || y == -1) {
		/* If we do not have enough measurements, make some more. */
		ssts_callout(sc);
		return 1;
	}

	sc->sc_sample_count = 0;

	tpcalib_trans(&sc->sc_tpcalib, x, y, &x, &y);

	wsmouse_input(sc->sc_wsmousedev, 0x1, x, y, 0, 0,
		      WSMOUSE_INPUT_ABSOLUTE_X | WSMOUSE_INPUT_ABSOLUTE_Y);

	/* Schedule a new adc measurement, unless the stylus has been lifed */
	if (sc->sc_next_stylus_intr == STYLUS_UP) {
		callout_schedule(&sc->sc_callout, hz/50);
	}

	/* Until measurement is to be performed, listen for stylus up-events */
	adctsc |= ADCTSC_YM_SEN | ADCTSC_YP_SEN | ADCTSC_XP_SEN |
		sc->sc_next_stylus_intr |
		3; /* 3 selects "Waiting for Interrupt Mode" */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCTSC, adctsc);

	return 1;
}

int
ssts_enable(void *arg)
{
	struct ssts_softc *sc = arg;

	DPRINTF(("%s\n", __func__));

	/* Enable clock signal to ADC module */
	s3c24x0_clkman_config(CLKCON_ADC, true);

	ssts_initialize(sc);

	return 0;
}

int
ssts_ioctl(void *v, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct ssts_softc *sc = v;

	DPRINTF(("%s\n", __func__));

	switch (cmd) {

	case WSMOUSEIO_GTYPE:
		*(uint *)data = WSMOUSE_TYPE_PSEUDO;
		break;

	case WSMOUSEIO_GCALIBCOORDS:
	case WSMOUSEIO_SCALIBCOORDS:
		return tpcalib_ioctl(&sc->sc_tpcalib, cmd, data, flag, l);

	default:
		return EPASSTHROUGH;

	}

	return 0;
}

void
ssts_disable(void *arg)
{
	struct ssts_softc *sc = arg;

	DPRINTF(("%s\n", __func__));

	/*
	 * By setting ADCCON register to 0, we also disable
	 * the prescaler, which should disable any interrupts.
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCCON, 0);

	/* Disable ADC module clock */
	s3c24x0_clkman_config(CLKCON_ADC, false);
}

void
ssts_callout(void *arg)
{
	struct ssts_softc *sc = arg;

	DPRINTF(("%s\n", __func__));

	/* If stylus is down, perform a measurement */
	if (sc->sc_next_stylus_intr == STYLUS_UP) {
		uint32_t reg;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCTSC,
				  ADCTSC_YM_SEN | ADCTSC_YP_SEN |
				  ADCTSC_XP_SEN | ADCTSC_PULL_UP |
				  ADCTSC_AUTO_PST);

		reg = bus_space_read_4(sc->sc_iot, sc->sc_ioh, ADC_ADCCON);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCCON,
				  reg | ADCCON_ENABLE_START);
	}

}

/* Do some very simple filtering on the measured values */
int
ssts_filter_values(int *vals, int val_count)
{
	int sum = 0;

	if (val_count < 5)
		return -1;

	for (int i=0; i<val_count; i++) {
		sum += vals[i];
	}

	return sum/val_count;
}

void
ssts_initialize(struct ssts_softc *sc)
{
	int prescaler;
	uint32_t adccon = 0;
	uint32_t adctsc = 0;

	/*
	 * ADC Conversion rate is calculated by:
	 * f(ADC) = PCLK/(prescaler+1)
	 * The ADC can operate at a maximum frequency of 2.5MHz for
	 * 500 KSPS.
	 */

	/* Set f(ADC) = 50MHz / 256 = 1,95MHz */
	prescaler = 0xff;

	adccon |= ((prescaler<<ADCCON_PRSCVL_SHIFT) &
		   ADCCON_PRSCVL_MASK);
	adccon |= ADCCON_PRSCEN;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCCON, adccon);

	/* Use Auto Sequential measurement of X and Y positions */
	adctsc |= ADCTSC_YM_SEN | ADCTSC_YP_SEN | ADCTSC_XP_SEN |
		sc->sc_next_stylus_intr |
		3; /* 3 selects "Waiting for Interrupt Mode" */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCTSC, adctsc);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCUPDN, 0x0);

	/* Time used to measure each X/Y position value? */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, ADC_ADCDLY, 10000);
}
