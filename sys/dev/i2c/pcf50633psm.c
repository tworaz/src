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

/*
 * Power Supply Module driver for NXP/Philips PCF50633.
 *
 * This driver is responsible for:
 * - configuration of PCF50633 linear voltage regulators and step-down converters,
 * - monitoring relevant output levels during drevice operation.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <dev/i2c/pcf50633reg.h>
#include <dev/i2c/pcf50633var.h>
#include <dev/sysmon/sysmon_taskq.h>

/* Forward declarations of local functions */
static int	psm_configure_converters(struct pcf50633_psm_softc *sc);
static int	psm_configure_ldos(struct pcf50633_psm_softc *sc);
static int	psm_configure_ldos_stby(struct pcf50633_psm_softc *sc);
static int	psm_configure_debpf(struct pcf50633_psm_softc *sc);
static int	psm_configure_sensors(struct pcf50633_psm_softc *sc);
static void	psm_status_refresh(void *);

static inline uint8_t	psm_auto_voltage_to_bits(unsigned mvolt);
static inline uint8_t	psm_auto_curlim_to_bits(unsigned mamp);
static inline uint8_t	psm_down_voltage_to_bits(unsigned mvolt);
static inline uint8_t	psm_down_curlim_to_bits(unsigned mamp);
static inline uint8_t	psm_ldo_voltage_to_bits(unsigned mvolt);

struct intr_def {
	uint8_t		reg;
	uint8_t 	bit;
	uint16_t	lvl;
	void		(* handler)(void *);
};

#define _(l, r, b) { .reg = r, .bit = b, .handler = psm_status_refresh, .lvl = l }
struct intr_def intr_defs[] = {
	_(10, PCF50633_INT4, INT4_AUTOPWRFAIL_BIT),
	_(10, PCF50633_INT4, INT4_DWN1PWRFAIL_BIT),
	_(10, PCF50633_INT4, INT4_DWN2PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO1PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO2PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO3PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO4PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO5PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_LDO6PWRFAIL_BIT),
	_(10, PCF50633_INT5, INT5_HCLDOPWRFAIL_BIT),
};
#undef _

int
pcf50633_psm_attach_sub(struct pcf50633_psm_softc *sc)
{
	struct intr_def *def;
	int i, error;

	aprint_normal(": PCF50633 Power Supply Module\n");

	if (psm_configure_converters(sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to configure step-down coverters\n");
		return 1;
	}

	if (psm_configure_ldos(sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to configure LDOs\n");
		return 1;
	}

	if (psm_configure_ldos_stby(sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to configure LDO standby mode\n");
		return 1;
	}

	if (psm_configure_debpf(sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to configure debounce filters\n");
		return 1;
	}

	for (i = 0; i < __arraycount(intr_defs); i++) {
		def = &intr_defs[i];
		error = pcf50633bus_intr_establish(def->reg, def->bit, def->lvl,
		                                   def->handler, sc);
		if (error) {
			aprint_error_dev(sc->sc_dev, "Failed to configure interrupts\n");
			return 1;
		}
	}

	sc->sc_sme = sysmon_envsys_create();

	if (psm_configure_sensors(sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to configure sensors\n");
		goto fail;
	}

	sc->sc_sme->sme_cookie = sc;
	sc->sc_sme->sme_name = device_xname(sc->sc_dev);
	sc->sc_sme->sme_flags = SME_DISABLE_REFRESH;

	if (sysmon_envsys_register(sc->sc_sme) != 0) {
		goto fail;
	}

	if (sysmon_task_queue_sched(0, psm_status_refresh, sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to schedule sensor refresh!");
		goto fail;
	}

	return 0;

fail:
	sysmon_envsys_destroy(sc->sc_sme);
	sc->sc_sme = NULL;
	return 1;
}

int
pcf50633_psm_detach_sub(struct pcf50633_psm_softc *sc)
{
	struct intr_def *def;
	int i;

	for (i = 0; i < __arraycount(intr_defs); i++) {
		def = &intr_defs[i];
		pcf50633bus_intr_disestablish(def->reg, def->bit);
	}

	if (sc->sc_sme != NULL) {
		sysmon_envsys_unregister(sc->sc_sme);
		sysmon_envsys_destroy(sc->sc_sme);
		sc->sc_sme = NULL;
	}

	return 0;
}

int
pcf50633_psm_ldo_enable(struct pcf50633_psm_softc *sc, pcf50633_psm_ldo_t ldo, bool en)
{
	pcf50633_ldo_cfg_t *cfg = sc->sc_ldo_cfg[ldo];
	uint8_t ldoena, destreg;
	int curV;

	if (cfg == NULL) {
		aprint_error_dev(sc->sc_dev, "LDO%d not configured.", ldo);
		return 1;
	}

	if (en && cfg->enable == PCF50633_PSM_OFF) {
		cfg->enable = PCF50633_PSM_ON;
	} else if (!en && cfg->enable == PCF50633_PSM_ON) {
		cfg->enable = PCF50633_PSM_OFF;
	} else {
		return 0;
	}

	ldoena = cfg->enable | (cfg->actph << LDOxENA_ACTPH_SHIFT);

	if (ldo == PCF50633_PSM_LDO_MEM) {
		destreg = PCF50633_MEMLDOENA;
	} else {
		destreg = PCF50633_LDOxENA(ldo);
	}

	if (pcf50633bus_write(destreg, &ldoena, 1, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write LDO%d config\n", ldo);
		return 1;
	}

	if (en) {
		curV = cfg->voltage * 1000;
	} else {
		curV = 0;
	}
	sc->sc_ldo_sensor[ldo].value_cur = curV;
	sc->sc_ldo_sensor[ldo].state = ENVSYS_SVALID;

	if (sysmon_task_queue_sched(0, psm_status_refresh, sc)) {
		aprint_error_dev(sc->sc_dev, "Failed to schedule sensor refresh!");
		return 1;
	}

	return 0;
}

static int
psm_configure_converters(struct pcf50633_psm_softc *sc)
{
	pcf50633_conv_cfg_t *cfg = NULL;
	uint8_t regs[4];

	if (sc->sc_conv_cfg[PCF50633_PSM_CONV_AUTO]) {
		cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_AUTO];
		regs[0] = psm_auto_voltage_to_bits(cfg->voltage);
		regs[1] = cfg->enable | (cfg->actph << AUTOENA_ACTPH_SHIFT);
		regs[2] = cfg->ctlreg_val;
		regs[3] = psm_auto_curlim_to_bits(cfg->curlimit_val) |
		          (cfg->curlimit_mode << AUTOMXC_MODE_SHIFT);
	} else {
		memset(regs, 0, 4);
		regs[1] = PCF50633_PSM_OFF;
	}

	if (pcf50633bus_write(PCF50633_AUTOOUT, regs, 4, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write AUTO converter config\n");
		return 1;
	}

	if (sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN1]) {
		cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN1];
		regs[0] = psm_down_voltage_to_bits(cfg->voltage);
		regs[1] = cfg->enable | (cfg->actph << DOWN1ENA_ACTPH_SHIFT);
		regs[2] = cfg->ctlreg_val;
		regs[3] = psm_down_curlim_to_bits(cfg->curlimit_val) |
		          (cfg->curlimit_mode << DOWN1MXC_MODE_SHIFT);
	} else {
		memset(regs, 0, 4);
		regs[1] = PCF50633_PSM_OFF;
	}

	if (pcf50633bus_write(PCF50633_DOWN1OUT, regs, 4, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write DOWN1 converter config\n");
		return 1;
	}

	if (sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN2]) {
		cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN2];
		regs[0] = psm_down_voltage_to_bits(cfg->voltage);
		regs[1] = cfg->enable | (cfg->actph << DOWN2ENA_ACTPH_SHIFT);
		regs[2] = cfg->ctlreg_val;
		regs[3] = psm_down_curlim_to_bits(cfg->curlimit_val) |
		          (cfg->curlimit_mode << DOWN2MXC_MODE_SHIFT);
	} else {
		memset(regs, 0, 4);
		regs[1] = PCF50633_PSM_OFF;
	}

	if (pcf50633bus_write(PCF50633_DOWN2OUT, regs, 4, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write DOWN2 converter config\n");
		return 1;
	}

	return 0;
}

static int
psm_configure_ldos(struct pcf50633_psm_softc *sc)
{
	pcf50633_ldo_cfg_t *cfg = NULL;
	pcf50633_psm_ldo_t ldo;
	uint8_t regs[2];
	uint8_t destreg;

	for (ldo = 0; ldo < PCF50633_PSM_LDO_COUNT; ldo++) {
		if (sc->sc_ldo_cfg[ldo]) {
			cfg = sc->sc_ldo_cfg[ldo];
			regs[0] = psm_ldo_voltage_to_bits(cfg->voltage) |
			          (cfg->mode << LDOxOUT_SWMOD_SHIFT);
			regs[1] = cfg->enable | (cfg->actph << LDOxENA_ACTPH_SHIFT);
		} else {
			memset(regs, 0, 2);
			regs[1] = PCF50633_PSM_OFF;
		}

		if (ldo == PCF50633_PSM_LDO_MEM) {
			destreg = PCF50633_MEMLDOOUT;
		} else {
			destreg = PCF50633_LDOxOUT(ldo);
		}

		if (pcf50633bus_write(destreg, regs, 2, I2C_F_POLL)) {
			aprint_error_dev(sc->sc_dev, "Failed to write LDO%d config\n", ldo);
			return 1;
		}
	}

	return 0;
}

static int
psm_configure_ldos_stby(struct pcf50633_psm_softc *sc)
{
	uint8_t regs[2];

	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_1] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_1]->standby_on) {
		regs[0] |= STBYCTL1_LDO1_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_2] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_2]->standby_on) {
		regs[0] |= STBYCTL1_LDO2_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_3] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_3]->standby_on) {
		regs[0] |= STBYCTL1_LDO3_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_4] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_4]->standby_on) {
		regs[0] |= STBYCTL1_LDO4_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_5] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_5]->standby_on) {
		regs[1] |= STBYCTL2_LDO5_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_6] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_6]->standby_on) {
		regs[1] |= STBYCTL2_LDO6_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_HC] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_HC]->standby_on) {
		regs[1] |= STBYCTL2_HCLDO_ENA;
	}
	if (sc->sc_ldo_cfg[PCF50633_PSM_LDO_MEM] &&
	    sc->sc_ldo_cfg[PCF50633_PSM_LDO_MEM]->standby_on) {
		regs[1] |= STBYCTL2_MEMLDO_ENA;
	}

	if (pcf50633bus_write(PCF50633_STBYCTL1, regs, 2, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write STBYCTL regs\n");
		return 1;
	}

	return 0;
}

static int
psm_configure_debpf(struct pcf50633_psm_softc *sc)
{
	pcf50633_conv_cfg_t *conv_cfg;
	pcf50633_ldo_cfg_t *ldo_cfg;
	uint8_t regs[3];

	conv_cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_AUTO];
	if (conv_cfg) {
		regs[0] |= conv_cfg->debpf << DEBPF1_AUTO_SHIFT;
	}
	conv_cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN1];
	if (conv_cfg) {
		regs[0] |= conv_cfg->debpf << DEBPF1_DOWN1_SHIFT;
	}
	conv_cfg = sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN2];
	if (conv_cfg) {
		regs[0] |= conv_cfg->debpf << DEBPF1_DOWN2_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_1];
	if (ldo_cfg) {
		regs[1] |= ldo_cfg->debpf << DEBPF2_LDO1_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_2];
	if (ldo_cfg) {
		regs[1] |= ldo_cfg->debpf << DEBPF2_LDO2_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_3];
	if (ldo_cfg) {
		regs[1] |= ldo_cfg->debpf << DEBPF2_LDO3_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_4];
	if (ldo_cfg) {
		regs[1] |= ldo_cfg->debpf << DEBPF2_LDO4_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_5];
	if (ldo_cfg) {
		regs[2] |= ldo_cfg->debpf << DEBPF3_LDO5_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_6];
	if (ldo_cfg) {
		regs[2] |= ldo_cfg->debpf << DEBPF3_LDO6_SHIFT;
	}
	ldo_cfg = sc->sc_ldo_cfg[PCF50633_PSM_LDO_HC];
	if (ldo_cfg) {
		regs[2] |= ldo_cfg->debpf << DEBPF3_HCLDO_SHIFT;
	}

	if (pcf50633bus_write(PCF50633_DEBPF1, regs, 3, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to write DEBPF regs\n");
		return 1;
	}

	return 0;
}

static int
psm_configure_sensors(struct pcf50633_psm_softc *sc)
{
	int itr, rv;
	int curV;

	for (itr = 0; itr < (PCF50633_PSM_CONV_COUNT); itr++) {
		if (sc->sc_conv_cfg[itr] == NULL) {
			continue;
		}

		if (sc->sc_conv_cfg[itr]->enable == PCF50633_PSM_OFF) {
			curV = 0;
		} else {
			curV = sc->sc_conv_cfg[itr]->voltage * 1000;
		}

		sc->sc_conv_sensor[itr].state = ENVSYS_SVALID;
		sc->sc_conv_sensor[itr].units = ENVSYS_SVOLTS_DC;
		sc->sc_conv_sensor[itr].flags = ENVSYS_FMONNOTSUPP;
		sc->sc_conv_sensor[itr].value_cur = curV;

		if (sc->sc_conv_sensor[itr].desc[0] == 0) {
			if (itr == 0) {
				(void)snprintf(sc->sc_conv_sensor[itr].desc,
				               ENVSYS_DESCLEN, "AUTO Voltage");
			} else {
				(void)snprintf(sc->sc_conv_sensor[itr].desc,
				               ENVSYS_DESCLEN, "DOWN%d Voltage", itr);
			}
		}

		rv = sysmon_envsys_sensor_attach(sc->sc_sme, &sc->sc_conv_sensor[itr]);
		if (rv != 0) {
			return 1;
		}
	}

	for (itr = 0; itr < (PCF50633_PSM_LDO_COUNT - 1); itr++) {
		if (sc->sc_ldo_cfg[itr] == NULL) {
			continue;
		}

		if (sc->sc_conv_cfg[itr]->enable == PCF50633_PSM_OFF) {
			curV = 0;
		} else {
			curV = sc->sc_ldo_cfg[itr]->voltage * 1000;
		}

		sc->sc_ldo_sensor[itr].state = ENVSYS_SVALID;
		sc->sc_ldo_sensor[itr].units = ENVSYS_SVOLTS_DC;
		sc->sc_ldo_sensor[itr].flags = ENVSYS_FMONNOTSUPP;
		sc->sc_ldo_sensor[itr].value_cur = curV;

		if (sc->sc_ldo_sensor[itr].desc[0] == 0) {
			(void)snprintf(sc->sc_ldo_sensor[itr].desc, ENVSYS_DESCLEN,
				       "LDO%d Voltage", itr);
		}

		rv = sysmon_envsys_sensor_attach(sc->sc_sme, &sc->sc_ldo_sensor[itr]);
		if (rv != 0) {
			return 1;
		}
	}

	return 0;
}

static void
psm_status_refresh(void *arg)
{
	struct pcf50633_psm_softc *sc = arg;
	envsys_data_t *sensor = NULL;
	uint8_t regs[2];
	int i;

	if (pcf50633bus_read(PCF50633_DCDCSTAT, regs, 2, 0)) {
		aprint_error_dev(sc->sc_dev, "Failed to read DCDCSTAT & LDOSTAT regs\n");
		return;
	}

	for (i = PCF50633_PSM_CONV_AUTO; i < PCF50633_PSM_CONV_COUNT; i++) {
		if (sc->sc_conv_cfg[i]->enable == PCF50633_PSM_OFF) {
			continue;
		}

		sensor = &(sc->sc_conv_sensor[i]);
		if (regs[0] & (1<<i)) {
			sensor->state = ENVSYS_SCRITUNDER;
			sensor->value_cur = sc->sc_conv_cfg[i]->voltage * 900;
			sysmon_envsys_sensor_event(sc->sc_sme, sensor, ENVSYS_SCRITUNDER);
		} else if (sc->sc_last_reading[0] & (1<<i)) {
			sensor->state = ENVSYS_SVALID;
			sensor->value_cur = sc->sc_conv_cfg[i]->voltage * 1000;
			sysmon_envsys_sensor_event(sc->sc_sme, sensor, ENVSYS_SVALID);
		}
	}

	for (i = PCF50633_PSM_LDO_1; i < PCF50633_PSM_LDO_MEM; i++) {
		if (sc->sc_ldo_cfg[i]->enable == PCF50633_PSM_OFF) {
			continue;
		}

		sensor = &(sc->sc_ldo_sensor[i]);
		if (regs[1] & (1<<i)) {
			sensor->state = ENVSYS_SCRITUNDER;
			sensor->value_cur = sc->sc_ldo_cfg[i]->voltage * 900;
			sysmon_envsys_sensor_event(sc->sc_sme, sensor, ENVSYS_SCRITUNDER);
		} else if (sc->sc_last_reading[1] & (1<<i)) {
			sensor->state = ENVSYS_SVALID;
			sensor->value_cur = sc->sc_ldo_cfg[i]->voltage * 1000;
			sysmon_envsys_sensor_event(sc->sc_sme, sensor, ENVSYS_SVALID);
		}
	}

	memcpy(sc->sc_last_reading, regs, 2);
}

static inline uint8_t
psm_auto_voltage_to_bits(unsigned mvolt)
{
	if (mvolt < 1800) {
		return 0;
	} else if (mvolt > 3800) {
		return 0x3f;
	} else {
		return ((mvolt - 625) / 25);
	}
}

static inline uint8_t
psm_auto_curlim_to_bits(unsigned mamp)
{
	if (mamp > 1100) {
		mamp = 1100;
	}
	return (mamp / 40);
}

static uint8_t
psm_down_voltage_to_bits(unsigned mvolt)
{
	if (mvolt < 625) {
		return 0;
	} else if (mvolt > 3000) {
		mvolt = 3000;
	}
	return ((mvolt - 625) / 25);
}

static uint8_t
psm_down_curlim_to_bits(unsigned mamp)
{
	if (mamp > 500) {
		mamp = 500;
	}
	return (mamp / 15);
}

static inline uint8_t
psm_ldo_voltage_to_bits(unsigned mvolt)
{
	if (mvolt < 900) {
		return 0;
	} else if (mvolt > 3600) {
		return 0x1b;
	} else {
		return ((mvolt - 900) / 100);
	}
}
