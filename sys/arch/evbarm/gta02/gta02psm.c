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

#include <dev/i2c/pcf50633var.h>

struct gta02psm_softc {
	struct pcf50633_psm_softc psm_sc;
};

pcf50633_conv_cfg_t gta02_auto_cfg = {
	.voltage	= 3300,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.ctlreg_val	= 0x00,
	.curlimit_val	= 400,
	.curlimit_mode	= PCF50633_PSM_CONV_LIMIT_ALWAYS,
	.debpf		= PCF50633_PSM_DEBPF_NONE
};

pcf50633_conv_cfg_t gta02_down1_cfg = {
	.voltage	= 1300,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.ctlreg_val	= 0x00,
	.curlimit_val	= 390,
	.curlimit_mode	= PCF50633_PSM_CONV_LIMIT_ALWAYS,
	.debpf		= PCF50633_PSM_DEBPF_NONE
};

pcf50633_conv_cfg_t gta02_down2_cfg = {
	.voltage	= 1800,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.ctlreg_val	= 0x00,
	.curlimit_val	= 390,
	.curlimit_mode	= PCF50633_PSM_CONV_LIMIT_ALWAYS,
	.debpf		= PCF50633_PSM_DEBPF_NONE
};

pcf50633_ldo_cfg_t gta02_ldo1_gsensor_cfg = {
	.voltage	= 3300,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_OFF,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= false,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_ldo2_codec_cfg = {
	.voltage	= 3300,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_OFF,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= false,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_ldo4_bt_cfg = {
	.voltage	= 3200,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_OFF,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= false,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_ldo5_rf_cfg = {
	.voltage	= 3000,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= true,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_ldo6_lcm_cfg = {
	.voltage	= 3000,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= false,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_hcldo_sd_cfg = {
	.voltage	= 3300,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_OFF,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= false,
	.debpf		= PCF50633_PSM_DEBPF_10MS
};

pcf50633_ldo_cfg_t gta02_memldo_cfg = {
	.voltage	= 1800,
	.mode		= PCF50633_PSM_LDO_LINEAR_REGULATOR,
	.enable		= PCF50633_PSM_ON,
	.actph		= PCF50633_PSM_ACTPH1,
	.standby_on	= true,
	.debpf		= PCF50633_PSM_DEBPF_NONE
};

static int	gta02psm_match(device_t, cfdata_t, void *);
static void	gta02psm_attach(device_t, device_t, void *);
static int	gta02psm_detach(device_t, int);

CFATTACH_DECL_NEW(gta02psm, sizeof (struct gta02psm_softc),
                  gta02psm_match, gta02psm_attach, gta02psm_detach, NULL);

static int
gta02psm_match(device_t parent, cfdata_t cf, void *aux)
{
	if (cf->cf_unit > 0)
		return 0;

	return 1;
}

static void
gta02psm_attach(device_t parent, device_t self, void *aux)
{
	struct gta02psm_softc *sc = device_private(self);
	struct pcf50633_psm_softc *psm_sc = &sc->psm_sc;

	memset(&sc->psm_sc, 0, sizeof(struct pcf50633_psm_softc));

	sc->psm_sc.sc_dev = self;

	/* Step-down converters */
	psm_sc->sc_conv_cfg[PCF50633_PSM_CONV_AUTO] = &gta02_auto_cfg;
	psm_sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN1] = &gta02_down1_cfg;
	psm_sc->sc_conv_cfg[PCF50633_PSM_CONV_DOWN2] = &gta02_down2_cfg;

	/* LDOs */
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_1] = &gta02_ldo1_gsensor_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_2] = &gta02_ldo2_codec_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_3] = NULL; /* Not used */
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_4] = &gta02_ldo4_bt_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_5] = &gta02_ldo5_rf_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_6] = &gta02_ldo6_lcm_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_HC] = &gta02_hcldo_sd_cfg;
	psm_sc->sc_ldo_cfg[PCF50633_PSM_LDO_MEM] = &gta02_memldo_cfg;

	(void)snprintf(psm_sc->sc_conv_sensor[PCF50633_PSM_CONV_AUTO].desc,
	               ENVSYS_DESCLEN, "IO 3V3");
	(void)snprintf(psm_sc->sc_conv_sensor[PCF50633_PSM_CONV_DOWN1].desc,
	               ENVSYS_DESCLEN, "CORE 1V3");
	(void)snprintf(psm_sc->sc_conv_sensor[PCF50633_PSM_CONV_DOWN2].desc,
	               ENVSYS_DESCLEN, "IO 1V8");

	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_1].desc,
	               ENVSYS_DESCLEN, "GSENSOR 3V3");
	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_2].desc,
	               ENVSYS_DESCLEN, "CODEC 3V3");
	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_4].desc,
	               ENVSYS_DESCLEN, "BT 3V2");
	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_5].desc,
	               ENVSYS_DESCLEN, "BT 3V");
	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_6].desc,
	               ENVSYS_DESCLEN, "LCM 3V");
	(void)snprintf(psm_sc->sc_ldo_sensor[PCF50633_PSM_LDO_HC].desc,
	               ENVSYS_DESCLEN, "SD 3V3");

	if (pcf50633_psm_attach_sub(psm_sc)) {
		aprint_error_dev(self, "Failed to attach PCF50633 PSM device\n");
	}
}

static int
gta02psm_detach(device_t self, int flags)
{
	struct gta02psm_softc *sc = device_private(self);

	return pcf50633_psm_detach_sub(&(sc->psm_sc));
}
