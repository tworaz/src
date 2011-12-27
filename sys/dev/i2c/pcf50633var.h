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

#ifndef _PCF50633VAR_H_
#define _PCF50633VAR_H_

#include <sys/mutex.h>
#include <sys/device.h>
#include <sys/condvar.h>

#include <dev/i2c/i2cvar.h>
#include <dev/i2c/pcf50633reg.h>

#include <dev/sysmon/sysmonvar.h>

/*
 * Pseudo BUS device used by sub-drivers.
 */
struct pcf50633bus_softc {
	device_t	sc_dev;
	i2c_tag_t	sc_tag;
	i2c_addr_t	sc_addr;

	uint8_t		sc_intmask[PCF50633_INT_REG_N];
	struct lwp	*sc_intr_lwp;
	int		sc_intr_count;
	kmutex_t	sc_lock;
	kcondvar_t	sc_cv;

	struct pcf50633_handler {
		void (* func)(void *);
		void *arg;
		int level;
	} sc_handler[PCF50633_INTRS_NO];
};

int	pcf50633bus_attach_sub(struct pcf50633bus_softc *);
int	pcf50633bus_detach_sub(struct pcf50633bus_softc *);
int	pcf50633bus_intr(void *);

/* Functions used by child devices */
int	pcf50633bus_read(uint8_t reg, uint8_t *data, size_t sz, int flags);
int	pcf50633bus_write(uint8_t reg , uint8_t *data, size_t sz, int flags);
int	pcf50633bus_intr_establish(int, int, int, void (*f)(void *), void *);
int	pcf50633bus_intr_disestablish(int, int);

/*
 * Power Supply Module
 */
typedef enum {
	PCF50633_PSM_CONV_AUTO,
	PCF50633_PSM_CONV_DOWN1,
	PCF50633_PSM_CONV_DOWN2,
	PCF50633_PSM_CONV_COUNT
} pcf50633_psm_conv_t;

typedef enum {
	PCF50633_PSM_LDO_1 = 0,
	PCF50633_PSM_LDO_2,
	PCF50633_PSM_LDO_3,
	PCF50633_PSM_LDO_4,
	PCF50633_PSM_LDO_5,
	PCF50633_PSM_LDO_6,
	PCF50633_PSM_LDO_HC,
	PCF50633_PSM_LDO_MEM,
	PCF50633_PSM_LDO_COUNT
} pcf50633_psm_ldo_t;

typedef enum {
	PCF50633_PSM_LDO_LINEAR_REGULATOR	= 0x0,
	PCF50633_PSM_LDO_SWITCH			= 0x1
} pcf50633_ldo_mode_t;

typedef enum {
	PCF50633_PSM_OFF 	= 0x0,
	PCF50633_PSM_ON		= 0x1,
	PCF50633_PSM_ON_GPIO1	= 0x2,
	PCF50633_PSM_ON_GPIO2	= 0x4,
	PCF50633_PSM_ON_GPIO3	= 0x8
} pcf50633_enab_t;

typedef enum {
	PCF50633_PSM_ACTPH1	= 0x0,
	PCF50633_PSM_ACTPH2	= 0x1,
	PCF50633_PSM_ACTPH3	= 0x2,
	PCF50633_PSM_ACTPH4	= 0x3
} pcf50633_actph_t;

typedef enum {
	PCF50633_PSM_CONV_LIMIT_STARTUP	= 0x0,
	PCF50633_PSM_CONV_LIMIT_ALWAYS	= 0x1
} pcf50633_conv_curlim_t;

typedef enum {
	PCF50633_PSM_DEBPF_NONE		= 0x0,
	PCF50633_PSM_DEBPF_1MS		= 0x1,
	PCF50633_PSM_DEBPF_10MS		= 0x2,
	PCF50633_PSM_DEBPF_100MS	= 0x3
} pcf50633_debpf_t;

typedef struct pcf50633_ldo_cfg {
	unsigned		voltage;
	pcf50633_ldo_mode_t	mode;
	pcf50633_enab_t		enable;
	pcf50633_actph_t	actph;
	bool			standby_on;
	pcf50633_debpf_t	debpf;
} pcf50633_ldo_cfg_t;

typedef struct pcf50633_conv_cfg {
	unsigned		voltage;
	pcf50633_enab_t		enable;
	pcf50633_actph_t	actph;
	uint8_t			ctlreg_val;
	unsigned		curlimit_val;
	pcf50633_conv_curlim_t	curlimit_mode;
	pcf50633_debpf_t	debpf;
} pcf50633_conv_cfg_t;

struct pcf50633_psm_softc {
	device_t	sc_dev;

	pcf50633_conv_cfg_t	*sc_conv_cfg[PCF50633_PSM_CONV_COUNT];
	pcf50633_ldo_cfg_t	*sc_ldo_cfg[PCF50633_PSM_LDO_COUNT];

	struct sysmon_envsys	*sc_sme;
	envsys_data_t		sc_conv_sensor[PCF50633_PSM_CONV_COUNT];
	envsys_data_t		sc_ldo_sensor[PCF50633_PSM_LDO_COUNT - 1];

	uint8_t			sc_last_reading[2];
};

int	pcf50633_psm_attach_sub(struct pcf50633_psm_softc *sc);
int	pcf50633_psm_detach_sub(struct pcf50633_psm_softc *sc);
int	pcf50633_psm_ldo_enable(struct pcf50633_psm_softc *sc, pcf50633_psm_ldo_t ldo,
                                bool enable);

#endif /* !_PCF50633VAR_H */
