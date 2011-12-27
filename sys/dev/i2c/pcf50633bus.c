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
 * Driver for NXP/Philips PCF50633 device. It acts as a pseudo bus to which
 * actual device drivers attach to. This way it's possiblewrite sub-drivers
 * utlilizing some parts of the chip in a generic way, while the rest is
 * handled by the platform specific sub-drivers.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kthread.h>

#include <dev/sysmon/sysmon_taskq.h>

#include <dev/i2c/pcf50633reg.h>
#include <dev/i2c/pcf50633var.h>

#ifdef PCF50633BUS_DEBUG
#define	DPRINTF(s)	printf s
#else
#define	DPRINTF(s)	do { } while (/*CONSTCOND*/0)
#endif /* PCF50633BUS_DEBUG */

/* Converting interrupt register & bit to interrupt number. */
#define REGBIT_TO_INTR(reg, bit) \
	((8 * (reg)) + (bit))

/* Global reference to the device state structure */
struct pcf50633bus_softc *pcf_sc = NULL;

/* Functions used internally in by the driver */
void	pcf50633bus_intr_thread(void *);
void	pcf50633bus_intr_handler(uint8_t *, uint8_t *);
int	pcf50633bus_search(device_t, cfdata_t, const int *, void *);
int	pcf50633bus_print(void *, const char *);

int
pcf50633bus_attach_sub(struct pcf50633bus_softc *sc)
{
	uint8_t intrs[PCF50633_INT_REG_N];
	uint8_t ver[2];
	int error;

	pcf_sc = sc;

	if (pcf50633bus_read(PCF50633_VERSION, ver, 2, I2C_F_POLL)) {
		aprint_error_dev(sc->sc_dev, "Failed to read PCF50633 version!\n");
		return -1;
	}

	aprint_normal_dev(sc->sc_dev, "NXP PCF50633 ver %d, rev %d\n", ver[0], ver[1]);

	/* Mask all interrupts */
	memset(sc->sc_intmask, 0xff, PCF50633_INT_REG_N);

	error = pcf50633bus_write(PCF50633_INT1MASK, sc->sc_intmask,
	                          PCF50633_INT_REG_N, I2C_F_POLL);
	if (error) {
		aprint_error_dev(sc->sc_dev, "Failed to set PCF50633 "
		                 "interrupt mask!\n");
		return -1;
	}

	/* Clear pending interrupts */
	error = pcf50633bus_read(PCF50633_INT1, intrs, PCF50633_INT_REG_N, I2C_F_POLL);
	if (error) {
		aprint_error_dev(sc->sc_dev, "Failed to clear pending PCF50633 "
		                 "interrupts!\n");
		return -1;
	}

	error = kthread_create(PRI_NONE, KTHREAD_MPSAFE, NULL, pcf50633bus_intr_thread,
	                       sc, &sc->sc_intr_lwp, "pcf50633bus_intr");
	if (error) {
		aprint_error_dev(sc->sc_dev, "Unable to create PCF50633 "
		                 "interrupt handling thread!\n");
		return -1;
	}

	sysmon_task_queue_init();

	cv_init(&sc->sc_cv, "pcf50633bus_cv");
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);

	config_search_ia(pcf50633bus_search, sc->sc_dev, "pcf50633bus", NULL);

	return 0;
}

int
pcf50633bus_detach_sub(struct pcf50633bus_softc *sc)
{
	int error;

	/* Mask all interrupts */
	memset(sc->sc_intmask, 0xff, PCF50633_INT_REG_N);

	error = pcf50633bus_write(PCF50633_INT1MASK, sc->sc_intmask,
	                          PCF50633_INT_REG_N, I2C_F_POLL);
	if (error) {
		device_printf(sc->sc_dev, "Failed to mask PCF50633 interrupts!\n");
		return -1;
	}

	mutex_spin_enter(&sc->sc_lock);
	sc->sc_intr_count = -1;
	cv_broadcast(&sc->sc_cv);
	mutex_spin_exit(&sc->sc_lock);

	mutex_destroy(&sc->sc_lock);
	cv_destroy(&sc->sc_cv);

	return 0;
}

int
pcf50633bus_intr(void *arg)
{
	struct pcf50633bus_softc *sc = arg;

	mutex_spin_enter(&sc->sc_lock);
	sc->sc_intr_count++;
	cv_broadcast(&sc->sc_cv);
	mutex_spin_exit(&sc->sc_lock);

	return 1;
}

int
pcf50633bus_read(uint8_t reg, uint8_t *buf, size_t len, int flags)
{
	struct pcf50633bus_softc *sc = pcf_sc;

	KASSERT(sc);

	if (iic_acquire_bus(sc->sc_tag, flags)) {
		return -1;
	}

	if (iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP, sc->sc_addr,
	             &reg, 1, buf, len, flags)) {
		iic_release_bus(sc->sc_tag, flags);
		return -1;
	}

	iic_release_bus(sc->sc_tag, flags);

	return 0;
}

int
pcf50633bus_write(uint8_t reg, uint8_t *buf, size_t len, int flags)
{
	struct pcf50633bus_softc *sc = pcf_sc;

	KASSERT(sc);

	if (iic_acquire_bus(sc->sc_tag, flags)) {
		return -1;
	}

	if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
	             &reg, 1, buf, len, flags)) {
		iic_release_bus(sc->sc_tag, flags);
		return -1;
	}

	iic_release_bus(sc->sc_tag, flags);

	return 0;
}

void
pcf50633bus_intr_thread(void *arg)
{
	struct pcf50633bus_softc *sc = arg;
	uint8_t intr[PCF50633_INT_REG_N];
	uint8_t mask[PCF50633_INT_REG_N];
	int intr_count;
	int error;

	DPRINTF(("PCF50633 interrupt thread started.\n"));

	for (;;) {
		mutex_spin_enter(&sc->sc_lock);
		intr_count = sc->sc_intr_count;
		memcpy(mask, sc->sc_intmask, PCF50633_INT_REG_N);
		sc->sc_intr_count = 0;
		mutex_spin_exit(&sc->sc_lock);

		if (intr_count > 0) {
			error = pcf50633bus_read(PCF50633_INT1, intr,
			                         PCF50633_INT_REG_N, 0);
			if (error) {
				device_printf(sc->sc_dev, "Failed to read PCF50633 "
				              "interrupt status registers!\n");
				continue;
			}
			pcf50633bus_intr_handler(intr, mask);
		} else if (intr_count == 0) {
			mutex_spin_enter(&sc->sc_lock);
			cv_wait(&sc->sc_cv, &sc->sc_lock);
			mutex_spin_exit(&sc->sc_lock);
		} else {
			break;
		}
	}

	DPRINTF(("PCF50633 interrupt thread exiting.\n"));

	kthread_exit(0);
}

void
pcf50633bus_intr_handler(uint8_t *intr, uint8_t *mask)
{
	struct pcf50633bus_softc *sc = pcf_sc;
	int bit, intno, level, i;
	void *farg, *func;
	uint8_t actintr;

	for (i = 0; i < PCF50633_INT_REG_N; i++) {
		actintr = (intr[i] & ~mask[i]);
		if (actintr) {
			for (bit = 0; bit < 8; bit++) {
				if ((1U << bit) & actintr) {
					intno = REGBIT_TO_INTR(i, bit);
					farg = sc->sc_handler[intno].arg;
					func = sc->sc_handler[intno].func;
					level = sc->sc_handler[intno].level;

					if (sysmon_task_queue_sched(level, func, farg)) {
						printf("Failed to schedule PCF50633"
						       "interupt task!");
					}
				}
			}
		}
	}
}

int
pcf50633bus_intr_establish(int reg, int bit, int lvl, void (*func)(void *), void *arg)
{
	struct pcf50633bus_softc *sc = pcf_sc;
	int intr;

	reg -= PCF50633_INT1;
	intr = REGBIT_TO_INTR(reg, bit);

	KASSERT(sc);
	KASSERT((intr < PCF50633_INTRS_NO) && (intr >= 0));

	if (sc->sc_handler[intr].func != NULL) {
		DPRINTF(("Interrupt handler for intr %d already registered!\n", intr));
		return -1;
	}

	sc->sc_intmask[reg] &= ~(1<<bit);
	sc->sc_handler[intr].func = func;
	sc->sc_handler[intr].arg = arg;
	sc->sc_handler[intr].level = lvl;

	if (pcf50633bus_write(PCF50633_INT1MASK, sc->sc_intmask,
	                      PCF50633_INT_REG_N, I2C_F_POLL) != 0) {
		DPRINTF(("Failed to unmask interrupt %d!\n", intr));
		sc->sc_handler[intr].func = NULL;
		sc->sc_intmask[reg] |= (1<<bit);
		return -1;
	}

	return 0;
}

int
pcf50633bus_intr_disestablish(int reg, int bit)
{
	struct pcf50633bus_softc *sc = pcf_sc;
	int intr;

	reg -= PCF50633_INT1;
	intr = REGBIT_TO_INTR(reg, bit);

	KASSERT(sc);
	KASSERT(intr < PCF50633_INTRS_NO);

	if (sc->sc_handler[intr].func == NULL) {
		DPRINTF(("Interrupt handler for intr %d not registered!\n", intr));
		return -1;
	}

	sc->sc_intmask[reg] |= (1<<bit);
	if (pcf50633bus_write(PCF50633_INT1MASK, sc->sc_intmask,
	                       PCF50633_INT_REG_N, I2C_F_POLL) != 0) {
		DPRINTF(("Failed to mask interrupt %d!\n", intr));
		sc->sc_intmask[reg] &= ~(1<<bit);
		return -1;
	}

	sc->sc_handler[intr].func = NULL;
	sc->sc_handler[intr].arg = NULL;
	sc->sc_handler[intr].level = 0;

	return 0;

}

int
pcf50633bus_search(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	if (config_match(parent, cf, NULL))
		config_attach(parent, cf, NULL, pcf50633bus_print);

	return 0;
}

int
pcf50633bus_print(void *aux, const char *name)
{

	return UNCONF;
}
