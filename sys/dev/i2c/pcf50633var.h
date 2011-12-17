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
int	pcf50633bus_read(uint8_t, uint8_t *, size_t, int);
int	pcf50633bus_write(uint8_t, uint8_t *, size_t, int);
int	pcf50633bus_intr_establish(int, int, int, void (*f)(void *), void *);
int	pcf50633bus_intr_disestablish(int, int);

#endif /* !_PCF50633VAR_H */
