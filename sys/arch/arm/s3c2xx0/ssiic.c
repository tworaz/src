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

#include <dev/i2c/i2cvar.h>

#include <arm/s3c2xx0/s3c24x0var.h>
#include <arm/s3c2xx0/s3c24x0_i2c.h>
#include <arm/s3c2xx0/s3c24x0_intr.h>

#include "locators.h"

#define S3C24X0_I2C_SIZE PAGE_SIZE

struct ssiic_softc {
	struct s3c24x0_i2c_softc sc_s3c_i2c;
	struct i2c_controller	sc_i2c;
	kmutex_t		sc_lock;
	int			sc_intr;
};

int	ssiic_match(device_t, cfdata_t, void *);
void	ssiic_attach(device_t, device_t, void *);
int	ssiic_acquire_bus(void *, int);
void	ssiic_release_bus(void *, int);
int	ssiic_exec(void *, i2c_op_t, i2c_addr_t, const void *, size_t,
	           void *, size_t, int);

CFATTACH_DECL_NEW(ssiic, sizeof(struct ssiic_softc), ssiic_match,
                  ssiic_attach, NULL, NULL);

int
ssiic_match(device_t parent, cfdata_t cf, void *aux)
{
	struct s3c2xx0_attach_args *sa = aux;

	sa->sa_size = S3C24X0_I2C_SIZE;

	if (sa->sa_intr == SSIOCF_INTR_DEFAULT)
		return 0;

	return 1;
}

void
ssiic_attach(device_t parent, device_t self, void *aux)
{
	struct ssiic_softc		*sc = device_private(self);
	struct s3c2xx0_attach_args 	*sa = aux;
	struct i2cbus_attach_args 	iba;

	aprint_normal("\n");
	aprint_naive("\n");

	sc->sc_s3c_i2c.sc_dev = self;
	sc->sc_s3c_i2c.sc_iot = sa->sa_iot;
	sc->sc_s3c_i2c.sc_addr = sa->sa_addr;
	sc->sc_s3c_i2c.sc_size = sa->sa_size;
	if (s3c24x0_i2c_attach_sub(&sc->sc_s3c_i2c)) {
		aprint_error_dev(self, "unable to attach S3C24x0 I2C\n");
		return;
	}

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	sc->sc_i2c.ic_cookie = sc;
	sc->sc_i2c.ic_acquire_bus = ssiic_acquire_bus;
	sc->sc_i2c.ic_release_bus = ssiic_release_bus;
	sc->sc_i2c.ic_send_start = NULL;
	sc->sc_i2c.ic_send_stop = NULL;
	sc->sc_i2c.ic_initiate_xfer = NULL;
	sc->sc_i2c.ic_read_byte = NULL;
	sc->sc_i2c.ic_write_byte = NULL;
	sc->sc_i2c.ic_exec = ssiic_exec;

	sc->sc_intr = sa->sa_intr;
	s3c24x0_intr_establish(sc->sc_intr, IPL_SERIAL, IST_LEVEL,
	                       s3c24x0_i2c_intr, sc);

	iba.iba_tag = &sc->sc_i2c;
	config_found_ia(self, "i2cbus", &iba, iicbus_print);
}

int
ssiic_acquire_bus(void *cookie, int flags)
{
	struct ssiic_softc *sc = cookie;

	KASSERT(!mutex_owned(&sc->sc_lock));

	mutex_enter(&sc->sc_lock);

	if (flags & I2C_F_POLL)
		s3c24x0_intr_mask(sc->sc_intr);

	s3c24x0_i2c_open(&sc->sc_s3c_i2c);

	return 0;
}

void
ssiic_release_bus(void *cookie, int flags)
{
	struct ssiic_softc *sc = cookie;

	KASSERT(mutex_owned(&sc->sc_lock));

	s3c24x0_i2c_close(&sc->sc_s3c_i2c);

	if (flags & I2C_F_POLL)
		s3c24x0_intr_unmask(sc->sc_intr);

	mutex_exit(&sc->sc_lock);

	return;
}

int
ssiic_exec(void *cookie, i2c_op_t op, i2c_addr_t addr, const void *vcmd,
	   size_t cmdlen, void *vbuf, size_t buflen, int flags)
{
	struct ssiic_softc	*sc = cookie;
	const uint8_t		*cmd = vcmd;
	uint8_t			*buf = vbuf;

	KASSERT(mutex_owned(&sc->sc_lock));

	sc->sc_s3c_i2c.sc_slave_addr = addr;

	if (I2C_OP_READ_P(op) && (cmdlen == 1) && (buflen >= 1)){
		return s3c24x0_i2c_read_x(&sc->sc_s3c_i2c, *cmd, buf, buflen, flags);
	}

	if (I2C_OP_WRITE_P(op) && (cmdlen == 1) && (buflen >= 1)) {
		return s3c24x0_i2c_write_x(&sc->sc_s3c_i2c, *cmd, buf, buflen, flags);
	}

	KASSERT(0 && "Transfer type not implemented!");

	return -1;
}
