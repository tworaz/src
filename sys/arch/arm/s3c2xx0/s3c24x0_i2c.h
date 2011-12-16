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

#ifndef _S3C24X0_I2C_H_
#define _S3C24X0_I2C_H_

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/queue.h>

#include <dev/i2c/i2cvar.h>

/* IIC operation types */
#define S3CIIC_OP_WRITE	0
#define S3CIIC_OP_READ	1

/* Error types */
#define S3CIIC_ERR_ACK	1
#define S3CIIC_ERR_ARB	2

enum s3c24x0_i2c_state {
	S3CIIC_START,
	S3CIIC_READ,
	S3CIIC_WRITE,
	S3CIIC_STOP
};

struct s3c24x0_i2c_op {
	SLIST_ENTRY(s3c24x0_i2c_op) op_link;
	uint8_t	op_type;
	uint8_t	op_data;
};

struct s3c24x0_i2c_softc {
	device_t		sc_dev;
	i2c_addr_t		sc_slave_addr;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	bus_addr_t		sc_addr;
	bus_size_t		sc_size;

	kmutex_t		sc_irq_lock;
	kcondvar_t		sc_irq_cv;
	enum s3c24x0_i2c_state	sc_state;
	int			sc_err;
	bool			sc_use_intr;

	SLIST_HEAD(, s3c24x0_i2c_op)	sc_oplist;
	struct s3c24x0_i2c_op		*sc_curop;
};

int	s3c24x0_i2c_attach_sub(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_detach_sub(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_open(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_close(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_intr(void *);
int	s3c24x0_i2c_read_x(struct s3c24x0_i2c_softc *, uint8_t,
                           uint8_t *, size_t, int);
int	s3c24x0_i2c_write_x(struct s3c24x0_i2c_softc*, uint8_t,
                            uint8_t *, size_t, int);

#ifdef _S3C24X0_I2C_PRIVATE
void	s3c24x0_i2c_init(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_calc_divs(uint32_t, uint32_t *, uint32_t *);
int	s3c24x0_i2c_xfer_intr(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_xfer_poll(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_busy_wait(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_irq_enable(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_irq_disable(struct s3c24x0_i2c_softc *);
int	s3c24x0_i2c_irq_wait(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_pending_clear(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_start(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_stop(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_ack_enable(struct s3c24x0_i2c_softc *);
void	s3c24x0_i2c_ack_disable(struct s3c24x0_i2c_softc *);
#endif /* _S3C24X0_I2C_PRIVATE */

#endif /* !_S3C24X0_I2C_H_ */
