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
 * I2C driver for Samsung S3C24X0 SOC's.
 *
 * Driver limitations:
 * - Only supports master mode,
 * - Doesn't support S3C2440 multi master feature.
 *
 * TODO:
 * - Transfer timeout can and probably should be calculated
 *   dynamically depending on current transmit frequency
 *   and the amount of data to transfer.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/kmem.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/condvar.h>

#include <arm/s3c2xx0/s3c2410reg.h>
#include <arm/s3c2xx0/s3c24x0var.h>
#define _S3C24X0_I2C_PRIVATE
#include <arm/s3c2xx0/s3c24x0_i2c.h>

#ifndef S3C24X0_IIC_CLK
#define S3C24X0_IIC_CLK 300000 /* Hz */
#endif /* !S3C24X0_IIC_CLK */

#ifndef S3C24X0_IIC_BUSY_WAIT
/* How long to wait for bus becoming free */
#define S3C24X0_IIC_BUSY_WAIT 1000 /* ms */
#endif /* !S3C24X0_IIC_BUSY_WAIT */

#ifndef S3C24X0_IIC_XFER_TIMEOUT
#define S3C24X0_IIC_XFER_TIMEOUT 400 /* ms */
#endif /* !S3C24X0_IIC_XFER_TIMEOUT */

/* How long to wait for interrupt in POLL mode. */
#define S3C24X0_IIC_IRQ_WAIT 400 /* ms */

/* Number of retries in situation where the transfer fails. */
#define S3C24X0_IIC_XFER_RETRIES 10

#ifdef S3C24X0_IIC_DEBUG
#define	DPRINTF(s)	printf s
#define DPRINT_REGS(sc) \
	printf("iiccon=%02x, iicstat=%02x\n", \
	       bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON), \
	       bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT) \
	      );
#else
#define	DPRINTF(s)	do { } while (/*CONSTCOND*/0)
#define DPRINT_REGS(sc) do { } while (/*CONSTCOND*/0)
#endif /* S3C24X0_IIC_DEBUG */

int
s3c24x0_i2c_attach_sub(struct s3c24x0_i2c_softc *sc)
{
	int error;

	error = bus_space_map(sc->sc_iot, sc->sc_addr, sc->sc_size, 0,
	    &sc->sc_ioh);
	if (error) {
		aprint_error_dev(sc->sc_dev, "unable to map registers!\n");
		sc->sc_size = 0;
		return error;
	}

	bus_space_barrier(sc->sc_iot, sc->sc_ioh, 0, sc->sc_size,
	    BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE);

	mutex_init(&sc->sc_irq_lock, MUTEX_DEFAULT, IPL_SERIAL);
	cv_init(&sc->sc_irq_cv, "iicirqwt");
	SLIST_INIT(&sc->sc_oplist);

	s3c24x0_i2c_init(sc);

	return 0;
}

int
s3c24x0_i2c_detach_sub(struct s3c24x0_i2c_softc *sc)
{
	s3c24x0_i2c_close(sc);

	if (sc->sc_size != 0) {
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_size);
		sc->sc_size = 0;
	}

	cv_destroy(&sc->sc_irq_cv);
	mutex_destroy(&sc->sc_irq_lock);

	s3c24x0_clkman_config(CLKCON_IIC, false);

	return 0;
}

void
s3c24x0_i2c_open(struct s3c24x0_i2c_softc *sc)
{
	/* Enable the clock to the standard I2C unit. */
	s3c24x0_clkman_config(CLKCON_IIC, true);
}

void
s3c24x0_i2c_close(struct s3c24x0_i2c_softc *sc)
{
	/* Disable the clock to the standard I2C unit. */
	s3c24x0_clkman_config(CLKCON_IIC, false);
}

int
s3c24x0_i2c_intr(void *arg)
{
	struct s3c24x0_i2c_softc *sc = arg;
	struct s3c24x0_i2c_op 	*op = sc->sc_curop;
	bus_space_tag_t		iot = sc->sc_iot;
	bus_space_handle_t	ioh = sc->sc_ioh;
	uint32_t		iicstat;

	KASSERT(sc->sc_state != S3CIIC_STOP);

	mutex_spin_enter(&sc->sc_irq_lock);

	iicstat = bus_space_read_4(iot, ioh, IIC_IICSTAT);
	if (iicstat & IICSTAT_ARBITR) {
		DPRINTF(("bus arbitration failure"));
		KASSERT(sc->sc_state == S3CIIC_START);

		sc->sc_err = S3CIIC_ERR_ARB;
		s3c24x0_i2c_irq_disable(sc);
		if (sc->sc_use_intr)
			cv_signal(&sc->sc_irq_cv);

		goto irqleave;
	} else if (iicstat & IICSTAT_LASTBIT) {
		DPRINTF(("transfer acknowledge failure"));

		sc->sc_err = S3CIIC_ERR_ACK;
		s3c24x0_i2c_stop(sc);
		if (sc->sc_use_intr)
			cv_signal(&sc->sc_irq_cv);

		goto irqleave;
	}

	/* Pefrorm action for current state */
	switch (sc->sc_state) {

	case S3CIIC_START:
		DPRINTF(("start->"));

		if (op->op_type == S3CIIC_OP_WRITE) {
			sc->sc_state = S3CIIC_WRITE;
			bus_space_write_1(iot, ioh, IIC_IICDS,
			                  op->op_data);
			delay(1);
		} else {
			sc->sc_state = S3CIIC_READ;
		}
		break;

	case S3CIIC_READ:
		op->op_data = bus_space_read_1(iot, ioh, IIC_IICDS);
		DPRINTF(("rx 0x%02x->", op->op_data));

		sc->sc_curop = op = SLIST_NEXT(sc->sc_curop, op_link);

		if (op != NULL) {
			if (op->op_type == S3CIIC_OP_WRITE)
				s3c24x0_i2c_start(sc);
		} else {
			DPRINTF(("stop"));
			s3c24x0_i2c_stop(sc);
			s3c24x0_i2c_ack_disable(sc);
			if (sc->sc_use_intr)
				cv_signal(&sc->sc_irq_cv);
		}
		break;

	case S3CIIC_WRITE:
		DPRINTF(("tx 0x%02x->", op->op_data));

		sc->sc_curop = op = SLIST_NEXT(sc->sc_curop, op_link);

		if (op != NULL) {
			if (op->op_type == S3CIIC_OP_WRITE)
				bus_space_write_1(iot, ioh, IIC_IICDS,
			                          op->op_data);
			else
				s3c24x0_i2c_start(sc);
		} else {
			DPRINTF(("stop"));
			s3c24x0_i2c_stop(sc);
			if (sc->sc_use_intr)
				cv_signal(&sc->sc_irq_cv);
		}
		break;

	default:
		KASSERT(0);

	}

irqleave:
	s3c24x0_i2c_pending_clear(sc);

	mutex_spin_exit(&sc->sc_irq_lock);

	return 1;
}

int
s3c24x0_i2c_read_x(struct s3c24x0_i2c_softc *sc, uint8_t cmd,
                   uint8_t *vp, size_t len, int flags)
{
	struct s3c24x0_i2c_op *cur, *nxt;
	int rv, i;

	mutex_spin_enter(&sc->sc_irq_lock);

	cur = kmem_alloc(sizeof(struct s3c24x0_i2c_op), KM_NOSLEEP);
	if (cur == NULL) {
		device_printf(sc->sc_dev, "Out of memory error\n");
		rv = ENOMEM;
		goto cleanup;
	}
	cur->op_type = S3CIIC_OP_WRITE;
	cur->op_data = cmd;

	SLIST_INSERT_HEAD(&sc->sc_oplist, cur, op_link);
	sc->sc_curop = cur;

	for (i = 0; i < len; i++) {
		nxt = kmem_alloc(sizeof(struct s3c24x0_i2c_op), KM_NOSLEEP);
		if (nxt == NULL) {
			device_printf(sc->sc_dev, "Out of memory error\n");
			rv = ENOMEM;
			goto cleanup;
		}
		nxt->op_type = S3CIIC_OP_READ;
		SLIST_INSERT_AFTER(cur, nxt, op_link);
		cur = nxt;
	}

	mutex_spin_exit(&sc->sc_irq_lock);

	/* Start transfer */
	if (flags & I2C_F_POLL) {
		sc->sc_use_intr = false;
		rv = s3c24x0_i2c_xfer_poll(sc);
	} else {
		sc->sc_use_intr = true;
		rv = s3c24x0_i2c_xfer_intr(sc);
	}

	if (rv != 0) {
		device_printf(sc->sc_dev, "transfer failed\n");
	}

	mutex_spin_enter(&sc->sc_irq_lock);

cleanup:
	/* First elm is a write op */
	cur = SLIST_FIRST(&sc->sc_oplist);
	if (cur) {
		SLIST_REMOVE_HEAD(&sc->sc_oplist, op_link);
		kmem_free(cur, sizeof(struct s3c24x0_i2c_op));
	}

	i = 0;
	SLIST_FOREACH(cur, &sc->sc_oplist, op_link) {
		if (cur == NULL)
			break;
		vp[i] = cur->op_data;
		SLIST_REMOVE(&sc->sc_oplist, cur, s3c24x0_i2c_op, op_link);
		kmem_free(cur, sizeof(struct s3c24x0_i2c_op));
		i++;
	}

	KASSERT(SLIST_EMPTY(&sc->sc_oplist));
	sc->sc_curop = NULL;
	mutex_spin_exit(&sc->sc_irq_lock);

	return rv;
}

int
s3c24x0_i2c_write_x(struct s3c24x0_i2c_softc *sc, uint8_t cmd,
                    uint8_t *vp, size_t len, int flags)
{
	struct s3c24x0_i2c_op *cur, *nxt;
	int rv, i;

	mutex_spin_enter(&sc->sc_irq_lock);

	cur = kmem_alloc(sizeof(struct s3c24x0_i2c_op), KM_NOSLEEP);
	if (cur == NULL) {
		device_printf(sc->sc_dev, "Out of memory error\n");
		rv = ENOMEM;
		goto cleanup;
	}
	cur->op_type = S3CIIC_OP_WRITE;
	cur->op_data = cmd;

	SLIST_INSERT_HEAD(&sc->sc_oplist, cur, op_link);
	sc->sc_curop = cur;

	for (i = 0; i < len; i++) {
		nxt = kmem_alloc(sizeof(struct s3c24x0_i2c_op), KM_NOSLEEP);
		if (nxt == NULL) {
			device_printf(sc->sc_dev, "Out of memory error\n");
			rv = ENOMEM;
			goto cleanup;
		}
		nxt->op_type = S3CIIC_OP_WRITE;
		nxt->op_data = vp[i];
		SLIST_INSERT_AFTER(cur, nxt, op_link);
		cur = nxt;
	}

	mutex_spin_exit(&sc->sc_irq_lock);

	/* Start transfer */
	if (flags & I2C_F_POLL) {
		sc->sc_use_intr = false;
		rv = s3c24x0_i2c_xfer_poll(sc);
	} else {
		sc->sc_use_intr = true;
		rv = s3c24x0_i2c_xfer_intr(sc);
	}

	if (rv != 0) {
		device_printf(sc->sc_dev, "transfer failed\n");
	}

	mutex_spin_enter(&sc->sc_irq_lock);

cleanup:
	SLIST_FOREACH(cur, &sc->sc_oplist, op_link) {
		if (cur == NULL)
			break;
		SLIST_REMOVE(&sc->sc_oplist, cur, s3c24x0_i2c_op, op_link);
		kmem_free(cur, sizeof(struct s3c24x0_i2c_op));
	}

	KASSERT(SLIST_EMPTY(&sc->sc_oplist));
	sc->sc_curop = NULL;
	mutex_spin_exit(&sc->sc_irq_lock);

	return rv;
}

void
s3c24x0_i2c_init(struct s3c24x0_i2c_softc *sc)
{
	bus_space_handle_t	gpioh = s3c2xx0_softc->sc_gpio_ioh;
	bus_space_tag_t		iot = sc->sc_iot;
	bus_space_handle_t	ioh = sc->sc_ioh;
	uint32_t		pclk = s3c2xx0_softc->sc_pclk;
	uint32_t		pecon, div0, div1, dv;
	uint32_t		iiccon = 0;

	pecon = bus_space_read_4(iot, gpioh, GPIO_PECON);
	pecon = GPIO_SET_FUNC(pecon, 15, PCON_ALTFUN); /* IICSDA */
	pecon = GPIO_SET_FUNC(pecon, 14, PCON_ALTFUN); /* IICSCL */
	bus_space_write_4(iot, gpioh, GPIO_PECON, pecon);

	dv = s3c24x0_i2c_calc_divs((pclk / S3C24X0_IIC_CLK), &div0, &div1);

	/* Set divisors to get desired IIC clock frequency */
	if (div0 == 512)
		iiccon |= IICCON_TXDIV;
	iiccon |= div1;

	bus_space_write_4(iot, ioh, IIC_IICCON, iiccon);

	DPRINTF(("IICCLK=%dHz, iicon=0x%02x\n", (pclk / dv), iiccon));
}

int
s3c24x0_i2c_calc_divs(uint32_t div_wanted, uint32_t *div0, uint32_t *div1)
{
	uint32_t _div0, _div1;

	if (div_wanted > (16*16))
		_div0 = 512;
	else
		_div0 = 16;

	_div1 = div_wanted / _div0;

	if (_div1 > 16)
		_div1 = 16;

	*div0 = _div0;
	*div1 = _div1;

	return (_div0 * (_div1 + 1));
}

int
s3c24x0_i2c_xfer_intr(struct s3c24x0_i2c_softc *sc)
{
	int ticks = mstohz(S3C24X0_IIC_XFER_TIMEOUT);
	int retry = 0;
	int rv;

retry_wait:
	if (s3c24x0_i2c_busy_wait(sc)) {
		device_printf(sc->sc_dev, "bus busy timeout\n");
		return EBUSY;
	}

	mutex_spin_enter(&sc->sc_irq_lock);
retry:
	if (retry++ >= S3C24X0_IIC_XFER_RETRIES) {
		rv = EIO;
		goto finish;
	}

	sc->sc_curop = SLIST_FIRST(&sc->sc_oplist);

	s3c24x0_i2c_ack_enable(sc);

	/* Transmit START condition */
	s3c24x0_i2c_start(sc);

	/* Wait until all the operations have been compleated */
	DPRINTF(("IIC_INTR: [0x%02x]->", sc->sc_slave_addr));
	rv = cv_timedwait(&sc->sc_irq_cv, &sc->sc_irq_lock, ticks);
	DPRINTF(("\n"));

	if (rv == EWOULDBLOCK) {
		DPRINTF(("IIC_INTR: timeout\n"));
		DPRINT_REGS(sc);
		goto retry;
	}

	if (sc->sc_err == S3CIIC_ERR_ACK) {
		s3c24x0_i2c_stop(sc);
		mutex_spin_exit(&sc->sc_irq_lock);
		goto retry_wait;
	} else if (sc->sc_err == S3CIIC_ERR_ARB) {
		DPRINT_REGS(sc);
		if (s3c24x0_i2c_busy_wait(sc) != 0)
			goto finish;
		else
			goto retry;
	}

finish:
	mutex_spin_exit(&sc->sc_irq_lock);

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT, 0);
	s3c24x0_i2c_irq_disable(sc);
	s3c24x0_i2c_pending_clear(sc);

	return rv;
}

int
s3c24x0_i2c_xfer_poll(struct s3c24x0_i2c_softc *sc)
{
	int retry = 0;
	int rv = -1;

retry_wait:
	if (s3c24x0_i2c_busy_wait(sc)) {
		device_printf(sc->sc_dev, "bus busy timeout\n");
		return EBUSY;
	}

retry:
	if (retry++ >= S3C24X0_IIC_XFER_RETRIES) {
		rv = EIO;
		goto finish;
	}

	sc->sc_curop = SLIST_FIRST(&sc->sc_oplist);

	s3c24x0_i2c_ack_enable(sc);

	/* Transmit START condition */
	s3c24x0_i2c_start(sc);

	/* Wait until all the operations have been compleated */
	DPRINTF(("IIC_POLL: [0x%02x]->", sc->sc_slave_addr));
	while (sc->sc_curop && (sc->sc_state != S3CIIC_STOP)) {

		rv = s3c24x0_i2c_irq_wait(sc);
		if (rv == EWOULDBLOCK) {
			DPRINTF(("timeout\n"));
			DPRINT_REGS(sc);
			goto retry;
		}

		s3c24x0_i2c_intr(sc);

		if (sc->sc_err == S3CIIC_ERR_ACK) {
			DPRINTF(("\n"));
			goto retry_wait;
		} else if (sc->sc_err == S3CIIC_ERR_ARB) {
			DPRINTF(("\n"));
			DPRINT_REGS(sc);
			goto retry;
		}
	}
	DPRINTF(("\n"));

finish:
	s3c24x0_i2c_busy_wait(sc);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT, 0);
	s3c24x0_i2c_pending_clear(sc);

	return rv;
}

int
s3c24x0_i2c_busy_wait(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iicstat;
	int timeout = S3C24X0_IIC_BUSY_WAIT;

	if (!sc->sc_use_intr)
		timeout = timeout * 1000;

	iicstat = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT);
	while ( (iicstat & IICSTAT_BUSBUSY) && (timeout-- > 0) ) {
		if (sc->sc_use_intr)
			kpause("iic_busywait", false, mstohz(1), NULL);
		else
			delay(1);
		iicstat = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT);
	}

	return (timeout < 0) ? EBUSY : 0;
}

void
s3c24x0_i2c_irq_enable(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	iiccon |= IICCON_IRQEN;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON, iiccon);
}

void
s3c24x0_i2c_irq_disable(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	iiccon &= ~IICCON_IRQEN;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON, iiccon);
}

int
s3c24x0_i2c_irq_wait(struct s3c24x0_i2c_softc *sc)
{
	int timeout = S3C24X0_IIC_IRQ_WAIT * 1000;
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	while (!(iiccon & IICCON_IRQPEND) && timeout--) {
		delay(1);
		iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	}

	return (iiccon & IICCON_IRQPEND) ? 0 : EWOULDBLOCK;
}

void
s3c24x0_i2c_pending_clear(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	iiccon &= ~IICCON_IRQPEND;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON, iiccon);
}

void
s3c24x0_i2c_start(struct s3c24x0_i2c_softc *sc)
{
	uint32_t	iicstat = 0;
	uint8_t 	addr = (sc->sc_slave_addr & 0x7f) << 1;

	iicstat |= IICSTAT_TXRXEN;
	if (sc->sc_curop->op_type == S3CIIC_OP_READ) {
		addr |= 1;
		iicstat |= IICSTAT_MASTER_RX;
	} else {
		iicstat |= IICSTAT_MASTER_TX;
	}
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT, iicstat);

	/* Write slave address */
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, IIC_IICDS, addr);

	delay(1);

	/* Send START */
	iicstat |= IICSTAT_START;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT, iicstat);

	sc->sc_state = S3CIIC_START;
	sc->sc_err = 0;

	s3c24x0_i2c_irq_enable(sc);
}

void
s3c24x0_i2c_stop(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iicstat;

	KASSERT(sc->sc_state != S3CIIC_STOP);

	iicstat = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT);
	iicstat &= ~IICSTAT_START;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICSTAT, iicstat);

	sc->sc_state = S3CIIC_STOP;

	s3c24x0_i2c_irq_disable(sc);
}

void
s3c24x0_i2c_ack_enable(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	iiccon |= IICCON_ACKEN;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON, iiccon);
}

void
s3c24x0_i2c_ack_disable(struct s3c24x0_i2c_softc *sc)
{
	uint32_t iiccon;

	iiccon = bus_space_read_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON);
	iiccon &= ~IICCON_ACKEN;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, IIC_IICCON, iiccon);
}
