/* $NetBSD: s3c2410reg.h,v 1.7 2005/12/11 12:16:51 christos Exp $ */

/*
 * Copyright (c) 2003, 2004  Genetec corporation.  All rights reserved.
 * Written by Hiroyuki Bessho for Genetec corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of Genetec corporation may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GENETEC CORP. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GENETEC CORP.
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * Samsung S3C2410X processor is ARM920T based integrated CPU
 *
 * Reference:
 *  S3C2410X User's Manual 
 */
#ifndef _ARM_S3C2XX0_S3C2410REG_H_
#define	_ARM_S3C2XX0_S3C2410REG_H_

#include <arm/s3c2xx0/s3c24x0reg.h>

/*
 * Memory Map
 */
#define	S3C2410_BANK_SIZE 	0x08000000
#define	S3C2410_BANK_START(n)	(S3C2410_BANK_SIZE*(n))
#define	S3C2410_SDRAM_START	S3C2410_BANK_START(6)

/*
 * Physical address of integrated peripherals
 */
#define	S3C2410_MEMCTL_BASE	0x48000000 /* memory controller */
#define	S3C2410_USBHC_BASE 	0x49000000 /* USB Host controller */
#define	S3C2410_INTCTL_BASE	0x4a000000 /* Interrupt controller */
#define	S3C2410_DMAC_BASE	0x4b000000
#define	S3C2410_DMAC_SIZE 	0xe4
#define	S3C2410_CLKMAN_BASE	0x4c000000 /* clock & power management */
#define	S3C2410_CLKMAN_SIZE     0x18
#define	S3C2410_LCDC_BASE 	0x4d000000 /* LCD controller */
#define	S3C2410_NANDFC_BASE	0x4e000000 /* NAND Flash controller */
#define	S3C2410_NANDFC_SIZE	0x18
#define	S3C2410_UART0_BASE	0x50000000
#define	S3C2410_UART_BASE(n)	(S3C2410_UART0_BASE+0x4000*(n))
#define	S3C2410_TIMER_BASE 	0x51000000
#define	S3C2410_USBDC_BASE 	0x5200140
#define	S3C2410_USBDC_SIZE 	0x130
#define	S3C2410_WDT_BASE 	0x53000000
#define	S3C2410_IIC_BASE 	0x54000000
#define	S3C2410_IIS_BASE 	0x55000000
#define	S3C2410_GPIO_BASE	0x56000000
#define	S3C2410_GPIO_SIZE	0xb4
#define	S3C2410_ADC_BASE 	0x58000000
#define	S3C2410_ADC_SIZE 	0x14
#define	S3C2410_SPI0_BASE 	0x59000000
#define	S3C2410_SPI1_BASE 	0x59000020
#define	S3C2410_SDI_BASE 	0x5a000000 /* SD Interface */
#define	S3C2410_SDI_SIZE 	0x44

/* interrupt control (additional defs for 2410) */
#define	ICU_LEN	(32+11)

#define	INTCTL_SUBSRCPND 	0x18	/* sub source pending (2410 only) */
#define	INTCTL_INTSUBMSK  	0x1c	/* sub mask (2410 only) */

/* 2410 has more than 32 interrupt sources.  These are sub-sources
 * that are OR-ed into main interrupt sources, and controlled via
 * SUBSRCPND and  SUBSRCMSK registers */

#define	S3C2410_SUBIRQ_MIN	32
#define	S3C2410_SUBIRQ_MAX	(32+10)

/* cascaded to INT_ADCTC */
#define	S3C2410_INT_ADC		(S3C2410_SUBIRQ_MIN+10)	/* AD converter */
#define	S3C2410_INT_TC 		(S3C2410_SUBIRQ_MIN+9)	/* Touch screen */
/* cascaded to INT_UART2 */
#define	S3C2410_INT_ERR2	(S3C2410_SUBIRQ_MIN+8)	/* UART2 Error interrupt */
#define	S3C2410_INT_TXD2	(S3C2410_SUBIRQ_MIN+7)	/* UART2 Tx interrupt */
#define	S3C2410_INT_RXD2	(S3C2410_SUBIRQ_MIN+6)	/* UART2 Rx interrupt */
/* cascaded to INT_UART1 */
#define	S3C2410_INT_ERR1	(S3C2410_SUBIRQ_MIN+5)	/* UART1 Error interrupt */
#define	S3C2410_INT_TXD1	(S3C2410_SUBIRQ_MIN+4)	/* UART1 Tx interrupt */
#define	S3C2410_INT_RXD1	(S3C2410_SUBIRQ_MIN+3)	/* UART1 Rx interrupt */
/* cascaded to INT_UART0 */
#define	S3C2410_INT_ERR0	(S3C2410_SUBIRQ_MIN+2)	/* UART0 Error interrupt */
#define	S3C2410_INT_TXD0	(S3C2410_SUBIRQ_MIN+1)	/* UART0 Tx interrupt */
#define	S3C2410_INT_RXD0	(S3C2410_SUBIRQ_MIN+0)	/* UART0 Rx interrupt */

#define	S3C2410_INTCTL_SIZE	0x20

/* NAND Flash controller */
#define	NANDFC_NFCONF	0x00	/* Configuration */
#define	NANDFC_NFCMD 	0x04	/* command */
#define	NANDFC_NFADDR 	0x08	/* address */
#define	NANDFC_NFDATA 	0x0c	/* data */
#define	NANDFC_NFSTAT 	0x10	/* operation status */
#define	NANDFC_NFECC	0x14	/* ecc */

/* GPIO */
#define	GPIO_PACON	0x00	/* port A configuration */
#define	 PCON_INPUT	0	/* Input port */
#define	 PCON_OUTPUT	1	/* Output port */
#define	 PCON_ALTFUN	2	/* Alternate function */
#define	 PCON_ALTFUN2	3	/* Alternate function */
#define	GPIO_PADAT	0x04	/* port A data */
#define	GPIO_PBCON	0x10
#define	GPIO_PBDAT	0x14
#define	GPIO_PBUP 	0x18
#define	GPIO_PCCON	0x20
#define	GPIO_PCDAT	0x24
#define	GPIO_PCUP	0x28
#define	GPIO_PDCON	0x30
#define	GPIO_PDDAT	0x34
#define	GPIO_PDUP	0x38
#define	GPIO_PECON	0x40
#define	GPIO_PEDAT	0x44
#define	GPIO_PEUP	0x48
#define	GPIO_PFCON	0x50
#define	GPIO_PFDAT	0x54
#define	GPIO_PFUP	0x58
#define	GPIO_PGCON	0x60
#define	GPIO_PGDAT	0x64
#define	GPIO_PGUP	0x68
#define	GPIO_PHCON	0x70
#define	GPIO_PHDAT	0x74
#define	GPIO_PHUP	0x78
#define	GPIO_MISCCR 	0x80	/* miscellaneous control */
#define	GPIO_DCLKCON 	0x84	/* DCLK 0/1 */
#define	GPIO_EXTINT(n)	(0x88+4*(n))	/* external int control 0/1/2 */
#define	GPIO_EINTFLT(n)	(0x94+4*(n))	/* external int filter control 0..3 */
#define	GPIO_EINTMASK	0xa4
#define	GPIO_EINTPEND	0xa8
#define	GPIO_GSTATUS0	0xac	/* external pin status */
#define	GPIO_GSTATUS1	0xb0	/* external pin status */

#define	GPIO_SET_FUNC(v,port,func)	\
		(((v) & ~(3<<(2*(port))))|((func)<<(2*(port))))

#define	 EXTINTR_LOW	 0x00
#define	 EXTINTR_HIGH	 0x01
#define	 EXTINTR_FALLING 0x02
#define	 EXTINTR_RISING  0x04
#define	 EXTINTR_BOTH    0x06

/* SD interface */
/* XXX */

/* ADC */
/* XXX: ADCCON register is common to both S3C2410 and S3C2400,
 *      but other registers are different.
 */
#define	ADC_ADCCON		0x00
#define	 ADCCON_ENABLE_START	(1<<0)
#define	 ADCCON_READ_START	(1<<1)
#define	 ADCCON_STDBM    	(1<<2)
#define	 ADCCON_SEL_MUX_SHIFT	3
#define	 ADCCON_SEL_MUX_MASK	(0x7<<ADCCON_SEL_MUX_SHIFT)
#define	 ADCCON_PRSCVL_SHIFT	6
#define	 ADCCON_PRSCVL_MASK	(0xff<<ADCCON_PRSCVL_SHIFT)
#define	 ADCCON_PRSCEN  	(1<<14)
#define	 ADCCON_ECFLG   	(1<<15)

#define	ADC_ADCTSC 		0x04
#define	 ADCTSC_XY_PST   	0x03
#define	 ADCTSC_AUTO_PST    	(1<<2)
#define	 ADCTSC_PULL_UP		(1<<3)
#define	 ADCTSC_XP_SEN		(1<<4)
#define	 ADCTSC_XM_SEN		(1<<5)
#define	 ADCTSC_YP_SEN		(1<<6)
#define	 ADCTSC_YM_SEN		(1<<7)
#define	 ADCTSC_UD_SEN		(1<<8)
#define	ADC_ADCDLY		0x08
#define	ADC_ADCDAT0		0x0c
#define	ADC_ADCDAT1		0x10

#define	ADCDAT_DATAMASK  	0x3ff

/* IIC Bus Interface */
#define IIC_IICCON		0x00
#define  IICCON_ACKEN		(1<<7)
#define  IICCON_TXDIV		(1<<6)
#define  IICCON_IRQEN		(1<<5)
#define  IICCON_IRQPEND		(1<<4)
#define  IICCON_SCALEMASK	0x0f
#define IIC_IICSTAT		0x04
#define  IICSTAT_MODEMASK	(3<<6)
#define  IICSTAT_MASTER_RX	(2<<6)
#define  IICSTAT_MASTER_TX	(3<<6)
#define  IICSTAT_SLAVE_RX	(0<<6)
#define  IICSTAT_SLAVE_TX	(1<<6)
#define  IICSTAT_START		(1<<5)
#define  IICSTAT_BUSBUSY	(1<<5)
#define  IICSTAT_TXRXEN		(1<<4)
#define  IICSTAT_ARBITR		(1<<3)
#define  IICSTAT_ADDRSLAVE	(1<<2)
#define  IICSTAT_ADDR0		(1<<1)
#define  IICSTAT_LASTBIT	(1<<0)
#define IIC_IICADD		0x08
#define IIC_IICDS		0x0C

#endif /* _ARM_S3C2XX0_S3C2410REG_H_ */
