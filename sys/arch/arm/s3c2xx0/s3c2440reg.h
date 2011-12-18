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
 * Samsung S3C244) processor is ARM920T based integrated CPU
 *
 * Reference:
 *  S3C2440 User's Manual
 */
#ifndef _ARM_S3C2XX0_S3C2440REG_H_
#define _ARM_S3C2XX0_S3C2440REG_H_

/* S3C2440 is almost identical to S3C2410 */
#include <arm/s3c2xx0/s3c2410reg.h>

/*
 * Physical address of integrated peripherals
 */
#define	S3C2440_CLKMAN_BASE	0x4c000000
#define S3C2440_CLKMAN_SIZE     0x20
#define	S3C2440_ADC_BASE 	0x58000000
#define	S3C2440_ADC_SIZE 	0x18

/* ADC */
#define ADC_ADCUPDN		0x14
#define  ADCUPDN_TSC_DN		(1<<0)
#define  ADCUPDN_TSC_UP		(1<<1)

/* Clock control */
#define CLKDIVN_HDIVN_MASK      0x6
#define CLKDIVN_DIVN_UPLL_SHIFT 3
#define CLKDIVN_DIVN_IPLL       (2<<CLKDIVN_DIVN_UPLL_SHIFT)

#define CLKMAN_CAMDIVN             0x18
#define   CAMDIVN_HCLK4_HALF_SHIFT 9
#define   CAMDIVN_HCLK4_HALF       (1<<CAMDIVN_HCLK4_HALF_SHIFT)
#define   CAMDIVN_HCLK3_HALF_SHIFT 8
#define   CAMDIVN_HCLK3_HALF       (1<<CAMDIVN_HCLK3_HALF_SHIFT)

#endif /* !_ARM_S3C2XX0_S3C2440REG_H_ */
