/*	$NetBSD: x25_19.h,v 1.1.1.1 2009/03/22 15:01:56 christos Exp $	*/

/*
 * Copyright (C) 2004, 2005, 2007  Internet Systems Consortium, Inc. ("ISC")
 * Copyright (C) 1999-2001  Internet Software Consortium.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef GENERIC_X25_19_H
#define GENERIC_X25_19_H 1

/* Id: x25_19.h,v 1.18 2007/06/19 23:47:17 tbox Exp */

/*!
 *  \brief Per RFC1183 */

typedef struct dns_rdata_x25 {
	dns_rdatacommon_t	common;
	isc_mem_t		*mctx;
	unsigned char		*x25;
	isc_uint8_t		x25_len;
} dns_rdata_x25_t;

#endif /* GENERIC_X25_19_H */