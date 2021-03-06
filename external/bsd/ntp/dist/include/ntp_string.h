/*	$NetBSD: ntp_string.h,v 1.1.1.1 2009/12/13 16:54:53 kardel Exp $	*/

/*
 * Define string ops: strchr strrchr memcmp memmove memset 
 */

#ifndef  _ntp_string_h
#define  _ntp_string_h

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef HAVE_MEMORY_H
# include <memory.h>
#endif

#ifdef HAVE_STRING_H
# include <string.h>
#endif

#ifdef HAVE_BSTRING_H
# include <bstring.h>
#endif

#ifndef STDC_HEADERS
# ifndef HAVE_STRCHR
#  include <strings.h>
#  define strchr index
#  define strrchr rindex
# endif
# ifndef __GNUC__
char *strchr(), *strrchr();
# endif
# ifndef HAVE_MEMCPY
#  define NTP_NEED_BOPS
# endif
#endif /* STDC_HEADERS */

#ifdef NTP_NEED_BOPS

#ifdef HAVE_STRINGS_H
# include <strings.h>		/* bcmp, bcopy, bzero */
#endif

void	ntp_memset	(char *, int, int);

#define memcmp(a, b, c)		bcmp(a, b, (int)(c))
#define memmove(t, f, c)	bcopy(f, t, (int)(c))
#define memcpy(t, f, c)		bcopy(f, t, (int)(c))
#define memset(a, x, c)		if (0 == (x)) \
					bzero(a, (int)(c)); \
				else \
					ntp_memset((char *)(a), x, c)
#endif /*  NTP_NEED_BOPS */

#endif /* _ntp_string_h */
