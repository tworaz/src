# $NetBSD: Makefile,v 1.8 2011/11/29 20:05:30 rmind Exp $

.include "../Makefile.inc"

.PATH:		${S}/net/npf

KMOD=		npf

SRCS=		npf.c npf_alg.c npf_ctl.c npf_handler.c
SRCS+=		npf_inet.c npf_instr.c npf_log.c npf_mbuf.c npf_nat.c
SRCS+=		npf_processor.c npf_ruleset.c npf_sendpkt.c npf_session.c
SRCS+=		npf_state.c npf_state_tcp.c npf_tableset.c

CPPFLAGS+=	-DINET6

.include <bsd.kmodule.mk>
