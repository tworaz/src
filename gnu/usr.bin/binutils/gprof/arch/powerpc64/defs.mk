# This file is automatically generated.  DO NOT EDIT!
# Generated from: 	NetBSD: mknative-binutils,v 1.5 2006/02/02 20:06:04 skrll Exp 
# Generated from: NetBSD: mknative.common,v 1.8 2006/05/26 19:17:21 mrg Exp 
#
G_DEFS=-DHAVE_CONFIG_H
G_gprof_OBJECTS=basic_blocks.o call_graph.o  cg_arcs.o cg_dfn.o cg_print.o  corefile.o gmon_io.o gprof.o  hertz.o hist.o source.o  search_list.o symtab.o sym_ids.o  utils.o i386.o alpha.o vax.o  tahoe.o sparc.o mips.o flat_bl.o  bsd_callg_bl.o fsf_callg_bl.o
G_INCLUDES=-D_GNU_SOURCE -DDEBUG -I../bfd -I${GNUHOSTDIST}/gprof/../include -I${GNUHOSTDIST}/gprof/../bfd -I${GNUHOSTDIST}/gprof/../intl -I../intl -I. -DLOCALEDIR="\"/usr/local/share/locale\""
G_TEXINFOS=gprof.texi