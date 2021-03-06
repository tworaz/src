/* Target-dependent code for NetBSD/hppa

   Copyright (C) 2007 Free Software Foundation, Inc.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

#include "defs.h"
#include "arch-utils.h"
#include "symtab.h"
#include "objfiles.h"
#include "osabi.h"
#include "regcache.h"
#include "regset.h"
#include "target.h"
#include "value.h"
#include "trad-frame.h"
#include "tramp-frame.h"

#include "gdb_assert.h"
#include "gdb_string.h"

#include "elf/common.h"

#include "hppa-tdep.h"
#include "solib-svr4.h"

/* From <machine/mcontext.h>.  */
static int hppanbsd_mc_reg_offset[] =
{
  /* r0 ... r31 */
      -1,   1 * 4,   2 * 4,   3 * 4,
   4 * 4,   5 * 4,   6 * 4,   7 * 4,
   8 * 4,   9 * 4,  10 * 4,  11 * 4, 
  12 * 4,  13 * 4,  14 * 4,  15 * 4,
  16 * 4,  17 * 4,  18 * 4,  19 * 4,
  20 * 4,  21 * 4,  22 * 4,  23 * 4,
  24 * 4,  25 * 4,  26 * 4,  27 * 4,
  28 * 4,  29 * 4,  30 * 4,  31 * 4,

  32 * 4,	/* HPPA_SAR_REGNUM */
  35 * 4,	/* HPPA_PCOQ_HEAD_REGNUM */
  33 * 4,	/* HPPA_PCSQ_HEAD_REGNUM */
  36 * 4,	/* HPPA_PCOQ_TAIL_REGNUM */
  34 * 4,	/* HPPA_PCSQ_TAIL_REGNUM */
  -1,		/* HPPA_EIEM_REGNUM */
  -1,		/* HPPA_IIR_REGNUM */
  -1,		/* HPPA_ISR_REGNUM */
  -1,		/* HPPA_IOR_REGNUM */
  0 * 4,	/* HPPA_IPSW_REGNUM */
  -1,		/* spare? */
  41 * 4,	/* HPPA_SR4_REGNUM */
  37 * 4,	/* sr0 */
  38 * 4,	/* sr1 */
  39 * 4,	/* sr2 */
  40 * 4,	/* sr3 */
  -1,		/* 48 */
  -1,		/* 49 */
  -1,		/* 50 */
  -1,		/* 51 */
  -1,		/* 52 */
  -1,		/* 53 */
  -1,		/* 54 */
  -1,		/* 55 */
  -1,		/* 56 */
  -1,		/* 57 CR24 */
  -1,		/* 58 CR25 */
  -1,		/* 59 CR26 */
  43 * 4,	/* HPPA_CR27_REGNUM */

  /* more tbd */
};

static void hppanbsd_sigtramp_cache_init (const struct tramp_frame *,
                                         struct frame_info *,
                                         struct trad_frame_cache *,
                                         CORE_ADDR);

static const struct tramp_frame hppanbsd_sigtramp_si4 =
{
  SIGTRAMP_FRAME,
  4,
  {
    { 0xc7d7c012, -1 },	/*	bb,>=,n %arg3, 30, 1f		*/
    { 0xd6e01c1e, -1 },	/*	 depwi 0,31,2,%arg3		*/
    { 0x0ee81093, -1 },	/*	ldw 4(%arg3), %r19		*/
    { 0x0ee01097, -1 },	/*	ldw 0(%arg3), %arg3		*/
			/* 1: 					*/
    { 0xe8404000, -1 },	/* 	blr %r0, %rp			*/
    { 0xeae0c002, -1 },	/*	bv,n %r0(%arg3)			*/
    { 0x08000240, -1 },	/*	 nop				*/

    { 0x0803025a, -1 },	/*	copy %r3, %arg0			*/
    { 0x20200801, -1 },	/*	ldil -40000000, %r1		*/
    { 0xe420e008, -1 },	/*	be,l 4(%sr7, %r1), %sr0, %r31	*/
    { 0x34160268, -1 },	/*	 ldi 134, %t1 ; SYS_setcontext	*/

    { 0x081c025a, -1 },	/*	copy ret0, %arg0		*/
    { 0x20200801, -1 },	/*	ldil -40000000, %r1		*/
    { 0xe420e008, -1 },	/*	be,l 4(%sr7, %r1), %sr0, %r31	*/
    { 0x34160002, -1 },	/*	 ldi 1, %t1 ; SYS_exit		*/
    { TRAMP_SENTINEL_INSN, -1 }
  },
  hppanbsd_sigtramp_cache_init
};


static void
hppanbsd_sigtramp_cache_init (const struct tramp_frame *self,
                             struct frame_info *next_frame,
                             struct trad_frame_cache *this_cache,
                             CORE_ADDR func)
{
  struct gdbarch *gdbarch = get_frame_arch (next_frame);
  struct gdbarch_tdep *tdep = gdbarch_tdep (gdbarch);
  CORE_ADDR sp = frame_unwind_register_unsigned (next_frame, HPPA_SP_REGNUM);
  CORE_ADDR base;
  int *reg_offset;
  int num_regs;
  int i;

  reg_offset = hppanbsd_mc_reg_offset;
  num_regs = ARRAY_SIZE (hppanbsd_mc_reg_offset);

  /* frame pointer */
  base = sp - 0x280;
  /* offsetof(struct sigframe_siginfo, sf_uc) = 128 */
  base += 128;
  /* offsetof(ucontext_t, uc_mcontext) == 40 */
  base += 40;

  for (i = 0; i < num_regs; i++)
    if (reg_offset[i] != -1)
      trad_frame_set_reg_addr (this_cache, i, base + reg_offset[i]);

  /* Construct the frame ID using the function start.  */
  trad_frame_set_id (this_cache, frame_id_build (sp, func));
}

/* Core file support.  */

/* Sizeof `struct reg' in <machine/reg.h>.  */
#define HPPANBSD_SIZEOF_GREGS	(46 * 4)

static int hppanbsd_reg_offset[] =
{
  /* r0 ... r31 */
      -1,   1 * 4,   2 * 4,   3 * 4,
   4 * 4,   5 * 4,   6 * 4,   7 * 4,
   8 * 4,   9 * 4,  10 * 4,  11 * 4, 
  12 * 4,  13 * 4,  14 * 4,  15 * 4,
  16 * 4,  17 * 4,  18 * 4,  19 * 4,
  20 * 4,  21 * 4,  22 * 4,  23 * 4,
  24 * 4,  25 * 4,  26 * 4,  27 * 4,
  28 * 4,  29 * 4,  30 * 4,  31 * 4,

  32 * 4,	/* HPPA_SAR_REGNUM */
  35 * 4,	/* HPPA_PCOQ_HEAD_REGNUM */
  33 * 4,	/* HPPA_PCSQ_HEAD_REGNUM */
  36 * 4,	/* HPPA_PCOQ_TAIL_REGNUM */
  34 * 4,	/* HPPA_PCSQ_TAIL_REGNUM */
  -1,		/* HPPA_EIEM_REGNUM */
  -1,		/* HPPA_IIR_REGNUM */
  -1,		/* HPPA_ISR_REGNUM */
  -1,		/* HPPA_IOR_REGNUM */
  0 * 4,	/* HPPA_IPSW_REGNUM */
  -1,		/* spare? */
  41 * 4,	/* HPPA_SR4_REGNUM */
  37 * 4,	/* sr0 */
  38 * 4,	/* sr1 */
  39 * 4,	/* sr2 */
  40 * 4,	/* sr3 */
  -1,		/* 48 */
  -1,		/* 49 */
  -1,		/* 50 */
  -1,		/* 51 */
  -1,		/* 52 */
  -1,		/* 53 */
  -1,		/* 54 */
  -1,		/* 55 */
  -1,		/* 56 */
  -1,		/* 57 */
  -1,		/* 58 */
  -1,		/* 59 */
  46 * 4,	/* HPPA_CR27_REGNUM */
};

/* Supply register REGNUM from the buffer specified by GREGS and LEN
   in the general-purpose register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

static void
hppanbsd_supply_gregset (const struct regset *regset, struct regcache *regcache,
		     int regnum, const void *gregs, size_t len)
{
  const gdb_byte *regs = gregs;
  size_t offset;
  int i;

  gdb_assert (len >= HPPANBSD_SIZEOF_GREGS);

  for (i = 0; i < ARRAY_SIZE (hppanbsd_reg_offset); i++)
    if (hppanbsd_reg_offset[i] != -1)
      if (regnum == -1 || regnum == i)
	regcache_raw_supply (regcache, i, regs + hppanbsd_reg_offset[i]);
}

/* NetBSD/hppa register set.  */

static struct regset hppanbsd_gregset =
{
  NULL,
  hppanbsd_supply_gregset
};

/* Return the appropriate register set for the core section identified
   by SECT_NAME and SECT_SIZE.  */

static const struct regset *
hppanbsd_regset_from_core_section (struct gdbarch *gdbarch,
				  const char *sect_name, size_t sect_size)
{
  if (strcmp (sect_name, ".reg") == 0 && sect_size >= HPPANBSD_SIZEOF_GREGS)
    return &hppanbsd_gregset;

  return NULL;
}

void hppabsd_init_abi (struct gdbarch_info, struct gdbarch *);

static void
hppanbsd_init_abi (struct gdbarch_info info, struct gdbarch *gdbarch)
{
  /* Obviously NetBSD is BSD-based.  */
  hppabsd_init_abi (info, gdbarch);

  /* Core file support.  */
  set_gdbarch_regset_from_core_section
    (gdbarch, hppanbsd_regset_from_core_section);

  tramp_frame_prepend_unwinder (gdbarch, &hppanbsd_sigtramp_si4);
}


/* Provide a prototype to silence -Wmissing-prototypes.  */
void _initialize_hppabsd_tdep (void);

void
_initialize_hppanbsd_tdep (void)
{
  gdbarch_register_osabi (bfd_arch_hppa, 0, GDB_OSABI_NETBSD_ELF,
			  hppanbsd_init_abi);
}
