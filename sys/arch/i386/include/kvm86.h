/* $NetBSD: kvm86.h,v 1.6 2008/10/26 06:57:30 mrg Exp $ */

void kvm86_init(void);
void kvm86_gpfault(struct trapframe *);
extern int kvm86_incall;

void *kvm86_bios_addpage(uint32_t);
void kvm86_bios_delpage(uint32_t, void *);
size_t kvm86_bios_read(uint32_t, char *, size_t);
int kvm86_bioscall(int, struct trapframe *);

/* for migration from bioscall() */
#include <i386/bioscall.h>
int kvm86_bioscall_simple(int, struct bioscallregs *);
