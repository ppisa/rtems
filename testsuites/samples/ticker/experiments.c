#include <bsp/vc.h>
#include <libcpu/arm-cp15.h>
#include <bsp.h>


/*        CRn opc1 CRm opc2
ATS12NSOPW c7 0 c8 5 32-bit WO Stages 1 and 2 Non-secure only PL1 write
ATS1CPW    c7 0 c8 1 32-bit WO Stage 1 Current state PL1 write
*/
/*
MCR p15, <opc1>, <Rt>, c7, c8, <opc2> ; Address translation operation, as defined by <opc1> and <opc2>
MRC p15, 0, <Rt>, c7, c4, 0 ; Read 32-bit PAR into Rt
MRRC p15, 0, <Rt>, <Rt2>, c7 ; Read 64-bit PAR into Rt (low word) and Rt2 (high word)
*/

ARM_CP15_TEXT_SECTION static inline uint32_t
arm_cp15_ATS1CPW(uint32_t virt_addr)
{
  ARM_SWITCH_REGISTERS;
  uint32_t phys_addr;

  __asm__ volatile (
    ARM_SWITCH_TO_ARM
    "mcr p15, 0, %[va], c7, c8, 1\n"
   #if defined(__ARM_ARCH_7A__)
    "isb\n"
   #endif
    "mrc p15, 0, %[pa], c7, c4, 0\n"
    ARM_SWITCH_BACK
    : [pa] "=&r" (phys_addr) ARM_SWITCH_ADDITIONAL_OUTPUT
    : [va] "r" (virt_addr)
  );

  return phys_addr;
}

int experiment_print_translations(int argc, char *argv[])
{
  uint32_t va;
  uint32_t pa;
  uint32_t mb = 1024 * 1024;

  for ( va = 0; va < 10 * mb; va += mb) {
    pa = arm_cp15_ATS1CPW(va);
    printf("0x%08x -> 0x%08x LPAE %i NOS %i NS %i ?? %i SH %i Inner %i Outer %i SS %i F %i\n",
           va, pa, (pa >> 11) & 1, (pa >> 10) & 1, (pa >> 9) & 1, (pa >> 8) & 1, (pa >> 7) & 1,
           (pa >> 4) & 7, (pa >> 2) & 3, (pa >> 1) & 1, (pa >> 0) & 1);
  }
  return 0;
}


void print_ver(void)
{
  int res;
  union {
    bcm2835_mailbox_get_fw_rev_entries fw_rev;
    bcm2835_get_board_spec_entries board_spec;
    bcm2835_get_board_spec_entries board_spec1;
  } u;
  printk("bcm2835_mailbox_get_firmware_revision ...\n");
  res = bcm2835_mailbox_get_firmware_revision(&u.fw_rev);
  printk("bcm2835_mailbox_get_firmware_revision %d %d\n", res, u.fw_rev.fw_rev);

  printk("bcm2835_mailbox_get_board_model ...\n");
  res = bcm2835_mailbox_get_board_model(&u.board_spec);
  printk("bcm2835_mailbox_get_board_model %d %d\n", res, u.board_spec.spec);

  printk("bcm2835_mailbox_get_board_revision ...\n");
  res = bcm2835_mailbox_get_board_revision(&u.board_spec1);
  printk("bcm2835_mailbox_get_board_revision %d %d\n", res, u.board_spec1.spec);
}

int experiment_print_ver(int argc, char *argv[])
{
  int i;

  for ( i = 0 ; i < 10; i++ )
    print_ver();

  return 0;
}

void experiments_run(void)
{
  experiment_print_translations(0, NULL);
  experiment_print_ver(0, NULL);
}
