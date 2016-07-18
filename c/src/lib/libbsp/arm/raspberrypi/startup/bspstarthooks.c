/**
 * @file
 *
 * @ingroup arm_start
 *
 * @brief Rasberry Pi startup code.
 */

/*
 * Copyright (c) 2013. Hesham AL-Matary
 * Copyright (c) 2013 by Alan Cudmore
 * based on work by:
 * Copyright (c) 2009
 * embedded brains GmbH
 * Obere Lagerstr. 30
 * D-82178 Puchheim
 * Germany
 * <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE
 */

#define BSP_START_HOOKS_WITH_LOADER_ARGS

#include <bspopts.h>
#include <bsp/start.h>
#include <bsp/raspberrypi.h>
#include <bsp/mm.h>
#include <libcpu/arm-cp15.h>
#include <bsp.h>
#include <bsp/rpi-gpio.h>
#include <bsp/usart.h>
#include <bsp/arm-cp15-start.h>

void ll_charout(char ch)
{
  if (ch == '\n')
    bcm2835_usart_fns.deviceWritePolled(0, '\r');
  bcm2835_usart_fns.deviceWritePolled(0, ch);
}

void ll_strout(char *str)
{
  char ch;
  while ((ch = *(str++)) != 0) {
    if (ch == '\n')
      bcm2835_usart_fns.deviceWritePolled(0, '\r');
    bcm2835_usart_fns.deviceWritePolled(0, ch);
  }
}

void ll_hexout(uint32_t val, int digits)
{
  int ch;
  while ( digits-- ) {
    ch = (val >> (digits * 4)) & 0xf;
    ch = ch <= 9? ch + '0': ch + 'A' - 10;
    ll_charout(ch);
  }
}

volatile int continue_execution;

void bsp_hyp_catch_handler(void)
{
  uint32_t spsr_val;
  uint32_t elr_val;
  uint32_t sw_reg;
  ll_strout("bsp_hyp_catch_handler:\n");

  __asm__ volatile (
#if defined(__thumb__) || defined(__thumb2__)
    "adr %[sw_reg], 1f\n"
    "bx %[sw_reg]\n"
    ".arm\n"
#endif
    "1:nop\n"
    "mrs %[spsr_val], spsr\n"
    ".inst 0xe10e0300\n"
    "mov %[elr_val], r0\n"
#if defined(__thumb__) || defined(__thumb2__)
    "adr %[sw_reg], 3f + 1\n"
    "bx %[sw_reg]\n"
    ".thumb\n"
    "3:"
#endif
    : [spsr_val] "=&r" (spsr_val), [elr_val] "=&r" (elr_val),  [sw_reg] "=&r" (sw_reg) : : "r0"
  );

  ll_strout("  psr ");
  ll_hexout(spsr_val, 8);
  ll_strout(" elr ");
  ll_hexout(elr_val, 8);
  ll_strout("\n");

  while(1);
}


/*
 * Save information that RTEMS has been started in hypervisor mode
 * Value has to be placed in initialized data section (-1) to not
 * be zeroed by BSS clear
 */
uint32_t bsp_start_invoked_in_hyp_mode = -1;

void bsp_arm_drop_hyp_mode(uint32_t new_mode)
{
  uint32_t lr_val, sp_val, cpsr_val, mpidr_val;

  __asm__ volatile (
    "mov %[lr_val], lr\n"
    "mov %[sp_val], sp\n"
    "mrs %[cpsr_val], cpsr\n"
    "mrc p15, 0, %[mpidr_val], c0, c0, 5\n" /* MPIDR */
    : [lr_val] "=&r" (lr_val),
      [sp_val] "=&r" (sp_val),
      [cpsr_val] "=&r" (cpsr_val),
      [mpidr_val] "=&r" (mpidr_val)
  );
  ll_strout(" lr ");
  ll_hexout(lr_val, 8);
  ll_strout(" sp ");
  ll_hexout(sp_val, 8);
  ll_strout(" cpsr ");
  ll_hexout(cpsr_val, 8);
  ll_strout(" mpidr ");
  ll_hexout(mpidr_val, 8);
  ll_strout("\n");

  __asm__ volatile (
        "mov     r1, #0\n"
        "mcr     p15, 4, r1, c1, c1, 0\n" /* HCR */
        "mcr     p15, 4, r1, c1, c1, 2\n" /* HCPTR */
        "mcr     p15, 4, r1, c1, c1, 3\n" /* HSTR */

#if defined(__thumb__) || defined(__thumb2__)
        //"orr     r0, #(1 << 30)\n"     /* HSCTLR.TE */
#endif
        "mcr     p15, 4, r1, c1, c0, 0\n" /* HSCTLR */
        "mrc     p15, 4, r1, c1, c1, 1\n" /* HDCR */
        "and     r1, #0x1f\n"             /* Preserve HPMN */
        "mcr     p15, 4, r1, c1, c1, 1\n" /* HDCR */
    : : : "r1"
  );

  ll_strout("bsp_arm_drop_hyp_mode 1\n");

  //new_mode = ARM_PSR_A | ARM_PSR_I | ARM_PSR_F | ARM_PSR_M_SVC;
#if defined(__thumb__) || defined(__thumb2__)
  //new_mode |= ARM_PSR_T;
#endif

  new_mode &= ~ARM_PSR_T;

  __asm__ volatile (
#if defined(__thumb__) || defined(__thumb2__)
    "adr r1, 1f\n"
    "bx r1\n"
    ".arm\n"
#endif
    "1:nop\n"
    "msr spsr_cxsf, %[new_mode]\n"
//#if defined(__thumb__) || defined(__thumb2__)
//    "adr r1, 2f + 1\n"
//#else
    "adr r1, 2f\n"
//#endif
    ".inst 0xe12ef301\n"   /* msr ELR_hyp, r1 */
    "mov r1, sp\n"
    "mov r2, lr\n"
    ".inst 0xe160006e\n"   /* eret */
    "2:\n"
    "mov sp, r1\n"
    "mov lr, r2\n"
#if defined(__thumb__) || defined(__thumb2__)
    "adr r1, 3f + 1\n"
    "bx r1\n"
    ".thumb\n"
    "3:"
#endif
     : : [new_mode] "r" (new_mode) : "r1", "r2", "memory"
  );

  ll_strout("bsp_arm_drop_hyp_mode 2\n");

  __asm__ volatile (
    "mov %[lr_val], lr\n"
    "mov %[sp_val], sp\n"
    "mrs %[cpsr_val], cpsr\n"
    "mrc p15, 0, %[mpidr_val], c0, c0, 5\n" /* MPIDR */
    : [lr_val] "=&r" (lr_val),
      [sp_val] "=&r" (sp_val),
      [cpsr_val] "=&r" (cpsr_val),
      [mpidr_val] "=&r" (mpidr_val)
  );
  ll_strout(" lr ");
  ll_hexout(lr_val, 8);
  ll_strout(" sp ");
  ll_hexout(sp_val, 8);
  ll_strout(" cpsr ");
  ll_hexout(cpsr_val, 8);
  ll_strout(" mpidr ");
  ll_hexout(mpidr_val, 8);
  ll_strout("\n");
}

void bsp_arm_invoke_hyp_hvc0()
{
  __asm__ volatile (
#if defined(__thumb__) || defined(__thumb2__)
    "adr r1, 1f\n"
    "bx r1\n"
    ".arm\n"
#endif
    "1:.inst 0xe1400070\n"
    ".thumb\n"
     : : : "memory"
  );
}


void bsp_arm_set_stack_for_mode(uint32_t mode, void *stack)
{
  ARM_SWITCH_REGISTERS;

  __asm__ volatile (
      ARM_SWITCH_TO_ARM
      "mov  r2, %[stack]\n"
      "mrs  r3, cpsr\n"
      "msr  cpsr_fc, %[mode]\n"
      "mov  sp, r2\n"
      "msr  cpsr_fc, r3\n"
      ARM_SWITCH_BACK
    : [mode] "+&r" (mode),
      [stack] "+&r" (stack)
      ARM_SWITCH_ADDITIONAL_OUTPUT
    : : "r2", "r3"
  );
}

void BSP_START_TEXT_SECTION bsp_start_hook_0(BSP_START_HOOKS_LOADER_ARGS)
{
  uint32_t sctlr_val, sctlr1_val;

  sctlr_val = arm_cp15_get_control();

  ll_strout("bsp_start_hook_0:\n");

  ll_strout("  saved psr      machid   DTB/ATAG addr\n");
  ll_strout("        ");
  ll_hexout(saved_psr, 8);
  ll_strout(" ");
  ll_hexout(saved_machid, 8);
  ll_strout(" ");
  ll_hexout(saved_dtb_adr, 8);
  ll_strout("\n");

  ll_strout("  sctlr ");
  ll_hexout(sctlr_val, 8);
  ll_strout("\n");

  {
    uint32_t cpsr_val;
    uint32_t cpu_id = 0;
    uint32_t stack_offset = 0;
    ARM_SWITCH_REGISTERS;

#ifdef RTEMS_SMP
    stack_offset = cpu_id * bsp_stack_all_size;
#endif

    __asm__ volatile (
      ARM_SWITCH_TO_ARM
      "mrs %[cpsr_val], cpsr\n"
      ARM_SWITCH_BACK
      : [cpsr_val] "=&r" (cpsr_val) ARM_SWITCH_ADDITIONAL_OUTPUT
    );

    cpsr_val |= ARM_PSR_I;
    cpsr_val |= ARM_PSR_F;

    bsp_start_invoked_in_hyp_mode = (cpsr_val & ARM_PSR_M_MASK) == ARM_PSR_M_HYP? 1 : 0;

    if ( bsp_start_invoked_in_hyp_mode ) {

      ll_strout("  dropping HYP mode\n");

      arm_cp15_set_hyp_vector_base_address(bsp_start_vector_table_begin);

      ll_strout("  switch to SVC mode\n");

      cpsr_val &= ~ARM_PSR_M_MASK;

      bsp_arm_drop_hyp_mode(cpsr_val | ARM_PSR_M_SVC);

      sctlr1_val = arm_cp15_get_control();
      ll_strout("  sctlr1 ");
      ll_hexout(sctlr1_val, 8);
      ll_strout("\n");

      bsp_arm_set_stack_for_mode(cpsr_val | ARM_PSR_M_IRQ, bsp_stack_irq_end + stack_offset);
      bsp_arm_set_stack_for_mode(cpsr_val | ARM_PSR_M_FIQ, bsp_stack_fiq_end + stack_offset);
      bsp_arm_set_stack_for_mode(cpsr_val | ARM_PSR_M_ABT, bsp_stack_abt_end + stack_offset);
      bsp_arm_set_stack_for_mode(cpsr_val | ARM_PSR_M_UND, bsp_stack_und_end + stack_offset);

    } else {
      __asm__ volatile (
        ARM_SWITCH_TO_ARM
        "msr cpsr, %[cpsr_val]\n"
        ARM_SWITCH_BACK
        : ARM_SWITCH_OUTPUT
        : [cpsr_val] "r" (cpsr_val)
      );

    }
    __asm__ volatile (
      ARM_SWITCH_TO_ARM
      "mrs %[cpsr_val], cpsr\n"
      ARM_SWITCH_BACK
      : [cpsr_val] "=&r" (cpsr_val) ARM_SWITCH_ADDITIONAL_OUTPUT
    );
    cpsr_val &= ARM_PSR_M_MASK;
    if (cpsr_val == ARM_PSR_M_SVC)
      ll_strout("  switch to SVC mode OK\n");
    else
      ll_strout("  switch to SVC mode failed\n");
  }

  /*
   * Current U-boot loader seems to start kernel image
   * with I and D caches on and MMU enabled.
   * If RTEMS application image finds that cache is on
   * during startup then disable caches.
   */
  if (sctlr_val & (ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M)) {
    if (sctlr_val & (ARM_CP15_CTRL_C | ARM_CP15_CTRL_M)) {
      /*
       * If the data cache is on then ensure that it is clean
       * before switching off to be extra carefull.
       */
      rtems_cache_flush_entire_data();
      rtems_cache_invalidate_entire_data();
    }
    arm_cp15_flush_prefetch_buffer();
    sctlr_val &= ~(ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M | ARM_CP15_CTRL_A);
    arm_cp15_set_control(sctlr_val);
  }
  rtems_cache_invalidate_entire_data();
  rtems_cache_invalidate_entire_instruction();
  arm_cp15_branch_predictor_invalidate_all();
  arm_cp15_tlb_invalidate();
  arm_cp15_flush_prefetch_buffer();

  //rtems_cache_invalidate_multiple_data_lines(0, 1024 *1024);

  /* Clear Translation Table Base Control Register */
  arm_cp15_set_translation_table_base_control_register(0);

  /* Clear Secure or Non-secure Vector Base Address Register */
  arm_cp15_set_vector_base_address(0);
  //arm_cp15_set_vector_base_address(0x8000);
  //arm_cp15_set_vector_base_address(bsp_start_vector_table_begin);

 #if defined(__ARM_ARCH_7A__)
  //arm_cp15_set_monitor_vector_base_address(0);
  /* Enable SMP in auxiliary control */
  uint32_t actlr = arm_cp15_get_auxiliary_control();
  actlr |= ARM_CORTEX_A9_ACTL_SMP;
  arm_cp15_set_auxiliary_control(actlr);
 #endif

  /* Enable JTAG */
  rtems_gpio_bsp_select_specific_io(0, 22, RPI_ALT_FUNC_4);
  rtems_gpio_bsp_select_specific_io(0, 4,  RPI_ALT_FUNC_5);
  rtems_gpio_bsp_select_specific_io(0, 27, RPI_ALT_FUNC_4);
  rtems_gpio_bsp_select_specific_io(0, 25, RPI_ALT_FUNC_4);
  rtems_gpio_bsp_select_specific_io(0, 23, RPI_ALT_FUNC_4);
  rtems_gpio_bsp_select_specific_io(0, 24, RPI_ALT_FUNC_4);

  if (0) {
    ll_strout("JTAG Enabled and waiting for GDB\n");

    continue_execution = 0;
    while (!continue_execution);
  }
  ll_strout("  bsp_start_hook_0 OK\n");
}

void BSP_START_TEXT_SECTION bsp_start_hook_1(BSP_START_HOOKS_LOADER_ARGS)
{
  uint32_t ctrl;

  ll_strout("bsp_start_hook_1:\n");
  bsp_start_copy_sections();
  ll_strout(" bsp_start_copy_sections OK\n");
  /* turn mmu off first in case it's on */
  ctrl = arm_cp15_start_setup_mmu_and_cache(
    ARM_CP15_CTRL_M | ARM_CP15_CTRL_A,  /* clear - mmu off */
    ARM_CP15_CTRL_AFE
  );

  ctrl |= ARM_CP15_CTRL_Z;

  arm_cp15_start_setup_translation_table_and_enable_mmu_and_cache(
    ctrl,
    (uint32_t *) bsp_translation_table_base,
    ARM_MMU_DEFAULT_CLIENT_DOMAIN,
    &arm_cp15_start_mmu_config_table[0],
    arm_cp15_start_mmu_config_table_size
  );
  ll_strout(" bsp_memory_management_initialize OK\n");

  ctrl = arm_cp15_get_control();
  ll_strout("  sctlr ");
  ll_hexout(ctrl, 8);
  ll_strout("\n");

  ll_strout("  saved psr      machid   DTB/ATAG addr\n");
  ll_strout("        ");
  ll_hexout(saved_psr, 8);
  ll_strout(" ");
  ll_hexout(saved_machid, 8);
  ll_strout(" ");
  ll_hexout(saved_dtb_adr, 8);
  ll_strout("\n");

  bsp_start_clear_bss();
  ll_strout(" bsp_start_clear_bss OK\n");

  rpi_video_init();
  ll_strout(" rpi_video_init OK\n");
}
