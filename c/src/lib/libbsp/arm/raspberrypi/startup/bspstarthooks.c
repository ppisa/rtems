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

#include <bspopts.h>
#include <bsp/start.h>
#include <bsp/raspberrypi.h>
#include <bsp/mm.h>
#include <libcpu/arm-cp15.h>
#include <bsp.h>
#include <bsp/rpi-gpio.h>

#ifdef RTEMS_SMP
#include <rtems/score/smp.h>
#endif

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
register char * stack_ptr asm ("sp");

void BSP_START_TEXT_SECTION bsp_start_hook_0(void)
{
  uint32_t sctlr_val;
#ifdef RTEMS_SMP
  uint32_t cpu_index_self = _SMP_Get_current_processor();

  ll_strout("boot on CPU ");
  ll_hexout(cpu_index_self, 1);
  ll_strout(" sp ");
  ll_hexout(stack_ptr, 8);
  ll_strout("\n\r");
#endif /* RTEMS_SMP */

  sctlr_val = arm_cp15_get_control();

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
#ifdef RTEMS_SMP
      if (cpu_index_self != 0) {
        arm_cp15_data_cache_clean_level(0);
        arm_cp15_cache_invalidate_level(0, 0);
      } else
#endif /* RTEMS_SMP */
      {
        rtems_cache_flush_entire_data();
        rtems_cache_invalidate_entire_data();
      }
    }
    arm_cp15_flush_prefetch_buffer();
    sctlr_val &= ~(ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M | ARM_CP15_CTRL_A);
    arm_cp15_set_control(sctlr_val);
  }
#ifdef RTEMS_SMP
  if (cpu_index_self != 0) {
    arm_cp15_cache_invalidate_level(0, 0);
  } else
#endif /* RTEMS_SMP */
  {
    rtems_cache_invalidate_entire_data();
  }
  rtems_cache_invalidate_entire_instruction();
  arm_cp15_branch_predictor_invalidate_all();
  arm_cp15_tlb_invalidate();
  arm_cp15_flush_prefetch_buffer();

  /* Clear Translation Table Base Control Register */
  arm_cp15_set_translation_table_base_control_register(0);

  /* Clear Secure or Non-secure Vector Base Address Register */
  arm_cp15_set_vector_base_address(0);

#ifdef RTEMS_SMP
  if (cpu_index_self == 0) {
    rpi_ipi_initialize();
  } else {
    rpi_start_rtems_on_secondary_processor();
  }
  ll_strout("CPU 0 OK\n\r");
#endif

  if (0) {
    /* Enable JTAG */
    rtems_gpio_bsp_select_specific_io(0, 22, RPI_ALT_FUNC_4);
    rtems_gpio_bsp_select_specific_io(0, 4,  RPI_ALT_FUNC_5);
    rtems_gpio_bsp_select_specific_io(0, 27, RPI_ALT_FUNC_4);
    rtems_gpio_bsp_select_specific_io(0, 25, RPI_ALT_FUNC_4);
    rtems_gpio_bsp_select_specific_io(0, 23, RPI_ALT_FUNC_4);
    rtems_gpio_bsp_select_specific_io(0, 24, RPI_ALT_FUNC_4);
  }

  if (0) {
    ll_strout("JTAG Enabled and waiting for GDB\n");

    continue_execution = 0;
    while (!continue_execution);
  }
  ll_strout("  bsp_start_hook_0 OK\n");
}

void BSP_START_TEXT_SECTION bsp_start_hook_1(void)
{
  ll_strout("bsp_start_hook_1\n\r");

  bsp_start_copy_sections();
  bsp_memory_management_initialize();
  bsp_start_clear_bss();

  rpi_video_init();
}
