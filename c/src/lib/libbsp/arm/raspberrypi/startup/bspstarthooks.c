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
#include <bsp/usart.h>

void ll_strout(char *str)
{
  char ch;
  while ((ch = *(str++)) != 0) {
    if (ch == '\n')
      bcm2835_usart_fns.deviceWritePolled(0, '\r');
    bcm2835_usart_fns.deviceWritePolled(0, ch);
  }
}

volatile int continue_execution;

void BSP_START_TEXT_SECTION bsp_start_hook_0(void)
{
  uint32_t sctlr_val;

  ll_strout("bsp_start_hook_0:\n");

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
      arm_cp15_drain_write_buffer();
      arm_cp15_data_cache_clean_and_invalidate();
    }
    arm_cp15_flush_prefetch_buffer();
    sctlr_val &= ~(ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M | ARM_CP15_CTRL_A);
    arm_cp15_set_control(sctlr_val);

    arm_cp15_tlb_invalidate();
    arm_cp15_flush_prefetch_buffer();
    arm_cp15_data_cache_invalidate();
    arm_cp15_instruction_cache_invalidate();
  }

  /* Clear Translation Table Base Control Register */
  arm_cp15_set_translation_table_base_control_register(0);

  /* Clear Secure or Non-secure Vector Base Address Register */
  arm_cp15_set_vector_base_address(0);

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

void BSP_START_TEXT_SECTION bsp_start_hook_1(void)
{
  ll_strout("bsp_start_hook_1:\n");
  bsp_start_copy_sections();
  ll_strout(" bsp_start_copy_sections OK\n");
  bsp_memory_management_initialize();
  ll_strout(" bsp_memory_management_initialize OK\n");
  bsp_start_clear_bss();
  ll_strout(" bsp_start_clear_bss OK\n");

  rpi_video_init();
  ll_strout(" rpi_video_init OK\n");
}
