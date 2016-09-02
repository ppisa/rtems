/*
 * Copyright (c) 2013 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Obere Lagerstr. 30
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include <rtems/score/cpu.h>
#include <rtems/fatal.h>

void _ARM_Exception_default( CPU_Exception_frame *frame )
{
  ll_strout("_ARM_Exception_default\n");
  ll_strout(" register_lr ");
  ll_hexout(frame->register_lr, 8);
  ll_strout("\n register_pc ");
  ll_hexout(frame->register_pc, 8);
  ll_strout("\n vector ");
  ll_hexout(frame->vector, 8);
  ll_strout("\n");

  rtems_fatal( RTEMS_FATAL_SOURCE_EXCEPTION, (rtems_fatal_code) frame );
}
