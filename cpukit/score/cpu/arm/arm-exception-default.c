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
  char s[30];
  ll_strout("_ARM_Exception_default\n");
  ll_strout(" register_lr ");
  itoa(frame->register_lr, s, 16);
  ll_strout(s);
  ll_strout("\n register_pc ");
  itoa(frame->register_pc, s, 16);
  ll_strout(s);
  ll_strout("\n vector ");
  itoa(frame->vector, s, 16);
  ll_strout(s);
  ll_strout("\n");

  rtems_fatal( RTEMS_FATAL_SOURCE_EXCEPTION, (rtems_fatal_code) frame );
}
