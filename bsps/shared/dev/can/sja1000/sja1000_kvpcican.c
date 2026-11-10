/* SPDX-License-Identifier: BSD-2-Clause OR Apache-2.0 OR GPL-2.0-or-later */

/**
 * @file  sja1000_kvpcican.c
 *
 * @ingroup CAN
 *
 * @brief This file is part of RTEMS CAN test application
 */

/*
 * Copyright (C) 2024-2026 Michal Lenc <michallenc@seznam.cz> FEE CTU
 * Copyright (C) 2024-2026 Pavel Pisa <pisa@cmp.felk.cvut.cz> FEE CTU
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <rtems/error.h>
#include <rtems/mw_uid.h>
#include <rtems/rtems/intr.h>
#include <rtems/untar.h>
#include <rtems/score/atomic.h>
#include <rtems/sysinit.h>
#include <rtems/bspIo.h>
#include <stdlib.h>
#include <stdio.h>

#include <dev/can/can.h>
#include <dev/can/can-bus.h>
#include <dev/can/sja1000.h>

#include <rtems/pci.h>

#ifndef SJA1000_WORKER_PRIORITY
#define SJA1000_WORKER_PRIORITY 40
#endif

#ifndef SJA1000_MAX_MINOR_COUNT
#define SJA1000_MAX_MINOR_COUNT 28
#endif

#if defined( __i386__ )
#define RTEMS_CAN_SJA1000_FOR_IO_PORTS 1
#include <i386_io.h>
#endif

#define KV_PCICAN_PCICAN_VENDOR 0x10e8
#define KV_PCICAN_PCICAN_ID     0x8406

/*AMCC 5920*/

#define S5920_OMB    0x0C
#define S5920_IMB    0x1C
#define S5920_MBEF   0x34
#define S5920_INTCSR 0x38
#define S5920_RCR    0x3C
#define S5920_PTCR   0x60

#define INTCSR_ADDON_INTENABLE_M    0x2000
#define INTCSR_INTERRUPT_ASSERTED_M 0x800000

#define KV_PCICAN_BYTES_PER_CIRCUIT 0x20

// Standard value: Pushpull  (OCTP1|OCTN1|OCTP0|OCTN0|OCM1)
#define KV_PCICAN_OCR_DEFAULT_STD 0xDA
// For Galathea piggyback.
#define KV_PCICAN_OCR_DEFAULT_GAL 0xDB

#define KV_PCICAN_CHANNELS_MAX 4

/*

You need to know the following:
" RX1 is connected to ground.
" TX1 is not connected.
" CLKO is not connected.
" Setting the OCR register to 0xDA is a good idea.
  This means  normal output mode , push-pull and the correct polarity.
" In the CDR register, you should set CBP to 1.
  You will probably also want to set the clock divider value to 0 (meaning
divide-by-2), the Pelican bit, and the clock-off bit (you have no need for
CLKOUT anyway.)

*/

static int can_sja1000_kvpcican_probe_instance( int instance )
{
  struct rtems_can_bus *can_bus[ KV_PCICAN_CHANNELS_MAX ];
  int                   bus;
  int                   dev;
  int                   fun;
  int                   status;
  uint32_t              base0;
  uint32_t              base1;
  uint32_t              base2;
  uint8_t               irq;
  uint32_t              tmp;
  char                  devpath[ 32 ];
  int                   ret;
  int                   channels_try = KV_PCICAN_CHANNELS_MAX;
  int                   channels = 0;
  int                   i;
  int                   idx = 0;

  status = pci_find_device( KV_PCICAN_PCICAN_VENDOR, // Kvaser SJA1000
                            KV_PCICAN_PCICAN_ID,     // Kvaser SJA1000
                            instance,
                            &bus,
                            &dev,
                            &fun );

  if ( status != PCIB_ERR_SUCCESS ) {
    return -1;
  }

  pci_read_config_dword( bus, dev, fun, PCI_BASE_ADDRESS_0, &base0 );
  pci_read_config_dword( bus, dev, fun, PCI_BASE_ADDRESS_1, &base1 );
  pci_read_config_dword( bus, dev, fun, PCI_BASE_ADDRESS_2, &base2 );
  pci_read_config_byte( bus, dev, fun, PCI_INTERRUPT_LINE, &irq );

  if ( ( ( base0 & PCI_BASE_ADDRESS_SPACE ) != PCI_BASE_ADDRESS_SPACE_IO ) ||
       ( ( base1 & PCI_BASE_ADDRESS_SPACE ) != PCI_BASE_ADDRESS_SPACE_IO ) ||
       ( ( base2 & PCI_BASE_ADDRESS_SPACE ) != PCI_BASE_ADDRESS_SPACE_IO ) ) {
    printk(
      "sja1000_pci_init: unexpected memory region in some of PCI BARs\n" );
    return -1;
  }

  base0 &= ~PCI_BASE_ADDRESS_SPACE;
  base1 &= ~PCI_BASE_ADDRESS_SPACE;
  base2 &= ~PCI_BASE_ADDRESS_SPACE;

  printk( "Kvaser PCICanX found, IO bases 0x%04x, 0x%04x, 0x%04x\n",
          (int) base0,
          (int) base1,
          (int) base2 );

  for ( i = 0; i < channels_try; i++ ) {
    can_bus[ i ] = malloc( sizeof( struct rtems_can_bus ) );
    if ( can_bus[ i ] == NULL ) {
      printk( "ERROR: can_sja1000_kvpcican_probe_instance malloc failed!\n" );
      break;
    }

    can_bus[ i ]->chip = rtems_can_sja1000_initialize(
      base1 + KV_PCICAN_BYTES_PER_CIRCUIT * i,
      RTEMS_SJA1000_HW_REG_IO_PORT | RTEMS_SJA1000_HW_REG_SPAN_1,
      irq,
      SJA1000_WORKER_PRIORITY,
      RTEMS_INTERRUPT_SHARED,
      16000000 );
    if ( can_bus[ i ]->chip == NULL ) {
      free( can_bus[ i ] );
      if ( i == 0 ) {
        printk( "ERROR: rtems_can_sja1000_initialize failed!\n" );
        return -1;
      } else {
        printk( "rtems_can_sja1000_initialize only %i chips found\n", i );
        break;
      }
    }
    channels = i + 1;
  }

  /* Enable interrupts from card */
  tmp = inw( base0 + S5920_INTCSR );
  tmp |= INTCSR_ADDON_INTENABLE_M;
  outw( tmp, base0 + S5920_INTCSR );

  for ( i = 0; i < channels; i++ ) {
    for ( idx = 0; idx < SJA1000_MAX_MINOR_COUNT; idx++ ) {
      snprintf( devpath, sizeof( devpath ) - 1, "dev/can%d", idx );

      ret = rtems_can_bus_register( can_bus[ i ], devpath );
      if ( ret == RTEMS_SUCCESSFUL ) {
        break;
      }
      if ( ret != RTEMS_SUCCESSFUL && idx == SJA1000_MAX_MINOR_COUNT - 1 ) {
        printk( "ERROR: rtems_can_bus_register failed\n" );
        channels = i;
        break;
      }
    }
  }

  return RTEMS_SUCCESSFUL;
}

void can_sja1000_kvpcican_probe( void );

void can_sja1000_kvpcican_probe( void )
{
  int instance = 0;

  while ( can_sja1000_kvpcican_probe_instance( instance ) ==
          RTEMS_SUCCESSFUL ) {
    instance++;
  }
}

/*
RTEMS_SYSINIT_ITEM(
  can_sja1000_kvpcican_probe,
  RTEMS_SYSINIT_DEVICE_DRIVERS,
  RTEMS_SYSINIT_ORDER_LAST
);
*/
