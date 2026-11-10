/* SPDX-License-Identifier: BSD-2-Clause OR Apache-2.0 OR GPL-2.0-or-later */

/**
 * @file  ctucanfd_pci.c
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
#include <dev/can/ctucanfd.h>

#include <rtems/pci.h>

#ifndef CTUCANFD_WORKER_PRIORITY
#define CTUCANFD_WORKER_PRIORITY 40
#endif

#ifndef CTUCANFD_PCI_MAX_MINOR_COUNT
#define CTUCANFD_PCI_MAX_MINOR_COUNT 28
#endif

#define CTUCANFD_PCI_VENDOR 0x1760
#define CTUCANFD_PCI_ID     0xff00

#define CTUCAN_BAR0_CTUCAN_ID 0x0000
#define CTUCAN_BAR0_CRA_BASE  0x4000
#define CYCLONE_IV_CRA_A2P_IE (0x0050)

#define CTUCANFD_PCI_CHANNELS_MAX      4
#define CTUCANFD_PCI_BYTES_PER_CHANNEL 0x4000

static int can_ctucanfd_pci_probe_instance( int instance )
{
  struct rtems_can_bus *can_bus[ CTUCANFD_PCI_CHANNELS_MAX ];
  int                   bus;
  int                   dev;
  int                   fun;
  int                   status;
  uint32_t              base0;
  uint32_t              base1;
  uint8_t               irq;
  uint32_t              cra_addr;
  uint32_t              cra_a2p_ie;
  int                   channels_try;
  int                   channels = 0;
  int                   i;
  int                   idx = 0;
  char                  devpath[ 32 ];
  int                   ret;

  status = pci_find_device(
    CTUCANFD_PCI_VENDOR,
    CTUCANFD_PCI_ID,
    instance,
    &bus,
    &dev,
    &fun );

  if ( status != PCIB_ERR_SUCCESS ) {
    printk( "can_ctucanfd_pci_probe_instance: pci_find_device instance %d failed: %d\n", instance, status );
    return -1;
  }

  pci_read_config_dword( bus, dev, fun, PCI_BASE_ADDRESS_0, &base0 );
  pci_read_config_dword( bus, dev, fun, PCI_BASE_ADDRESS_1, &base1 );
  pci_read_config_byte( bus, dev, fun, PCI_INTERRUPT_LINE, &irq );

  printk( "can_ctucanfd_pci_probe_instance base0 0x%08lx\n", (unsigned long)base0 );
  printk( "can_ctucanfd_pci_probe_instance base1 0x%08lx\n", (unsigned long)base1 );

  cra_addr = base0 + CTUCAN_BAR0_CRA_BASE;
  channels_try = *(volatile uint32_t *) ( base0 + CTUCAN_BAR0_CTUCAN_ID );
  channels_try = channels_try & 0xf;
  printk( "can_ctucanfd_pci_probe_instance trying %d channels\n", channels_try );

  if ( channels_try > CTUCANFD_PCI_CHANNELS_MAX )
    channels_try = CTUCANFD_PCI_CHANNELS_MAX;

  for ( i = 0; i < channels_try; i++ ) {
    can_bus[ i ] = malloc( sizeof( struct rtems_can_bus ) );
    if ( can_bus[ i ] == NULL ) {
      printk( "ERROR: can_ctucanfd_pci_probe_instance malloc failed!\n" );
      break;
    }

    can_bus[ i ]->chip = rtems_can_ctucanfd_initialize(
      base1 + CTUCANFD_PCI_BYTES_PER_CHANNEL * i,
      irq,
      CTUCANFD_WORKER_PRIORITY,
      4,
      RTEMS_INTERRUPT_SHARED,
      100000000 );

    if ( can_bus[ i ]->chip == NULL ) {
      free( can_bus[ i ] );
      if ( i == 0 ) {
        printk( "ERROR: can_ctucanfd_pci_probe_instance rtems_can_sja1000_initialize failed!\n" );
        return -1;
      } else {
        printk( "can_ctucanfd_pci_probe_instance only %i channels found\n", i );
        break;
      }
    }
    channels = i + 1;
  }

  for ( i = 0; i < channels; i++ ) {
    for ( idx = 0; idx < CTUCANFD_PCI_MAX_MINOR_COUNT; idx++ ) {
      snprintf( devpath, sizeof( devpath ) - 1, "dev/can%d", idx );

      ret = rtems_can_bus_register( can_bus[ i ], devpath );
      if ( ret == RTEMS_SUCCESSFUL ) {
        break;
      }
      if ( ret != RTEMS_SUCCESSFUL && idx == CTUCANFD_PCI_MAX_MINOR_COUNT - 1 ) {
        printk( "ERROR: rtems_can_bus_register failed\n" );
        channels = i;
        break;
      }
    }
  }

  cra_a2p_ie = *(volatile uint32_t *) ( cra_addr + CYCLONE_IV_CRA_A2P_IE );
  printk( "can_ctucanfd_pci_probe_instance cra_a2p_ie 0x%08lx\n", (unsigned long) cra_a2p_ie );
  cra_a2p_ie |= 1;
  *(volatile uint32_t *)  ( cra_addr + CYCLONE_IV_CRA_A2P_IE ) = cra_a2p_ie;

  return RTEMS_SUCCESSFUL;
}

void can_ctucanfd_pci_probe( void );

void can_ctucanfd_pci_probe( void )
{
  int instance = 0;

  while ( can_ctucanfd_pci_probe_instance( instance ) ==
          RTEMS_SUCCESSFUL ) {
    instance++;
  }
}

/*
RTEMS_SYSINIT_ITEM(
  can_ctucanfd_pci_probe,
  RTEMS_SYSINIT_DEVICE_DRIVERS,
  RTEMS_SYSINIT_ORDER_LAST
);
*/
