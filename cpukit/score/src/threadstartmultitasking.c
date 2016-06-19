/**
 *  @file
 *
 *  @brief Start Thread Multitasking
 *  @ingroup ScoreThread
 */

/*
 *  COPYRIGHT (c) 1989-2006.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/score/threadimpl.h>
#include <rtems/score/assert.h>

void _Thread_Start_multitasking( void )
{
  Per_CPU_Control *cpu_self = _Per_CPU_Get();
  Thread_Control  *heir;

  ll_strout("_Thread_Start_multitasking:\n");
  ll_strout(" printk test ...\n");
  printk(" printk test OK\n");

#if defined(RTEMS_SMP)
  _Per_CPU_State_change( cpu_self, PER_CPU_STATE_UP );

  /*
   * Threads begin execution in the _Thread_Handler() function.   This
   * function will set the thread dispatch disable level to zero.
   */
  cpu_self->thread_dispatch_disable_level = 1;
#endif

  ll_strout(" _Thread_Get_heir_and_make_it_executing ...\n");
  heir = _Thread_Get_heir_and_make_it_executing( cpu_self );

  ll_strout(" _Profiling_Thread_dispatch_disable ...\n");
  _Profiling_Thread_dispatch_disable( cpu_self, 0 );

#if defined(RTEMS_SMP)
  ll_strout(" _CPU_SMP_Prepare_start_multitasking ...\n");
  _CPU_SMP_Prepare_start_multitasking();
#endif

#if defined(_CPU_Start_multitasking)
  _CPU_Start_multitasking( &heir->Registers );
#elif defined(RTEMS_SMP)
  {
    Context_Control trash;

    /*
     * Mark the trash context as executing to not confuse the
     * _CPU_Context_switch().  On SMP configurations the context switch
     * contains a special hand over section to atomically switch from the
     * executing to the currently selected heir thread.
     */
    ll_strout(" _CPU_Context_Set_is_executing ...\n");
    _CPU_Context_Set_is_executing( &trash, true );
    ll_strout(" _CPU_Context_switch ...\n");
    _CPU_Context_switch( &trash, &heir->Registers );
    ll_strout(" _CPU_Context_switch failed\n");
     RTEMS_UNREACHABLE();
  }
#else
  _CPU_Context_Restart_self( &heir->Registers );
#endif
}
