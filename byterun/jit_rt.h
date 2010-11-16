/***********************************************************************/
/*                                                                     */
/*                           Objective Caml                            */
/*                                                                     */
/*            Benedikt Meurer, University of Siegen                    */
/*                                                                     */
/*  Copyright 2010 Department of Compiler Construction and Software    */
/*  Analysis, University of Siegen.  All rights reserved.  This file   */
/*  is distributed under the terms of the GNU Library General Public   */
/*  License, with the special exception on linking described in file   */
/*  ../LICENSE.                                                        */
/*                                                                     */
/***********************************************************************/

/* $Id$ */

#ifndef CAML_JIT_RT_H
#define CAML_JIT_RT_H

#include "jit.h"

#ifdef CAML_JIT

/* from jit_rt_amd64.S/jit_rt_i386.S */
CAML_JIT_INTERNAL void  caml_jit_rt_call_gc();
CAML_JIT_INTERNAL void  caml_jit_rt_allocN();
CAML_JIT_INTERNAL void  caml_jit_rt_apply1();
CAML_JIT_INTERNAL void  caml_jit_rt_apply2();
CAML_JIT_INTERNAL void  caml_jit_rt_apply3();
CAML_JIT_INTERNAL void  caml_jit_rt_apply();
CAML_JIT_INTERNAL void  caml_jit_rt_closure();
CAML_JIT_INTERNAL void  caml_jit_rt_grab_closure_and_return();
CAML_JIT_INTERNAL void  caml_jit_rt_makefloatblock();
CAML_JIT_INTERNAL void  caml_jit_rt_process_signal();
CAML_JIT_INTERNAL void  caml_jit_rt_raise();
CAML_JIT_INTERNAL void  caml_jit_rt_restart();
CAML_JIT_INTERNAL void  caml_jit_rt_return();
CAML_JIT_INTERNAL value caml_jit_rt_start(code_t, value, value, value, value *);
CAML_JIT_INTERNAL void  caml_jit_rt_stop();

CAML_JIT_INTERNAL void  caml_jit_rt_copy_double();
CAML_JIT_INTERNAL void  caml_jit_rt_add_float();
CAML_JIT_INTERNAL void  caml_jit_rt_sub_float();
CAML_JIT_INTERNAL void  caml_jit_rt_mul_float();
CAML_JIT_INTERNAL void  caml_jit_rt_div_float();

CAML_JIT_INTERNAL void  caml_jit_rt_copy_floats();

#ifdef DEBUG
CAML_JIT_INTERNAL void  caml_jit_rt_trace();
#endif

#endif /* CAML_JIT */

#endif /* !CAML_JIT_RT_H */
