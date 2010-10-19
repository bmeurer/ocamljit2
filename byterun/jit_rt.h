#ifndef CAML_JIT_RT_H
#define CAML_JIT_RT_H

#include "jit.h"

/* from jit_rt_amd64.S */
CAML_JIT_INTERNAL void  caml_jit_rt_alloc1();
CAML_JIT_INTERNAL void  caml_jit_rt_alloc2();
CAML_JIT_INTERNAL void  caml_jit_rt_alloc3();
CAML_JIT_INTERNAL void  caml_jit_rt_allocN();
CAML_JIT_INTERNAL void  caml_jit_rt_apply1();
CAML_JIT_INTERNAL void  caml_jit_rt_apply2();
CAML_JIT_INTERNAL void  caml_jit_rt_apply3();
CAML_JIT_INTERNAL void  caml_jit_rt_apply();
CAML_JIT_INTERNAL void  caml_jit_rt_closure();
CAML_JIT_INTERNAL void  caml_jit_rt_grab_closure_and_return();
CAML_JIT_INTERNAL void  caml_jit_rt_makeblock1();
CAML_JIT_INTERNAL void  caml_jit_rt_makeblock2();
CAML_JIT_INTERNAL void  caml_jit_rt_makeblock3();
CAML_JIT_INTERNAL void  caml_jit_rt_makeblockN();
CAML_JIT_INTERNAL void  caml_jit_rt_makefloatblock();
CAML_JIT_INTERNAL void  caml_jit_rt_makeshrblock();
CAML_JIT_INTERNAL void  caml_jit_rt_process_signal();
CAML_JIT_INTERNAL void  caml_jit_rt_raise();
CAML_JIT_INTERNAL void  caml_jit_rt_restart();
CAML_JIT_INTERNAL void  caml_jit_rt_return();
CAML_JIT_INTERNAL value caml_jit_rt_start(code_t, value, value, value, value *);
CAML_JIT_INTERNAL void  caml_jit_rt_stop();

#endif /* !CAML_JIT_RT_H */
