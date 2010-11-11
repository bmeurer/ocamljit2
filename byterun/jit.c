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

/* The interface of this file is in "jit.h" */

#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "callback.h"
#include "fail.h"
#include "instruct.h"
#include "instrtrace.h"
#include "interp.h"
#include "jit.h"
#include "jit_rt.h"
#include "jx86.h"
#include "memory.h"
#include "misc.h"
#include "mlvalues.h"
#include "prims.h"
#include "signals.h"
#include "stacks.h"


/* from array.c */
extern value caml_array_unsafe_get_float(value, value);
extern value caml_array_unsafe_set_float(value, value, value);

/* from floats.c */
extern value caml_asin_float(value);
extern value caml_cos_float(value);
extern value caml_sin_float(value);
extern value caml_sqrt_float(value);
extern value caml_add_float(value, value);
extern value caml_sub_float(value, value);
extern value caml_mul_float(value, value);
extern value caml_div_float(value, value);
extern value caml_atan2_float(value, value);
extern value caml_eq_float(value, value);
extern value caml_neq_float(value, value);
extern value caml_le_float(value, value);
extern value caml_lt_float(value, value);
extern value caml_ge_float(value, value);
extern value caml_gt_float(value, value);

/* from obj.c */
extern value caml_cache_public_method2(value *, value, opcode_t *);


static caml_jit_uint8_t *caml_jit_compile(code_t pc);


static caml_jit_block_t *caml_jit_block_head = NULL;
caml_jit_uint8_t *caml_jit_code_base = NULL;
caml_jit_uint8_t *caml_jit_code_end = NULL;
caml_jit_uint8_t *caml_jit_code_ptr = NULL;
static caml_jit_uint8_t *caml_jit_code_raise_zero_divide = NULL;
opcode_t caml_jit_callback_return = 0; /* for caml_callbackN_exn */
unsigned caml_jit_enabled = 0;


#ifdef DEBUG
static caml_jit_uint8_t *caml_jit_debug_addr = NULL;
static long caml_bcodcount = 0;

static void caml_jit_debug(opcode_t instr, code_t pc, code_t prog, asize_t prog_size, value accu, value extra_args, value env, value *sp)
{
  opcode_t save = *pc;

  *pc = instr;
  caml_bcodcount++;
  if (caml_trace_flag > 1) printf("\n##%ld\n", caml_bcodcount);
  if (caml_trace_flag) caml_disasm_instr(pc);
  if (caml_trace_flag > 1) {
    printf("extra_args=%ld\n", Long_val(extra_args));
    printf("env=");
    caml_trace_value_file(env, prog, prog_size, stdout);
    putchar('\n');
    caml_trace_accu_sp_file(accu, sp, prog, prog_size, stdout);
    fflush(stdout);
  }
  Assert(sp >= caml_stack_low);
  Assert(sp <= caml_stack_high);
  *pc = save;
}
#endif


void caml_jit_init()
{
  caml_jit_uint8_t *cp;

  /* check if already initialized */
  if (caml_jit_code_base != NULL)
    return;

  /* the JIT engine is incompatible with ocamldebug */
  caml_jit_enabled = (getenv("CAML_DEBUG_SOCKET") == NULL);
  if (CAML_JIT_GNUC_UNLIKELY (!caml_jit_enabled))
    return;

  /* allocate memory for the JIT code */
  cp = (caml_jit_uint8_t *) mmap(NULL, CAML_JIT_CODE_SIZE,
                                 PROT_EXEC | PROT_READ | PROT_WRITE,
                                 MAP_ANON | MAP_PRIVATE, 0, (off_t) 0);
  if (cp == (caml_jit_uint8_t *) MAP_FAILED)
    caml_fatal_error_arg("Failed to allocate JIT code area (%s)!\n", strerror(errno));
  caml_jit_code_ptr = cp;

  /* Generate the compile trampoline code at the end. If
   * caml_jit_compile is within +/-2GB of code area, we
   * use the shorter (and faster) sequence:
   *
   *   xchgq %rax, %rbx
   *   call  caml_jit_compile(%rip)
   *   xchgq %rax, %rbx
   *   jmpq *%rbx
   *   ud2
   *
   * Otherwise we have to use an indirect call via a 64bit
   * pointer:
   *
   *   xchgq %rax, %rbx
   *   callq 6(%rip)
   *   xchgq %rax, %rbx
   *   jmpq  *%rbx
   *   ud2
   *   .quad caml_jit_compile
   *
   * Assumes byte code address to compile is in %rdi and
   * C stack aligned on 16-byte boundary. Preserves %rax
   * since the compile trampoline may also be used to
   * return to not yet compiled code, where %rax contains
   * the return value.
   *
   * The UD2 instruction following the indirect jump is
   * used to stop the processor from decoding down the
   * fall-through path.
   */
  caml_jit_code_end = cp + CAML_JIT_CODE_SIZE - (2 + 5 + 2 + 2 + 2);
  if (JX86_IS_IMM32((caml_jit_uint8_t *) &caml_jit_compile - caml_jit_code_end - 2 - 5)) {
    cp = caml_jit_code_end;
    jx86_xchgq_reg_reg(cp, JX86_RBX, JX86_RAX); /* xchgq %rax, %rbx */
    jx86_call(cp, &caml_jit_compile);           /* call  caml_jit_compile(%rip) */
    jx86_xchgq_reg_reg(cp, JX86_RAX, JX86_RBX); /* xchgq %rbx, %rax */
    jx86_jmpq_reg(cp, JX86_RBX);                /* jmpq *%rbx */
    jx86_ud2(cp);                               /* ud2 */
  }
  else {
    caml_jit_code_end = (caml_jit_code_end + 5) - (6 + 8);
    cp = caml_jit_code_end;
    jx86_xchgq_reg_reg(cp, JX86_RBX, JX86_RAX); /* xchgq %rax, %rbx */
    jx86_emit_uint8(cp, 0xff);                  /* callq 6(%rip) */
    jx86_emit_address_byte(cp, 0, 2, 5);
    jx86_emit_int32(cp, 6);
    jx86_xchgq_reg_reg(cp, JX86_RAX, JX86_RBX); /* xchgq %rbx, %rax */
    jx86_jmpq_reg(cp, JX86_RBX);                /* jmpq  *%rbx */
    jx86_ud2(cp);                               /* ud2 */
    jx86_emit_uint64(cp, &caml_jit_compile);    /* .quad caml_jit_compile */
  }
  caml_jit_assert(cp == caml_jit_code_ptr + CAML_JIT_CODE_SIZE);

  /* Generate the NOPs in front of the compile trampoline.
   * Each byte code instruction gets one NOP, so that in the
   * end, all not yet compiled instructions will redirect
   * to the compile trampoline.
   */
  cp = caml_jit_code_end;
  caml_jit_code_end -= BREAK;
  while (cp > caml_jit_code_end)
    *--cp = 0x90;
  caml_jit_assert(cp == caml_jit_code_end);

  cp = caml_jit_code_ptr;

  /* Setup the "raise zero divide" code (used by DIVINT/MODINT) */
  caml_jit_code_raise_zero_divide = cp;
  jx86_movq_membase_reg(cp, JX86_RSP, 0 * 8, JX86_R11); /* saved_pc = pc */
  jx86_subq_reg_imm(cp, JX86_R14, 1 * 8);               /* *--sp = env */
  jx86_movq_membase_reg(cp, JX86_R14, 0 * 8, JX86_R12);
  jx86_movq_reg_imm(cp, JX86_R11, &caml_extern_sp);     /* caml_extern_sp = sp */
  jx86_movq_membase_reg(cp, JX86_R11, 0, JX86_R14);
  jx86_movq_reg_imm(cp, JX86_R11, &caml_young_ptr);     /* caml_young_ptr = %r15 */
  jx86_movq_membase_reg(cp, JX86_R11, 0, JX86_R15);
  jx86_call(cp, &caml_raise_zero_divide);

  /* setup the callback return code */
  caml_jit_callback_return = cp - caml_jit_code_end;
  jx86_jmp(cp, &caml_jit_rt_stop);

#ifdef DEBUG
  caml_jit_debug_addr = cp;
  jx86_pushq_reg(cp, JX86_R8);
  jx86_pushq_reg(cp, JX86_R9);
  jx86_pushq_reg(cp, JX86_R10);
  jx86_pushq_reg(cp, JX86_R11);
  jx86_pushq_reg(cp, JX86_RAX);
  jx86_movq_reg_imm(cp, JX86_R11, &caml_young_ptr);
  jx86_movq_membase_reg(cp, JX86_R11, 0, JX86_R15);
  jx86_movq_reg_reg(cp, JX86_R15, JX86_RSP);
  jx86_movq_reg_reg(cp, JX86_R8, JX86_RAX);
  jx86_movq_reg_reg(cp, JX86_R9, JX86_R13);
  jx86_andq_reg_imm(cp, JX86_RSP, -16);
  jx86_pushq_reg(cp, JX86_R14);
  jx86_pushq_reg(cp, JX86_R12);
  jx86_call(cp, &caml_jit_debug);
  jx86_movq_reg_reg(cp, JX86_RSP, JX86_R15);
  jx86_movq_reg_imm(cp, JX86_R11, &caml_young_ptr);
  jx86_movq_reg_membase(cp, JX86_R15, JX86_R11, 0);
  jx86_popq_reg(cp, JX86_RAX);
  jx86_popq_reg(cp, JX86_R11);
  jx86_popq_reg(cp, JX86_R10);
  jx86_popq_reg(cp, JX86_R9);
  jx86_popq_reg(cp, JX86_R8);
  jx86_ret(cp);
#endif

  caml_jit_code_base = cp - (BREAK + 1);
  caml_jit_code_ptr = cp;
}


static code_t *caml_jit_pending_buffer = NULL;
static int caml_jit_pending_capacity = 0;
static int caml_jit_pending_size = 0;

static inline void caml_jit_pending_add(code_t pc)
{
  caml_jit_assert(pc != NULL);
  caml_jit_assert(caml_jit_pending_size >= 0);
  caml_jit_assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  caml_jit_assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  caml_jit_assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

  if (CAML_JIT_GNUC_UNLIKELY (caml_jit_pending_size == caml_jit_pending_capacity)) {
    caml_jit_pending_capacity += 32;
    caml_jit_pending_buffer = realloc(caml_jit_pending_buffer, caml_jit_pending_capacity * sizeof(code_t));
  }
  caml_jit_pending_buffer[caml_jit_pending_size] = pc;
  caml_jit_pending_size += 1;
}

static inline int caml_jit_pending_contains(code_t pc)
{
  int i;

  caml_jit_assert(pc != NULL);
  caml_jit_assert(caml_jit_pending_size >= 0);
  caml_jit_assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  caml_jit_assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  caml_jit_assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

  for (i = 0; i < caml_jit_pending_size; ++i)
    if (caml_jit_pending_buffer[i] == pc)
      return 1;
  return 0;
}

static caml_jit_uint8_t *caml_jit_compile(code_t pc)
{
  void *addr;
  caml_jit_uint8_t *cp;
  opcode_t instr;
  unsigned op;
  int sp = 0;
#ifdef DEBUG
  caml_jit_block_t *block;
#endif
  unsigned state = 0;

  caml_jit_assert(pc != NULL);
  caml_jit_assert(caml_jit_enabled);
  caml_jit_assert(*pc >= 0 && *pc <= BREAK);
  caml_jit_assert(caml_jit_pending_size >= 0);
  caml_jit_assert(caml_jit_code_base != NULL);
  caml_jit_assert(caml_jit_code_ptr < caml_jit_code_end);
  caml_jit_assert(caml_jit_code_ptr > caml_jit_code_base);
  caml_jit_assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  caml_jit_assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  caml_jit_assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

#ifdef DEBUG
  for (block = caml_jit_block_head;; block = block->block_next) {
    caml_jit_assert(block != NULL);
    if (block->block_prog <= pc && pc < block->block_pend)
      break;
  }
#endif

  for (cp = caml_jit_code_ptr;; ) {
    caml_jit_assert((sp % 8) == 0);
    caml_jit_assert(cp < caml_jit_code_end);
    caml_jit_assert(cp > caml_jit_code_base);
    caml_jit_assert(pc < block->block_pend);
    caml_jit_assert(pc >= block->block_prog);

    /* determine the next instruction */
    instr = *pc;

    /* check if this instruction is already compiled */
    if (CAML_JIT_GNUC_UNLIKELY (instr < 0)) {
      /* determine the native code address */
      addr = caml_jit_code_end + instr;

    flush_jmp_stop_generation:
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }

      /* flush the pending float box */
      if (state != 0) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }

    jmp_stop_generation:
      /* generate a jmp to the known native code */
      caml_jit_assert(sp == 0);
      caml_jit_assert(state == 0);
      jx86_jmp(cp, addr);

    stop_generation:
      caml_jit_assert(sp == 0);
      caml_jit_assert(state == 0);
      if (caml_jit_pending_size > 0) {
        pc = caml_jit_pending_buffer[--caml_jit_pending_size];
        /* check if this pending item is already compiled */
        if (CAML_JIT_GNUC_UNLIKELY (*pc < 0))
          goto stop_generation;
        continue;
      }
      break;
    }

    /* patch forward jccs/jmps to this byte code address */
    while (CAML_JIT_GNUC_UNLIKELY (instr > BREAK)) {
      caml_jit_uint8_t *jcp = caml_jit_code_base + instr;

      /* flush the stack pointer */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }

      /* flush the pending float box */
      if (state != 0) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }

      caml_jit_assert(jcp > caml_jit_code_base);
      caml_jit_assert(jcp + 4 < caml_jit_code_end);
      caml_jit_assert(jcp[0] == 0
                      || (jcp[0] == 0x0f && jcp[1] >= 0x80 && jcp[1] <= 0x8f));

      if (CAML_JIT_GNUC_UNLIKELY (jcp[0] == 0)) {
        instr = *((caml_jit_int32_t *) (jcp + 1));
        *((caml_jit_uintptr_t *) jcp) = (caml_jit_uintptr_t) cp;
      }
      else {
        instr = *((caml_jit_int32_t *) (jcp + 2));
        jx86_jcc32_patch(cp, jcp);
      }
    }

    caml_jit_assert(instr >= 0);
    caml_jit_assert(instr <= BREAK);
    caml_jit_assert((sp % 8) == 0);
    caml_jit_assert(cp < caml_jit_code_end);
    caml_jit_assert(cp > caml_jit_code_base);

    /* setup the native code offset for this instruction */
    if (CAML_JIT_GNUC_LIKELY (sp == 0 && state == 0))
      *((caml_jit_int32_t *) pc) = (caml_jit_int32_t) (cp - caml_jit_code_end);

#ifdef DEBUG
    if (caml_trace_flag && caml_jit_debug_addr != NULL) {
      if (sp != 0) {
        jx86_pushq_reg(cp, JX86_R14);
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
      }

      jx86_pushq_reg(cp, JX86_RDI);
      jx86_pushq_reg(cp, JX86_RSI);
      jx86_pushq_reg(cp, JX86_RDX);
      jx86_pushq_reg(cp, JX86_RCX);
      jx86_movl_reg_imm(cp, JX86_EDI, instr);
      jx86_movq_reg_imm(cp, JX86_RSI, pc);
      jx86_movq_reg_imm(cp, JX86_RDX, block->block_prog);
      jx86_movq_reg_imm(cp, JX86_RCX, (block->block_pend - block->block_prog) * sizeof(*block->block_prog));
      jx86_call(cp, caml_jit_debug_addr);
      jx86_popq_reg(cp, JX86_RCX);
      jx86_popq_reg(cp, JX86_RDX);
      jx86_popq_reg(cp, JX86_RSI);
      jx86_popq_reg(cp, JX86_RDI);

      if (sp != 0) {
        jx86_popq_reg(cp, JX86_R14);
      }
    }
#endif

    ++pc;

  again:
    switch (instr) {

    state1_to_state0:
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      jx86_call(cp, &caml_jit_rt_copy_double);
      state = 0;
      goto again;

/* Basic stack operations */

    case PUSHACC:
      if (state != 0)
        goto state1_to_state0;
      instr = ACC0 + *pc++;
      goto pushaccX;
    case PUSHACC1:
    case PUSHACC2:
    case PUSHACC3:
    case PUSHACC4:
    case PUSHACC5:
    case PUSHACC6:
    case PUSHACC7:
      if (state != 0)
        goto state1_to_state0;
      instr = (instr - PUSHACC0) + ACC0;
    pushaccX:
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case ACC0:
    case ACC1:
    case ACC2:
    case ACC3:
    case ACC4:
    case ACC5:
    case ACC6:
    case ACC7:
    accX:
      jx86_movq_reg_membase(cp, JX86_RAX, JX86_R14, sp + (instr - ACC0) * 8);
      state = 0;
      break;

    case ACC:
      instr = ACC0 + *pc++;
      goto accX;

    case PUSH:
    case PUSHACC0:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      break;

    case ASSIGN:
      if (state != 0)
        goto state1_to_state0;
      jx86_movq_membase_reg(cp, JX86_R14, sp + *pc++ * 8, JX86_RAX);
    unit:
      jx86_movl_reg_imm(cp, JX86_EAX, Val_unit);
      state = 0;
      break;

    case POP:
      sp += *pc++ * 8;
      break;

/* Access in heap-allocated environment */

    case PUSHENVACC:
      if (state != 0)
        goto state1_to_state0;
      instr = (ENVACC1 - 1) + *pc++;
      goto pushenvaccX;
    case PUSHENVACC1:
    case PUSHENVACC2:
    case PUSHENVACC3:
    case PUSHENVACC4:
      if (state != 0)
        goto state1_to_state0;
      instr = (instr - PUSHENVACC1) + ENVACC1;
    pushenvaccX:
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case ENVACC1:
    case ENVACC2:
    case ENVACC3:
    case ENVACC4:
    envaccX:
      jx86_movq_reg_membase(cp, JX86_RAX, JX86_R12, (instr - (ENVACC1 - 1)) * 8);
      state = 0;
      break;

    case ENVACC:
      instr = (ENVACC1 - 1) + *pc++;
      goto envaccX;

/* Function application */

    case PUSH_RETADDR:
      sp -= 3 * 8;
      jx86_movq_reg_imm(cp, JX86_RDX, (value) (pc + *pc));
      jx86_movq_membase_reg(cp, JX86_R14, sp + 0 * 8, JX86_RDX); /* return address */
      jx86_movq_membase_reg(cp, JX86_R14, sp + 1 * 8, JX86_R12); /* env */
      jx86_movq_membase_reg(cp, JX86_R14, sp + 2 * 8, JX86_R13); /* extra args */
      pc++;
      break;

    case APPLY1:
      addr = &caml_jit_rt_apply1;
    applyX:
      jx86_movq_reg_imm(cp, JX86_RDI, (caml_jit_uintptr_t) pc);
    apply:
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      caml_jit_assert(state == 0);
      jx86_jmp(cp, addr);
      break;

    case APPLY2:
      addr = &caml_jit_rt_apply2;
      goto applyX;

    case APPLY3:
      addr = &caml_jit_rt_apply3;
      goto applyX;

    case APPLY:
      jx86_movl_reg_imm(cp, JX86_R13, Val_int(*pc++ - 1));
      addr = &caml_jit_rt_apply;
      goto apply;

    case APPTERM:
      instr = (APPTERM1 - 1) + *pc++;
      /* FALL-THROUGH */
    case APPTERM1:
    case APPTERM2:
    case APPTERM3: {
      int i;
      int num_args = instr - (APPTERM1 - 1);
      int newsp;

      /* slide the num_args bottom words of the current frame to the
       * top of the frame, and discard the remainder of the frame.
       */
      newsp = sp + (*pc++ - num_args) * 8;
      for (i = num_args - 1; i >= 0; --i) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp + i * 8);
        jx86_movq_membase_reg(cp, JX86_R14, newsp + i * 8, JX86_RSI);
      }
      sp = newsp;

      /* calculate the extra args */
      if (num_args > 1)
        jx86_addq_reg_imm(cp, JX86_R13, (unsigned) (num_args - 1) << 1);

      /* perform the application (closure in %rax) */
      addr = &caml_jit_rt_apply;
      goto flush_jmp_stop_generation;
    }

    case RETURN:
      sp += *pc++ * 8;
      addr = &caml_jit_rt_return;
      goto flush_jmp_stop_generation;

    case RESTART:
      addr = &caml_jit_rt_restart;
      goto flush_call_addr;

    case GRAB: {
      caml_jit_uint8_t *jae8;
      int required = *pc++;

      caml_jit_assert(sp == 0);
      caml_jit_assert(state == 0);

      /* check if we have too few extra args */
      jx86_cmpq_reg_imm(cp, JX86_R13, Val_int(required));
      jae8 = cp;
      jx86_jae8_forward(cp);

      /* generate a restart closure and return to caller */
      jx86_movq_reg_imm(cp, JX86_RAX, pc - 3);
      jx86_jmp(cp, &caml_jit_rt_grab_closure_and_return);

      /* calculate remaining extra arguments */
      jx86_jcc8_patch(cp, jae8);
      jx86_addq_reg_imm(cp, JX86_R13, -required << 1);
      break;
    }

    case CLOSURE: {
      int num_vars = *pc++;
      if (num_vars == 0) {
        addr = &caml_jit_rt_makeblock1;
      }
      else {
        if (state != 0) {
          pc--;
          goto state1_to_state0;
        }

        /* push accu onto the Caml stack */
        sp -= 8;
        jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);

        if (num_vars == 1) {
          addr = &caml_jit_rt_makeblock2;
        }
        else if (num_vars == 2) {
          addr = &caml_jit_rt_makeblock3;
        }
        else {
          jx86_movl_reg_imm(cp, JX86_EDI, num_vars + 1);
          addr = &caml_jit_rt_makeblockN;
        }
      }
      jx86_movl_reg_imm(cp, JX86_ESI, Closure_tag);
      jx86_movq_reg_imm(cp, JX86_RAX, pc + *pc);
      state = 0;
      pc++;
      goto flush_call_addr;
    }

    case CLOSUREREC: {
      int i;
      int num_funs = *pc++;
      int num_vars = *pc++;
      mlsize_t wosize = num_funs * 2 - 1 + num_vars;
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      if (state != 0)
        wosize += 2;
      switch (wosize) {
      case 1:
        addr = &caml_jit_rt_alloc1;
        break;

      case 2:
        addr = &caml_jit_rt_alloc2;
        break;

      case 3:
        addr = &caml_jit_rt_alloc3;
        break;

      default:
        jx86_movq_reg_imm(cp, JX86_R10, Bhsize_wosize(wosize));
        addr = &caml_jit_rt_allocN;
        break;
      }
      jx86_call(cp, addr);
      if (state != 0) {
        wosize -= 2;
        jx86_movq_membase_imm(cp, JX86_R15, (wosize + 1) * 8, Make_header(Double_wosize, Double_tag, Caml_black));
        jx86_leaq_reg_membase(cp, JX86_RAX, JX86_R15, (wosize + 1) * 8);
        jx86_movlpd_membase_xmm(cp, JX86_R15, (wosize + 2) * 8, JX86_XMM0);
        state = 0;
      }
      jx86_movq_membase_imm(cp, JX86_R15, 0 * 8, Make_header(wosize, Closure_tag, Caml_black));
      jx86_movq_reg_imm(cp, JX86_RDX, (caml_jit_intptr_t) (pc + pc[0]));
      if (--num_vars >= 0) {
        jx86_movq_membase_reg(cp, JX86_R15, num_funs * 16, JX86_RAX);
        for (i = 0; i < num_vars; ++i, sp += 8) {
          jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
          jx86_movq_membase_reg(cp, JX86_R15, num_funs * 16 + (i + 1) * 8, JX86_RSI);
        }
      }
      jx86_leaq_reg_membase(cp, JX86_RAX, JX86_R15, 1 * 8);
      jx86_movq_membase_reg(cp, JX86_RAX, 0, JX86_RDX);
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      for (i = 1; i < num_funs; ++i) {
        jx86_leaq_reg_membase(cp, JX86_RSI, JX86_R15, (i * 2 + 1) * 8);
        jx86_movq_reg_imm(cp, JX86_RDI, pc + pc[i]);
        jx86_movq_membase_imm(cp, JX86_RSI, -1 * 8, Make_header(i * 2, Infix_tag, Caml_white));
        jx86_movq_membase_reg(cp, JX86_RSI,  0 * 8, JX86_RDI);
        sp -= 8;
        jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RSI);
      }
      pc += num_funs;
      break;
    }

    case PUSHOFFSETCLOSURE:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE:
      jx86_leaq_reg_membase(cp, JX86_RAX, JX86_R12, *pc++ * 8);
      state = 0;
      break;

    case PUSHOFFSETCLOSUREM2:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case OFFSETCLOSUREM2:
      jx86_leaq_reg_membase(cp, JX86_RAX, JX86_R12, -2 * 8);
      state = 0;
      break;

    case PUSHOFFSETCLOSURE0:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE0:
      jx86_movq_reg_reg(cp, JX86_RAX, JX86_R12);
      state = 0;
      break;

    case PUSHOFFSETCLOSURE2:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE2:
      jx86_leaq_reg_membase(cp, JX86_RAX, JX86_R12, 2 * 8);
      state = 0;
      break;

/* Access to global variables */

    case PUSHGETGLOBAL:
    case PUSHGETGLOBALFIELD:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case GETGLOBAL:
    case GETGLOBALFIELD:
      caml_jit_assert(state == 0);
      jx86_movq_reg_imm(cp, JX86_RSI, &caml_global_data);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_RSI, 0);
      jx86_movq_reg_membase(cp, JX86_RAX, JX86_RSI, *pc++ * 8);
      if (instr >= GETGLOBALFIELD)
        goto getfield;
      break;

    case SETGLOBAL:
      if (state != 0)
        goto state1_to_state0;
      jx86_movq_reg_imm(cp, JX86_RDI, &caml_global_data);
      jx86_movq_reg_reg(cp, JX86_RSI, JX86_RAX);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_RDI, 0);
      jx86_addq_reg_imm(cp, JX86_RDI, *pc++ * 8);
      goto modify;

/* Allocation of blocks */

    case PUSHATOM:
      if (state != 0)
        goto state1_to_state0;
      instr = ATOM0 + *pc++;
      goto pushatomX;
    case PUSHATOM0:
      if (state != 0)
        goto state1_to_state0;
      instr = ATOM0;
    pushatomX:
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case ATOM0:
    atomX:
      jx86_movq_reg_imm(cp, JX86_RAX, Atom(instr - ATOM0));
      state = 0;
      break;

    case ATOM:
      instr = ATOM0 + *pc++;
      goto atomX;

    case MAKEBLOCK: {
      mlsize_t wosize = *pc++;
      jx86_movq_reg_imm(cp, JX86_RDI, wosize);
      if (*pc == 0)
        jx86_xorl_reg_reg(cp, JX86_ESI, JX86_ESI);
      else
        jx86_movl_reg_imm(cp, JX86_ESI, *pc);
      pc++;
      addr = (wosize <= Max_young_wosize) ? &caml_jit_rt_makeblockN : &caml_jit_rt_makeshrblock;
      goto flush_call_addr;
    }

    case MAKEFLOATBLOCK:
      jx86_movq_reg_imm(cp, JX86_RDI, *pc++);
      addr = &caml_jit_rt_makefloatblock;
      goto flush_call_addr;

    case MAKEBLOCK1:
      addr = &caml_jit_rt_makeblock1;
    makeblockX:
      if (*pc == 0)
        jx86_xorl_reg_reg(cp, JX86_ESI, JX86_ESI);
      else
        jx86_movl_reg_imm(cp, JX86_ESI, *pc);
      pc++;
    flush_call_addr:
      if (CAML_JIT_GNUC_UNLIKELY (sp != 0)) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      if (CAML_JIT_GNUC_UNLIKELY (state != 0)) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }
    call_addr:
      jx86_call(cp, addr);
      break;
    case MAKEBLOCK2:
      addr = &caml_jit_rt_makeblock2;
      goto makeblockX;
    case MAKEBLOCK3:
      addr = &caml_jit_rt_makeblock3;
      goto makeblockX;

/* Access to components of blocks */

    case GETFIELD:
    getfield:
      instr = GETFIELD0 + *pc++;
      /* FALL-THROUGH */
    case GETFIELD0:
    case GETFIELD1:
    case GETFIELD2:
    case GETFIELD3:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RAX, JX86_RAX, (instr - GETFIELD0) * 8);
      break;

    case GETFLOATFIELD:
      caml_jit_assert(state == 0);
      jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, *pc++ * 8);
      state = 1;
      break;

    case SETFIELD:
      instr = SETFIELD0 + *pc++;
      /* FALL-THROUGH */
    case SETFIELD0:
    case SETFIELD1:
    case SETFIELD2:
    case SETFIELD3:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
      sp += 8;
      jx86_leaq_reg_membase(cp, JX86_RDI, JX86_RAX, (instr - SETFIELD0) * 8);
    modify:
      jx86_call(cp, &caml_modify);
      goto unit;

    case SETFLOATFIELD:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
      sp += 8;
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_RSI, 0);
      jx86_movq_membase_reg(cp, JX86_RAX, *pc++ * 8, JX86_RSI);
      goto unit;

/* Array operations */

    case VECTLENGTH:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RAX, JX86_RAX, -1 * 8);
      jx86_shrq_reg_imm(cp, JX86_RAX, 9);
      jx86_orq_reg_imm(cp, JX86_RAX, 1);
      break;

    case GETVECTITEM:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
      jx86_shrq_reg_imm(cp, JX86_RSI, 1);
      jx86_movq_reg_memindex(cp, JX86_RAX, JX86_RAX, 0, JX86_RSI, 3);
      goto pop1;

    case SETVECTITEM:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_R14, sp + 0 * 8);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp + 1 * 8);
      sp += 2 * 8;
      jx86_shrq_reg_imm(cp, JX86_RDI, 1);
      jx86_leaq_reg_memindex(cp, JX86_RDI, JX86_RAX, 0, JX86_RDI, 3);
      goto modify;

/* String operations */

    case GETSTRINGCHAR:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
      jx86_shrq_reg_imm(cp, JX86_RSI, 1);
      jx86_movb_reg_memindex(cp, JX86_AL, JX86_RAX, 0, JX86_RSI, 0);
      jx86_movzxlb_reg_reg(cp, JX86_EAX, JX86_AL);
    shl1_or1_pop1:
      jx86_shlq_reg_imm(cp, JX86_RAX, 1);
      goto or1_pop1;

    case SETSTRINGCHAR:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_R14, sp + 0 * 8);
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, sp + 1 * 8);
      sp += 2 * 8;
      jx86_shrq_reg_imm(cp, JX86_RDI, 1);
      jx86_shrl_reg_imm(cp, JX86_ECX, 1);
      jx86_movb_memindex_reg(cp, JX86_RAX, 0, JX86_RDI, 0, JX86_CL);
      goto unit;

/* Branches and conditional branches */

    case BRANCH: {
      code_t dpc = pc + *pc;

      /* flush the stack pointer */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }

      /* flush the float box */
      if (state != 0) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }

      /* check if target already compiled */
      if (*dpc < 0) {
        /* jump to known target */
        addr = caml_jit_code_end + *dpc;
        goto jmp_stop_generation;
      }
      else {
        /* compile the target code */
        pc = dpc;
      }
      break;
    }
        
    case BRANCHIF:
      op = JX86_JNE;
    branchif0:
      jx86_cmpq_reg_imm(cp, JX86_RAX, Val_false);
    branchif1: {
        code_t dpc = pc + *pc;
        pc++;

        /* flush the stack pointer */
        if (sp != 0) {
          jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
          sp = 0;
        }

        /* flush the float box */
        if (state != 0) {
          jx86_call(cp, &caml_jit_rt_copy_double);
          state = 0;
        }

        if (*dpc < 0) {
          /* "then" address is known */
          jx86_jcc(cp, op, caml_jit_code_end + *dpc);
        }
        else if (*pc < 0) {
          /* "else" address is known, negate condition
           * generate jcc to "else" address and continue
           * generation with the "then" address
           */
          jx86_jcc(cp, op ^ 1, caml_jit_code_end + *pc);
          pc = dpc;
        }
        else {
          /* neither "then" nor "else" are known */
          caml_jit_intptr_t offset = cp - caml_jit_code_base;

          caml_jit_assert(*pc >= 0);
          caml_jit_assert(*dpc >= 0);
          caml_jit_assert(offset > STOP);
          caml_jit_assert(offset <= CAML_JIT_INT32_MAX);

          if (*pc > BREAK) {
            /* "then" address is already on the pending list */
            caml_jit_assert(caml_jit_pending_contains(pc));
            jx86_jcc32_forward(cp, op ^ 1);
            *((caml_jit_int32_t *) cp - 1) = *pc;
            *pc = offset;
            pc = dpc;
          }
          else {
            /* add "else" address if not already pending */
            if (*dpc <= BREAK)
              caml_jit_pending_add(dpc);
            caml_jit_assert(caml_jit_pending_contains(dpc));
            jx86_jcc32_forward(cp, op);
            *((caml_jit_int32_t *) cp - 1) = *dpc;
            *dpc = offset;
          }
        }
        break;
      }

    case BRANCHIFNOT:
      op = JX86_JE;
      goto branchif0;

    case SWITCH: {
      unsigned i;
      unsigned sizes = *pc++;
      unsigned num_consts = sizes & 0xffff;
      unsigned num_blocks = sizes >> 16;
      caml_jit_uint8_t *jz8;
      caml_jit_uint8_t *leaq;

      /* flush the stack pointer */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }

      /* flush the float box */
      if (state != 0) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }

      leaq = cp;
      jx86_leaq_reg_forward(cp, JX86_RDX);
      if (num_consts) {
        jx86_testb_reg_imm(cp, JX86_AL, 1);
        jz8 = cp;
        jx86_jz8_forward(cp);
        jx86_jmpq_memindex(cp, JX86_RDX, -4, JX86_RAX, 2);
        jx86_jcc8_patch(cp, jz8);
      }
      if (num_blocks) {
        jx86_xorl_reg_reg(cp, JX86_ECX, JX86_ECX);
        jx86_movb_reg_membase(cp, JX86_CL, JX86_RAX, -8);
        jx86_jmpq_memindex(cp, JX86_RDX, num_consts * 8, JX86_RCX, 3);
      }
      /* stuff an UD2 instruction after the indirect branch to stop
       * the processor from decoding down the fall-through path.
       */
      jx86_ud2(cp);
      jx86_leaq_reg_patch(cp, leaq);
      for (i = 0; i < num_consts + num_blocks; ++i) {
        code_t dpc = pc + pc[i];

        /* check if target is known */
        if (*dpc < 0) {
          jx86_emit_uint64(cp, caml_jit_code_end + *dpc);
        }
        else {
          caml_jit_intptr_t offset = cp - caml_jit_code_base;

          caml_jit_assert(*dpc >= 0);
          caml_jit_assert(offset > BREAK);
          caml_jit_assert(offset <= CAML_JIT_INT32_MAX);

          /* add address if not already pending */
          if (*dpc <= BREAK)
            caml_jit_pending_add(dpc);
          caml_jit_assert(caml_jit_pending_contains(dpc));
          jx86_emit_uint8(cp, 0);
          jx86_emit_int32(cp, *dpc);
          *dpc = offset;
          cp += 3;
        }
      }
      goto stop_generation;
    }

    case BOOLNOT:
      caml_jit_assert(state == 0);
      jx86_xorb_reg_imm(cp, JX86_AL, 2);
      break;

/* Exceptions */

    case PUSHTRAP:
      jx86_movq_reg_imm(cp, JX86_RDX, pc + *pc);
      jx86_movq_reg_imm(cp, JX86_RDI, &caml_trapsp);
      sp -= 4 * 8;
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_RDI, 0);
      jx86_movq_membase_reg(cp, JX86_R14, 0 * 8, JX86_RDX); /* trap address */
      jx86_movq_membase_reg(cp, JX86_R14, 1 * 8, JX86_RCX); /* previous trap stack pointer */
      jx86_movq_membase_reg(cp, JX86_R14, 2 * 8, JX86_R12); /* env */
      jx86_movq_membase_reg(cp, JX86_R14, 3 * 8, JX86_R13); /* extra args */
      jx86_movq_membase_reg(cp, JX86_RDI, 0, JX86_R14);
      pc++;
      break;

    case CHECK_SIGNALS:
    case POPTRAP: {
      caml_jit_uint8_t *jnz8;

      /* CHECK_SIGNALS is likely a branch target for loops,
       * so ensure to assign a native address to avoid
       * duplicate compilation.
       */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      if (state != 0) {
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }
      *((caml_jit_int32_t *) pc - 1) = (caml_jit_int32_t) (cp - caml_jit_code_end);
      caml_jit_assert(state == 0);
      caml_jit_assert(pc[-1] < 0);

      jx86_movq_reg_imm(cp, JX86_RDX, &caml_something_to_do);
      jx86_testl_membase_imm(cp, JX86_RDX, 0, -1);
      jnz8 = cp;
      jx86_jz8_forward(cp);
      jx86_movq_reg_imm(cp, JX86_RDI, pc - 1);
      jx86_jmp(cp, &caml_jit_rt_process_signal);
      jx86_jcc8_patch(cp, jnz8);
      if (instr == POPTRAP) {
        caml_jit_assert(sp == 0);
        jx86_movq_reg_imm(cp, JX86_RDI, &caml_trapsp);
        jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, 1 * 8);
        sp = 4 * 8;
        jx86_movq_membase_reg(cp, JX86_RDI, 0, JX86_RCX);
      }
      break;
    }

    case RAISE:
      /* ensure that RAISE instructions remain
       * untouched (necessary for backtrace)
       */
      *(pc - 1) = RAISE;

      /* load %rsi with pc */
      jx86_movq_reg_imm(cp, JX86_RSI, pc);

      /* throw the exception in %rax back to C code */
      addr = &caml_jit_rt_raise;
      goto flush_jmp_stop_generation;

/* Calling C functions */

    case C_CALLN:
      instr = (C_CALL1 - 1) + *pc++;
      addr = Primitive(*pc++);
      goto c_call;
    case C_CALL1:
      addr = Primitive(*pc++);
      if (addr == &caml_asin_float) {
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_call(cp, &asin);
        break;
      }
      else if (addr == &caml_cos_float) {
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_call(cp, &cos);
        break;
      }
      else if (addr == &caml_sin_float) {
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_call(cp, &sin);
        break;
      }
      else if (addr == &caml_sqrt_float) {
        if (state == 0) {
          jx86_sqrtsd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        else {
          jx86_sqrtsd_xmm_xmm(cp, JX86_XMM0, JX86_XMM0);
        }
        break;
      }
      else {
        goto c_call;
      }
    case C_CALL2:
      addr = Primitive(*pc++);
      if (addr == &caml_add_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_addsd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        break;
      }
      else if (addr == &caml_sub_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_subsd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        break;
      }
      else if (addr == &caml_mul_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_mulsd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        break;
      }
      else if (addr == &caml_div_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_divsd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        break;
      }
      else if (addr == &caml_atan2_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        if (state == 0) {
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
          state = 1;
        }
        jx86_movlpd_xmm_membase(cp, JX86_XMM1, JX86_RSI, 0);
        jx86_call(cp, &atan2);
        break;
      }
      else if (addr == &caml_eq_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
        if (state == 0)
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
        state = 0;
        jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        jx86_sete_reg(cp, JX86_AL);
        jx86_setnp_reg(cp, JX86_DL);
        jx86_andb_reg_reg(cp, JX86_AL, JX86_DL);
        op = JX86_SETNZ;
        goto cmpint2;
      }
      else if (addr == &caml_neq_float) {
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
        if (state == 0)
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
        state = 0;
        jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        jx86_setne_reg(cp, JX86_AL);
        jx86_setp_reg(cp, JX86_DL);
        jx86_orb_reg_reg(cp, JX86_AL, JX86_DL);
        op = JX86_SETNZ;
        goto cmpint2;
      }
      else if (addr == &caml_le_float) {
        op = JX86_SETAE;
      cmpfloat_rev:
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
        jx86_movlpd_xmm_membase(cp, JX86_XMM1, JX86_RSI, 0);
        if (state == 0)
          jx86_ucomisd_xmm_membase(cp, JX86_XMM1, JX86_RAX, 0);
        else
          jx86_ucomisd_xmm_xmm(cp, JX86_XMM1, JX86_XMM0);
        goto cmpint1;
      }
      else if (addr == &caml_lt_float) {
        op = JX86_SETA;
        goto cmpfloat_rev;
      }
      else if (addr == &caml_ge_float) {
        op = JX86_SETAE;
      cmpfloat:
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
        if (state == 0)
          jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RAX, 0);
        jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        goto cmpint1;
      }
      else if (addr == &caml_gt_float) {
        op = JX86_SETA;
        goto cmpfloat;
      }
      else if (addr == &caml_array_unsafe_get_float) {
        caml_jit_assert(state == 0);
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        jx86_leaq_reg_memindex(cp, JX86_RSI, JX86_RAX, -4, JX86_RSI, 2);
        jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        state = 1;
        break;
      }
      else {
        goto c_call;
      }
    case C_CALL3:
      addr = Primitive(*pc++);
      if (addr == &caml_array_unsafe_set_float) {
        caml_jit_assert(state == 0);
        jx86_movq_reg_membase(cp, JX86_RDI, JX86_R14, sp); sp += 8;
        jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8;
        jx86_leaq_reg_memindex(cp, JX86_RDI, JX86_RAX, -4, JX86_RDI, 2);
        jx86_movlpd_xmm_membase(cp, JX86_XMM0, JX86_RSI, 0);
        jx86_movlpd_membase_xmm(cp, JX86_RDI, 0, JX86_XMM0);
        goto unit;
      }
      else {
        goto c_call;
      }
    case C_CALL4:
    case C_CALL5:
      addr = Primitive(*pc++);
    c_call: {
      int num_args = instr - (C_CALL1 - 1);

      if (state != 0) {
        if (sp != 0) {
          jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
          sp = 0;
        }
        jx86_call(cp, &caml_jit_rt_copy_double);
        state = 0;
      }

      /* load %rbp with the address of caml_young_ptr */
      jx86_movq_reg_imm(cp, JX86_RBP, &caml_young_ptr);
      
      /* load %rbx with the address of caml_extern_sp */
      jx86_movq_reg_imm(cp, JX86_RBX, &caml_extern_sp);

      /* call semantics differ if num_args <= 5 */
      if (num_args <= 5) {
        /* setup up to five arguments */
        jx86_movq_reg_reg(cp, JX86_RDI, JX86_RAX);
        if (num_args >= 2) { jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp); sp += 8; }
        if (num_args >= 3) { jx86_movq_reg_membase(cp, JX86_RDX, JX86_R14, sp); sp += 8; }
        if (num_args >= 4) { jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, sp); sp += 8; }
        if (num_args == 5) { jx86_movq_reg_membase(cp, JX86_R8,  JX86_R14, sp); sp += 8; }

        /* reserve stack space for the environment pointer */
        sp -= 8;
      }
      else {
        /* arguments are stack pointer and num_args */
        sp -= 2 * 8;
        jx86_movl_reg_imm(cp, JX86_ESI, num_args);
        jx86_movq_membase_reg(cp, JX86_R14, sp + 1 * 8, JX86_RAX);
        jx86_leaq_reg_membase(cp, JX86_RDI, JX86_R14, sp + 1 * 8);
      }

      /* flush the stack pointer */
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }

      /* save environment pointer */
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_R12);

      /* save young ptr to caml_young_ptr */
      jx86_movq_membase_reg(cp, JX86_RBP, 0, JX86_R15);

      /* save stack pointer to caml_extern_sp */
      jx86_movq_membase_reg(cp, JX86_RBX, 0, JX86_R14);

      /* save the pc to the reserved saved_pc area 0*8(%rsp) */
      jx86_movq_reg_imm(cp, JX86_R11, pc + 1);
      jx86_movq_membase_reg(cp, JX86_RSP, 0 * 8, JX86_R11);

      /* call the primitive C function */
      jx86_call(cp, addr);

      /* reset saved_pc area 0*8(%rsp) */
      jx86_movq_membase_imm(cp, JX86_RSP, 0 * 8, 0);

      /* restore local state */
      jx86_movq_reg_membase(cp, JX86_R15, JX86_RBP, 0);
      jx86_movq_reg_membase(cp, JX86_R14, JX86_RBX, 0);
      jx86_movq_reg_membase(cp, JX86_R12, JX86_R14, 0);

      /* pop arguments/environment pointer off the stack */
      if (num_args > 5)
        sp += (num_args + 1) * 8;
      else
        sp += 1 * 8;
      break;
    }

/* Integer constants */

    case PUSHCONSTINT:
      if (state != 0)
        goto state1_to_state0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case CONSTINT:
      jx86_movq_reg_imm(cp, JX86_RAX, Val_int(*pc++));
      state = 0;
      break;

    case PUSHCONST0:
    case PUSHCONST1:
    case PUSHCONST2:
    case PUSHCONST3:
      if (state != 0)
        goto state1_to_state0;
      instr = (instr - PUSHCONST0) + CONST0;
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      /* FALL-THROUGH */
    case CONST0:
    case CONST1:
    case CONST2:
    case CONST3:
      jx86_movq_reg_imm(cp, JX86_RAX, Val_int(instr - CONST0));
      state = 0;
      break;

/* Integer arithmetic */

    case NEGINT:
      caml_jit_assert(state == 0);
      jx86_negq_reg(cp, JX86_RAX);
      jx86_addq_reg_imm(cp, JX86_RAX, 2);
      break;

    case ADDINT:
      caml_jit_assert(state == 0);
      jx86_addq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
      jx86_subq_reg_imm(cp, JX86_RAX, 1);
    pop1:
      sp += 8;
      break;

    case SUBINT:
      caml_jit_assert(state == 0);
      jx86_subq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
    or1_pop1:
      jx86_orb_reg_imm(cp, JX86_AL, 1);
      goto pop1;

    case MULINT:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, sp);
      jx86_sarq_reg_imm(cp, JX86_RAX, 1);
      jx86_sarq_reg_imm(cp, JX86_RCX, 1);
      jx86_imulq_reg_reg(cp, JX86_RAX, JX86_RCX);
      goto shl1_or1_pop1;

    case DIVINT:
    case MODINT:
      caml_jit_assert(state == 0);
      /* load %r11 with the effective pc (for the exception case) */
      jx86_movq_reg_imm(cp, JX86_R11, pc + 2);
      /* load the operands */
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, sp);
      sp += 8;
      jx86_sarq_reg_imm(cp, JX86_RAX, 1);
      jx86_sarq_reg_imm(cp, JX86_RCX, 1);
      /* %rdx:%rax <- sign-extend of %rax */
      jx86_cqo(cp);
      /* flush the stack pointer */  
      if (sp != 0) {
        jx86_leaq_reg_membase(cp, JX86_R14, JX86_R14, sp);
        sp = 0;
      }
      /* check for "zero divide" */
      jx86_jz(cp, caml_jit_code_raise_zero_divide);
      jx86_idivq_reg(cp, JX86_RCX);
      if (instr == MODINT)
        jx86_movq_reg_reg(cp, JX86_RAX, JX86_RDX);
      jx86_shlq_reg_imm(cp, JX86_RAX, 1);
      jx86_orq_reg_imm(cp, JX86_RAX, 1);
      break;

    case ANDINT:
      caml_jit_assert(state == 0);
      jx86_andq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
      goto pop1;

    case ORINT:
      caml_jit_assert(state == 0);
      jx86_orq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
      goto pop1;

    case XORINT:
      caml_jit_assert(state == 0);
      jx86_xorq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
      goto or1_pop1;

    case LSLINT:
      op = JX86_SHL;
    shiftint:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_R14, sp);
      jx86_subq_reg_imm(cp, JX86_RAX, 1);
      jx86_shrb_reg_imm(cp, JX86_CL, 1);
      jx86_shfq_reg_reg(cp, op, JX86_RAX, JX86_CL);
      goto or1_pop1;
    case LSRINT:
      op = JX86_SHR;
      goto shiftint;
    case ASRINT:
      op = JX86_SAR;
      goto shiftint;

    case EQ:
      if (state != 0)
        goto state1_to_state0;
      op = JX86_SETE;
    cmpint:
      caml_jit_assert(state == 0);
      jx86_cmpq_reg_membase(cp, JX86_RAX, JX86_R14, sp);
    cmpint1:
      jx86_setcc_reg(cp, op, JX86_AL);
      state = 0;
    cmpint2:
      caml_jit_assert(state == 0);
      jx86_movzxlb_reg_reg(cp, JX86_EAX, JX86_AL);
      if (*pc == BRANCHIF || *pc == BRANCHIFNOT) {
        /* leal 1(, %eax, 2), %eax */
        jx86_emit_uint8((cp), 0x8d);
        jx86_emit_address_byte((cp), 0, JX86_EAX, 4);
        jx86_emit_address_byte((cp), 1, JX86_EAX, 5);
        jx86_emit_int32((cp), 1);
        op = (op + JX86_JA) - JX86_SETA;
        if (*pc++ == BRANCHIFNOT)
          op = op ^ 1;
        sp += 8;
        goto branchif1;
      }
      else
        goto shl1_or1_pop1;
    case NEQ:
      if (state != 0)
        goto state1_to_state0;
      op = JX86_SETNE;
      goto cmpint;
    case LTINT:
      op = JX86_SETL;
      goto cmpint;
    case LEINT:
      op = JX86_SETLE;
      goto cmpint;
    case GTINT:
      op = JX86_SETG;
      goto cmpint;
    case GEINT:
      op = JX86_SETGE;
      goto cmpint;
    case ULTINT:
      op = JX86_SETB;
      goto cmpint;
    case UGEINT:
      op = JX86_SETAE;
      goto cmpint;

    case BEQ:
      if (state != 0)
        goto state1_to_state0;
      op = JX86_JE;
    branchint:
      jx86_cmpq_reg_imm(cp, JX86_RAX, Val_int(*pc++));
      goto branchif1;
    case BNEQ:
      if (state != 0)
        goto state1_to_state0;
      op = JX86_JNE;
      goto branchint;
    case BLTINT:
      op = JX86_JG;
      goto branchint;
    case BLEINT:
      op = JX86_JGE;
      goto branchint;
    case BGTINT:
      op = JX86_JL;
      goto branchint;
    case BGEINT:
      op = JX86_JLE;
      goto branchint;
    case BULTINT:
      op = JX86_JA;
      goto branchint;
    case BUGEINT:
      op = JX86_JBE;
      goto branchint;

    case OFFSETINT:
      caml_jit_assert(state == 0);
      jx86_addq_reg_imm(cp, JX86_RAX, *pc++ << 1);
      break;

    case OFFSETREF:
      caml_jit_assert(state == 0);
      jx86_addq_membase_imm(cp, JX86_RAX, 0, *pc++ << 1);
      goto unit;

    case ISINT:
      if (state == 0) {
        jx86_andl_reg_imm(cp, JX86_EAX, 1);
        jx86_shll_reg_imm(cp, JX86_EAX, 1);
        jx86_orb_reg_imm(cp, JX86_AL, 1);
      }
      else {
        jx86_movl_reg_imm(cp, JX86_EAX, Val_false);
        state = 0;
      }
      break;

/* Object-oriented operations */

    case GETMETHOD:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_R14, sp);
      jx86_movq_reg_membase(cp, JX86_RSI, JX86_RSI, 0);
      jx86_movq_reg_memindex(cp, JX86_RAX, JX86_RSI, -4, JX86_RAX, 2);
      break;

    case GETPUBMET:
      caml_jit_assert(state == 0);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_RAX, 0);
      sp -= 8;
      jx86_movq_membase_reg(cp, JX86_R14, sp, JX86_RAX);
      jx86_movq_reg_imm(cp, JX86_RSI, Val_int(*pc++));
      jx86_movq_reg_imm(cp, JX86_RDX, (caml_jit_intptr_t) pc);
      pc++;
      addr = &caml_cache_public_method2;
      goto call_addr;

    case GETDYNMET:
      caml_jit_assert(state == 0);
      jx86_movq_reg_reg(cp, JX86_RSI, JX86_RAX);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_R14, sp);
      addr = &caml_get_public_method;
      goto call_addr;

/* Debugging and machine control */
    case STOP:
      addr = &caml_jit_rt_stop;
      goto flush_jmp_stop_generation;

    case EVENT:
    case BREAK:
      break;

    default:
#if _MSC_VER >= 1200      
      __assum(0);
#else
      caml_fatal_error_arg("Fatal error: bad opcode (%"
                           ARCH_INTNAT_PRINTF_FORMAT "x)\n",
                           (char *) (caml_jit_intptr_t) instr);
#endif
    }
  }

  addr = caml_jit_code_ptr;
  caml_jit_code_ptr = cp;

  caml_jit_assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);
  caml_jit_assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  caml_jit_assert(caml_jit_pending_capacity >= 0);
  caml_jit_assert(caml_jit_pending_size == 0);
  caml_jit_assert(state == 0);
  caml_jit_assert(sp == 0);

  return (caml_jit_uint8_t *) addr;
}


void caml_prepare_bytecode(code_t prog, asize_t prog_size)
{
  caml_jit_block_t  *block;
  caml_jit_block_t **blockp;
  code_t             pend;

  caml_jit_assert(prog != NULL);
  caml_jit_assert(prog_size > 0);
  caml_jit_assert((prog_size % sizeof(*prog)) == 0);

  if (CAML_JIT_GNUC_LIKELY (caml_jit_enabled)) {
    pend = prog + (prog_size / sizeof(*prog));
    
    /* register the code block (if not already registered) */
    for (blockp = &caml_jit_block_head;;) {
      block = *blockp;
      if (block == NULL) {
        /* we have a new code block here */
        block = (caml_jit_block_t *) malloc(sizeof(*block));
        block->block_prog = prog;
        block->block_pend = pend;
        block->block_next = NULL;
        *blockp = block;
        break;
      }
      else if (block->block_prog == prog && block->block_pend == pend) {
        /* we know this block already */
        break;
      }
      caml_jit_assert(prog >= block->block_pend
                      || pend < block->block_prog);
      blockp = &block->block_next;
    }
  }
}


void caml_release_bytecode(code_t prog, asize_t prog_size)
{
  caml_jit_block_t  *block;
  caml_jit_block_t **blockp;
  code_t             pend;

  caml_jit_assert(prog != NULL);
  caml_jit_assert(prog_size > 0);
  caml_jit_assert((prog_size % sizeof(*prog)) == 0);

  if (CAML_JIT_GNUC_LIKELY (caml_jit_enabled)) {
    pend = prog + (prog_size / sizeof(*prog));

    /* unregister the code block */
    for (blockp = &caml_jit_block_head;;) {
      block = *blockp;
      caml_jit_assert(block != NULL);
      if (block->block_prog == prog && block->block_pend == pend) {
        /* drop the block from the list */
        *blockp = block->block_next;
        free(block);
        break;
      }
      blockp = &block->block_next;
    }
  }
}

