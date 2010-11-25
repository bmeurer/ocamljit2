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
#include <assert.h>
#include <errno.h>
#include <limits.h>
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
extern value caml_abs_float(value);
extern value caml_neg_float(value);
extern value caml_sqrt_float(value);
extern value caml_cos_float(value);
extern value caml_sin_float(value);
extern value caml_asin_float(value);
extern value caml_float_of_int(value);
extern value caml_int_of_float(value);
extern value caml_add_float(value, value);
extern value caml_sub_float(value, value);
extern value caml_mul_float(value, value);
extern value caml_div_float(value, value);
extern value caml_eq_float(value, value);
extern value caml_neq_float(value, value);
extern value caml_le_float(value, value);
extern value caml_lt_float(value, value);
extern value caml_ge_float(value, value);
extern value caml_gt_float(value, value);
extern value caml_atan2_float(value, value);
extern value caml_fmod_float(value, value);

/* from obj.c */
extern value caml_cache_public_method2(value *, value, opcode_t *);

/* from str.c */
extern value caml_ml_string_length(value);
extern value caml_string_equal(value, value);
extern value caml_string_notequal(value, value);
extern value caml_string_compare(value, value);
extern value caml_string_lessthan(value, value);
extern value caml_string_lessequal(value, value);
extern value caml_string_greaterthan(value, value);
extern value caml_string_greaterequal(value, value);
extern value caml_blit_string(value, value, value, value, value);
extern value caml_fill_string(value, value, value, value);

/* forward declarations */
static void *caml_jit_compile(code_t pc);

/* native floating point primitive names (emulation on i386 to cope with %xmm registers) */
#if defined(TARGET_amd64)
# define CAML_JIT_FPPRIM(name) name
#else
# define CAML_JIT_FPPRIM(name) caml_jit_rt_##name##_float
#endif

/* native register assignment */
#if defined(TARGET_amd64)
# define CAML_JIT_NEP        JX86_R12 /* env */
# define CAML_JIT_NEA        JX86_R13 /* extra args */
# define CAML_JIT_NSP        JX86_R14 /* sp */
# define CAML_JIT_NYP        JX86_R15 /* young ptr */
# define CAML_JIT_WORD_SHIFT 3
#else
# define CAML_JIT_NEP        JX86_EBP /* env */
# define CAML_JIT_NEA        JX86_EBX /* extra args */
# define CAML_JIT_NSP        JX86_ESI /* sp */
# define CAML_JIT_NYP        JX86_EDI /* young ptr */
# define CAML_JIT_WORD_SHIFT 2
#endif
#define CAML_JIT_WORD_SIZE (1 << CAML_JIT_WORD_SHIFT)

static caml_jit_segment_t *caml_jit_segment_head = NULL;
unsigned char *caml_jit_code_base = NULL;
unsigned char *caml_jit_code_end = NULL;
static unsigned char *caml_jit_code_raise_zero_divide = NULL;
opcode_t caml_jit_callback_return = 0; /* for caml_callbackN_exn */
unsigned caml_jit_enabled = 0;

/* translation of EQ..GEINT opcodes to setcc codes */
static const unsigned char caml_jit_cmp2x86setcc[6] = {
  JX86_SETE,  /* EQ */
  JX86_SETNE, /* NEQ */
  JX86_SETL,  /* LTINT */
  JX86_SETLE, /* LEINT */
  JX86_SETG,  /* GTINT */
  JX86_SETGE  /* GEINT */
};

/* translation of BEQ..BGEINT opcodes to jcc opcodes (reversed operands) */
static const unsigned char caml_jit_bcmp2x86jcc[6] = {
  JX86_JE,  /* BEQ */
  JX86_JNE, /* BNEQ */
  JX86_JG,  /* BLTINT */
  JX86_JGE, /* BLEINT */
  JX86_JL,  /* BGTINT */
  JX86_JLE  /* BGEINT */
};


#ifdef DEBUG
static long caml_bcodcount = 0;

void caml_jit_trace(opcode_t instr, code_t pc, value accu, value extra_args, value env, value *sp)
{
  opcode_t save = *pc;
  caml_jit_segment_t *segment;
  code_t prog;
  asize_t prog_size;

  /* lookup the byte-code segment for the pc */
  for (segment = caml_jit_segment_head;; segment = segment->segment_next) {
    assert(segment != NULL);
    if (segment->segment_prog <= pc && pc < segment->segment_pend)
      break;
  }

  prog = segment->segment_prog;
  prog_size = (segment->segment_pend - prog) * sizeof(*prog);

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


static unsigned char    *caml_jit_chunk_next = NULL;
static unsigned char    *caml_jit_chunk_limit = NULL;
static caml_jit_chunk_t *caml_jit_chunk_free = NULL;

static caml_jit_chunk_t *caml_jit_chunk_alloc() CAML_JIT_GNUC_MALLOC CAML_JIT_GNUC_WARN_UNUSED_RESULT;

static inline void caml_jit_chunk_clear(caml_jit_chunk_t *chunk)
{
#if defined(DEBUG)
  unsigned char *cp;

  assert(chunk != NULL);

  memset(&chunk->chunk_next, 0xaa, sizeof(chunk->chunk_next));
  for (cp = chunk->chunk_data; cp < CAML_JIT_CHUNK_END(chunk); )
    jx86_ud2(cp);

  assert(cp == CAML_JIT_CHUNK_END(chunk));
#else
  (void) chunk;
#endif
}

static caml_jit_chunk_t *caml_jit_chunk_alloc()
{
  caml_jit_chunk_t *chunk = caml_jit_chunk_free;

  if (CAML_JIT_GNUC_UNLIKELY (chunk != NULL)) {
    caml_jit_chunk_free = chunk->chunk_next;
  }
  else if (CAML_JIT_GNUC_LIKELY (caml_jit_chunk_next < caml_jit_chunk_limit)) {
    chunk = (caml_jit_chunk_t *) caml_jit_chunk_next;
    caml_jit_chunk_next += CAML_JIT_CHUNK_SIZE;
  }
  else {
    caml_fatal_error("Fatal error: Native code space exhausted!\n");
  }

  caml_jit_chunk_clear(chunk);

  return chunk;
}


void caml_jit_init()
{
  unsigned char *bp;
  unsigned char *cp;
#ifdef TARGET_i386
  unsigned int eax, edx;
#endif

  /* check if already initialized */
  if (caml_jit_code_base != NULL)
    return;

  /* the JIT engine is incompatible with ocamldebug */
  caml_jit_enabled = (getenv("CAML_DEBUG_SOCKET") == NULL);

  /* we need a SSE2 capable CPU */
#ifdef TARGET_i386
  if (CAML_JIT_GNUC_LIKELY (caml_jit_enabled)) {
    __asm__ __volatile__("pushl %%ebx; cpuid; popl %%ebx"
                         : "=a"(eax), "=d"(edx)
                         : "a"(0x00000001u)
                         : "ecx");
    if (CAML_JIT_GNUC_UNLIKELY ((edx & 0x04000000) == 0))
      caml_jit_enabled = 0;
  }
#endif

  /* check if JIT is enabled */
  if (CAML_JIT_GNUC_UNLIKELY (!caml_jit_enabled))
    return;

  /* allocate memory for the JIT code */
  bp = (unsigned char *) mmap(NULL, CAML_JIT_CODE_SIZE,
                              PROT_EXEC | PROT_READ | PROT_WRITE,
                              MAP_ANON | MAP_PRIVATE, 0, (off_t) 0);
  if (bp == (unsigned char *) MAP_FAILED)
    caml_fatal_error_arg("Fatal error: Failed to allocate JIT code area (%s)!\n", strerror(errno));
  cp = bp;

  /* divide code space into chunks (last one reserved for special stuff) */
  caml_jit_chunk_next = bp;
  caml_jit_chunk_limit = bp + (CAML_JIT_CODE_SIZE - CAML_JIT_CHUNK_SIZE);

#ifdef TARGET_amd64
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
   * Assumes byte-code address to compile is in %rdi and
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
  if (JX86_IS_IMM32((unsigned char *) &caml_jit_compile - caml_jit_code_end - 2 - 5)) {
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
#else
  /* Generate the compile trampoline code at the end:
   *
   *   pushl %eax
   *   pushl %ecx
   *   call  caml_jit_compile
   *   xchgl %eax, %edx
   *   popl  %ecx
   *   popl  %eax
   *   jmpl *%edx
   *   ud2
   *
   * Assumes byte-code address to compile is in %ecx.
   * Preserves %eax since the compile trampoline may also
   * be used to return to not yet compiled code, where
   * %eax contains the return value.
   *
   * The UD2 instruction following the indirect jump is
   * used to stop the processor from decoding down the
   * fall-through path.
   */
  caml_jit_code_end = (cp += CAML_JIT_CODE_SIZE - (1 + 1 + 5 + 1 + 1 + 1 + 2 + 2));
  jx86_pushl_reg(cp, JX86_EAX);
  jx86_pushl_reg(cp, JX86_ECX);
  jx86_call(cp, &caml_jit_compile);
  jx86_xchgl_reg_reg(cp, JX86_EDX, JX86_EAX);
  jx86_popl_reg(cp, JX86_ECX);
  jx86_popl_reg(cp, JX86_EAX);
  jx86_jmpl_reg(cp, JX86_EDX);
  jx86_ud2(cp);
#endif
  assert(cp == bp + CAML_JIT_CODE_SIZE);

  /* Generate the NOPs in front of the compile trampoline.
   * Each byte code instruction gets one NOP, so that in the
   * end, all not yet compiled instructions will redirect
   * to the compile trampoline.
   */
  cp = caml_jit_code_end;
  caml_jit_code_end -= STOP;
  while (cp > caml_jit_code_end)
    *--cp = 0x90;
  assert(cp == caml_jit_code_end);

  cp = caml_jit_chunk_limit;

  /* Setup the "raise zero divide" code (used by DIVINT/MODINT) */
  caml_jit_code_raise_zero_divide = cp;
  jx86_subn_reg_imm(cp, CAML_JIT_NSP, 1 * CAML_JIT_WORD_SIZE);                   /* *--sp = env */
  jx86_movn_membase_reg(cp, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE, CAML_JIT_NEP);
#ifdef TARGET_amd64
  jx86_movq_membase_reg(cp, JX86_RSP, 0 * CAML_JIT_WORD_SIZE, JX86_R11);         /* saved_pc = pc */
  jx86_movq_reg_imm(cp, JX86_R8, &caml_extern_sp);                               /* caml_extern_sp = sp */
  jx86_movq_membase_reg(cp, JX86_R8, 0, CAML_JIT_NSP);
  jx86_movq_reg_imm(cp, JX86_R9, &caml_young_ptr);                               /* caml_young_ptr = yp */
  jx86_movq_membase_reg(cp, JX86_R9, 0, CAML_JIT_NYP);
#else
  jx86_movl_mem_reg(cp, &caml_extern_sp, CAML_JIT_NSP);                          /* caml_extern_sp = sp */
  jx86_movl_mem_reg(cp, &caml_young_ptr, CAML_JIT_NYP);                          /* caml_young_ptr = yp */
#endif
  jx86_call(cp, &caml_raise_zero_divide);

  /* setup the callback return code */
  caml_jit_callback_return = cp - caml_jit_code_end;
  jx86_jmp(cp, &caml_jit_rt_stop);

  caml_jit_code_base = bp - (STOP + 1);
}


static code_t *caml_jit_pending_buffer = NULL;
static int caml_jit_pending_capacity = 0;
static int caml_jit_pending_size = 0;

static inline void caml_jit_pending_add(code_t pc)
{
  assert(pc != NULL);
  assert(caml_jit_pending_size >= 0);
  assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

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

  assert(pc != NULL);
  assert(caml_jit_pending_size >= 0);
  assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

  for (i = 0; i < caml_jit_pending_size; ++i)
    if (caml_jit_pending_buffer[i] == pc)
      return 1;
  return 0;
}


static void caml_jit_segment_alloc_chunk(caml_jit_segment_t *segment)
{
  caml_jit_chunk_t *chunk;

  assert(segment != NULL);
  assert(((size_t) segment->segment_current % sizeof(void *)) == 0);

  chunk = caml_jit_chunk_alloc();
  chunk->chunk_next = segment->segment_chunks;
  segment->segment_chunks = chunk;
  segment->segment_current = chunk->chunk_data;
  segment->segment_limit = CAML_JIT_CHUNK_END(chunk) - CAML_JIT_CODE_RESERVE;

  assert(((size_t) segment->segment_current % sizeof(void *)) == 0);
}


static unsigned char *caml_jit_segment_continue(caml_jit_segment_t *segment, unsigned char *cp)
{
  assert(segment != NULL);
  assert(cp >= segment->segment_current);
  assert(segment->segment_chunks != NULL);
  assert(cp < CAML_JIT_CHUNK_END(segment->segment_chunks) - 5);
  assert(((size_t) segment->segment_current % sizeof(void *)) == 0);

  caml_jit_segment_alloc_chunk(segment);
  jx86_jmp(cp, segment->segment_current);

  assert(((size_t) segment->segment_current % sizeof(void *)) == 0);

  return segment->segment_current;
}


/* check if a C primitive allocates/raises */
static inline int caml_jit_noalloc(const void *prim)
{
  return (prim == &caml_ml_string_length
          || prim == &caml_string_equal
          || prim == &caml_string_notequal
          || prim == &caml_string_compare
          || prim == &caml_string_lessthan
          || prim == &caml_string_lessequal
          || prim == &caml_string_greaterthan
          || prim == &caml_string_greaterequal
          || prim == &caml_blit_string
          || prim == &caml_fill_string);
}


static void *caml_jit_compile(code_t pc)
{
  unsigned char *start;
  void *addr;
  register unsigned char *cp;
  opcode_t instr;
  unsigned op;
  int state = 0; /* used for the float optimization loop */
  register int sp = 0;
  caml_jit_segment_t *segment;

  assert(pc != NULL);
  assert(caml_jit_enabled);
  assert(*pc >= 0 && *pc <= STOP);
  assert(caml_jit_pending_size >= 0);
  assert(caml_jit_code_base != NULL);
  assert(caml_jit_pending_capacity >= caml_jit_pending_size);
  assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);

  /* lookup the byte-code segment for the pc */
  for (segment = caml_jit_segment_head;; segment = segment->segment_next) {
    assert(segment != NULL);
    if (segment->segment_prog <= pc && pc < segment->segment_pend)
      break;
  }

  /* make sure that reasonable space is available for this segment */
  if (CAML_JIT_GNUC_UNLIKELY (segment->segment_current >= segment->segment_limit))
    caml_jit_segment_alloc_chunk(segment);

  assert(segment != NULL);
  assert(segment->segment_current != NULL);
  assert(((size_t) segment->segment_current % sizeof(void *)) == 0);

  /* setup the native start address */
  cp = start = segment->segment_current;
  instr = *pc;
  *pc = (opcode_t) (cp - caml_jit_code_end);
  goto next;

  for (;; ) {
    assert((sp % CAML_JIT_WORD_SIZE) == 0);
    assert(cp >= segment->segment_current);
    assert(cp < CAML_JIT_CHUNK_END(segment->segment_chunks));
    assert(pc < segment->segment_pend);
    assert(pc >= segment->segment_prog);

    /* determine the next instruction */
    instr = *pc;

  begin:
    /* check if this instruction is already compiled */
    if (CAML_JIT_GNUC_UNLIKELY (instr < 0)) {
      /* determine the native code address */
      addr = caml_jit_code_end + instr;

    flush_jmp_stop_generation:
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }

    jmp_stop_generation:
      /* generate a jmp to the known native code */
      assert(sp == 0);
      jx86_jmp(cp, addr);

    stop_generation:
      assert(sp == 0);
      if (caml_jit_pending_size > 0) {
        pc = caml_jit_pending_buffer[--caml_jit_pending_size];
        /* check if this pending item is already compiled */
        if (CAML_JIT_GNUC_UNLIKELY (*pc < 0))
          goto stop_generation;

        /* allocate a new chunk as necessary */
        if (CAML_JIT_GNUC_UNLIKELY (cp >= segment->segment_limit)) {
          caml_jit_segment_alloc_chunk(segment);
          cp = segment->segment_current;
        }
        continue;
      }
      break;
    }

    /* patch forward jumps to this byte-code address */
    if (CAML_JIT_GNUC_UNLIKELY (instr > STOP)) {
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }

      /* patch the forward jumps */
      do {
        unsigned char *jcp = caml_jit_code_base + instr;

        assert(jcp > caml_jit_code_base);
        assert(jcp + 4 < caml_jit_code_end);
        assert(jcp[0] == 0
               || (jcp[0] == 0x0f && jcp[1] >= 0x80 && jcp[1] <= 0x8f));

        if (CAML_JIT_GNUC_UNLIKELY (jcp[0] == 0)) {
          instr = *((opcode_t *) (jcp + 1));
          *((unsigned char **) jcp) = cp;
        }
        else {
          instr = *((opcode_t *) (jcp + 2));
          jx86_jcc32_patch(cp, jcp);
        }
      } while (CAML_JIT_GNUC_UNLIKELY (instr > STOP));

      /* setup the native code offset for this instruction */
      *pc = (opcode_t) (cp - caml_jit_code_end);
    }

    /* make sure we have reasonable space available */
    if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
      cp = caml_jit_segment_continue(segment, cp);

  next:
    assert(instr >= 0);
    assert(instr <= STOP);
    assert((sp % CAML_JIT_WORD_SIZE) == 0);
    assert(cp >= segment->segment_current);
    assert(cp < CAML_JIT_CHUNK_END(segment->segment_chunks));

    /* instruction tracing */
#ifdef DEBUG
    if (caml_trace_flag) {
      if (sp != 0)
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);

#ifdef TARGET_amd64
      jx86_movl_reg_imm(cp, JX86_EDI, instr);
      jx86_movq_reg_imm(cp, JX86_RSI, pc);
#else
      jx86_push_imm(cp, instr);
      jx86_push_imm(cp, pc);
#endif
      jx86_call(cp, caml_jit_rt_trace);

      if (sp != 0) 
        jx86_subn_reg_imm(cp, CAML_JIT_NSP, sp);

      /* make sure we have reasonable space available */
      if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
        cp = caml_jit_segment_continue(segment, cp);
    }
#endif

    ++pc;

    switch (instr) {

/* Basic stack operations */

    case PUSHACC:
      instr = PUSHACC0 + *pc++;
      /* FALL-THROUGH */
    case PUSHACC1:
    case PUSHACC2:
    case PUSHACC3:
    case PUSHACC4:
    case PUSHACC5:
    case PUSHACC6:
    case PUSHACC7:
      instr = (instr - PUSHACC0) + ACC0;
      sp -= CAML_JIT_WORD_SIZE;
      /* [Peephole Optimization] PUSHACCx followed by EQ..GEINT */
      if (CAML_JIT_GNUC_UNLIKELY (*pc >= EQ && *pc <= GEINT)) {
        jx86_cmpn_membase_reg(cp, CAML_JIT_NSP, sp + (instr - ACC0) * CAML_JIT_WORD_SIZE, JX86_NAX);
        op = caml_jit_cmp2x86setcc[*pc++ - EQ];
        goto cmpint1;
      }
      /* [Peephole Optimization] PUSHACCx followed by GETVECTITEM */
      if (CAML_JIT_GNUC_UNLIKELY (*pc == GETVECTITEM)) {
        jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp + (instr - ACC0) * CAML_JIT_WORD_SIZE);
      getvectitem_po1:
        jx86_movn_reg_memindex(cp, JX86_NAX, JX86_NDX, -(CAML_JIT_WORD_SIZE / 2), JX86_NAX, CAML_JIT_WORD_SHIFT - 1);
        pc++;
        goto pop1;
      }
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
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
      jx86_movn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp + (instr - ACC0) * CAML_JIT_WORD_SIZE);
      break;

    case ACC:
      instr = ACC0 + *pc++;
      goto accX;

    case PUSH:
    case PUSHACC0:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      break;

    case ASSIGN:
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp + *pc++ * CAML_JIT_WORD_SIZE, JX86_NAX);
    unit:
      jx86_movl_reg_imm(cp, JX86_EAX, Val_unit);
      break;

    case POP:
      sp += *pc++ * CAML_JIT_WORD_SIZE;
      break;

/* Access in heap-allocated environment */

    case PUSHENVACC:
      instr = (PUSHENVACC1 - 1) + *pc++;
      /* FALL-THROUGH */
    case PUSHENVACC1:
    case PUSHENVACC2:
    case PUSHENVACC3:
    case PUSHENVACC4:
      instr = (instr - PUSHENVACC1) + ENVACC1;
      sp -= CAML_JIT_WORD_SIZE;
      /* [Peephole Optimization] PUSHENVACCx followed by GETVECTITEM */
      if (CAML_JIT_GNUC_UNLIKELY (*pc == GETVECTITEM)) {
        jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NEP, (instr - (ENVACC1 - 1)) * CAML_JIT_WORD_SIZE);
        goto getvectitem_po1;
      }
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case ENVACC1:
    case ENVACC2:
    case ENVACC3:
    case ENVACC4:
    envaccX:
      jx86_movn_reg_membase(cp, JX86_NAX, CAML_JIT_NEP, (instr - (ENVACC1 - 1)) * CAML_JIT_WORD_SIZE);
      break;

    case ENVACC:
      instr = (ENVACC1 - 1) + *pc++;
      goto envaccX;

/* Function application */

    case PUSH_RETADDR: {
      code_t dpc = pc + *pc;
      /* ensure to translate the return address */
      if (CAML_JIT_GNUC_LIKELY (*dpc >= 0)) {
        if (*dpc <= STOP)
          caml_jit_pending_add(dpc);
        assert(caml_jit_pending_contains(dpc));
      }
      sp -= 3 * CAML_JIT_WORD_SIZE;
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDX, (value) dpc);
      jx86_movq_membase_reg(cp, CAML_JIT_NSP, sp + 0 * CAML_JIT_WORD_SIZE, JX86_RDX);     /* return address */
#else
      jx86_movl_membase_imm(cp, CAML_JIT_NSP, sp + 0 * CAML_JIT_WORD_SIZE, (value) dpc);  /* return address */
#endif
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp + 1 * CAML_JIT_WORD_SIZE, CAML_JIT_NEP); /* env */
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp + 2 * CAML_JIT_WORD_SIZE, CAML_JIT_NEA); /* extra args */
      pc++;
      break;
    }

    case APPLY1:
      addr = &caml_jit_rt_apply1;
    applyX:
      /* ensure to translate the return address */
      if (CAML_JIT_GNUC_LIKELY (*pc >= 0)) {
        if (CAML_JIT_GNUC_LIKELY (*pc <= STOP))
          caml_jit_pending_add(pc);
        assert(caml_jit_pending_contains(pc));
      }
      jx86_movn_reg_imm(cp, JX86_NDX, pc);
      goto flush_jmp_stop_generation;

    case APPLY2:
      addr = &caml_jit_rt_apply2;
      goto applyX;

    case APPLY3:
      addr = &caml_jit_rt_apply3;
      goto applyX;

    case APPLY:
      jx86_movl_reg_imm(cp, CAML_JIT_NEA, Val_int(*pc++ - 1));
      addr = &caml_jit_rt_apply;
      goto flush_jmp_stop_generation;

    case APPTERM:
      instr = (APPTERM1 - 1) + *pc++;
      /* FALL-THROUGH */
    case APPTERM1:
    case APPTERM2:
    case APPTERM3: {
      int i;
      int num_args = instr - (APPTERM1 - 1);
      int newsp;

      /* make sure reasonable space is available */
      if (CAML_JIT_GNUC_UNLIKELY (cp + (2 * num_args) * 15 > segment->segment_limit))
        cp = caml_jit_segment_continue(segment, cp);

      /* slide the num_args bottom words of the current frame to the
       * top of the frame, and discard the remainder of the frame.
       */
      newsp = sp + (*pc++ - num_args) * CAML_JIT_WORD_SIZE;
      for (i = num_args - 1; i >= 0; --i) {
        jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp + i * CAML_JIT_WORD_SIZE);
        jx86_movn_membase_reg(cp, CAML_JIT_NSP, newsp + i * CAML_JIT_WORD_SIZE, JX86_NDX);
      }
      sp = newsp;

      /* calculate the extra args */
      if (num_args > 1)
        jx86_addn_reg_imm(cp, CAML_JIT_NEA, (unsigned) (num_args - 1) << 1);

      /* perform the application (closure in %rax) */
      addr = &caml_jit_rt_apply;
      goto flush_jmp_stop_generation;
    }

    case RETURN:
      sp += *pc++ * CAML_JIT_WORD_SIZE;
      addr = &caml_jit_rt_return;
      goto flush_jmp_stop_generation;

    case RESTART:
      addr = &caml_jit_rt_restart;
    flush_call_addr:
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      jx86_call(cp, addr);
      break;

    case GRAB: {
      unsigned char *jae8;
      int required = *pc++;

      assert(sp == 0);

      /* check if we have too few extra args */
      jx86_cmpn_reg_imm(cp, CAML_JIT_NEA, Val_int(required));
      jae8 = cp;
      jx86_jae8_forward(cp);

      /* generate a restart closure and return to caller */
      jx86_movn_reg_imm(cp, JX86_NAX, pc - 3);
      jx86_jmp(cp, &caml_jit_rt_grab_closure_and_return);

      /* calculate remaining extra arguments */
      jx86_jcc8_patch(cp, jae8);
      jx86_addn_reg_imm(cp, CAML_JIT_NEA, -required << 1);
      break;
    }

    case CLOSURE: {
      int i, num_vars = *pc++;
      const mlsize_t wosize = num_vars + 1;
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      /* allocate space in the minor heap */
      switch (num_vars) {
      case 0:
        addr = &caml_jit_rt_alloc1;
        break;

      case 1:
        addr = &caml_jit_rt_alloc2;
        break;

      case 2:
        addr = &caml_jit_rt_alloc3;
        break;

      default:
        jx86_movn_reg_imm(cp, JX86_NDX, Bhsize_wosize(wosize));
        addr = &caml_jit_rt_allocN;
        break;
      }
      jx86_call(cp, addr);
      /* initialize the closure */
      jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(wosize, Closure_tag, Caml_black));
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_R10, pc + pc[0]);
      jx86_movq_membase_reg(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_R10);
#else
      jx86_movl_membase_imm(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, pc + pc[0]);
#endif
      if (--num_vars >= 0) {
        assert(sp == 0);
        jx86_movn_membase_reg(cp, CAML_JIT_NYP, 2 * CAML_JIT_WORD_SIZE, JX86_NAX);
        if (num_vars <= 5) {
          for (i = 0; i < num_vars; ++i, sp += CAML_JIT_WORD_SIZE) {
            jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp);
            jx86_movn_membase_reg(cp, CAML_JIT_NYP, (i + 3) * CAML_JIT_WORD_SIZE, JX86_NCX);
          }
        }
        else {
          jx86_movn_reg_imm(cp, JX86_NCX, num_vars);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, JX86_RSI, CAML_JIT_NSP);
#else
          jx86_pushl_reg(cp, JX86_EDI);
#endif
          jx86_lean_reg_membase(cp, JX86_NDI, CAML_JIT_NYP, 3 * CAML_JIT_WORD_SIZE);
          jx86_rep_movsn(cp);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, CAML_JIT_NSP, JX86_RSI);
#else
          jx86_popl_reg(cp, JX86_EDI);
#endif
        }
      }
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
      pc++;
      break;
    }

    case CLOSUREREC: {
      int i;
      int num_funs = *pc++;
      int num_vars = *pc++;
      const mlsize_t wosize = num_funs * 2 - 1 + num_vars;
      /* flush stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      /* allocate space in the minor heap */
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
        jx86_movn_reg_imm(cp, JX86_NDX, Bhsize_wosize(wosize));
        addr = &caml_jit_rt_allocN;
        break;
      }
      jx86_call(cp, addr);
      /* initialize the recursive closure */
      jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(wosize, Closure_tag, Caml_black));
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDX, pc + pc[0]);
#endif
      if (--num_vars >= 0) {
        assert(sp == 0);
        jx86_movn_membase_reg(cp, CAML_JIT_NYP, (2 * num_funs) * CAML_JIT_WORD_SIZE, JX86_NAX);
        if (num_vars <= 5) {
          for (i = 0; i < num_vars; ++i, sp += CAML_JIT_WORD_SIZE) {
            jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp);
            jx86_movn_membase_reg(cp, CAML_JIT_NYP, (2 * num_funs + i + 1) * CAML_JIT_WORD_SIZE, JX86_NCX);
          }
        }
        else {
          jx86_movn_reg_imm(cp, JX86_NCX, num_vars);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, JX86_RSI, CAML_JIT_NSP);
#else
          jx86_pushl_reg(cp, JX86_EDI);
#endif
          jx86_lean_reg_membase(cp, JX86_NDI, CAML_JIT_NYP, (2 * num_funs + 1) * CAML_JIT_WORD_SIZE);
          jx86_rep_movsn(cp);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, CAML_JIT_NSP, JX86_RSI);
#else
          jx86_popl_reg(cp, JX86_EDI);
#endif
        }
      }
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
#ifdef TARGET_amd64
      jx86_movq_membase_reg(cp, JX86_RAX, 0, JX86_RDX);
#else
      jx86_movl_membase_imm(cp, JX86_EAX, 0, pc + pc[0]);
#endif
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      for (i = 1; i < num_funs; ++i) {
        if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
          cp = caml_jit_segment_continue(segment, cp);
        jx86_lean_reg_membase(cp, JX86_NDX, CAML_JIT_NYP, (i * 2 + 1) * CAML_JIT_WORD_SIZE);
        jx86_movn_membase_imm(cp, JX86_NDX, -1 * CAML_JIT_WORD_SIZE, Make_header(i * 2, Infix_tag, Caml_white));
#ifdef TARGET_amd64
        jx86_movn_reg_imm(cp, JX86_NCX, pc + pc[i]);
        jx86_movn_membase_reg(cp, JX86_NDX, 0 * CAML_JIT_WORD_SIZE, JX86_NCX);
#else
        jx86_movn_membase_imm(cp, JX86_NDX, 0 * CAML_JIT_WORD_SIZE, pc + pc[i]);
#endif
        sp -= CAML_JIT_WORD_SIZE;
        jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NDX);
      }
      pc += num_funs;
      break;
    }

    case PUSHOFFSETCLOSURE:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE:
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NEP, *pc++ * CAML_JIT_WORD_SIZE);
      break;

    case PUSHOFFSETCLOSUREM2:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case OFFSETCLOSUREM2:
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NEP, -2 * CAML_JIT_WORD_SIZE);
      break;

    case PUSHOFFSETCLOSURE0:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE0:
      jx86_movn_reg_reg(cp, JX86_NAX, CAML_JIT_NEP);
      break;

    case PUSHOFFSETCLOSURE2:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case OFFSETCLOSURE2:
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NEP, 2 * CAML_JIT_WORD_SIZE);
      break;

/* Access to global variables */

    case PUSHGETGLOBAL:
    case PUSHGETGLOBALFIELD:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case GETGLOBAL:
    case GETGLOBALFIELD:
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDX, &caml_global_data);
      jx86_movq_reg_membase(cp, JX86_RDX, JX86_RDX, 0);
#else
      jx86_movl_reg_mem(cp, JX86_EDX, &caml_global_data);
#endif
      jx86_movn_reg_membase(cp, JX86_NAX, JX86_NDX, *pc++ * CAML_JIT_WORD_SIZE);
      if (instr >= GETGLOBALFIELD)
        goto getfield;
      break;

    case SETGLOBAL:
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDI, &caml_global_data);
      jx86_movq_reg_reg(cp, JX86_RSI, JX86_RAX);
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_RDI, 0);
      jx86_addq_reg_imm(cp, JX86_RDI, *pc++ * CAML_JIT_WORD_SIZE);
#else
      jx86_movl_reg_mem(cp, JX86_EDX, &caml_global_data);
      jx86_addl_reg_imm(cp, JX86_EDX, *pc++ * CAML_JIT_WORD_SIZE);
      jx86_pushl_reg(cp, JX86_EAX);
      jx86_pushl_reg(cp, JX86_EDX);
#endif
      goto modify;

/* Allocation of blocks */

    case PUSHATOM:
      instr = ATOM0 + *pc++;
      goto pushatomX;
    case PUSHATOM0:
      instr = ATOM0;
    pushatomX:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case ATOM0:
    atomX:
      jx86_movn_reg_imm(cp, JX86_NAX, Atom(instr - ATOM0));
      break;

    case ATOM:
      instr = ATOM0 + *pc++;
      goto atomX;

    case MAKEBLOCK: {
      const mlsize_t wosize = *pc++;
      const tag_t tag = *pc++;
      if (CAML_JIT_GNUC_LIKELY (wosize <= Max_young_wosize)) {
        mlsize_t i;

        assert(wosize > 1);

        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* allocate space in the minor heap */
        jx86_movn_reg_imm(cp, JX86_NDX, Bhsize_wosize(wosize));
        jx86_call(cp, &caml_jit_rt_allocN);
        /* initialize the block */
        jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(wosize, tag, Caml_black));
        jx86_movn_membase_reg(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_NAX);
        if (wosize <= 6) {
          for (i = 1; i++ < wosize; sp += CAML_JIT_WORD_SIZE) {
            jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp);
            jx86_movn_membase_reg(cp, CAML_JIT_NYP, i * CAML_JIT_WORD_SIZE, JX86_NCX);
          }
        }
        else {
          jx86_movn_reg_imm(cp, JX86_NCX, wosize - 1);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, JX86_RSI, CAML_JIT_NSP);
#else
          jx86_pushl_reg(cp, JX86_EDI);
#endif
          jx86_lean_reg_membase(cp, JX86_NDI, CAML_JIT_NYP, 2 * CAML_JIT_WORD_SIZE);
          jx86_rep_movsn(cp);
#ifdef TARGET_amd64
          jx86_movq_reg_reg(cp, CAML_JIT_NSP, JX86_RSI);
#else
          jx86_popl_reg(cp, JX86_EDI);
#endif
        }
        jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
      }
      else {
        /* push accu onto the stack */
        sp -= CAML_JIT_WORD_SIZE;
        jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
#ifdef TARGET_amd64
        /* allocate space in the major heap */
        jx86_movq_reg_imm(cp, JX86_RDI, wosize);
        jx86_movl_reg_imm(cp, JX86_ESI, tag);
        jx86_call(cp, &caml_alloc_shr);
        /* initialize the block */
        jx86_movq_reg_reg(cp, JX86_RDI, JX86_RAX);
        jx86_leaq_reg_membase(cp, JX86_RSI, CAML_JIT_NSP, sp);
        jx86_movq_reg_imm(cp, JX86_RDX, wosize);
        jx86_call(cp, &caml_initialize_n);
#else
        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* allocate space in the major heap */
        jx86_push_imm(cp, tag);
        jx86_push_imm(cp, wosize);
        jx86_call(cp, &caml_alloc_shr);
        /* initialize the block */
        jx86_movl_membase_imm(cp, JX86_ESP, 1 * CAML_JIT_WORD_SIZE, wosize);
        jx86_movl_membase_reg(cp, JX86_ESP, 0 * CAML_JIT_WORD_SIZE, CAML_JIT_NSP);
        jx86_pushl_reg(cp, JX86_EAX);
        jx86_call(cp, &caml_initialize_n);
        jx86_addn_reg_imm(cp, JX86_ESP, 3 * CAML_JIT_WORD_SIZE);
#endif
        sp += wosize * CAML_JIT_WORD_SIZE;
      }
      break;
    }

    case MAKEFLOATBLOCK: {
      mlsize_t wosize = *pc++ * Double_wosize;
      if (CAML_JIT_GNUC_LIKELY (wosize <= Max_young_wosize)) {
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
        jx86_movn_reg_imm(cp, JX86_NDX, Bhsize_wosize(wosize));
        jx86_call(cp, &caml_jit_rt_allocN);
        jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE,
                              Make_header(wosize, Double_array_tag, Caml_black));
        jx86_movsd_membase_xmm(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_XMM0);
        jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
        wosize -= Double_wosize;
        if (wosize == 2 * Double_wosize) {
          jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NCX, 0);
          jx86_movsd_xmm_membase(cp, JX86_XMM1, JX86_NDX, 0);
          jx86_movsd_membase_xmm(cp, JX86_NAX, 1 * sizeof(double), JX86_XMM0);
          jx86_movsd_membase_xmm(cp, JX86_NAX, 2 * sizeof(double), JX86_XMM1);
        }
        else if (wosize != 0) {
          jx86_movn_reg_imm(cp, JX86_NCX, wosize / Double_wosize);
          jx86_lean_reg_membase(cp, JX86_NDX, JX86_NAX, sizeof(double));
          jx86_call(cp, &caml_jit_rt_copy_floats);
        }
      }
      else {
        /* push accu onto the stack */
        sp -= CAML_JIT_WORD_SIZE;
        jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
#ifdef TARGET_amd64
        /* allocate space in the major heap */
        jx86_movq_reg_imm(cp, JX86_RDI, wosize);
        jx86_movl_reg_imm(cp, JX86_ESI, Double_array_tag);
        jx86_call(cp, &caml_alloc_shr);
#else
        /* allocate space in the major heap */
        jx86_push_imm(cp, Double_array_tag);
        jx86_push_imm(cp, wosize);
        jx86_call(cp, &caml_alloc_shr);
        jx86_addn_reg_imm(cp, JX86_ESP, 2 * CAML_JIT_WORD_SIZE);
#endif
        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* initialize the block */
        jx86_movn_reg_reg(cp, JX86_NDX, JX86_NAX);
        jx86_movn_reg_imm(cp, JX86_NCX, wosize / Double_wosize);
        jx86_call(cp, &caml_jit_rt_copy_floats);
      }
      break;
    }

    case MAKEBLOCK1:
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      /* allocate space in the minor heap */
      jx86_call(cp, &caml_jit_rt_alloc1);
      /* initialize the block */
      jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(1, *pc, Caml_black));
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_NAX);
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
      pc++;
      break;

    case MAKEBLOCK2:
      /* flush the stack pointer */
      if (sp != 0)
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
      /* allocate space in the minor heap */
      jx86_call(cp, &caml_jit_rt_alloc2);
      /* fetch elements */
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE);
      sp = CAML_JIT_WORD_SIZE;
      /* initialize the block */
      jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(2, *pc, Caml_black));
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_NAX);
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 2 * CAML_JIT_WORD_SIZE, JX86_NCX);
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
      pc++;
      break;

    case MAKEBLOCK3:
      /* flush the stack pointer */
      if (sp != 0)
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
      /* allocate space in the minor heap */
      jx86_call(cp, &caml_jit_rt_alloc3);
      /* fetch elements */
#ifdef TARGET_amd64
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE);
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, 1 * CAML_JIT_WORD_SIZE);
#else
      jx86_movsd_xmm_membase(cp, JX86_XMM0, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE);
#endif
      sp = 2 * CAML_JIT_WORD_SIZE;
      /* initialize the block */
      jx86_movn_membase_imm(cp, CAML_JIT_NYP, 0 * CAML_JIT_WORD_SIZE, Make_header(3, *pc, Caml_black));
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE, JX86_NAX);
#ifdef TARGET_amd64
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 2 * CAML_JIT_WORD_SIZE, JX86_NCX);
      jx86_movn_membase_reg(cp, CAML_JIT_NYP, 3 * CAML_JIT_WORD_SIZE, JX86_NDX);
#else
      jx86_movsd_membase_xmm(cp, CAML_JIT_NYP, 2 * CAML_JIT_WORD_SIZE, JX86_XMM0);
#endif
      jx86_lean_reg_membase(cp, JX86_NAX, CAML_JIT_NYP, 1 * CAML_JIT_WORD_SIZE);
      pc++;
      break;

/* Access to components of blocks */

    case GETFIELD:
    getfield:
      instr = GETFIELD0 + *pc++;
      /* FALL-THROUGH */
    case GETFIELD0:
    case GETFIELD1:
    case GETFIELD2:
    case GETFIELD3:
      jx86_movn_reg_membase(cp, JX86_NAX, JX86_NAX, (instr - GETFIELD0) * CAML_JIT_WORD_SIZE);
      break;
    case GETFLOATFIELD:
      jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, *pc++ * sizeof(double));
      goto float_optloop;

    case SETFIELD:
      instr = SETFIELD0 + *pc++;
      /* FALL-THROUGH */
    case SETFIELD0:
    case SETFIELD1:
    case SETFIELD2:
    case SETFIELD3:
#ifdef TARGET_amd64
      jx86_movq_reg_membase(cp, JX86_RSI, CAML_JIT_NSP, sp);
      jx86_leaq_reg_membase(cp, JX86_RDI, JX86_RAX, (instr - SETFIELD0) * CAML_JIT_WORD_SIZE);
#else
      jx86_addl_reg_imm(cp, JX86_EAX, (instr - SETFIELD0) * CAML_JIT_WORD_SIZE);
      jx86_pushl_membase(cp, CAML_JIT_NSP, sp);
      jx86_pushl_reg(cp, JX86_EAX);
#endif
      sp += CAML_JIT_WORD_SIZE;
    modify:
      jx86_call(cp, &caml_modify);
#ifdef TARGET_i386
      jx86_addl_reg_imm(cp, JX86_ESP, 2 * CAML_JIT_WORD_SIZE);
#endif
      goto unit;

    case SETFLOATFIELD:
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
      jx86_movsd_membase_xmm(cp, JX86_NAX, *pc++ * sizeof(double), JX86_XMM0);
      goto unit;

/* Array operations */

    case VECTLENGTH:
      jx86_movn_reg_membase(cp, JX86_NAX, JX86_NAX, -1 * CAML_JIT_WORD_SIZE);
#ifdef TARGET_i386
      do {
        unsigned char *jne8;

        jx86_cmpb_reg_imm(cp, JX86_AL, Double_array_tag);
        jne8 = cp; jx86_jne8_forward(cp);
        jx86_shrl_reg_imm(cp, JX86_EAX, 1);
        jx86_jcc8_patch(cp, jne8);
      } while (0);
#endif
      jx86_shrn_reg_imm(cp, JX86_NAX, 9);
      jx86_orn_reg_imm(cp, JX86_NAX, 1);
      break;

    case GETVECTITEM:
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
      jx86_movn_reg_memindex(cp, JX86_NAX, JX86_NAX, -(CAML_JIT_WORD_SIZE / 2), JX86_NDX, CAML_JIT_WORD_SHIFT - 1);
      goto pop1;

    case SETVECTITEM:
#ifdef TARGET_amd64
      jx86_movq_reg_membase(cp, JX86_RDI, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_movq_reg_membase(cp, JX86_RSI, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_leaq_reg_memindex(cp, JX86_RDI, JX86_RAX, -(CAML_JIT_WORD_SIZE / 2), JX86_RDI, CAML_JIT_WORD_SHIFT - 1);
#else
      jx86_movl_reg_membase(cp, JX86_EDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_pushl_membase(cp, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_leal_reg_memindex(cp, JX86_EDX, JX86_EAX, -(CAML_JIT_WORD_SIZE / 2), JX86_EDX, CAML_JIT_WORD_SHIFT - 1);
      jx86_pushl_reg(cp, JX86_EDX);
#endif
      goto modify;

/* String operations */

    case GETSTRINGCHAR:
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
      jx86_shrn_reg_imm(cp, JX86_NDX, 1);
      jx86_movb_reg_memindex(cp, JX86_AL, JX86_NAX, 0, JX86_NDX, 0);
      jx86_movzxlb_reg_reg(cp, JX86_EAX, JX86_AL);
    shl1_or1_pop1:
      jx86_shln_reg_imm(cp, JX86_NAX, 1);
      goto or1_pop1;

    case SETSTRINGCHAR:
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_shrn_reg_imm(cp, JX86_NDX, 1);
      jx86_shrl_reg_imm(cp, JX86_ECX, 1);
      jx86_movb_memindex_reg(cp, JX86_NAX, 0, JX86_NDX, 0, JX86_CL);
      goto unit;

/* Branches and conditional branches */

    case BRANCH: {
      code_t dpc = pc + *pc;
      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      /* check if target is already compiled */
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
      jx86_cmpn_reg_imm(cp, JX86_NAX, Val_false);
    branchif1: {
        code_t dpc = pc + *pc;
        pc++;

        /* flush the stack pointer */
        if (sp != 0) {
          jx86_lean_reg_membase(cp, CAML_JIT_NSP, CAML_JIT_NSP, sp);
          sp = 0;
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
          ptrdiff_t offset = cp - caml_jit_code_base;

          assert(*pc >= 0);
          assert(*dpc >= 0);
          assert(offset > STOP);
          assert(offset <= INT_MAX);

          if (*pc > STOP) {
            /* "then" address is already on the pending list */
            assert(caml_jit_pending_contains(pc));
            jx86_jcc32_forward(cp, op ^ 1);
            *((opcode_t *) cp - 1) = *pc;
            *pc = offset;
            pc = dpc;
          }
          else {
            /* add "else" address if not already pending */
            if (*dpc <= STOP)
              caml_jit_pending_add(dpc);
            assert(caml_jit_pending_contains(dpc));
            jx86_jcc32_forward(cp, op);
            *((opcode_t *) cp - 1) = *dpc;
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
      unsigned char *jz8;
      unsigned char *leaq;

      /* flush the stack pointer */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }

      /* reserve space for the jumptable (each entry is 8-byte, even on x86) */
      if (CAML_JIT_GNUC_UNLIKELY (cp + (num_consts + num_blocks) * 8 > segment->segment_limit))
        cp = caml_jit_segment_continue(segment, cp);

      leaq = cp;
      jx86_lean_reg_forward(cp, JX86_NDX);
      if (num_consts) {
        jx86_testb_reg_imm(cp, JX86_AL, 1);
        jz8 = cp;
        jx86_jz8_forward(cp);
        jx86_jmpn_memindex(cp, JX86_NDX, -4, JX86_NAX, 2);
        jx86_jcc8_patch(cp, jz8);
      }
      if (num_blocks) {
        jx86_movzxlb_reg_membase(cp, JX86_ECX, JX86_NAX, -CAML_JIT_WORD_SIZE);
        jx86_jmpn_memindex(cp, JX86_NDX, num_consts * 8, JX86_NCX, 3);
      }
      /* stuff an UD2 instruction after the indirect branch to stop
       * the processor from decoding down the fall-through path.
       */
      jx86_ud2(cp);
      jx86_lean_reg_patch(cp, leaq);
      for (i = 0; i < num_consts + num_blocks; ++i) {
        code_t dpc = pc + pc[i];

        /* check if target is known */
        if (*dpc < 0) {
#ifdef TARGET_amd64
          jx86_emit_uint64(cp, caml_jit_code_end + *dpc);
#else
          jx86_emit_uint32(cp, caml_jit_code_end + *dpc);
          cp += 4;
#endif
        }
        else {
          ptrdiff_t offset = cp - caml_jit_code_base;

          assert(*dpc >= 0);
          assert(offset > STOP);
          assert(offset <= INT_MAX);

          /* add address if not already pending */
          if (*dpc <= STOP)
            caml_jit_pending_add(dpc);
          assert(caml_jit_pending_contains(dpc));
          jx86_emit_uint8(cp, 0);
          jx86_emit_int32(cp, *dpc);
          *dpc = offset;
          cp += 3;
        }
      }
      goto stop_generation;
    }

    case BOOLNOT:
      jx86_xorn_reg_imm(cp, JX86_NAX, 2);
      break;

/* Exceptions */

    case PUSHTRAP:
      sp -= 4 * CAML_JIT_WORD_SIZE;
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDX, pc + *pc);
      jx86_movq_reg_imm(cp, JX86_RDI, &caml_trapsp);
      jx86_movq_reg_membase(cp, JX86_RCX, JX86_RDI, 0);
      jx86_movq_membase_reg(cp, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE, JX86_RDX);     /* trap address */
      jx86_movq_membase_reg(cp, CAML_JIT_NSP, 1 * CAML_JIT_WORD_SIZE, JX86_RCX);     /* previous trap stack pointer */
      jx86_movq_membase_reg(cp, CAML_JIT_NSP, 2 * CAML_JIT_WORD_SIZE, CAML_JIT_NEP); /* env */
      jx86_movq_membase_reg(cp, CAML_JIT_NSP, 3 * CAML_JIT_WORD_SIZE, CAML_JIT_NEA); /* extra args */
      jx86_movq_membase_reg(cp, JX86_RDI, 0, CAML_JIT_NSP);
#else
      jx86_movl_reg_mem(cp, JX86_ECX, &caml_trapsp);
      jx86_movl_membase_imm(cp, CAML_JIT_NSP, 0 * CAML_JIT_WORD_SIZE, pc + *pc);     /* trap address */
      jx86_movl_membase_reg(cp, CAML_JIT_NSP, 1 * CAML_JIT_WORD_SIZE, JX86_ECX);     /* previous trap stack pointer */
      jx86_movl_membase_reg(cp, CAML_JIT_NSP, 2 * CAML_JIT_WORD_SIZE, CAML_JIT_NEP); /* env */
      jx86_movl_membase_reg(cp, CAML_JIT_NSP, 3 * CAML_JIT_WORD_SIZE, CAML_JIT_NEA); /* extra args */
      jx86_movl_mem_reg(cp, &caml_trapsp, CAML_JIT_NSP);
#endif
      pc++;
      break;

    case CHECK_SIGNALS:
    case POPTRAP: {
      unsigned char *jnz8;

      /* CHECK_SIGNALS is likely a branch target for loops,
       * so ensure to assign a native address to avoid
       * duplicate compilation.
       */
      if (sp != 0) {
        jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
        sp = 0;
      }
      pc[-1] = (opcode_t) (cp - caml_jit_code_end);
      assert(pc[-1] < 0);

#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDX, &caml_something_to_do);
      jx86_testl_membase_imm(cp, JX86_RDX, 0, -1);
#else
      jx86_testl_mem_imm(cp, &caml_something_to_do, -1);
#endif
      jnz8 = cp;
      jx86_jz8_forward(cp);
#ifdef TARGET_amd64
      jx86_movq_reg_imm(cp, JX86_RDI, pc - 1);
#else
      jx86_movl_reg_imm(cp, JX86_ECX, pc - 1);
#endif
      jx86_jmp(cp, &caml_jit_rt_process_signal);
      jx86_jcc8_patch(cp, jnz8);
      if (instr == POPTRAP) {
        assert(sp == 0);
        jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, 1 * CAML_JIT_WORD_SIZE);
#ifdef TARGET_amd64
        jx86_movq_reg_imm(cp, JX86_RDI, &caml_trapsp);
        jx86_movq_membase_reg(cp, JX86_RDI, 0, JX86_RCX);
#else
        jx86_movl_mem_reg(cp, &caml_trapsp, JX86_ECX);
#endif
        sp = 4 * CAML_JIT_WORD_SIZE;
      }
      break;
    }

    case RAISE:
      /* ensure that RAISE instructions remain
       * untouched (necessary for backtrace)
       */
      *(pc - 1) = RAISE;

#ifdef TARGET_amd64
      /* load %rsi with pc */
      jx86_movq_reg_imm(cp, JX86_RSI, pc);
#else
      /* store pc to saved_pc */
      jx86_movl_membase_imm(cp, JX86_ESP, 0, pc);
#endif

      /* throw the exception in %rax back to C code */
      addr = &caml_jit_rt_raise;
      goto flush_jmp_stop_generation;

/* Calling C functions */

    case C_CALL1:
    case C_CALL2:
    case C_CALL3: {
      state = 0; /* state == 0: float value in (%nax), state != 0: float value in %xmm0 */
      for (;;) {
        addr = Primitive(*pc++);
        if (addr == &caml_sqrt_float) {
          if (!state)
            jx86_sqrtsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          else
            jx86_sqrtsd_xmm_xmm(cp, JX86_XMM0, JX86_XMM0);
          state = 1;
        }
        else if (addr == &caml_float_of_int) {
          assert(!state);
          jx86_sarn_reg_imm(cp, JX86_NAX, 1);
          jx86_cvtsi2sdn_xmm_reg(cp, JX86_XMM0, JX86_NAX);
          state = 1;
        }
        else if (addr == &caml_int_of_float) {
          if (!state)
            jx86_cvttsd2sin_reg_membase(cp, JX86_NAX, JX86_NAX, 0);
          else
            jx86_cvttsd2sin_reg_xmm(cp, JX86_NAX, JX86_XMM0);
          goto shl1_or1;
        }
        else if (addr == &caml_abs_float) {
          addr = &caml_jit_rt_abs_float;
        c_call1_floatext:
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_call(cp, addr);
          state = 1;
        }
        else if (addr == &caml_neg_float) {
          addr = &caml_jit_rt_neg_float;
          goto c_call1_floatext;
        }
        else if (addr == &caml_cos_float) {
          addr = &CAML_JIT_FPPRIM(cos);
          goto c_call1_floatext;
        }
        else if (addr == &caml_sin_float) {
          addr = &CAML_JIT_FPPRIM(sin);
          goto c_call1_floatext;
        }
        else if (addr == &caml_asin_float) {
          addr = &CAML_JIT_FPPRIM(asin);
          goto c_call1_floatext;
        }
        else if (addr == &caml_add_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_addsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          state = 1;
        }
        else if (addr == &caml_sub_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_subsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          state = 1;
        }
        else if (addr == &caml_mul_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_mulsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          state = 1;
        }
        else if (addr == &caml_div_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_divsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          state = 1;
        }
        else if (addr == &caml_eq_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          jx86_sete_reg(cp, JX86_AL);
          jx86_setnp_reg(cp, JX86_DL);
          jx86_andb_reg_reg(cp, JX86_AL, JX86_DL);
          op = JX86_SETNZ;
          goto cmpint2;
        }
        else if (addr == &caml_neq_float) {
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          jx86_setne_reg(cp, JX86_AL);
          jx86_setp_reg(cp, JX86_DL);
          jx86_orb_reg_reg(cp, JX86_AL, JX86_DL);
          op = JX86_SETNZ;
          goto cmpint2;
        }
        else if (addr == &caml_le_float) {
          op = JX86_SETAE;
        cmpfloat_rev:
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
          jx86_movsd_xmm_membase(cp, JX86_XMM1, JX86_NDX, 0);
          if (!state)
            jx86_ucomisd_xmm_membase(cp, JX86_XMM1, JX86_NAX, 0);
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
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
          if (!state)
            jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NAX, 0);
          jx86_ucomisd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          goto cmpint1;
        }
        else if (addr == &caml_gt_float) {
          op = JX86_SETA;
          goto cmpfloat;
        }
        else if (addr == &caml_atan2_float) {
          addr = &CAML_JIT_FPPRIM(atan2);
        c_call2_floatext:
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movsd_xmm_membase(cp, JX86_XMM1, JX86_NDX, 0);
          goto c_call1_floatext;
        }
        else if (addr == &caml_fmod_float) {
          addr = &CAML_JIT_FPPRIM(fmod);
          goto c_call2_floatext;
        }
        else if (addr == &caml_array_unsafe_get_float) {
          assert(!state);
          jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movsd_xmm_memindex(cp, JX86_XMM0, JX86_NAX, -4, JX86_NCX, 2);
      float_optloop:
          state = 1;
        }
        else if (addr == &caml_array_unsafe_set_float) {
          assert(!state);
          jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
          jx86_movsd_xmm_membase(cp, JX86_XMM0, JX86_NDX, 0);
          jx86_movsd_memindex_xmm(cp, JX86_NAX, -4, JX86_NCX, 2, JX86_XMM0);
          goto unit;
        }
        else {
          /* flush the double value */
          if (state) {
            /* flush the stack pointer */
            if (sp != 0) {
              jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
              sp = 0;
            }
            jx86_call(cp, &caml_jit_rt_copy_double);
          }
          goto c_call;
        }
        instr = *pc;
#ifdef DEBUG
        /* support proper instruction tracing */
        if (caml_trace_flag)
          goto begin;
#endif
        if (instr < C_CALL1 || instr > C_CALL3) {
          /* [Peephole Optimization] PUSHCONSTx PUSHACCy C_CALL3 caml_array_unsafe_set_float */
          if (state
              && instr >= PUSHCONST0 && instr <= PUSHCONST3
              && pc[1] >= PUSHACC2 && pc[1] <= PUSHACC7
              && pc[2] == C_CALL3 && Primitive(pc[3]) == &caml_array_unsafe_set_float) {
            jx86_movn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp + (pc[1] - PUSHACC0 - 2) * CAML_JIT_WORD_SIZE);
            jx86_movsd_membase_xmm(cp, JX86_NAX, (instr - PUSHCONST0) * sizeof(double), JX86_XMM0);
            pc += 4;
            goto unit;
          }
          else if (state) {
            addr = &caml_jit_rt_copy_double;
            goto flush_call_addr;
          }
          else
            goto begin;
        }
        pc++;
        if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
          cp = caml_jit_segment_continue(segment, cp);
      }
      break;
    }

    case C_CALLN:
      instr = (C_CALL1 - 1) + *pc++;
      /* FALL-THROUGH */
    case C_CALL4:
    case C_CALL5:
      addr = Primitive(*pc++);
    c_call: {
      int num_args = instr - (C_CALL1 - 1);
      int alloc = !caml_jit_noalloc(addr);
#ifdef TARGET_amd64
      if (alloc) {
        /* load %rbp with the address of caml_young_ptr */
        jx86_movq_reg_imm(cp, JX86_RBP, &caml_young_ptr);
        /* load %rbx with the address of caml_extern_sp */
        jx86_movq_reg_imm(cp, JX86_RBX, &caml_extern_sp);
      }
      /* call semantics differ if num_args <= 5 */
      if (num_args <= 5) {
        /* setup up to five arguments */
        jx86_movq_reg_reg(cp, JX86_RDI, JX86_RAX);
        if (num_args >= 2) { jx86_movq_reg_membase(cp, JX86_RSI, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE; }
        if (num_args >= 3) { jx86_movq_reg_membase(cp, JX86_RDX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE; }
        if (num_args >= 4) { jx86_movq_reg_membase(cp, JX86_RCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE; }
        if (num_args == 5) { jx86_movq_reg_membase(cp, JX86_R8,  CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE; }
        /* reserve stack space for the environment pointer */
        sp -= CAML_JIT_WORD_SIZE;
      }
      else {
        /* arguments are stack pointer and num_args */
        sp -= CAML_JIT_WORD_SIZE;
        jx86_movl_reg_imm(cp, JX86_ESI, num_args);
        jx86_movq_membase_reg(cp, CAML_JIT_NSP, sp, JX86_RAX);
        jx86_leaq_reg_membase(cp, JX86_RDI, CAML_JIT_NSP, sp);
        sp -= CAML_JIT_WORD_SIZE;
      }
      if (alloc) {
        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* make sure reasonable code space is available */
        if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
          cp = caml_jit_segment_continue(segment, cp);
        /* save environment pointer */
        jx86_movq_membase_reg(cp, CAML_JIT_NSP, 0, CAML_JIT_NEP);
        /* save young ptr to caml_young_ptr */
        jx86_movq_membase_reg(cp, JX86_RBP, 0, CAML_JIT_NYP);
        /* save stack pointer to caml_extern_sp */
        jx86_movq_membase_reg(cp, JX86_RBX, 0, CAML_JIT_NSP);
        /* save the pc to the reserved saved_pc area 0*8(%rsp) */
        jx86_movq_reg_imm(cp, JX86_R11, pc + 1);
        jx86_movq_membase_reg(cp, JX86_RSP, 0 * 8, JX86_R11);
      }
      /* call the primitive C function */
      jx86_call(cp, addr);
      if (alloc) {
        /* reset saved_pc area 0*8(%rsp) */
        jx86_movq_membase_imm(cp, JX86_RSP, 0 * 8, 0);
        /* restore local state */
        jx86_movq_reg_membase(cp, CAML_JIT_NYP, JX86_RBP, 0);
        jx86_movq_reg_membase(cp, CAML_JIT_NSP, JX86_RBX, 0);
        jx86_movq_reg_membase(cp, CAML_JIT_NEP, CAML_JIT_NSP, 0);
      }
#else
      if (alloc) {
        /* save the pc to the reserved saved_pc area 0*4(%esp) */
        jx86_movl_membase_imm(cp, JX86_ESP, 0 * 4, pc + 1);
      }
      /* call semantics differ if num_args <= 5 */
      if (num_args <= 5) {
        int i;
        /* setup up to five arguments */
        for (i = num_args - 1; --i >= 0; )
          jx86_pushl_membase(cp, CAML_JIT_NSP, sp + i * CAML_JIT_WORD_SIZE);
        jx86_pushl_reg(cp, JX86_EAX);
        /* pop stack arguments and reserve space for env */
        sp += (num_args - 2) * CAML_JIT_WORD_SIZE;
      }
      else {
        /* push accu onto Caml stack */
        sp -= CAML_JIT_WORD_SIZE;
        jx86_movl_membase_reg(cp, CAML_JIT_NSP, sp, JX86_EAX);
        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* arguments are stack pointer and num_args */
        jx86_push_imm(cp, num_args);
        jx86_pushl_reg(cp, CAML_JIT_NSP);
        /* reserve space for the environment pointer */
        sp -= CAML_JIT_WORD_SIZE;
      }
      if (alloc) {
        /* flush the stack pointer */
        if (sp != 0) {
          jx86_addn_reg_imm(cp, CAML_JIT_NSP, sp);
          sp = 0;
        }
        /* make sure reasonable code space is available */
        if (CAML_JIT_GNUC_UNLIKELY (cp > segment->segment_limit))
          cp = caml_jit_segment_continue(segment, cp);
        /* save environment pointer */
        jx86_movn_membase_reg(cp, CAML_JIT_NSP, 0, CAML_JIT_NEP);
        /* make stack and young pointer available to C code */
        jx86_movl_mem_reg(cp, &caml_extern_sp, CAML_JIT_NSP);
        jx86_movl_mem_reg(cp, &caml_young_ptr, CAML_JIT_NYP);
      }
      /* call the primitive C function */
      jx86_call(cp, addr);
      /* pop C stack arguments */
      if (num_args > 5)
        jx86_addn_reg_imm(cp, JX86_ESP, 2 * CAML_JIT_WORD_SIZE);
      else
        jx86_addn_reg_imm(cp, JX86_ESP, num_args * CAML_JIT_WORD_SIZE);
      if (alloc) {
        /* reset saved_pc area 0*4(%esp) */
        jx86_movl_membase_imm(cp, JX86_ESP, 0 * 4, 0);
        /* restore stack, young and environment pointer */
        jx86_movl_reg_mem(cp, CAML_JIT_NSP, &caml_extern_sp);
        jx86_movl_reg_mem(cp, CAML_JIT_NYP, &caml_young_ptr);
        jx86_movl_reg_membase(cp, CAML_JIT_NEP, CAML_JIT_NSP, 0);
      }
#endif
      /* pop arguments/environment pointer off the stack */
      if (num_args > 5)
        sp += (num_args + 1) * CAML_JIT_WORD_SIZE;
      else
        sp += 1 * CAML_JIT_WORD_SIZE;
      break;
    }

/* Integer constants */

    case PUSHCONSTINT:
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case CONSTINT:
      jx86_movn_reg_imm(cp, JX86_NAX, Val_int(*pc++));
      break;

    case PUSHCONST0:
    case PUSHCONST1:
    case PUSHCONST2:
    case PUSHCONST3:
      instr = (instr - PUSHCONST0) + CONST0;
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
      /* FALL-THROUGH */
    case CONST0:
    case CONST1:
    case CONST2:
    case CONST3:
      jx86_movn_reg_imm(cp, JX86_NAX, Val_int(instr - CONST0));
      break;

/* Integer arithmetic */

    case NEGINT:
      jx86_negn_reg(cp, JX86_NAX);
      jx86_addn_reg_imm(cp, JX86_NAX, 2);
      break;

    case ADDINT:
      jx86_addn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
      jx86_subn_reg_imm(cp, JX86_NAX, 1);
    pop1:
      sp += CAML_JIT_WORD_SIZE;
      break;

    case SUBINT:
      jx86_subn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
    or1_pop1:
      jx86_orn_reg_imm(cp, JX86_NAX, 1);
      goto pop1;

    case MULINT:
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp);
      jx86_sarn_reg_imm(cp, JX86_NAX, 1);
      jx86_sarn_reg_imm(cp, JX86_NCX, 1);
      jx86_imuln_reg_reg(cp, JX86_NAX, JX86_NCX);
      goto shl1_or1_pop1;

    case DIVINT:
    case MODINT:
      /* load the operands */
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp); sp += CAML_JIT_WORD_SIZE;
      jx86_sarn_reg_imm(cp, JX86_NAX, 1);
      jx86_sarn_reg_imm(cp, JX86_NCX, 1);
#ifdef TARGET_amd64
      /* load %r11 with the effective pc (for the exception case) */
      jx86_movq_reg_imm(cp, JX86_R11, pc + 2);
      /* %rdx:%rax <- sign-extend of %rax */
      jx86_cqo(cp);
#else
      /* save effective pc to saved_pc (for the exception case) */
      jx86_movl_membase_imm(cp, JX86_ESP, 0 * CAML_JIT_WORD_SIZE, pc + 2);
      /* %edx:%eax <- sign-extend of %eax */
      jx86_cdq(cp);
#endif
      /* flush the stack pointer */  
      if (sp != 0) {
        jx86_lean_reg_membase(cp, CAML_JIT_NSP, CAML_JIT_NSP, sp);
        sp = 0;
      }
      /* check for "zero divide" */
      jx86_jz(cp, caml_jit_code_raise_zero_divide);
      jx86_idivn_reg(cp, JX86_NCX);
#ifdef TARGET_i386
      /* reset saved_pc */
      jx86_movl_membase_imm(cp, JX86_ESP, 0 * CAML_JIT_WORD_SIZE, NULL);
#endif
      if (instr == MODINT)
        jx86_movn_reg_reg(cp, JX86_NAX, JX86_NDX);
    shl1_or1:
      jx86_shln_reg_imm(cp, JX86_NAX, 1);
      jx86_orn_reg_imm(cp, JX86_NAX, 1);
      break;

    case ANDINT:
      jx86_andn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
      goto pop1;

    case ORINT:
      jx86_orn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
      goto pop1;

    case XORINT:
      jx86_xorn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
      goto or1_pop1;

    case LSLINT:
      op = JX86_SHL;
    shiftint:
      jx86_movn_reg_membase(cp, JX86_NCX, CAML_JIT_NSP, sp);
      jx86_subn_reg_imm(cp, JX86_NAX, 1);
      jx86_shrn_reg_imm(cp, JX86_NCX, 1);
      jx86_shfn_reg_reg(cp, op, JX86_NAX, JX86_CL);
      goto or1_pop1;
    case LSRINT:
      op = JX86_SHR;
      goto shiftint;
    case ASRINT:
      op = JX86_SAR;
      goto shiftint;

    case EQ:
    case NEQ:
    case LTINT:
    case LEINT:
    case GTINT:
    case GEINT:
      op = caml_jit_cmp2x86setcc[instr - EQ];
    cmpint:
      jx86_cmpn_reg_membase(cp, JX86_NAX, CAML_JIT_NSP, sp);
    cmpint1:
      jx86_setcc_reg(cp, op, JX86_AL);
    cmpint2:
      jx86_movzxlb_reg_reg(cp, JX86_EAX, JX86_AL);
      /* [Peephole Optimization] Comparison followed by BRANCHIF/BRANCHIFNOT */
      if (CAML_JIT_GNUC_LIKELY (*pc == BRANCHIF || *pc == BRANCHIFNOT)) {
        /* leal 1(, %eax, 2), %eax */
        jx86_emit_uint8((cp), 0x8d);
        jx86_emit_address_byte((cp), 0, JX86_EAX, 4);
        jx86_emit_address_byte((cp), 1, JX86_EAX, 5);
        jx86_emit_int32((cp), 1);
        op = (op + JX86_JA) - JX86_SETA;
        if (*pc++ == BRANCHIFNOT)
          op = op ^ 1;
        sp += CAML_JIT_WORD_SIZE;
        goto branchif1;
      }
      else
        goto shl1_or1_pop1;
    case ULTINT:
      op = JX86_SETB;
      goto cmpint;
    case UGEINT:
      op = JX86_SETAE;
      goto cmpint;

    case BEQ:
    case BNEQ:
    case BLTINT:
    case BLEINT:
    case BGTINT:
    case BGEINT:
      op = caml_jit_bcmp2x86jcc[instr - BEQ];
    branchint:
      jx86_cmpn_reg_imm(cp, JX86_NAX, Val_int(*pc++));
      goto branchif1;
    case BULTINT:
      op = JX86_JA;
      goto branchint;
    case BUGEINT:
      op = JX86_JBE;
      goto branchint;

    case OFFSETINT:
      jx86_addn_reg_imm(cp, JX86_NAX, *pc++ << 1);
      break;

    case OFFSETREF:
      jx86_addn_membase_imm(cp, JX86_NAX, 0, *pc++ << 1);
      goto unit;

    case ISINT:
      jx86_andl_reg_imm(cp, JX86_EAX, 1);
      /* [Peephole Optimization] ISINT followed by BRANCHIF/BRANCHIFNOT */
      if (CAML_JIT_GNUC_LIKELY (*pc == BRANCHIF || *pc == BRANCHIFNOT)) {
        /* leal 1(, %eax, 2), %eax */
        jx86_emit_uint8((cp), 0x8d);
        jx86_emit_address_byte((cp), 0, JX86_EAX, 4);
        jx86_emit_address_byte((cp), 1, JX86_EAX, 5);
        jx86_emit_int32((cp), 1);
        op = (*pc++ == BRANCHIFNOT) ? JX86_JZ : JX86_JNZ;
        goto branchif1;
      }
      else {
        jx86_shll_reg_imm(cp, JX86_EAX, 1);
        jx86_orn_reg_imm(cp, JX86_NAX, 1);
      }
      break;

/* Object-oriented operations */

    case GETMETHOD:
      jx86_movn_reg_membase(cp, JX86_NDX, CAML_JIT_NSP, sp);
      jx86_movn_reg_membase(cp, JX86_NDX, JX86_NDX, 0);
      jx86_movn_reg_memindex(cp, JX86_NAX, JX86_NDX, -(CAML_JIT_WORD_SIZE / 2), JX86_NAX, CAML_JIT_WORD_SHIFT - 1);
      break;

    case GETPUBMET: {
      const value tag = Val_int(*pc++);
      sp -= CAML_JIT_WORD_SIZE;
      jx86_movn_membase_reg(cp, CAML_JIT_NSP, sp, JX86_NAX);
#ifdef TARGET_i386
      jx86_push_imm(cp, pc);
      jx86_push_imm(cp, tag);
      jx86_pushl_membase(cp, JX86_EAX, 0);
#else
      jx86_movq_reg_membase(cp, JX86_RDI, JX86_RAX, 0);
      jx86_movq_reg_imm(cp, JX86_RSI, tag);
      jx86_movq_reg_imm(cp, JX86_RDX, pc);
#endif
      pc++;
      jx86_call(cp, &caml_cache_public_method2);
#ifdef TARGET_i386
      jx86_addl_reg_imm(cp, JX86_ESP, 3 * CAML_JIT_WORD_SIZE);
#endif
      break;
    }

    case GETDYNMET:
#ifdef TARGET_i386
      jx86_pushl_reg(cp, JX86_EAX);
      jx86_pushl_membase(cp, CAML_JIT_NSP, sp);
#else
      jx86_movq_reg_reg(cp, JX86_RSI, JX86_RAX);
      jx86_movq_reg_membase(cp, JX86_RDI, CAML_JIT_NSP, sp);
#endif
      jx86_call(cp, &caml_get_public_method);
#ifdef TARGET_i386
      jx86_addl_reg_imm(cp, JX86_ESP, 2 * CAML_JIT_WORD_SIZE);
#endif
      break;

/* Debugging and machine control */

    case STOP:
      addr = &caml_jit_rt_stop;
      goto flush_jmp_stop_generation;

    default:
#if _MSC_VER >= 1200      
      __assum(0);
#else
      caml_fatal_error_arg("Fatal error: bad opcode (%"
                           ARCH_INTNAT_PRINTF_FORMAT "x)\n",
                           (char *) (intnat) instr);
#endif
    }
  }

  /* enforce proper alignment for segment code pointer */
  segment->segment_current = (unsigned char *) (((size_t) cp + (sizeof(void *) - 1)) & ~(sizeof(void *) - 1));

  assert(caml_jit_pending_capacity == 0 || caml_jit_pending_buffer != NULL);
  assert(caml_jit_pending_buffer == NULL || caml_jit_pending_capacity > 0);
  assert(caml_jit_pending_capacity >= 0);
  assert(caml_jit_pending_size == 0);
  assert(sp == 0);

  return start;
}


void caml_prepare_bytecode(code_t prog, asize_t prog_size)
{
  caml_jit_segment_t  *segment;
  caml_jit_segment_t **segmentp;
  code_t               pend;

  assert(prog != NULL);
  assert(prog_size > 0);
  assert((prog_size % sizeof(*prog)) == 0);

  if (CAML_JIT_GNUC_LIKELY (caml_jit_enabled)) {
    pend = prog + (prog_size / sizeof(*prog));
    
    /* register the byte-code segment (if not already registered) */
    for (segmentp = &caml_jit_segment_head;;) {
      segment = *segmentp;
      if (segment == NULL) {
        /* we have a new code segment here */
        segment = (caml_jit_segment_t *) calloc(1, sizeof(caml_jit_segment_t));
        if (CAML_JIT_GNUC_UNLIKELY (segment == NULL))
          caml_fatal_error("Fatal error: Unable to alloc new byte-code segment!\n");
        segment->segment_prog = prog;
        segment->segment_pend = pend;
        *segmentp = segment;
        break;
      }
      else if (segment->segment_prog == prog && segment->segment_pend == pend) {
        /* we know this segment already */
        break;
      }
      assert(prog >= segment->segment_pend
             || pend < segment->segment_prog);
      segmentp = &segment->segment_next;
    }
  }
}


void caml_release_bytecode(code_t prog, asize_t prog_size)
{
  caml_jit_chunk_t    *chunk;
  caml_jit_chunk_t    *cnext;
  caml_jit_segment_t  *segment;
  caml_jit_segment_t **segmentp;
  code_t               pend;

  assert(prog != NULL);
  assert(prog_size > 0);
  assert((prog_size % sizeof(*prog)) == 0);

  if (CAML_JIT_GNUC_LIKELY (caml_jit_enabled)) {
    pend = prog + (prog_size / sizeof(*prog));

    /* unregister the byte-code segment */
    for (segmentp = &caml_jit_segment_head;;) {
      segment = *segmentp;
      assert(segment != NULL);
      if (segment->segment_prog == prog && segment->segment_pend == pend) {
        /* release the code chunks */
        for (chunk = segment->segment_chunks; chunk != NULL; chunk = cnext) {
          cnext = chunk->chunk_next;
          caml_jit_chunk_clear(chunk);
          chunk->chunk_next = caml_jit_chunk_free;
          caml_jit_chunk_free = chunk;
        }

        /* drop the segment from the list */
        *segmentp = segment->segment_next;
        free(segment);
        break;
      }
      segmentp = &segment->segment_next;
    }
  }
}

