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

/* Asm part of the JIT runtime system, AMD64 processor */
/* Must be preprocessed by cpp */

#include "../config/m.h"
#include "../config/s.h"

#ifdef SYS_macosx

#define G(r) _##r
#define GREL(r) _##r@GOTPCREL
#define GCALL(r) _##r
#define FUNCTION(name) \
        .globl name; \
        .private_extern name; \
        .p2align 3; \
        name:

#else

#define G(r) r
#define GREL(r) r@GOTPCREL
#define GCALL(r) r@PLT
#define FUNCTION(name) \
        .globl name; \
        .type name,@function; \
        .hidden name; \
        .p2align 3; \
        name:

#endif

#ifdef __PIC__

/* Position-independent operations on global variables. */

/* Store [srcreg] in global [dstlabel]. Clobbers %r11. */
#define STORE_VAR(srcreg, dstlabel) \
        movq    GREL(dstlabel)(%rip), %r11; \
        movq    srcreg, (%r11)

/* Load global [srclabel] in register [dstreg]. Clobbers %r11. */
#define LOAD_VAR(srclabel, dstreg) \
        movq    GREL(srclabel)(%rip), %r11; \
        movq    (%r11), dstreg

/* Add global [srclabel] to register [dstreg]. Clobbers %r11. */
#define ADD_VAR(srclabel, dstreg) \
        movq    GREL(srclabel)(%rip), %r11; \
        addq    (%r11), dstreg

/* Compare global [label] with register [reg]. Clobbers %r11. */
#define CMP_VAR(label, reg) \
        movq    GREL(label)(%rip), %r11; \
        cmpq    (%r11), reg

/* Store [srcimm] in 32-bit global [dstlabel]. Clobbers %r11. */
#define STOREL_VAR(srcimm, dstlabel) \
        movq    GREL(dstlabel)(%rip), %r11; \
        movl    srcimm, (%r11)

/* Test 32-bit global [label] against mask [imm]. Clobbers %r11. */
#define TESTL_VAR(imm, label) \
        movq    GREL(label)(%rip), %r11; \
        testl   imm, (%r11)

/* Decrements 32-bit global [dstlabel]. Clobbers %r11. */
#define DECL_VAR(dstlabel) \
        movq    GREL(dstlabel)(%rip), %r11; \
        decl    (%r11)

/* Increments 32-bit global [dstlabel]. Clobbers %r11. */
#define INCL_VAR(dstlabel) \
        movq    GREL(dstlabel)(%rip), %r11; \
        incl    (%r11)

#else

/* Non-PIC operations on global variables. Slightly faster. */

/* Store [srcreg] in global [dstlabel]. */
#define STORE_VAR(srcreg, dstlabel) \
        movq    srcreg, G(dstlabel)(%rip)

/* Load global [srclabel] in register [dstreg]. */
#define LOAD_VAR(srclabel, dstreg) \
        movq    G(srclabel)(%rip), dstreg

/* Add global [srclabel] to register [dstreg]. */
#define ADD_VAR(srclabel, dstreg) \
        addq    G(srclabel)(%rip), dstreg

/* Compare global [label] with register [reg]. */
#define CMP_VAR(label, reg) \
        cmpq    G(label)(%rip), reg

/* Store [srcimm] in 32-bit global [dstlabel]. */
#define STOREL_VAR(srcimm, dstlabel) \
        movl    srcimm, G(dstlabel)(%rip)

/* Test 32-bit global [label] against mask [imm]. */
#define TESTL_VAR(imm, label) \
        testl   imm, G(label)(%rip)

/* Decrements 32-bit global [dstlabel]. */
#define DECL_VAR(dstlabel) \
        decl    G(dstlabel)(%rip)

/* Increments 32-bit global [dstlabel]. */
#define INCL_VAR(dstlabel) \
        incl    G(dstlabel)(%rip)

#endif

        .text

/* Allocation */

        /* caml_jit_rt_alloc1:
	 *
	 * Allocates a block of wosize 1 in the minor heap. Upon
	 * return %r15 points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc1_gc:
        addq    $(2*8), %r15
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc1))
.Lcaml_jit_rt_alloc1:
        subq    $(2*8), %r15
        CMP_VAR(caml_young_limit, %r15)
        jb      .Lcaml_jit_rt_alloc1_gc
        ret

        /* caml_jit_rt_alloc2:
	 *
	 * Allocates a block of wosize 2 in the minor heap. Upon
	 * return %r15 points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc2_gc:
        addq    $(3*8), %r15
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc2))
.Lcaml_jit_rt_alloc2:
        subq    $(3*8), %r15
        CMP_VAR(caml_young_limit, %r15)
        jb      .Lcaml_jit_rt_alloc2_gc
        ret

        /* caml_jit_rt_alloc3:
	 *
	 * Allocates a block of wosize 3 in the minor heap. Upon
	 * return %r15 points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc3_gc:
        addq    $(4*8), %r15
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc3))
.Lcaml_jit_rt_alloc3:
        subq    $(4*8), %r15
        CMP_VAR(caml_young_limit, %r15)
        jb      .Lcaml_jit_rt_alloc3_gc
        ret

        /* caml_jit_rt_allocN:
	 *
	 * Allocates a block of bhsize %rdx in the minor heap.
	 * Upon return %r15 points to the header of the newly
	 * allocated block.
	 */
.Lcaml_jit_rt_allocN_gc:        
        pushq   %rdx                    /* preserve desired size */
        addq    %rdx, %r15
        call    .Lcaml_jit_rt_call_gc
        popq    %rdx                    /* recover desired size */
FUNCTION(G(caml_jit_rt_allocN))
.Lcaml_jit_rt_allocN:
        subq    %rdx, %r15
        CMP_VAR(caml_young_limit, %r15)
        jb      .Lcaml_jit_rt_allocN_gc
        ret

        /* caml_jit_rt_call_gc:
	 *
	 * Invoked by the caml_jit_rt_allocX functions above when
	 * the minor heap runs out of space.
	 *
	 * Preserves %rdi, %rsi, %xmm0.
         */
FUNCTION(G(caml_jit_rt_call_gc))
.Lcaml_jit_rt_call_gc:
    /* Push preserved registers */
        pushq   %rdi
        pushq   %rsi
        subq    $(1*8), %rsp
        movlpd  %xmm0, 0*8(%rsp)
    /* Push environment and acc onto the Caml stack */
        movq    %r12, -1*8(%r14)
        movq    %rax, -2*8(%r14)
    /* Make the young and stack pointers available to the C code */
        STORE_VAR(%r15, caml_young_ptr)
        leaq    -2*8(%r14), %r10
        STORE_VAR(%r10, caml_extern_sp)
    /* Invoke the garbage collector */
        movq    %rsp, %rbp      /* 16-align C stack */
        andq    $-16, %rsp
        call    GCALL(caml_minor_collection)
        movq    %rbp, %rsp
    /* Restore young pointer */
        LOAD_VAR(caml_young_ptr, %r15)
    /* Restore environment and accu from the Caml stack */
        movq    -1*8(%r14), %r12
        movq    -2*8(%r14), %rax
    /* Restore preserved registers */
        movlpd  0*8(%rsp), %xmm0
        addq    $(1*8), %rsp
        popq    %rsi
        popq    %rdi
        ret

        
/* Function return */

        /* caml_jit_rt_grab_closure_and_return:
	 *
	 * Upon entry, %rax contains the restart byte code address.
         */
FUNCTION(G(caml_jit_rt_grab_closure_and_return))
    /* Determine the wosize from the extra arguments */
        shrq    $1, %r13
        addq    $3, %r13
    /* Allocate space in the minor heap */
        leaq    8(, %r13, 8), %rdx
        call    .Lcaml_jit_rt_allocN
    /* Setup for initialization below */
        leaq    -2(%r13), %rcx
        leaq    3*8(%r15), %rdi
        movq    %r14, %rsi
    /* Store the closure header (%r13 << 10 | Closure_tag | Caml_black) */
        shlq    $10, %r13
        orq     $1015, %r13
        movq    %r13, 0*8(%r15)
    /* Store the restart function pointer */
        movq    %rax, 1*8(%r15)
    /* Store the environment pointer */
        movq    %r12, 2*8(%r15)
    /* Move the arguments from the stack to the closure */
        rep movsq
    /* Restore stack pointer */
        movq    %rsi, %r14
    /* Return the closure as result */
        leaq    1*8(%r15), %rax
        jmp     .Lcaml_jit_rt_return_to_caller

        /* caml_jit_rt_return:
	 *
	 * Returns to caller or tail calls to result, depending on
	 * the number of extra arguments present.
         */
FUNCTION(G(caml_jit_rt_return))
    /* Check if we have extra args left */
        cmpq    $1, %r13
        jbe     .Lcaml_jit_rt_return_to_caller
    /* Decrement extra args and tail call to result */
        addq    $-2, %r13
        movq    0*8(%rax), %rdi
        movq    %rax, %r12
        jmp     .Lcaml_jit_rt_trampoline
.Lcaml_jit_rt_return_to_caller:
    /* Restore extra args/env from Caml stack and return to caller */
        movq    0*8(%r14), %rdi
        movq    1*8(%r14), %r12
        movq    2*8(%r14), %r13
        addq    $(3*8), %r14
        jmp     .Lcaml_jit_rt_trampoline

        /* caml_jit_rt_trampoline:
	 *
	 * Jumps to the byte-code address %rdi, compiling the byte
	 * code on-demand if it wasn't already compiled.
	 */
FUNCTION(G(caml_jit_rt_trampoline))
.Lcaml_jit_rt_trampoline:
        movslq  (%rdi), %rdx
        ADD_VAR(caml_jit_code_end, %rdx)
        jmpq    *%rdx
        

/* Function restart */

        /* caml_jit_rt_restart:
	 *
	 * Pushes the arguments saved in the closure %r12 by
	 * caml_jit_rt_grab_closure_and_return back onto the
	 * Caml stack.
	 *
	 * Clobbers %r11, %rcx, %rdi and %rsi.
         */
FUNCTION(G(caml_jit_rt_restart))
    /* Determine the number of arguments from the environment */
        movq    -1*8(%r12), %rcx
        shrq    $10, %rcx
        subq    $2, %rcx
    /* Add the number of arguments to the extra args */
        leaq    (%r13, %rcx, 2), %r13
    /* Load the new stack pointer */
        leaq    (, %rcx, 8), %r11
        subq    %r11, %r14
    /* Push the arguments back onto the Caml stack */
        leaq    2*8(%r12), %rsi
        movq    %r14, %rdi
        rep movsq
    /* Load the new environment pointer */
        movq    1*8(%r12), %r12
        ret

        
/* Function application */

        /* caml_jit_rt_apply1:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY1 opcode.
	 * Expects Caml return address in %rdx.
         */
FUNCTION(G(caml_jit_rt_apply1))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdx, 1*8(%r14) /* return address */
        movq    %r12, 2*8(%r14) /* environment */
        movq    %r13, 3*8(%r14) /* extra args */
    /* Record extra arguments */
        movq    $((0 << 1) | 1), %r13
    /* Perform the application */
        jmp     .Lcaml_jit_rt_apply

        /* caml_jit_rt_apply2:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY2 opcode.
	 * Expects Caml return address in %rdx.
         */
FUNCTION(G(caml_jit_rt_apply2))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
        movq    1*8(%r14), %rdi /* arg2 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdi, 1*8(%r14) /* arg2 */
        movq    %rdx, 2*8(%r14) /* return address */
        movq    %r12, 3*8(%r14) /* environment */
        movq    %r13, 4*8(%r14) /* extra args */
    /* Record extra arguments */
        movq    $((1 << 1) | 1), %r13
    /* Perform the application */
        jmp     .Lcaml_jit_rt_apply

        /* caml_jit_rt_apply3:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY3 opcode.
	 * Expects Caml return address in %rdx.
         */
FUNCTION(G(caml_jit_rt_apply3))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
        movq    1*8(%r14), %rdi /* arg2 */
        movq    2*8(%r14), %rsi /* arg3 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdi, 1*8(%r14) /* arg2 */
        movq    %rsi, 2*8(%r14) /* arg3 */
        movq    %rdx, 3*8(%r14) /* return address */
        movq    %r12, 4*8(%r14) /* environment */
        movq    %r13, 5*8(%r14) /* extra args */
    /* Record extra arguments */
        movq    $((2 << 1) | 1), %r13
    /* Perform the application */
        jmp     .Lcaml_jit_rt_apply
        
        /* caml_jit_rt_apply:
	 *
	 * Upon entry %rax contains the closure. The function then
	 * checks whether to enlarge the Caml stack and also processes
	 * pending signals. Afterwards it jumps to the closure's code
	 * address.
	 */
FUNCTION(G(caml_jit_rt_apply))
.Lcaml_jit_rt_apply:        
    /* Load environment pointer with closure */
        movq    %rax, %r12
    /* Check if we need to enlarge the Caml stack */
        CMP_VAR(caml_stack_threshold, %r14)
        movq    0*8(%rax), %rdi /* load %rdi with closure address */
        jb      .Lcaml_jit_rt_apply_realloc_stack
.Lcaml_jit_rt_apply_check_signals:
    /* Prefetch the closure byte code */
        prefetcht0 (%rdi)
    /* Check for pending signal and jump to the closure's code address */
        TESTL_VAR($-1, caml_something_to_do)
        jnz     .Lcaml_jit_rt_process_signal
        jmp     .Lcaml_jit_rt_trampoline
.Lcaml_jit_rt_apply_realloc_stack:
    /* Make stack pointer available to C code */
        STORE_VAR(%r14, caml_extern_sp)
    /* Enlarge the stack by (Stack_threshold / sizeof(value)) */
        movq    %rsp, %r14      /* 16-align C stack */
        andq    $-16, %rsp
        movl    $256, %edi
        call    GCALL(caml_realloc_stack)
        movq    %r14, %rsp
    /* Restore stack pointer */
        LOAD_VAR(caml_extern_sp, %r14)
    /* Restore closure in %rax and closure address in %rdi */
        movq    %r12, %rax
        movq    0*8(%r12), %rdi
        jmp     .Lcaml_jit_rt_apply_check_signals


/* Signal processing */

        /* caml_jit_rt_process_signal:
	 *
	 * Process a pending signal and return to the
	 * bytecode address in %rdi.
	 */
FUNCTION(G(caml_jit_rt_process_signal))
.Lcaml_jit_rt_process_signal:
    /* Reset state of caml_something_to_do */
        STOREL_VAR($0, caml_something_to_do)
    /* An event frame must look like accu + a C_CALL frame + a RETURN 1 frame */
        subq    $(6*8), %r14
        movq    %rax, 0*8(%r14)  /* accu */
        movq      $1, 1*8(%r14)  /* C_CALL frame: dummy environment */
        movq      $1, 2*8(%r14)  /* RETURN frame: dummy local 0 (Val_unit) */
        movq    %rdi, 3*8(%r14)  /* RETURN frame: saved return address */
        movq    %r12, 4*8(%r14)  /* RETURN frame: saved environment */
        movq    %r13, 5*8(%r14)  /* RETURN frame: saved extra args */
    /* Make the stack and young pointer available to C code */
        STORE_VAR(%r14, caml_extern_sp)
        STORE_VAR(%r15, caml_young_ptr)
    /* Call into C to process the event */
        movq    %rsp, %rbp      /* 16-align C stack */
        andq    $-16, %rsp
        call    GCALL(caml_process_event)
        movq    %rbp, %rsp
    /* Restore stack and young pointer */
        LOAD_VAR(caml_extern_sp, %r14)
        LOAD_VAR(caml_young_ptr, %r15)
    /* Restore local state from event frame */
        movq    0*8(%r14), %rax  /* accu */
        movq    3*8(%r14), %rdi  /* return address */
        movq    4*8(%r14), %r12  /* environment */
        movq    5*8(%r14), %r13  /* extra args */
        addq    $(6*8), %r14
    /* Return via bytecode address */
        jmp     .Lcaml_jit_rt_trampoline


/* Interaction with C code */

        /* caml_jit_rt_raise:
	 *
	 * Mark the block in %rax as exception result and
	 * throw the exception (either back to C code or
	 * handle it with the active catch code).
         */
FUNCTION(G(caml_jit_rt_raise))        
.Lcaml_jit_rt_raise:
    /* Stash backtrace if enabled (pc in %rsi) */
        TESTL_VAR($-1, caml_backtrace_active)
        jnz     .Lcaml_jit_rt_raise_2
.Lcaml_jit_rt_raise_1:
    /* Check if we need to throw the exception back to C code */
        LOAD_VAR(caml_stack_high, %r14)
        LOAD_VAR(caml_trapsp, %r10)
        subq    1*8(%rsp), %r14
        cmpq    %r14, %r10              /* caml_trapsp >= caml_stack_high - initial_sp_offset */
        jge     .Lcaml_jit_rt_stop_exn
    /* Exception is caught within Caml code */
        movq    0*8(%r10), %rdi         /* Trap_pc(sp) */
        movq    1*8(%r10), %rsi         /* Trap_link(sp) */
        movq    2*8(%r10), %r12         /* env */
        movq    3*8(%r10), %r13         /* extra args */
        leaq    4*8(%r10), %r14         /* sp */
    /* Restore trap stack pointer */
        STORE_VAR(%rsi, caml_trapsp)
        jmp     .Lcaml_jit_rt_start_enter
.Lcaml_jit_rt_raise_2:
        movq    %rax, %rbx              /* preserve %rax */
        movq    %rax, %rdi
        movq    %r14, %rdx
        call    GCALL(caml_stash_backtrace)
        movq    %rbx, %rax              /* restore %rax */
        jmp     .Lcaml_jit_rt_raise_1

        /* caml_jit_rt_start:
	 *
	 * Entry point into the JIT runtime. Invoked from C via
	 *
	 *  caml_jit_rt_start(code, accu, Val_int(extra_args), env, sp)
	 *
	 * hence the initial register assignment is:
	 *
         *  %rdi - byte code address
         *  %rsi - accu
         *  %rdx - extra arguments
         *  %rcx - environment
         *  %r8  - stack pointer
	 *
	 * C stack layout during execution:
	 *
	 *  0*8(%rsp) - saved_pc (for backtrace)
	 *  1*8(%rsp) - initial sp offset
	 *  2*8(%rsp) - initial local roots
	 *  3*8(%rsp) - initial external raise
	 *  4*8(%rsp) - start of sigjmp buffer (SIZEOF_SIGJMPBUF bytes + padding)
	 */
FUNCTION(G(caml_jit_rt_start))
    /* Push callee-save registers */
        pushq   %rbp
        pushq   %rbx
        pushq   %r12
        pushq   %r13
        pushq   %r14
        pushq   %r15
    /* Load initial caml_external_raise, caml_local_roots and stack high */
        LOAD_VAR(caml_external_raise, %r13)
        LOAD_VAR(caml_local_roots, %r14)
        LOAD_VAR(caml_stack_high, %r15)
    /* Record callback invocation */
        INCL_VAR(caml_callback_depth)
    /* Calculate initial stack pointer offset */
        subq    %r8, %r15
    /* Reserve C stack space for:
     *
     * - sigjmp_buf             (SIZEOF_SIGJMPBUF byte)
     * - initial external raise (8 byte)
     * - initial local roots    (8 byte)
     * - initial sp offset      (8 byte)
     * - saved_pc               (8 byte)
     *
     * Round everything to an odd number of longs, so C stack
     * is aligned on a 16-byte boundary afterwards.
     */
        subq    $(((SIZEOF_SIGJMPBUF + 15) / 16 + 2) * 16 + 8), %rsp
    /* Save initial sp offset, local roots and external raise */
        movq    %r15, 1*8(%rsp) /* initial sp offset */
        movq    %r14, 2*8(%rsp) /* initial local roots */
        movq    %r13, 3*8(%rsp) /* initial external raise */
    /* Setup machine state */
        movq    %rsi, %rax      /* accu */
        movq    %r8,  %r14      /* stack pointer */
        movq    %rdx, %r13      /* extra arguments */
        movq    %rcx, %r12      /* environment */
    /* Load young pointer */
        LOAD_VAR(caml_young_ptr, %r15)
.Lcaml_jit_rt_start_enter:
    /* Preserve accu in %rbx and code pointer on C stack */
        movq    %rax, %rbx
        movq    %rdi, 0*8(%rsp)
    /* Setup the sigjmp buffer */
        leaq    4*8(%rsp), %rdi /* sigjmp_buf */
        xorl    %esi, %esi
#if defined(HAS_SIGSETJMP)
        call    GCALL(sigsetjmp)
#elif defined(HAS___SIGSETJMP)
        call    GCALL(__sigsetjmp)
#else
#error "Neither sigsetjmp() nor __sigsetjmp() found."
#endif
        testl   %eax, %eax
    /* Check if we caught an exception from C code */
        jnz     .Lcaml_jit_rt_start_caught
    /* Restore accu and code pointer */
        movq    0*8(%rsp), %rdi
        movq    %rbx, %rax
    /* Reset saved_pc to NULL */
        movq    $0, 0*8(%rsp)
    /* Setup the new external raise buffer */
        leaq    4*8(%rsp), %r10
        STORE_VAR(%r10, caml_external_raise)
    /* Jump to the code */
        jmp     .Lcaml_jit_rt_trampoline
.Lcaml_jit_rt_start_caught:
    /* Load %rsi with the saved pc */
        movq    0*8(%rsp), %rsi
    /* Reload stack and young pointer */
        LOAD_VAR(caml_extern_sp, %r14)
        LOAD_VAR(caml_young_ptr, %r15)
    /* Restore the local roots to its initial state */
        movq    2*8(%rsp), %r10
        STORE_VAR(%r10, caml_local_roots)
    /* Load the exception object */
        LOAD_VAR(caml_exn_bucket, %rax)
        jmp     .Lcaml_jit_rt_raise

        /* caml_jit_rt_stop:
	 *
	 * Return to the calling C code with the result
	 * in %rax.
	 */
.Lcaml_jit_rt_stop_exn:
    /* Mark %rax as exception result */
        orb     $2, %al
FUNCTION(G(caml_jit_rt_stop))
.Lcaml_jit_rt_stop:        
    /* Make stack and young pointer available to C code */
        STORE_VAR(%r14, caml_extern_sp)
        STORE_VAR(%r15, caml_young_ptr)
    /* Restore caml_external_raise and caml_local_roots to its initial state */
        movq    2*8(%rsp), %r8          /* initial local roots */
        movq    3*8(%rsp), %r9          /* initial external raise */
        STORE_VAR(%r8, caml_local_roots)
        STORE_VAR(%r9, caml_external_raise)
    /* Decrement callback counter */
        DECL_VAR(caml_callback_depth)
    /* Pop callee-save registers */
        addq    $(((SIZEOF_SIGJMPBUF + 15) / 16 + 2) * 16 + 8), %rsp
        popq    %r15
        popq    %r14
        popq    %r13
        popq    %r12
        popq    %rbx
        popq    %rbp
        ret


/* Floating point operations */

FUNCTION(G(caml_jit_rt_copy_double))
.Lcaml_jit_rt_copy_double:
        call    .Lcaml_jit_rt_alloc1
        movq    $((1 << 10) | (3 << 8) | 256), 0*8(%r15)
        movlpd  %xmm0, 1*8(%r15)
        leaq    1*8(%r15), %rax
        ret

        /* caml_jit_rt_add_float:
	 *
	 * Adds the floating point values in (%rax) and
	 * -1*8(%r14) and stores the result into a newly
	 * allocated float block. Upon return %rax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_add_float))
        movq    -1*8(%r14), %rsi
        movlpd  (%rax), %xmm0
        addsd   (%rsi), %xmm0
        jmp     .Lcaml_jit_rt_copy_double

        /* caml_jit_rt_sub_float:
	 *
	 * Subtracts the floating point values in (%rax) and
	 * -1*8(%r14) and stores the result into a newly
	 * allocated float block. Upon return %rax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_sub_float))
        movq    -1*8(%r14), %rsi
        movlpd  (%rax), %xmm0
        subsd   (%rsi), %xmm0
        jmp     .Lcaml_jit_rt_copy_double

        /* caml_jit_rt_mul_float:
	 *
	 * Multiplies the floating point values in (%rax) and
	 * -1*8(%r14) and stores the result into a newly
	 * allocated float block. Upon return %rax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_mul_float))
        movq    -1*8(%r14), %rsi
        movlpd  (%rax), %xmm0
        mulsd   (%rsi), %xmm0
        jmp     .Lcaml_jit_rt_copy_double

        /* caml_jit_rt_div_float:
	 *
	 * Divides the floating point values in (%rax) and
	 * -1*8(%r14) and stores the result into a newly
	 * allocated float block. Upon return %rax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_div_float))
        movq    -1*8(%r14), %rsi
        movlpd  (%rax), %xmm0
        divsd   (%rsi), %xmm0
        jmp     .Lcaml_jit_rt_copy_double


/* Copying operations */

        /* caml_jit_rt_copy_floats:
	 *
	 * Copies %rcx double values from the stack %r14 to
	 * the double array at %rdx. %rcx is assumed to be
	 * non-zero.
	 *
	 * Upon return, %r14 will be incremented by %rcx*8.
	 *
	 * Clobbers %rcx, %rdx, %r11 and %xmm0.
	 */
FUNCTION(G(caml_jit_rt_copy_floats))
.Lcaml_jit_rt_copy_floats_1:
        subl    $1, %ecx
        movq    0*8(%r14), %r11
        movlpd  0*8(%r11), %xmm0
        movlpd  %xmm0, 0*8(%rdx)
        leaq    1*8(%r14), %r14
        leaq    1*8(%rdx), %rdx
        jnz     .Lcaml_jit_rt_copy_floats_1
        ret


/* Instruction tracing */
        
#ifdef DEBUG
        /* caml_jit_rt_trace:
	 *
	 * Expects instruction opcode in %rdi and byte-code
	 * pc in %rsi. Invokes caml_jit_trace() with the
	 * appropriate parameters, preserving the current state
	 * except for %rdi and %rsi.
	 */
FUNCTION(G(caml_jit_rt_trace))
    /* Push preserved registers */
        pushq   %rax
        pushq   %rcx
        pushq   %rdx
        pushq   %r8
        pushq   %r9
        pushq   %r10
        pushq   %r11
    /* Save floating-point registers */
        subq    $(16*8), %rsp
        movlpd  %xmm0, 0*8(%rsp)
        movlpd  %xmm1, 1*8(%rsp)
        movlpd  %xmm2, 2*8(%rsp)
        movlpd  %xmm3, 3*8(%rsp)
        movlpd  %xmm4, 4*8(%rsp)
        movlpd  %xmm5, 5*8(%rsp)
        movlpd  %xmm6, 6*8(%rsp)
        movlpd  %xmm7, 7*8(%rsp)
        movlpd  %xmm8, 8*8(%rsp)
        movlpd  %xmm9, 9*8(%rsp)
        movlpd  %xmm10, 10*8(%rsp)
        movlpd  %xmm11, 11*8(%rsp)
        movlpd  %xmm12, 12*8(%rsp)
        movlpd  %xmm13, 13*8(%rsp)
        movlpd  %xmm14, 14*8(%rsp)
        movlpd  %xmm15, 15*8(%rsp)
    /* Make stack and young ptr available to C code */
        STORE_VAR(%r14, caml_extern_sp)
        STORE_VAR(%r15, caml_young_ptr)
    /* Setup the trace parameters */
        movq    %rax, %rdx      /* accu */
        movq    %r13, %rcx      /* extra args */
        movq    %r12, %r8       /* env */
        movq    %r14, %r9       /* sp */
    /* Invoke the trace function */
        pushq   %rbp
        movq    %rsp, %rbp      /* 16-align C stack */
        andq    $-16, %rsp
        call    G(caml_jit_trace)
        movq    %rbp, %rsp
        popq    %rbp
    /* Restore floating-point registers */
        movlpd  15*8(%rsp), %xmm15
        movlpd  14*8(%rsp), %xmm14
        movlpd  13*8(%rsp), %xmm13
        movlpd  12*8(%rsp), %xmm12
        movlpd  11*8(%rsp), %xmm11
        movlpd  10*8(%rsp), %xmm10
        movlpd  9*8(%rsp), %xmm9
        movlpd  8*8(%rsp), %xmm8
        movlpd  7*8(%rsp), %xmm7
        movlpd  6*8(%rsp), %xmm6
        movlpd  5*8(%rsp), %xmm5
        movlpd  4*8(%rsp), %xmm4
        movlpd  3*8(%rsp), %xmm3
        movlpd  2*8(%rsp), %xmm2
        movlpd  1*8(%rsp), %xmm1
        movlpd  0*8(%rsp), %xmm0
        addq    $(16*8), %rsp
    /* Restore preserved registers */
        popq    %r11
        popq    %r10
        popq    %r9
        popq    %r8
        popq    %rdx
        popq    %rcx
        popq    %rax
        ret
#endif
