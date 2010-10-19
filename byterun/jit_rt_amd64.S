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
	 * return %rbx points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc1_gc:
        addq    $(2*8), %rbx
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc1))
.Lcaml_jit_rt_alloc1:
        subq    $(2*8), %rbx
        cmpq    %rbp, %rbx
        jb      .Lcaml_jit_rt_alloc1_gc
        ret

        /* caml_jit_rt_alloc2:
	 *
	 * Allocates a block of wosize 2 in the minor heap. Upon
	 * return %rbx points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc2_gc:
        addq    $(3*8), %rbx
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc2))
.Lcaml_jit_rt_alloc2:
        subq    $(3*8), %rbx
        cmpq    %rbp, %rbx
        jb      .Lcaml_jit_rt_alloc2_gc
        ret

        /* caml_jit_rt_alloc3:
	 *
	 * Allocates a block of wosize 3 in the minor heap. Upon
	 * return %rbx points to the header of the newly allocated
	 * block.
	 */
.Lcaml_jit_rt_alloc3_gc:
        addq    $(4*8), %rbx
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_alloc3))
.Lcaml_jit_rt_alloc3:
        subq    $(4*8), %rbx
        cmpq    %rbp, %rbx
        jb      .Lcaml_jit_rt_alloc3_gc
        ret

        /* caml_jit_rt_allocN:
	 *
	 * Allocates a block of bhsize %r11 in the minor heap.
	 * Upon return %rbx points to the header of the newly
	 * allocated block. %r11 is preserved across calls.
	 */
.Lcaml_jit_rt_allocN_gc:        
        pushq   %r11                    /* preserve desired size */
        addq    %r11, %rbx
        call    .Lcaml_jit_rt_call_gc
        popq    %r11                    /* recover desired size */
FUNCTION(G(caml_jit_rt_allocN))
.Lcaml_jit_rt_allocN:
        subq    %r11, %rbx
        cmpq    %rbp, %rbx
        jb      .Lcaml_jit_rt_allocN_gc
        ret

        /* .Lcaml_jit_rt_call_gc:
	 *
	 * Invoked by the caml_jit_rt_allocX functions above when
	 * the minor heap runs out of space.
	 *
	 * Preserves %rdi, %rsi.
         */
.Lcaml_jit_rt_call_gc:
    /* Push preserved registers */
        pushq   %rdi
        pushq   %rsi
    /* Push environment and acc onto the Caml stack */
        movq    %r12, -1*8(%r14)
        movq    %rax, -2*8(%r14)
    /* Make the young and stack pointers available to the C code */
        STORE_VAR(%rbx, caml_young_ptr)
        leaq    -2*8(%r14), %r10
        STORE_VAR(%r10, caml_extern_sp)
    /* Invoke the garbage collector */
        movq    %rsp, %rbp      /* 16-align C stack */
        andq    $-16, %rsp
        call    GCALL(caml_minor_collection)
        movq    %rbp, %rsp
    /* Restore young limit and pointer */
        LOAD_VAR(caml_young_limit, %rbp)
        LOAD_VAR(caml_young_ptr, %rbx)
    /* Restore environment and accu from the Caml stack */
        movq    -1*8(%r14), %r12
        movq    -2*8(%r14), %rax
    /* Restore preserved registers */
        popq    %rsi
        popq    %rdi
        ret

        /* caml_jit_rt_makeblock1:
	 *
	 * Allocates and initializes a 1-item block in the
	 * minor heap. Upon entry %esi contains the tag for
	 * the block and %rax contains the element to store
	 * into the block. Upon return %rax contains the
	 * block pointer.
         */
FUNCTION(G(caml_jit_rt_makeblock1))
    /* Generate the block header */
        orl     $((1 << 10) | (3 << 8)), %esi   /* Make_header(1, %esi, Caml_black) */
    /* Allocate space in the minor heap */
        call    .Lcaml_jit_rt_alloc1
    /* Initialize the block */
        movq    %rsi, 0*8(%rbx)
        movq    %rax, 1*8(%rbx)
        leaq    1*8(%rbx), %rax
        ret

        /* caml_jit_rt_makeblock2:
	 *
	 * Allocates and initializes a 2-item block in the
	 * minor heap. Upon entry %esi contains the tag for
	 * the block, %rax contains the first element and
	 * 0*8(%r14) contains the second element to store
	 * into the block. Upon return %rax contains the
	 * block pointer and %r14 is incremented by 1*8.
         */
FUNCTION(G(caml_jit_rt_makeblock2))
    /* Generate the block header */
        orl     $((2 << 10) | (3 << 8)), %esi   /* Make_header(2, %esi, Caml_black) */
    /* Allocate space in the minor heap */
        call    .Lcaml_jit_rt_alloc2
    /* Fetch elements */
        movq    0*8(%r14), %rcx
    /* Initialize the block */
        movq    %rsi, 0*8(%rbx)
        movq    %rax, 1*8(%rbx)
        movq    %rcx, 2*8(%rbx)
        addq    $(1*8), %r14
        leaq    1*8(%rbx), %rax
        ret

        /* caml_jit_rt_makeblock3:
	 *
	 * Allocates and initializes a 3-item block in the
	 * minor heap. Upon entry %esi contains the tag for
	 * the block, %rax contains the first, 0*8(%r14)
	 * contains the second and 1*8(%r14) contains the
	 * third element to store into the block. Upon
	 * return %rax contains the block pointer and %r14
	 * is incremented by 2*8.
	 */
FUNCTION(G(caml_jit_rt_makeblock3))
    /* Generate the block header */
        orl     $((3 << 10) | (3 << 8)), %esi   /* Make_header(3, %esi, Caml_black) */
    /* Allocate space in the minor heap */
        call    .Lcaml_jit_rt_alloc3
    /* Fetch elements */
        movq    0*8(%r14), %rcx
        movq    1*8(%r14), %rdx
    /* Initialize the block */
        movq    %rsi, 0*8(%rbx)
        movq    %rax, 1*8(%rbx)
        movq    %rcx, 2*8(%rbx)
        movq    %rdx, 3*8(%rbx)
        addq    $(2*8), %r14
        leaq    1*8(%rbx), %rax
        ret

        /* caml_jit_rt_makeblockN:
	 *
	 * Allocates and initializes a %rdi-item block in the
	 * minor heap. Upon entry %esi contains the tag for the
	 * block, %rdi contains the wosize, %rax contains the
	 * first element and the Caml stack holds the remaining
	 * elements.
	 *
	 * Upon return %rax contains the block pointer and %r14
	 * is incremented by (%rdi-1)*8.
	 */
FUNCTION(G(caml_jit_rt_makeblockN))
    /* Allocate space in the minor heap */
        leaq    8(, %rdi, 8), %r11
        call    .Lcaml_jit_rt_allocN
    /* Calculate the number of stack arguments */
        leaq    -1(%rdi), %rcx
    /* Generate the block header */
        shlq    $10, %rdi
        orl     $(3 << 8), %esi
        orq     %rdi, %rsi
    /* Initialize the block */
        cld
        leaq    2*8(%rbx), %rdi
        movq    %rsi, 0*8(%rbx)
        movq    %r14, %rsi
        movq    %rax, 1*8(%rbx)
        rep movsq
        leaq    1*8(%rbx), %rax
        movq    %rsi, %r14
        ret

        /* caml_jit_rt_makefloatblock:
	 *
	 * Allocates a new float block with Double_array_tag. Upon
	 * entry %rdi contains the desired wosize of the new float
	 * block, %rax contains the pointer to the first float value
	 * and the Caml stack contains the pointers to the remaining
	 * values.
	 *
	 * Upon return %rax contains the block pointer to the newly
	 * allocated block and %r14 is incremented by wosize * 8.
	 *
	 * Note that this implementation differs slightly from the
	 * one found in the interpreter in that it attempts to allocate
	 * the block in the minor heap without looking at the wosize
	 * first and only falls back to major heap allocation if the
	 * block would not fit into the minor heap.
	 */
.Lcaml_jit_rt_makefloatblock_gc:
    /* Fallback to major heap allocation if %rcx > Max_young_wosize */
        cmpq    $256, %rcx
        leaq    (%rbx, %r11, 1), %rbx   /* reset young ptr */
        jg      .Lcaml_jit_rt_makefloatblock_shr
        call    .Lcaml_jit_rt_call_gc
FUNCTION(G(caml_jit_rt_makefloatblock))
    /* Allocate in the minor heap */
        leaq    8(, %rdi, 8), %r11
        subq    %r11, %rbx
        cmpq    %rbp, %rbx
    /* Setup for block initializing */
        movq    %rdi, %rcx      /* wosize */
        movlpd  (%rax), %xmm0   /* 1st double value */
        leaq    1*8(%rbx), %rdx /* block pointer */
    /* Check if minor heap exhausted */
        jl      .Lcaml_jit_rt_makefloatblock_gc
    /* Generate the block header */
        shlq    $10, %rdi
        orq     $((3 << 8) | 254), %rdi
    /* Initialize the float block */
        movq    %rdx, %rax
        movq    %rdi, 0*8(%rbx)
        jmp     .Lcaml_jit_rt_makefloatblock_loop_1
        .p2align 3
.Lcaml_jit_rt_makefloatblock_loop:
        movq    0*8(%r14), %r11
        leaq    1*8(%rdx), %rdx
        movlpd  (%r11), %xmm0
        leaq    1*8(%r14), %r14
.Lcaml_jit_rt_makefloatblock_loop_1:
        subq    $1, %rcx
        movlpd  %xmm0, 0*8(%rdx)
        jnz     .Lcaml_jit_rt_makefloatblock_loop
        ret
    /* Allocate the float block in the major heap */
.Lcaml_jit_rt_makefloatblock_shr:
        subq    $(3*8), %rsp
        movq    %rcx,  0*8(%rsp)
        movq    %rbp,  1*8(%rsp)
        movlpd  %xmm0, 2*8(%rsp)
        movl    $254, %esi
        movq    %rsp, %rbp
        andq    $-16, %rsp
        call    GCALL(caml_alloc_shr)
        movq    %rbp, %rsp
        movq    0*8(%rsp), %rcx
        movq    1*8(%rsp), %rbp
        movlpd  2*8(%rsp), %xmm0
        addq    $(3*8), %rsp
        movq    %rax, %rdx
        jmp     .Lcaml_jit_rt_makefloatblock_loop_1
        

        /* caml_jit_rt_makeblock_shr:
	 *
	 * Allocates a shared block in the major heap. Upon entry
	 * %rdi contains the wosize of the new block and %rsi
	 * contains the tag. Upon return, %rax contains the pointer
	 * to the newly allocated block.
         *
	 * The first block field is initialized with the value in
	 * %rax, the remaining fields are initialized with values
	 * popped off the stack.
	 */
FUNCTION(G(caml_jit_rt_makeshrblock))
    /* Align C stack on 16-byte boundary */
        pushq   %r12
        pushq   %r13
        pushq   %rbp
        pushq   %rbx
        movq    %rsp, %rbp
        andq    $-16, %rsp
    /* Push %rax onto the Caml stack */
        subq    $(1*8), %r14
        movq    %rax, 0*8(%r14)
    /* Allocate a block in the major heap */
        movq    %rdi, %r13      /* preserve wosize in %r13 */
        call    GCALL(caml_alloc_shr)
        movq    %rax, %rbx      /* save block pointer in %rbx */
        movq    %rax, %r12
        .p2align 3
.Lcaml_jit_rt_makeshrblock_regular_loop:
        movq    %r12, %rdi
        movq    0*8(%r14), %rsi
        call    GCALL(caml_initialize)
        subq    $1, %r13
        leaq    1*8(%r14), %r14
        leaq    1*8(%r12), %r12
        jnz     .Lcaml_jit_rt_makeshrblock_regular_loop
.Lcaml_jit_rt_makeshrblock_done:
        movq    %rbx, %rax
        movq    %rbp, %rsp
        popq    %rbx
        popq    %rbp
        popq    %r13
        popq    %r12
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
        leaq    8(, %r13, 8), %r11
        call    .Lcaml_jit_rt_allocN
    /* Setup for initialization below */
        cld
        leaq    -2(%r13), %rcx
        leaq    3*8(%rbx), %rdi
        movq    %r14, %rsi
    /* Store the closure header (%r13 << 10 | Closure_tag | Caml_black) */
        shlq    $10, %r13
        orq     $1015, %r13
        movq    %r13, 0*8(%rbx)
    /* Store the restart function pointer */
        movq    %rax, 1*8(%rbx)
    /* Store the environment pointer */
        movq    %r12, 2*8(%rbx)
    /* Move the arguments from the stack to the closure */
        rep movsq
    /* Restore stack pointer */
        movq    %rsi, %r14
    /* Return the closure as result */
        leaq    1*8(%rbx), %rax
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

        .p2align 3
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
        cld
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
	 * Expects Caml return address in %rdi.
         */
FUNCTION(G(caml_jit_rt_apply1))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdi, 1*8(%r14) /* return address */
        movq    %r12, 2*8(%r14) /* environment */
        movq    %r13, 3*8(%r14) /* extra args */
    /* Record extra arguments */
        movq    $((0 << 1) | 1), %r13
    /* Perform the application */
        jmp     .Lcaml_jit_rt_apply

        /* caml_jit_rt_apply2:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY2 opcode.
	 * Expects Caml return address in %rdi.
         */
FUNCTION(G(caml_jit_rt_apply2))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
        movq    1*8(%r14), %rdx /* arg2 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdx, 1*8(%r14) /* arg2 */
        movq    %rdi, 2*8(%r14) /* return address */
        movq    %r12, 3*8(%r14) /* environment */
        movq    %r13, 4*8(%r14) /* extra args */
    /* Record extra arguments */
        movq    $((1 << 1) | 1), %r13
    /* Perform the application */
        jmp     .Lcaml_jit_rt_apply

        /* caml_jit_rt_apply3:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY3 opcode.
	 * Expects Caml return address in %rdi.
         */
FUNCTION(G(caml_jit_rt_apply3))
    /* Save arguments */
        movq    0*8(%r14), %rcx /* arg1 */
        movq    1*8(%r14), %rdx /* arg2 */
        movq    2*8(%r14), %rsi /* arg3 */
    /* Prepare stack frame */
        subq    $(3*8), %r14
        movq    %rcx, 0*8(%r14) /* arg1 */
        movq    %rdx, 1*8(%r14) /* arg2 */
        movq    %rsi, 2*8(%r14) /* arg3 */
        movq    %rdi, 3*8(%r14) /* return address */
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
        cmpq    %r15, %r14
        movq    0*8(%rax), %rdi /* load %rdi with closure address */
        jb      .Lcaml_jit_rt_apply_realloc_stack
.Lcaml_jit_rt_apply_check_signals:
    /* Prefetch the closure byte code */
        prefetcht0 (%rdi)
    /* Check for pending signal and jump to the closure's code address */
        TESTL_VAR($0xffffffff, caml_something_to_do)
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
    /* Restore stack pointer and threshold */
        LOAD_VAR(caml_extern_sp, %r14)
        LOAD_VAR(caml_stack_threshold, %r15)
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
    /* Make the young and stack pointers available to C code */
        STORE_VAR(%rbx, caml_young_ptr)
        STORE_VAR(%r14, caml_extern_sp)
    /* Call into C to process the event */
        movq    %rsp, %rbp      /* 16-align C stack */
        andq    $-16, %rsp
        call    GCALL(caml_process_event)
        movq    %rbp, %rsp
    /* Restore stack pointer and threshold */
        LOAD_VAR(caml_extern_sp, %r14)
        LOAD_VAR(caml_stack_threshold, %r15)
    /* Restore young limit and pointer */
        LOAD_VAR(caml_young_limit, %rbp)
        LOAD_VAR(caml_young_ptr, %rbx)
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
        jz      .Lcaml_jit_rt_raise1
        movq    %rax, 0*8(%rsp)         /* preserve %rax */
        movq    %rax, %rdi
        movq    %r14, %rdx
        call    GCALL(caml_stash_backtrace)
        movq    0*8(%rsp), %rax         /* restore %rax */
.Lcaml_jit_rt_raise1:        
    /* Check if we need to throw the exception back to C code */
        LOAD_VAR(caml_stack_high, %r14)
        LOAD_VAR(caml_trapsp, %r10)
        subq    2*8(%rsp), %r14
        cmpq    %r14, %r10              /* caml_trapsp >= caml_stack_high - initial_sp_offset */
        jge     .Lcaml_jit_rt_stop_exn
.Lcaml_jit_rt_raise2:        
    /* Exception is caught within Caml code */
        movq    0*8(%r10), %rdi         /* Trap_pc(sp) */
        movq    1*8(%r10), %rsi         /* Trap_link(sp) */
        movq    2*8(%r10), %r12         /* env */
        movq    3*8(%r10), %r13         /* extra args */
        leaq    4*8(%r10), %r14         /* sp */
    /* Restore trap stack pointer */
        STORE_VAR(%rsi, caml_trapsp)
        jmp     .Lcaml_jit_rt_start_enter

        /* caml_jit_rt_start:
	 *
	 * Entry point into the JIT runtime. Invoked from C via
	 *
	 *  caml_jit_rt_start(code, accu, Val_int(extra_args), env, sp)
	 *
	 * hence the initial register assignment ist:
	 *
         *  %rdi - byte code address
         *  %rsi - accu
         *  %rdx - extra arguments
         *  %rcx - environment
         *  %r8  - stack pointer
	 *
	 * C stack layout during execution:
	 *
	 *  0*8(%rsp) - saved accu (for compile trampoline)
	 *  1*8(%rsp) - saved_pc (for backtrace)
	 *  2*8(%rsp) - initial sp offset
	 *  3*8(%rsp) - initial local roots
	 *  4*8(%rsp) - initial external raise
	 *  5*8(%rsp) - start of sigjmp buffer (SIZEOF_SIGJMPBUF bytes + padding)
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
     * - saved accu             (8 byte)
     *
     * Round everything to an odd number of longs, so C stack
     * is aligned on a 16-byte boundary afterwards.
     */
        subq    $((((SIZEOF_SIGJMPBUF + 7) / 8 + 5) | 1) * 8), %rsp
    /* Save initial sp offset, local roots and external raise */
        movq    %r15, 2*8(%rsp) /* initial sp offset */
        movq    %r14, 3*8(%rsp) /* initial local roots */
        movq    %r13, 4*8(%rsp) /* initial external raise */
    /* Setup machine state */
        movq    %rsi, %rax      /* accu */
        movq    %r8,  %r14      /* stack pointer */
        movq    %rdx, %r13      /* extra arguments */
        movq    %rcx, %r12      /* environment */
    /* Load young limit/pointer and stack threshold */
        LOAD_VAR(caml_young_limit, %rbp)
        LOAD_VAR(caml_young_ptr, %rbx)
        LOAD_VAR(caml_stack_threshold, %r15)
.Lcaml_jit_rt_start_enter:
    /* Preserve accu and code pointer on C stack */
        movq    %rax, 0*8(%rsp)
        movq    %rdi, 1*8(%rsp)
    /* Setup the sigjmp buffer */
        leaq    5*8(%rsp), %rdi /* sigjmp_buf */
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
        movq    0*8(%rsp), %rax
        movq    1*8(%rsp), %rdi
    /* Reset saved_pc */
        movq    $0, 1*8(%rsp)
    /* Setup the new external raise buffer */
        leaq    5*8(%rsp), %r10
        STORE_VAR(%r10, caml_external_raise)
    /* Jump to the code */
        jmp     .Lcaml_jit_rt_trampoline
.Lcaml_jit_rt_start_caught:
    /* Load %rsi with the saved pc */
        movq    1*8(%rsp), %rsi
    /* Reload young limit/pointer and stack pointer/threshold */
        LOAD_VAR(caml_young_limit, %rbp)
        LOAD_VAR(caml_young_ptr, %rbx)
        LOAD_VAR(caml_extern_sp, %r14)
        LOAD_VAR(caml_stack_threshold, %r15)
    /* Restore the local roots to its initial state */
        movq    3*8(%rsp), %r10
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
    /* Make young and stack pointer available to C code */
        STORE_VAR(%rbx, caml_young_ptr)
        STORE_VAR(%r14, caml_extern_sp)
    /* Restore caml_external_raise and caml_local_roots to its initial state */
        movq    3*8(%rsp), %r8          /* initial local roots */
        movq    4*8(%rsp), %r9          /* initial external raise */
        STORE_VAR(%r8, caml_local_roots)
        STORE_VAR(%r9, caml_external_raise)
    /* Decrement callback counter */
        DECL_VAR(caml_callback_depth)
    /* Pop callee-save registers */
        addq    $((((SIZEOF_SIGJMPBUF + 7) / 8 + 5) | 1) * 8), %rsp
        popq    %r15
        popq    %r14
        popq    %r13
        popq    %r12
        popq    %rbx
        popq    %rbp
        ret

