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

/* Asm part of the JIT runtime system, Intel 386 processor */
/* Must be preprocessed by cpp */

#include "../config/m.h"
#include "../config/s.h"

/* Linux/BSD with ELF binaries and Solaris do not prefix identifiers with _.
   Linux/BSD with a.out binaries and NextStep do. */
        
#if defined(SYS_solaris)
#define CONCAT(a,b) a/**/b
#else
#define CONCAT(a,b) a##b
#endif

#if defined(SYS_linux_elf) || defined(SYS_bsd_elf) || defined(SYS_solaris) || defined(SYS_beos) || defined(SYS_gnu)
#define G(x) x
#define L(x) CONCAT(.L,x)
#else
#define G(x) CONCAT(_,x)
#define L(x) CONCAT(L,x)
#endif
        
#ifdef SYS_macosx

#define FUNCTION(name) \
        .globl name; \
        .private_extern name; \
        .p2align 4; \
        name:

#else

#define FUNCTION(name) \
        .globl name; \
        .type name,@function; \
        .hidden name; \
        .p2align 4; \
        name:

#endif


        .text

/* Allocation */

        /* caml_jit_rt_alloc1:
	 *
	 * Allocates a block of wosize 1 in the minor heap. Upon
	 * return %edi points to the header of the newly allocated
	 * block.
	 */
L(caml_jit_rt_alloc1_gc):
        addl    $(2*4), %edi
        call    L(caml_jit_rt_call_gc)
FUNCTION(G(caml_jit_rt_alloc1))
L(caml_jit_rt_alloc1):
        subl    $(2*4), %edi
        cmpl    G(caml_young_limit), %edi
        jb      L(caml_jit_rt_alloc1_gc)
        ret

        /* caml_jit_rt_alloc2:
	 *
	 * Allocates a block of wosize 2 in the minor heap. Upon
	 * return %edi points to the header of the newly allocated
	 * block.
	 */
L(caml_jit_rt_alloc2_gc):
        addl    $(3*4), %edi
        call    L(caml_jit_rt_call_gc)
FUNCTION(G(caml_jit_rt_alloc2))
L(caml_jit_rt_alloc2):
        subl    $(3*4), %edi
        cmpl    G(caml_young_limit), %edi
        jb      L(caml_jit_rt_alloc2_gc)
        ret

        /* caml_jit_rt_alloc3:
	 *
	 * Allocates a block of wosize 3 in the minor heap. Upon
	 * return %edi points to the header of the newly allocated
	 * block.
	 */
L(caml_jit_rt_alloc3_gc):
        addl    $(4*4), %edi
        call    L(caml_jit_rt_call_gc)
FUNCTION(G(caml_jit_rt_alloc3))
L(caml_jit_rt_alloc3):
        subl    $(4*4), %edi
        cmpl    G(caml_young_limit), %edi
        jb      L(caml_jit_rt_alloc3_gc)
        ret

        /* caml_jit_rt_allocN:
	 *
	 * Allocates a block of bhsize %edx in the minor heap.
	 * Upon return %edi points to the header of the newly
	 * allocated block.
	 */
L(caml_jit_rt_allocN_gc):
        pushl   %edx                    /* preserve desired size */
        addl    %edx, %edi
        call    L(caml_jit_rt_call_gc)
        popl    %edx                    /* recover desired size */
FUNCTION(G(caml_jit_rt_allocN))
L(caml_jit_rt_allocN):
        subl    %edx, %edi
        cmpl    G(caml_young_limit), %edi
        jb      L(caml_jit_rt_allocN_gc)
        ret

        /* caml_jit_rt_call_gc:
	 *
	 * Invoked by the caml_jit_rt_allocX functions above when
	 * the minor heap runs out of space.
	 *
	 * Preserves %xmm0.
         */
FUNCTION(G(caml_jit_rt_call_gc))
L(caml_jit_rt_call_gc):
    /* Push preserved registers */
        subl    $(1*8), %esp
        movsd   %xmm0, 0*8(%esp)
    /* Push environment and acc onto the Caml stack */
        movl    %ebp, -1*4(%esi)
        movl    %eax, -2*4(%esi)
    /* Make the young and stack pointers available to the C code */
        movl    %edi, G(caml_young_ptr)
        leal    -2*4(%esi), %ecx
        movl    %ecx, G(caml_extern_sp)
    /* Invoke the garbage collector */
        call    G(caml_minor_collection)
    /* Restore young pointer */
        movl    G(caml_young_ptr), %edi
    /* Restore environment and accu from the Caml stack */
        movl    -1*4(%esi), %ebp
        movl    -2*4(%esi), %eax
    /* Restore preserved registers */
        movsd   0*8(%esp), %xmm0
        addl    $(1*8), %esp
        ret


/* Function return */

        /* caml_jit_rt_grab_closure_and_return:
	 *
	 * Upon entry, %eax contains the restart byte code address.
         */
FUNCTION(G(caml_jit_rt_grab_closure_and_return))
    /* Determine the wosize from the extra arguments */
        shrl    $1, %ebx
        addl    $3, %ebx
    /* Allocate space in the minor heap */
        leal    4(, %ebx, 4), %edx
        call    L(caml_jit_rt_allocN)
    /* Setup for initialization below */
        leal    -2(%ebx), %ecx
        pushl   %edi
    /* Store the closure header (%ebx << 10 | Closure_tag | Caml_black) */
        shll    $10, %ebx
        orl     $((3 << 8) | 247), %ebx
        movl    %ebx, 0*4(%edi)
    /* Store the restart function pointer */
        movl    %eax, 1*4(%edi)
    /* Store the environment pointer */
        movl    %ebp, 2*4(%edi)
    /* Move the arguments from the stack to the closure */
        addl    $(3*4), %edi
        rep movsl
    /* Restore young pointer */
        popl    %edi
    /* Return the closure as result */
        leal    1*4(%edi), %eax
        jmp     L(caml_jit_rt_return_to_caller)

        /* caml_jit_rt_return:
	 *
	 * Returns to caller or tail calls to result, depending on
	 * the number of extra arguments present.
         */
FUNCTION(G(caml_jit_rt_return))
    /* Check if we have extra args left */
        cmpl    $1, %ebx
        jbe     L(caml_jit_rt_return_to_caller)
    /* Decrement extra args and tail call to result */
        movl    0*4(%eax), %ecx
        addl    $-2, %ebx
        movl    %eax, %ebp
        jmp     L(caml_jit_rt_trampoline)
L(caml_jit_rt_return_to_caller):
    /* Restore extra args/env from Caml stack and return to caller */
        movl    0*4(%esi), %ecx
        movl    1*4(%esi), %ebp
        movl    2*4(%esi), %ebx
        addl    $(3*4), %esi
        jmp     L(caml_jit_rt_trampoline)

        /* caml_jit_rt_trampoline:
	 *
	 * Jumps to the byte-code address %ecx, compiling the byte
	 * code on-demand if it wasn't already compiled.
	 */
FUNCTION(G(caml_jit_rt_trampoline))
L(caml_jit_rt_trampoline):
        movl    (%ecx), %edx
        addl    G(caml_jit_code_end), %edx
        jmpl    *%edx
        

/* Function restart */

        /* caml_jit_rt_restart:
	 *
	 * Pushes the arguments saved in the closure %ebp by
	 * caml_jit_rt_grab_closure_and_return back onto the
	 * Caml stack.
         */
FUNCTION(G(caml_jit_rt_restart))
    /* Determine the number of arguments from the environment */
        movl    -1*4(%ebp), %ecx
        shrl    $10, %ecx
        subl    $2, %ecx
    /* Add the number of arguments to the extra args */
        leal    (%ebx, %ecx, 2), %ebx
    /* Load the new stack pointer */
        leal    (, %ecx, 4), %edx
        subl    %edx, %esi
    /* Push the arguments back onto the Caml stack */
        pushl   %edi
        pushl   %esi
        movl    %esi, %edi
        leal    2*4(%ebp), %esi
        rep movsl
        popl    %esi
        popl    %edi
    /* Load the new environment pointer */
        movl    1*4(%ebp), %ebp
        ret

        
/* Function application */

        /* caml_jit_rt_apply1:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY1 opcode.
	 * Expects Caml return address in %edx.
         */
FUNCTION(G(caml_jit_rt_apply1))
    /* Save arguments */
        movl    0*4(%esi), %ecx /* arg1 */
    /* Prepare stack frame */
        subl    $(3*4), %esi
        movl    %ecx, 0*4(%esi) /* arg1 */
        movl    %edx, 1*4(%esi) /* return address */
        movl    %ebp, 2*4(%esi) /* environment */
        movl    %ebx, 3*4(%esi) /* extra args */
    /* Record extra arguments */
        movl    $((0 << 1) | 1), %ebx
    /* Perform the application */
        jmp     L(caml_jit_rt_apply)

        /* caml_jit_rt_apply2:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY2 opcode.
	 * Expects Caml return address in %edx.
         */
FUNCTION(G(caml_jit_rt_apply2))
    /* Save arguments */
        movsd   0*4(%esi), %xmm0 /* arg1 + arg2 */
    /* Prepare stack frame */
        subl    $(3*4), %esi
        movsd   %xmm0, 0*4(%esi) /* arg1 + arg2 */
        movl    %edx, 2*4(%esi)  /* return address */
        movl    %ebp, 3*4(%esi)  /* environment */
        movl    %ebx, 4*4(%esi)  /* extra args */
    /* Record extra arguments */
        movl    $((1 << 1) | 1), %ebx
    /* Perform the application */
        jmp     L(caml_jit_rt_apply)

        /* caml_jit_rt_apply3:
	 *
	 * Special case of caml_jit_rt_apply for the APPLY3 opcode.
	 * Expects Caml return address in %edx.
         */
FUNCTION(G(caml_jit_rt_apply3))
    /* Save arguments */
        movsd   0*4(%esi), %xmm0 /* arg1 + arg2 */
        movl    2*4(%esi), %ecx  /* arg3 */
    /* Prepare stack frame */
        subl    $(3*4), %esi
        movsd   %xmm0, 0*4(%esi) /* arg1 + arg2 */
        movl    %ecx, 2*4(%esi)  /* arg3 */
        movl    %edx, 3*4(%esi)  /* return address */
        movl    %ebp, 4*4(%esi)  /* environment */
        movl    %ebx, 5*4(%esi)  /* extra args */
    /* Record extra arguments */
        movl    $((2 << 1) | 1), %ebx
    /* Perform the application */
        jmp     L(caml_jit_rt_apply)
        
        /* caml_jit_rt_apply:
	 *
	 * Upon entry %eax contains the closure. The function then
	 * checks whether to enlarge the Caml stack and also processes
	 * pending signals. Afterwards it jumps to the closure's code
	 * address.
	 */
FUNCTION(G(caml_jit_rt_apply))
L(caml_jit_rt_apply):
    /* Load environment pointer with closure */
        movl    %eax, %ebp
    /* Check if we need to enlarge the Caml stack */
        cmpl    G(caml_stack_threshold), %esi
        movl    0*4(%eax), %ecx /* load %ecx with closure address */
        jb      L(caml_jit_rt_apply_realloc_stack)
L(caml_jit_rt_apply_check_signals):
    /* Prefetch the closure byte code */
        prefetcht0 (%ecx)
    /* Check for pending signal and jump to the closure's code address */
        testl   $-1, G(caml_something_to_do)
        jnz     L(caml_jit_rt_process_signal)
        jmp     L(caml_jit_rt_trampoline)
L(caml_jit_rt_apply_realloc_stack):
    /* Make stack pointer available to C code */
        movl    %esi, G(caml_extern_sp)
    /* Enlarge the stack by (Stack_threshold / sizeof(value)) */
        pushl   $256
        call    G(caml_realloc_stack)
        addl    $(1*4), %esp
    /* Restore stack pointer */
        movl    G(caml_extern_sp), %esi
    /* Restore closure in %eax and closure address in %ecx */
        movl    %ebp, %eax
        movl    0*4(%ebp), %ecx
        jmp     L(caml_jit_rt_apply_check_signals)


/* Signal processing */

        /* caml_jit_rt_process_signal:
	 *
	 * Process a pending signal and return to the
	 * bytecode address in %ecx.
	 */
FUNCTION(G(caml_jit_rt_process_signal))
L(caml_jit_rt_process_signal):
    /* Reset state of caml_something_to_do */
        movl    $0, G(caml_something_to_do)
    /* An event frame must look like accu + a C_CALL frame + a RETURN 1 frame */
        subl    $(6*4), %esi
        movl    %eax, 0*4(%esi)  /* accu */
        movl      $1, 1*4(%esi)  /* C_CALL frame: dummy environment */
        movl      $1, 2*4(%esi)  /* RETURN frame: dummy local 0 (Val_unit) */
        movl    %ecx, 3*4(%esi)  /* RETURN frame: saved return address */
        movl    %ebp, 4*4(%esi)  /* RETURN frame: saved environment */
        movl    %ebx, 5*4(%esi)  /* RETURN frame: saved extra args */
    /* Make the stack and young pointer available to C code */
        movl    %esi, G(caml_extern_sp)
        movl    %edi, G(caml_young_ptr)
    /* Call into C to process the event */
        call    G(caml_process_event)
    /* Restore stack and young pointer */
        movl    G(caml_extern_sp), %esi
        movl    G(caml_young_ptr), %edi
    /* Restore local state from event frame */
        movl    0*4(%esi), %eax  /* accu */
        movl    3*4(%esi), %ecx  /* return address */
        movl    4*4(%esi), %ebp  /* environment */
        movl    5*4(%esi), %ebx  /* extra args */
        addl    $(6*4), %esi
    /* Return via bytecode address */
        jmp     L(caml_jit_rt_trampoline)


/* Interaction with C code */

        /* caml_jit_rt_raise:
	 *
	 * Mark the block in %eax as exception result and
	 * throw the exception (either back to C code or
	 * handle it with the active catch code).
         */
FUNCTION(G(caml_jit_rt_raise))        
L(caml_jit_rt_raise):
    /* Stash backtrace if enabled */
        testl   $-1, G(caml_backtrace_active)
        jnz     L(caml_jit_rt_raise_2)
L(caml_jit_rt_raise_1):
    /* Check if we need to throw the exception back to C code */
        movl    G(caml_stack_high), %ecx
        movl    G(caml_trapsp), %edx
        subl    1*4(%esp), %ecx
        cmpl    %ecx, %edx              /* caml_trapsp >= caml_stack_high - initial_sp_offset */
        jge     L(caml_jit_rt_stop_exn)
    /* Exception is caught within Caml code */
        leal    4*4(%edx), %esi         /* sp */
        movl    -4*4(%esi), %ecx        /* Trap_pc(sp) */
        movl    -3*4(%esi), %edx        /* Trap_link(sp) */
        movl    -2*4(%esi), %ebp        /* env */
        movl    -1*4(%esi), %ebx        /* extra args */
    /* Restore trap stack pointer */
        movl    %edx, G(caml_trapsp)
        jmp     L(caml_jit_rt_start_enter)
L(caml_jit_rt_raise_2):
    /* Stack backtrace using caml_stash_backtrace, pc in 0*4(%esp) */
        movl    0*4(%esp), %ecx         /* load %ecx with saved_pc */
        pushl   %eax                    /* preserve %eax */
        pushl   %esi
        pushl   %ecx
        pushl   %eax
        call    G(caml_stash_backtrace)
        addl    $(3*4), %esp
        popl    %eax                    /* restore %eax */
        jmp     L(caml_jit_rt_raise_1)

        /* caml_jit_rt_start:
	 *
	 * Entry point into the JIT runtime. Invoked from C via
	 *
	 *  caml_jit_rt_start(code, accu, Val_int(extra_args), env, sp)
	 *
	 * hence the initial C stack layout is:
	 *
	 *  0*4(%esp) - return address
	 *  1*4(%esp) - byte code address
	 *  2*4(%esp) - accu
	 *  3*4(%esp) - extra arguments
	 *  4*4(%esp) - environment
	 *  5*4(%esp) - stack pointer
	 *
	 * C stack layout during execution:
	 *
	 *  0*4(%esp) - saved_pc (for backtrace)
	 *  1*4(%esp) - initial sp offset
	 *  2*4(%esp) - initial local roots
	 *  3*4(%esp) - initial external raise
	 *  4*4(%esp) - saved_accu (spill area)
	 *  5*4(%esp) - start of sigjmp buffer (SIZEOF_SIGJMPBUF bytes + padding)
	 */
FUNCTION(G(caml_jit_rt_start))
    /* Prologue */
        pushl   %ebp
        movl    %esp, %ebp
    /* Push callee-save registers */
        pushl   %ebx
        pushl   %edi
        pushl   %esi
    /* Load initial caml_external_raise, caml_local_roots and stack high */
        movl    G(caml_external_raise), %ebx
        movl    G(caml_local_roots), %edi
        movl    G(caml_stack_high), %esi
    /* Record callback invocation */
        incl    G(caml_callback_depth)
    /* Calculate initial stack pointer offset */
        subl    6*4(%ebp), %esi
    /* Reserve C stack space for:
     *
     * - sigjmp_buf             (SIZEOF_SIGJMPBUF byte)
     * - saved_accu             (4 byte)
     * - initial external raise (4 byte)
     * - initial local roots    (4 byte)
     * - initial sp offset      (4 byte)
     * - saved_pc               (4 byte)
     */
        subl    $(((SIZEOF_SIGJMPBUF + 3) / 4 + 5) * 4), %esp
    /* Save initial sp offset, local roots and external raise */
        movl    %esi, 1*4(%esp) /* initial sp offset */
        movl    %edi, 2*4(%esp) /* initial local roots */
        movl    %ebx, 3*4(%esp) /* initial external raise */
    /* Setup machine state */
        movl    2*4(%ebp), %ecx /* byte code address */
        movl    3*4(%ebp), %eax /* accu */
        movl    4*4(%ebp), %ebx /* extra arguments */
        movl    6*4(%ebp), %esi /* stack pointer */
        movl    5*4(%ebp), %ebp /* environment */
    /* Load young pointer */
        movl    G(caml_young_ptr), %edi
L(caml_jit_rt_start_enter):
    /* Preserve accu and code pointer on C stack */
        movl    %eax, 4*4(%esp)
        movl    %ecx, 0*4(%esp)
    /* Setup the sigjmp buffer */
        leal    5*4(%esp), %eax
        pushl   $0
        pushl   %eax
#if defined(HAS_SIGSETJMP)
        call    G(sigsetjmp)
#elif defined(HAS___SIGSETJMP)
        call    G(__sigsetjmp)
#else
#error "Neither sigsetjmp() nor __sigsetjmp() found."
#endif
        addl    $(2*4), %esp
        testl   %eax, %eax
    /* Check if we caught an exception from C code */
        jnz     L(caml_jit_rt_start_caught)
    /* Restore accu and code pointer */
        movl    4*4(%esp), %eax
        movl    0*4(%esp), %ecx
    /* Reset saved_pc to NULL */
        movl    $0, 0*4(%esp)
    /* Setup the new external raise buffer */
        leal    5*4(%esp), %edx
        movl    %edx, G(caml_external_raise)
    /* Jump to the code */
        jmp     L(caml_jit_rt_trampoline)
L(caml_jit_rt_start_caught):
    /* Reload stack and young pointer */
        movl    G(caml_extern_sp), %esi
        movl    G(caml_young_ptr), %edi
    /* Restore the local roots to its initial state */
        movl    2*4(%esp), %ecx
        movl    %ecx, G(caml_local_roots)
    /* Load the exception object */
        movl    G(caml_exn_bucket), %eax
        jmp     L(caml_jit_rt_raise)

        /* caml_jit_rt_stop:
	 *
	 * Return to the calling C code with the result
	 * in %eax.
	 */
L(caml_jit_rt_stop_exn):
    /* Mark %rax as exception result */
        orb     $2, %al
FUNCTION(G(caml_jit_rt_stop))
L(caml_jit_rt_stop):
    /* Make stack and young pointer available to C code */
        movl    %esi, G(caml_extern_sp)
        movl    %edi, G(caml_young_ptr)
    /* Restore caml_external_raise and caml_local_roots to its initial state */
        movl    2*4(%esp), %ecx         /* initial local roots */
        movl    3*4(%esp), %edx         /* initial external raise */
        movl    %ecx, G(caml_local_roots)
        movl    %edx, G(caml_external_raise)
    /* Decrement callback counter */
        decl    G(caml_callback_depth)
    /* Pop callee-save registers */
        addl    $(((SIZEOF_SIGJMPBUF + 3) / 4 + 5) * 4), %esp
        popl    %esi
        popl    %edi
        pop     %ebx
    /* Epilogue */
        popl    %ebp
        ret


/* Floating point operations */

FUNCTION(G(caml_jit_rt_copy_double))
L(caml_jit_rt_copy_double):
        call    L(caml_jit_rt_alloc2)
        movl    $((2 << 10) | (3 << 8) | 253), 0*4(%edi)
        movsd   %xmm0, 1*4(%edi)
        leal    1*4(%edi), %eax
        ret

        /* caml_jit_rt_add_float:
	 *
	 * Adds the floating point values in (%eax) and
	 * -1*4(%esi) and stores the result into a newly
	 * allocated float block. Upon return %eax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_add_float))
        movl    -1*4(%esi), %edx
        movsd   (%eax), %xmm0
        addsd   (%edx), %xmm0
        jmp     L(caml_jit_rt_copy_double)

        /* caml_jit_rt_sub_float:
	 *
	 * Subtracts the floating point values in (%eax) and
	 * -1*4(%esi) and stores the result into a newly
	 * allocated float block. Upon return %eax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_sub_float))
        movl    -1*4(%esi), %edx
        movsd   (%eax), %xmm0
        subsd   (%edx), %xmm0
        jmp     L(caml_jit_rt_copy_double)

        /* caml_jit_rt_mul_float:
	 *
	 * Multiplies the floating point values in (%eax) and
	 * -1*4(%esi) and stores the result into a newly
	 * allocated float block. Upon return %eax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_mul_float))
        movl    -1*4(%esi), %edx
        movsd   (%eax), %xmm0
        mulsd   (%edx), %xmm0
        jmp     L(caml_jit_rt_copy_double)

        /* caml_jit_rt_div_float:
	 *
	 * Divides the floating point values in (%eax) and
	 * -1*4(%esi) and stores the result into a newly
	 * allocated float block. Upon return %eax points
	 * to the newly allocated float block with the
	 * result.
         */
FUNCTION(G(caml_jit_rt_div_float))
        movl    -1*4(%esi), %edx
        movsd   (%eax), %xmm0
        divsd   (%edx), %xmm0
        jmp     L(caml_jit_rt_copy_double)


/* Copying operations */

        /* caml_jit_rt_copy_floats:
	 *
	 * Copies %ecx double values from the stack %esi to
	 * the double array at %edx. %ecx is assumed to be
	 * non-zero.
	 *
	 * Upon return, %esi will be incremented by %ecx*4.
	 *
	 * Clobbers %ecx, %edx and %xmm0.
	 */
FUNCTION(G(caml_jit_rt_copy_floats))
        pushl   %eax
L(caml_jit_rt_copy_floats_1):
        subl    $1, %ecx
        movl    0*4(%esi), %eax
        movsd   0*8(%eax), %xmm0
        movsd   %xmm0, 0*8(%edx)
        leal    1*4(%esi), %esi
        leal    1*8(%edx), %edx
        jnz     L(caml_jit_rt_copy_floats_1)
        popl    %eax
        ret


/* Instruction tracing */
        
#ifdef DEBUG
        /* caml_jit_rt_trace:
	 *
	 * Expects instruction opcode and byte-code pc on
	 * C stack. Invokes caml_jit_trace() with the
	 * appropriate parameters, preserving the current
	 * state.
	 *
	 * Upon return, the instruction opcode and byte-code
	 * pc will be popped off the C stack.
	 */
FUNCTION(G(caml_jit_rt_trace))
    /* Prologue */
        pushl   %ebp
        movl    %esp, %ebp
    /* Push preserved registers */
        pushl   %eax
        pushl   %ecx
        pushl   %edx
    /* Save floating-point registers */
        subl    $(8*8), %esp
        movsd   %xmm0, 0*8(%esp)
        movsd   %xmm1, 1*8(%esp)
        movsd   %xmm2, 2*8(%esp)
        movsd   %xmm3, 3*8(%esp)
        movsd   %xmm4, 4*8(%esp)
        movsd   %xmm5, 5*8(%esp)
        movsd   %xmm6, 6*8(%esp)
        movsd   %xmm7, 7*8(%esp)
    /* Make stack and young ptr available to C code */
        movl    %esi, G(caml_extern_sp)
        movl    %edi, G(caml_young_ptr)
    /* Setup the trace parameters */
        pushl   %esi            /* sp */
        pushl   0*4(%ebp)       /* env */
        pushl   %ebx            /* extra args */
        pushl   %eax            /* accu */
        pushl   2*4(%ebp)       /* pc */
        pushl   3*4(%ebp)       /* instr */
    /* Invoke the trace function */
        call    G(caml_jit_trace)
    /* Pop the parameters off the C stack */
        addl    $(6*4), %esp
    /* Restore floating-point registers */
        movsd   7*8(%esp), %xmm7
        movsd   6*8(%esp), %xmm6
        movsd   5*8(%esp), %xmm5
        movsd   4*8(%esp), %xmm4
        movsd   3*8(%esp), %xmm3
        movsd   2*8(%esp), %xmm2
        movsd   1*8(%esp), %xmm1
        movsd   0*8(%esp), %xmm0
        addl    $(8*8), %esp
    /* Restore preserved registers */
	popl    %edx
	popl    %ecx
        popl    %eax
    /* Epilogue */
        popl    %ebp
        ret     $(2*4)
#endif
