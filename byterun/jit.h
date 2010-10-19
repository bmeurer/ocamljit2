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

#ifndef CAML_JIT_H
#define CAML_JIT_H

#include <stddef.h>
#include <stdlib.h>

/* System detection */
#if defined(__APPLE__) || defined(__GLIBC__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
# include <stdint.h>
typedef int8_t    caml_jit_int8_t;
typedef uint8_t   caml_jit_uint8_t;
typedef int16_t   caml_jit_int16_t;
typedef uint16_t  caml_jit_uint16_t;
typedef int32_t   caml_jit_int32_t;
typedef uint32_t  caml_jit_uint32_t;
typedef int64_t   caml_jit_int64_t;
typedef uint64_t  caml_jit_uint64_t;
typedef intptr_t  caml_jit_intptr_t;
typedef uintptr_t caml_jit_uintptr_t;
# define CAML_JIT_INT8_MAX   INT8_MAX
# define CAML_JIT_INT8_MIN   INT8_MIN
# define CAML_JIT_UINT8_MAX  UINT8_MAX
# define CAML_JIT_INT16_MAX  INT16_MAX
# define CAML_JIT_INT16_MIN  INT16_MIN
# define CAML_JIT_UINT16_MAX UINT16_MAX
# define CAML_JIT_INT32_MAX  INT32_MAX
# define CAML_JIT_INT32_MIN  INT32_MIN
# define CAML_JIT_UINT32_MAX UINT32_MAX
# define CAML_JIT_INT64_MAX  INT64_MAX
# define CAML_JIT_INT64_MIN  INT64_MIN
# define CAML_JIT_UINT64_MAX UINT64_MAX
#else
# error "Unsupported system"
#endif

/* Assertions */
#if defined(DEBUG)
# include <assert.h>
# define caml_jit_assert               assert
# define caml_jit_assert_not_reached() assert(0)
#else
# define caml_jit_assert(x)            ((void) 0)
# define caml_jit_assert_not_reached() ((void) 0)
#endif

/* GNU CC detection */
#if defined(__GNUC__) && defined(__GNUC_MINOR__)
# define CAML_JIT_GNUC_PREREQ(major, minor) ((__GNUC__ << 16) + __GNUC_MINOR__ >= ((major) << 16) + (minor))
#else
# define CAML_JIT_GNUC_PREREQ(major, minor) (0)
#endif

/* Internal functions/variables */
#if defined(DEBUG)
# define CAML_JIT_INTERNAL extern
#else
# define CAML_JIT_INTERNAL extern CAML_JIT_GNUC_HIDDEN
#endif

/* GNU CC symbol attributes */
#if CAML_JIT_GNUC_PREREQ(3,0)
# define CAML_JIT_GNUC_HIDDEN __attribute__((visibility("hidden")))
#else
# define CAML_JIT_GNUC_HIDDEN
#endif
#if CAML_JIT_GNUC_PREREQ(2,96)
# define CAML_JIT_GNUC_MALLOC __attribute__((__malloc__)) CAML_JIT_GNUC_WARN_UNUSED_RESULT
#else
# define CAML_JIT_GNUC_MALLOC
#endif
#if CAML_JIT_GNUC_PREREQ(2,8)
# define CAML_JIT_GNUC_NOINLINE __attribute__((noinline))
#else
# define CAML_JIT_GNUC_NOINLINE
#endif
#if CAML_JIT_GNUC_PREREQ(2,8)
# define CAML_JIT_GNUC_UNUSED __attribute__((unused))
#else
# define CAML_JIT_GNUC_UNUSED
#endif
#if CAML_JIT_GNUC_PREREQ(3,4)
# define CAML_JIT_GNUC_WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#else
# define CAML_JIT_GNUC_WARN_UNUSED_RESULT
#endif

/* GNU CC features */
#if CAML_JIT_GNUC_PREREQ(3,0)
# define CAML_JIT_GNUC_LIKELY(x)   __builtin_expect(!!(x), 1)
# define CAML_JIT_GNUC_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
# define CAML_JIT_GNUC_LIKELY(x)   (x)
# define CAML_JIT_GNUC_UNLIKELY(x) (x)
#endif


/* maximum native code size */
#define CAML_JIT_CODE_SIZE (128 * 1024 * 1024)


//
// TODO
//

CAML_JIT_INTERNAL caml_jit_uint8_t *caml_jit_code_base;
CAML_JIT_INTERNAL caml_jit_uint8_t *caml_jit_code_end;
CAML_JIT_INTERNAL caml_jit_uint8_t *caml_jit_code_ptr;

/* Caml byte code blocks */
typedef struct caml_jit_block_t caml_jit_block_t;
struct caml_jit_block_t
{
  code_t            block_prog;
  code_t            block_pend;
  caml_jit_block_t *block_next;
};


#endif /* !CAML_JIT_H */
