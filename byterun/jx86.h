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

#ifndef CAML_JX86_H
#define CAML_JX86_H

#if defined(TARGET_amd64) || defined(TARGET_i386)

/* X86-32/64 architecture detection
 * --------------------------------
 */
#ifdef TARGET_amd64
# define JX86_64
#else
# define JX86_32
#endif


/* Native types
 * ------------
 */
typedef signed char        jx86_int8_t;
typedef unsigned char      jx86_uint8_t;
typedef signed short       jx86_int16_t;
typedef unsigned short     jx86_uint16_t;
typedef signed int         jx86_int32_t;
typedef unsigned int       jx86_uint32_t;
#ifdef JX86_64
typedef signed long        jx86_int64_t;
typedef unsigned long      jx86_uint64_t;
#else
typedef signed long long   jx86_int64_t;
typedef unsigned long long jx86_uint64_t;
#endif
#ifdef JX86_64
typedef jx86_int64_t  jx86_intptr_t;
typedef jx86_uint64_t jx86_uintptr_t;
#else
typedef jx86_int32_t  jx86_intptr_t;
typedef jx86_uint32_t jx86_uintptr_t;
#endif
/* native word size */
#define JX86_NWS (sizeof(jx86_intptr_t))


/* Assertions
 * ----------
 */
#ifdef DEBUG
# include <assert.h>
# define jx86_assert assert
#else
# define jx86_assert(x) do { if (0) { (void) (x); } } while (0)
#endif


/* General purpose registers
 * -------------------------
 */
typedef enum
{
  /* 8bit register names */
  JX86_AL   =  0,
  JX86_CL   =  1,
  JX86_DL   =  2,
  JX86_BL   =  3,
#ifdef JX86_64
  JX86_SPL  =  4, 
  JX86_BPL  =  5,
  JX86_SIL  =  6,
  JX86_DIL  =  7,
  JX86_R8B  =  8,
  JX86_R9B  =  9,
  JX86_R10B = 10,
  JX86_R11B = 11,
  JX86_R12B = 12,
  JX86_R13B = 13,
  JX86_R14B = 14,
  JX86_R15B = 15,
#endif /* JX86_64 */

  /* 32bit register names */
  JX86_EAX  =  0,
  JX86_ECX  =  1,
  JX86_EDX  =  2,
  JX86_EBX  =  3,
  JX86_ESP  =  4,
  JX86_EBP  =  5,
  JX86_ESI  =  6,
  JX86_EDI  =  7,
#ifdef JX86_64
  JX86_R8D  =  8,
  JX86_R9D  =  9,
  JX86_R10D = 10,
  JX86_R11D = 11,
  JX86_R12D = 12,
  JX86_R13D = 13,
  JX86_R14D = 14,
  JX86_R15D = 15,

  /* 64bit register names */
  JX86_RAX =  0,
  JX86_RCX =  1,
  JX86_RDX =  2,
  JX86_RBX =  3,
  JX86_RSP =  4,
  JX86_RBP =  5,
  JX86_RSI =  6,
  JX86_RDI =  7,
  JX86_R8  =  8,
  JX86_R9  =  9,
  JX86_R10 = 10,
  JX86_R11 = 11,
  JX86_R12 = 12,
  JX86_R13 = 13,
  JX86_R14 = 14,
  JX86_R15 = 15,
#endif /* JX86_64 */

  /* native register names */
  JX86_NAX  =  0,
  JX86_NCX  =  1,
  JX86_NDX  =  2,
  JX86_NBX  =  3,
  JX86_NSP  =  4,
  JX86_NBP  =  5,
  JX86_NSI  =  6,
  JX86_NDI  =  7,
} jx86_reg_t;

#ifdef JX86_64
# define JX86_IS_REG(reg)   ((jx86_reg_t) (reg) >= JX86_RAX && (jx86_reg_t) (reg) <= JX86_R15)
# define JX86_REG_CODE(reg) ((jx86_reg_t) ((reg) & 0x07))
#else
# define JX86_IS_REG(reg)   ((jx86_reg_t) (reg) >= JX86_EAX && (jx86_reg_t) (reg) <= JX86_EDI)
# define JX86_REG_CODE(reg) ((jx86_reg_t) (reg))
#endif

/* SSE2 registers
 * --------------
 */
typedef enum
{
  JX86_XMM0  =  0,
  JX86_XMM1  =  1,
  JX86_XMM2  =  2,
  JX86_XMM3  =  3,
  JX86_XMM4  =  4,
  JX86_XMM5  =  5,
  JX86_XMM6  =  6,
  JX86_XMM7  =  7,
#ifdef JX86_64
  JX86_XMM8  =  8,
  JX86_XMM9  =  9,
  JX86_XMM10 = 10,
  JX86_XMM11 = 11,
  JX86_XMM12 = 12,
  JX86_XMM13 = 13,
  JX86_XMM14 = 14,
  JX86_XMM15 = 15
#endif /* JX86_64 */
} jx86_xmm_t;

#ifdef JX86_64
# define JX86_IS_XMM(xmm)   ((jx86_xmm_t) (reg) >= JX86_XMM0 && (jx86_xmm_t) (reg) <= JX86_XMM15)
# define JX86_XMM_CODE(xmm) ((jx86_xmm_t) ((reg) & 0x07))
#else
# define JX86_IS_XMM(xmm)   ((jx86_xmm_t) (reg) >= JX86_XMM0 && (jx86_xmm_t) (reg) <= JX86_XMM7)
# define JX86_XMM_CODE(xmm) ((jx86_xmm_t) (reg))
#endif
  

/* Integer condition codes
 * -----------------------
 */
#define JX86_DECLARE_ENUM_CC(N, b)              \
  enum {                                        \
    JX86_##N##A   = (unsigned) (b) + 0x07U,     \
    JX86_##N##AE  = (unsigned) (b) + 0x03U,     \
    JX86_##N##B   = (unsigned) (b) + 0x02U,     \
    JX86_##N##BE  = (unsigned) (b) + 0x06U,     \
    JX86_##N##C   = JX86_##N##B,                \
    JX86_##N##E   = (unsigned) (b) + 0x04U,     \
    JX86_##N##G   = (unsigned) (b) + 0x0fU,     \
    JX86_##N##GE  = (unsigned) (b) + 0x0dU,     \
    JX86_##N##L   = (unsigned) (b) + 0x0cU,     \
    JX86_##N##LE  = (unsigned) (b) + 0x0eU,     \
    JX86_##N##NA  = JX86_##N##BE,               \
    JX86_##N##NAE = JX86_##N##B,                \
    JX86_##N##NB  = JX86_##N##AE,               \
    JX86_##N##NBE = JX86_##N##A,                \
    JX86_##N##NC  = JX86_##N##NB,               \
    JX86_##N##NE  = (unsigned) (b) + 0x05U,     \
    JX86_##N##NG  = JX86_##N##LE,               \
    JX86_##N##NGE = JX86_##N##L,                \
    JX86_##N##NL  = JX86_##N##GE,               \
    JX86_##N##NLE = JX86_##N##G,                \
    JX86_##N##NO  = (unsigned) (b) + 0x01U,     \
    JX86_##N##NP  = (unsigned) (b) + 0x0bU,     \
    JX86_##N##NS  = (unsigned) (b) + 0x09U,     \
    JX86_##N##NZ  = JX86_##N##NE,               \
    JX86_##N##O   = (unsigned) (b) + 0x00U,     \
    JX86_##N##P   = (unsigned) (b) + 0x0aU,     \
    JX86_##N##PE  = JX86_##N##P,                \
    JX86_##N##PO  = JX86_##N##NP,               \
    JX86_##N##S   = (unsigned) (b) + 0x08U,     \
    JX86_##N##Z   = JX86_##N##E                 \
  }


/* Emit data
 * ---------
 */
#define jx86_emit_int8(cp, v)                           \
  do {                                                  \
    *((jx86_int8_t *) (cp)) = (jx86_int8_t) (v);        \
    cp += sizeof(jx86_int8_t);                          \
  } while (0)
#define jx86_emit_uint8(cp, v)                          \
  do {                                                  \
    *((jx86_uint8_t *) (cp)) = (jx86_uint8_t) (v);      \
    cp += sizeof(jx86_uint8_t);                         \
  } while (0)
#define jx86_emit_int16(cp, v)                            \
  do {                                                    \
    *((jx86_int16_t *) (cp)) = (jx86_int16_t) (v);        \
    cp += sizeof(jx86_int16_t);                           \
  } while (0)
#define jx86_emit_uint16(cp, v)                           \
  do {                                                    \
    *((jx86_uint16_t *) (cp)) = (jx86_uint16_t) (v);      \
    cp += sizeof(jx86_uint16_t);                          \
  } while (0)
#define jx86_emit_int32(cp, v)                            \
  do {                                                    \
    *((jx86_int32_t *) (cp)) = (jx86_int32_t) (v);        \
    cp += sizeof(jx86_int32_t);                           \
  } while (0)
#define jx86_emit_uint32(cp, v)                           \
  do {                                                    \
    *((jx86_uint32_t *) (cp)) = (jx86_uint32_t) (v);      \
    cp += sizeof(jx86_uint32_t);                          \
  } while (0)
#define jx86_emit_int64(cp, v)                            \
  do {                                                    \
    *((jx86_int64_t *) (cp)) = (jx86_int64_t) (v);        \
    cp += sizeof(jx86_int64_t);                           \
  } while (0)
#define jx86_emit_uint64(cp, v)                           \
  do {                                                    \
    *((jx86_uint64_t *) (cp)) = (jx86_uint64_t) (v);      \
    cp += sizeof(jx86_uint64_t);                          \
  } while (0)


/* Immediates
 * ----------
 */
#define JX86_IS_IMM8(imm) ((int) (imm) >= -128 && (int) (imm) <= 127)
#ifdef JX86_32
# define JX86_IS_IMM32(imm) (1)
#else /* !JX86_32 */
# define JX86_IS_IMM32(imm) ((jx86_int64_t) (imm) >= -((jx86_int64_t) 1 << 31) \
                             && ((jx86_int64_t) (imm) <= ((jx86_int64_t) 1 << 31) - 1))
#endif /* JX86_32 */

#define jx86_emit_imm(cp, size, imm)            \
  do {                                          \
    switch ((size)) {                           \
    default:                                    \
      jx86_assert(!"jx86_emit_imm");            \
    case 1:                                     \
      jx86_emit_int8((cp), (imm));              \
      break;                                    \
    case 2:                                     \
      jx86_emit_int16((cp), (imm));             \
      break;                                    \
    case 8:                                     \
      jx86_assert(JX86_NWS == 8);               \
    case 4:                                     \
      jx86_emit_int32((cp), (imm));             \
      break;                                    \
    }                                           \
  } while (0)


/* Instruction prefixes
 * --------------------
 */
#define jx86_emit_osp(cp, size)                 \
  do {                                          \
    if ((size) == 2)                            \
      jx86_emit_uint8((cp), 0x66);              \
  } while (0)

#ifdef JX86_64
typedef enum
{
  JX86_REX_B = 1, /* The register in R/M, base register in SIB or register in opcode is 8-15 */
  JX86_REX_X = 2, /* The index register in SIB is 8-15 */
  JX86_REX_R = 4, /* The register in ModR/M is 8-15 */
  JX86_REX_W = 8  /* Operation is 64-bits */
} jx86_rex_t;

#define jx86_emit_rex(cp, size, reg_modrm, reg_index, reg_rm_base_opcode) \
  do {                                                                  \
    const jx86_uint8_t _rex_ =                                          \
      (((size) > 4) ? JX86_REX_W : 0) |                                 \
      (((reg_modrm) > 7) ? JX86_REX_R : 0) |                            \
      (((reg_index) > 7) ? JX86_REX_X : 0) |                            \
      (((reg_rm_base_opcode) > 7) ? JX86_REX_B : 0);                    \
    jx86_assert((size) <= sizeof(void *));                              \
    jx86_assert(JX86_IS_REG(reg_modrm));                                \
    jx86_assert(JX86_IS_REG(reg_index));                                \
    jx86_assert(JX86_IS_REG(reg_rm_base_opcode));                       \
    if (_rex_ || (size) == 1)                                           \
      jx86_emit_uint8((cp), _rex_ | 0x40);                              \
  } while (0)
#else /* !JX86_64 */
#define jx86_emit_rex(cp, size, reg_modrm, reg_index, reg_rm_base_opcode) \
  do {                                                                  \
    jx86_assert((size) <= sizeof(void *));                              \
    jx86_assert(JX86_IS_REG(reg_modrm));                                \
    jx86_assert(JX86_IS_REG(reg_index));                                \
    jx86_assert(JX86_IS_REG(reg_rm_base_opcode));                       \
  } while (0)
#endif /* JX86_64 */

#define jx86_emit_osprex(cp, size, reg_modrm, reg_index, reg_rm_base_opcode) \
  do {                                                                  \
    jx86_emit_osp((cp), (size));                                        \
    jx86_emit_rex((cp), (size), (reg_modrm),                            \
                  (reg_index), (reg_rm_base_opcode));                   \
  } while (0)


/* Instruction encoding
 * --------------------
 */
#define jx86_emit_opcode1(cp, size, opc)                \
  do {                                                  \
    if ((size) == 1)                                    \
      jx86_emit_uint8((cp), (jx86_uint8_t) (opc));      \
    else                                                \
      jx86_emit_uint8((cp), (jx86_uint8_t) (opc) + 1);  \
  } while (0)

#define jx86_emit_address_byte(cp, m, o, r)             \
  jx86_emit_uint8((cp), (((m) & 0x03) << 6) | (((o) & 0x07) << 3) | ((r) & 0x07))

#ifdef JX86_32
# define jx86_emit_mem(cp, r, mem)                              \
  do {                                                          \
    jx86_emit_address_byte((cp), 0, (r), 5);                    \
    jx86_emit_int32((cp), (mem));                               \
  } while (0)
#endif /* JX86_32 */

#define jx86_emit_membase(cp, r, breg, disp)                    \
  do {                                                          \
    if (JX86_REG_CODE(breg) != JX86_EBP && (disp) == 0) {       \
      jx86_emit_address_byte((cp), 0, (r), (breg));             \
      if (JX86_REG_CODE(breg) == JX86_ESP)                      \
        jx86_emit_address_byte((cp), 0, JX86_ESP, JX86_ESP);    \
    }                                                           \
    else if (JX86_IS_IMM8(disp)) {                              \
      jx86_emit_address_byte((cp), 1, (r), (breg));             \
      if (JX86_REG_CODE(breg) == JX86_ESP)                      \
        jx86_emit_address_byte((cp), 0, JX86_ESP, JX86_ESP);    \
      jx86_emit_int8((cp), (disp));                             \
    }                                                           \
    else {                                                      \
      jx86_emit_address_byte((cp), 2, (r), (breg));             \
      if (JX86_REG_CODE(breg) == JX86_ESP)                      \
        jx86_emit_address_byte((cp), 0, JX86_ESP, JX86_ESP);    \
      jx86_emit_int32((cp), (disp));                            \
    }                                                           \
  } while (0)

#define jx86_emit_memindex(cp, r, breg, disp, ireg, shift)              \
  do {                                                                  \
    jx86_assert((shift) >= 0 && (shift) <= 3);                          \
    if (JX86_REG_CODE(breg) != JX86_EBP && (disp) == 0) {               \
      jx86_emit_address_byte((cp), 0, (r), 4);                          \
      jx86_emit_address_byte((cp), (shift), (ireg), (breg));            \
    }                                                                   \
    else if (JX86_IS_IMM8(disp)) {                                      \
      jx86_emit_address_byte((cp), 1, (r), 4);                          \
      jx86_emit_address_byte((cp), (shift), (ireg), (breg));            \
      jx86_emit_int8((cp), (disp));                                     \
    }                                                                   \
    else {                                                              \
      jx86_emit_address_byte((cp), 2, (r), 4);                          \
      jx86_emit_address_byte((cp), (shift), (ireg), (breg));            \
      jx86_emit_int32((cp), (disp));                                    \
    }                                                                   \
  } while (0)

#define jx86_emit_reg(cp, r, reg)               \
  jx86_emit_address_byte((cp), 3, (r), (reg))


/* ALU instructions
 * ----------------
 */

typedef enum
{
  JX86_ADD = 0,
  JX86_OR  = 1,
  JX86_ADC = 2,
  JX86_SBB = 3,
  JX86_AND = 4,
  JX86_SUB = 5,
  JX86_XOR = 6,
  JX86_CMP = 7
} jx86_alu_t;

#define jx86_alu_membase_imm(cp, alu, breg, disp, simm, size)           \
  do {                                                                  \
    const jx86_alu_t _alu_ = (const jx86_alu_t) (alu);                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);            \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);                       \
    if (_size_ != 1 && JX86_IS_IMM8(_simm_)) {                          \
      jx86_emit_uint8((cp), 0x83);                                      \
      jx86_emit_membase((cp), _alu_, _breg_, _disp_);                   \
      jx86_emit_int8((cp), _simm_);                                     \
    }                                                                   \
    else {                                                              \
      jx86_emit_opcode1((cp), _size_, 0x80);                            \
      jx86_emit_membase((cp), _alu_, _breg_, _disp_);                   \
      jx86_emit_imm((cp), _size_, _simm_);                              \
    }                                                                   \
  } while (0)

#define jx86_alu_membase_reg(cp, alu, breg, disp, sreg, size)           \
  do {                                                                  \
    const jx86_alu_t _alu_ = (const jx86_alu_t) (alu);                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);                \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, _sreg_, 0, _breg_);                  \
    jx86_emit_opcode1((cp), _size_, _alu_ << 3);                        \
    jx86_emit_membase((cp), _sreg_, _breg_, _disp_);                    \
  } while (0)

#define jx86_alu_reg_imm(cp, alu, dreg, simm, size)             \
  do {                                                          \
    const jx86_alu_t _alu_ = (const jx86_alu_t) (alu);          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);               \
    if (_size_ != 1 && JX86_IS_IMM8(_simm_)) {                  \
      jx86_emit_uint8((cp), 0x83);                              \
      jx86_emit_reg((cp), _alu_, _dreg_);                       \
      jx86_emit_int8((cp), _simm_);                             \
    }                                                           \
    else {                                                      \
      if (_dreg_ == JX86_EAX)                                   \
        jx86_emit_opcode1((cp), _size_, (_alu_ << 3) + 4);      \
      else {                                                    \
        jx86_emit_opcode1((cp), _size_, 0x80);                   \
        jx86_emit_reg((cp), _alu_, _dreg_);                     \
      }                                                         \
      jx86_emit_imm((cp), _size_, _simm_);                      \
    }                                                           \
  } while (0)

#define jx86_alu_reg_membase(cp, alu, dreg, breg, disp, size)           \
  do {                                                                  \
    const jx86_alu_t _alu_ = (const jx86_alu_t) (alu);                  \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _breg_);                  \
    jx86_emit_opcode1((cp), _size_, (_alu_ << 3) + 2);                  \
    jx86_emit_membase((cp), _dreg_, _breg_, _disp_);                    \
  } while (0)

#define jx86_alu_reg_reg(cp, alu, dreg, sreg, size)             \
  do {                                                          \
    const jx86_alu_t _alu_ = (const jx86_alu_t) (alu);          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _sreg_);          \
    jx86_emit_opcode1((cp), _size_, (_alu_ << 3) + 2);          \
    jx86_emit_reg((cp), _dreg_, _sreg_);                        \
  } while (0)

#define jx86_addb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADD, (breg), (disp), (simm), 1)
#define jx86_addb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADD, (breg), (disp), (sreg), 1)
#define jx86_addb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADD, (dreg), (simm), 1)
#define jx86_addb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADD, (dreg), (breg), (disp), 1)
#define jx86_addb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADD, (dreg), (sreg), 1)
#define jx86_addw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADD, (breg), (disp), (simm), 2)
#define jx86_addw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADD, (breg), (disp), (sreg), 2)
#define jx86_addw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADD, (dreg), (simm), 2)
#define jx86_addw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADD, (dreg), (breg), (disp), 2)
#define jx86_addw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADD, (dreg), (sreg), 2)
#define jx86_addl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADD, (breg), (disp), (simm), 4)
#define jx86_addl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADD, (breg), (disp), (sreg), 4)
#define jx86_addl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADD, (dreg), (simm), 4)
#define jx86_addl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADD, (dreg), (breg), (disp), 4)
#define jx86_addl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADD, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_addq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_ADD, (breg), (disp), (simm), 8)
# define jx86_addq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_ADD, (breg), (disp), (sreg), 8)
# define jx86_addq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_ADD, (dreg), (simm), 8)
# define jx86_addq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_ADD, (dreg), (breg), (disp), 8)
# define jx86_addq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_ADD, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_addn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADD, (breg), (disp), (simm), JX86_NWS)
#define jx86_addn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADD, (breg), (disp), (sreg), JX86_NWS)
#define jx86_addn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADD, (dreg), (simm), JX86_NWS)
#define jx86_addn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADD, (dreg), (breg), (disp), JX86_NWS)
#define jx86_addn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADD, (dreg), (sreg), JX86_NWS)

#define jx86_orb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_OR, (breg), (disp), (simm), 1)
#define jx86_orb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_OR, (breg), (disp), (sreg), 1)
#define jx86_orb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_OR, (dreg), (simm), 1)
#define jx86_orb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_OR, (dreg), (breg), (disp), 1)
#define jx86_orb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_OR, (dreg), (sreg), 1)
#define jx86_orw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_OR, (breg), (disp), (simm), 2)
#define jx86_orw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_OR, (breg), (disp), (sreg), 2)
#define jx86_orw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_OR, (dreg), (simm), 2)
#define jx86_orw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_OR, (dreg), (breg), (disp), 2)
#define jx86_orw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_OR, (dreg), (sreg), 2)
#define jx86_orl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_OR, (breg), (disp), (simm), 4)
#define jx86_orl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_OR, (breg), (disp), (sreg), 4)
#define jx86_orl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_OR, (dreg), (simm), 4)
#define jx86_orl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_OR, (dreg), (breg), (disp), 4)
#define jx86_orl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_OR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_orq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_OR, (breg), (disp), (simm), 8)
# define jx86_orq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_OR, (breg), (disp), (sreg), 8)
# define jx86_orq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_OR, (dreg), (simm), 8)
# define jx86_orq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_OR, (dreg), (breg), (disp), 8)
# define jx86_orq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_OR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_orn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_OR, (breg), (disp), (simm), JX86_NWS)
#define jx86_orn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_OR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_orn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_OR, (dreg), (simm), JX86_NWS)
#define jx86_orn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_OR, (dreg), (breg), (disp), JX86_NWS)
#define jx86_orn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_OR, (dreg), (sreg), JX86_NWS)

#define jx86_adcb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADC, (breg), (disp), (simm), 1)
#define jx86_adcb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADC, (breg), (disp), (sreg), 1)
#define jx86_adcb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADC, (dreg), (simm), 1)
#define jx86_adcb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADC, (dreg), (breg), (disp), 1)
#define jx86_adcb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADC, (dreg), (sreg), 1)
#define jx86_adcw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADC, (breg), (disp), (simm), 2)
#define jx86_adcw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADC, (breg), (disp), (sreg), 2)
#define jx86_adcw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADC, (dreg), (simm), 2)
#define jx86_adcw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADC, (dreg), (breg), (disp), 2)
#define jx86_adcw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADC, (dreg), (sreg), 2)
#define jx86_adcl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADC, (breg), (disp), (simm), 4)
#define jx86_adcl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADC, (breg), (disp), (sreg), 4)
#define jx86_adcl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADC, (dreg), (simm), 4)
#define jx86_adcl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADC, (dreg), (breg), (disp), 4)
#define jx86_adcl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADC, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_adcq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_ADC, (breg), (disp), (simm), 8)
# define jx86_adcq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_ADC, (breg), (disp), (sreg), 8)
# define jx86_adcq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_ADC, (dreg), (simm), 8)
# define jx86_adcq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_ADC, (dreg), (breg), (disp), 8)
# define jx86_adcq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_ADC, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_adcn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_ADC, (breg), (disp), (simm), JX86_NWS)
#define jx86_adcn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_ADC, (breg), (disp), (sreg), JX86_NWS)
#define jx86_adcn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_ADC, (dreg), (simm), JX86_NWS)
#define jx86_adcn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_ADC, (dreg), (breg), (disp), JX86_NWS)
#define jx86_adcn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_ADC, (dreg), (sreg), JX86_NWS)

#define jx86_sbbb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SBB, (breg), (disp), (simm), 1)
#define jx86_sbbb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SBB, (breg), (disp), (sreg), 1)
#define jx86_sbbb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SBB, (dreg), (simm), 1)
#define jx86_sbbb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SBB, (dreg), (breg), (disp), 1)
#define jx86_sbbb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SBB, (dreg), (sreg), 1)
#define jx86_sbbw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SBB, (breg), (disp), (simm), 2)
#define jx86_sbbw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SBB, (breg), (disp), (sreg), 2)
#define jx86_sbbw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SBB, (dreg), (simm), 2)
#define jx86_sbbw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SBB, (dreg), (breg), (disp), 2)
#define jx86_sbbw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SBB, (dreg), (sreg), 2)
#define jx86_sbbl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SBB, (breg), (disp), (simm), 4)
#define jx86_sbbl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SBB, (breg), (disp), (sreg), 4)
#define jx86_sbbl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SBB, (dreg), (simm), 4)
#define jx86_sbbl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SBB, (dreg), (breg), (disp), 4)
#define jx86_sbbl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SBB, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_sbbq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_SBB, (breg), (disp), (simm), 8)
# define jx86_sbbq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_SBB, (breg), (disp), (sreg), 8)
# define jx86_sbbq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_SBB, (dreg), (simm), 8)
# define jx86_sbbq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_SBB, (dreg), (breg), (disp), 8)
# define jx86_sbbq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_SBB, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_sbbn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SBB, (breg), (disp), (simm), JX86_NWS)
#define jx86_sbbn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SBB, (breg), (disp), (sreg), JX86_NWS)
#define jx86_sbbn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SBB, (dreg), (simm), JX86_NWS)
#define jx86_sbbn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SBB, (dreg), (breg), (disp), JX86_NWS)
#define jx86_sbbn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SBB, (dreg), (sreg), JX86_NWS)

#define jx86_andb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_AND, (breg), (disp), (simm), 1)
#define jx86_andb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_AND, (breg), (disp), (sreg), 1)
#define jx86_andb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_AND, (dreg), (simm), 1)
#define jx86_andb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_AND, (dreg), (breg), (disp), 1)
#define jx86_andb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_AND, (dreg), (sreg), 1)
#define jx86_andw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_AND, (breg), (disp), (simm), 2)
#define jx86_andw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_AND, (breg), (disp), (sreg), 2)
#define jx86_andw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_AND, (dreg), (simm), 2)
#define jx86_andw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_AND, (dreg), (breg), (disp), 2)
#define jx86_andw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_AND, (dreg), (sreg), 2)
#define jx86_andl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_AND, (breg), (disp), (simm), 4)
#define jx86_andl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_AND, (breg), (disp), (sreg), 4)
#define jx86_andl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_AND, (dreg), (simm), 4)
#define jx86_andl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_AND, (dreg), (breg), (disp), 4)
#define jx86_andl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_AND, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_andq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_AND, (breg), (disp), (simm), 8)
# define jx86_andq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_AND, (breg), (disp), (sreg), 8)
# define jx86_andq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_AND, (dreg), (simm), 8)
# define jx86_andq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_AND, (dreg), (breg), (disp), 8)
# define jx86_andq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_AND, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_andn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_AND, (breg), (disp), (simm), JX86_NWS)
#define jx86_andn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_AND, (breg), (disp), (sreg), JX86_NWS)
#define jx86_andn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_AND, (dreg), (simm), JX86_NWS)
#define jx86_andn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_AND, (dreg), (breg), (disp), JX86_NWS)
#define jx86_andn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_AND, (dreg), (sreg), JX86_NWS)

#define jx86_subb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SUB, (breg), (disp), (simm), 1)
#define jx86_subb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SUB, (breg), (disp), (sreg), 1)
#define jx86_subb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SUB, (dreg), (simm), 1)
#define jx86_subb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SUB, (dreg), (breg), (disp), 1)
#define jx86_subb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SUB, (dreg), (sreg), 1)
#define jx86_subw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SUB, (breg), (disp), (simm), 2)
#define jx86_subw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SUB, (breg), (disp), (sreg), 2)
#define jx86_subw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SUB, (dreg), (simm), 2)
#define jx86_subw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SUB, (dreg), (breg), (disp), 2)
#define jx86_subw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SUB, (dreg), (sreg), 2)
#define jx86_subl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SUB, (breg), (disp), (simm), 4)
#define jx86_subl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SUB, (breg), (disp), (sreg), 4)
#define jx86_subl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SUB, (dreg), (simm), 4)
#define jx86_subl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SUB, (dreg), (breg), (disp), 4)
#define jx86_subl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SUB, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_subq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_SUB, (breg), (disp), (simm), 8)
# define jx86_subq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_SUB, (breg), (disp), (sreg), 8)
# define jx86_subq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_SUB, (dreg), (simm), 8)
# define jx86_subq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_SUB, (dreg), (breg), (disp), 8)
# define jx86_subq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_SUB, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_subn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_SUB, (breg), (disp), (simm), JX86_NWS)
#define jx86_subn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_SUB, (breg), (disp), (sreg), JX86_NWS)
#define jx86_subn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_SUB, (dreg), (simm), JX86_NWS)
#define jx86_subn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_SUB, (dreg), (breg), (disp), JX86_NWS)
#define jx86_subn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_SUB, (dreg), (sreg), JX86_NWS)

#define jx86_xorb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_XOR, (breg), (disp), (simm), 1)
#define jx86_xorb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_XOR, (breg), (disp), (sreg), 1)
#define jx86_xorb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_XOR, (dreg), (simm), 1)
#define jx86_xorb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_XOR, (dreg), (breg), (disp), 1)
#define jx86_xorb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_XOR, (dreg), (sreg), 1)
#define jx86_xorw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_XOR, (breg), (disp), (simm), 2)
#define jx86_xorw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_XOR, (breg), (disp), (sreg), 2)
#define jx86_xorw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_XOR, (dreg), (simm), 2)
#define jx86_xorw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_XOR, (dreg), (breg), (disp), 2)
#define jx86_xorw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_XOR, (dreg), (sreg), 2)
#define jx86_xorl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_XOR, (breg), (disp), (simm), 4)
#define jx86_xorl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_XOR, (breg), (disp), (sreg), 4)
#define jx86_xorl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_XOR, (dreg), (simm), 4)
#define jx86_xorl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_XOR, (dreg), (breg), (disp), 4)
#define jx86_xorl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_XOR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_xorq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_XOR, (breg), (disp), (simm), 8)
# define jx86_xorq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_XOR, (breg), (disp), (sreg), 8)
# define jx86_xorq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_XOR, (dreg), (simm), 8)
# define jx86_xorq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_XOR, (dreg), (breg), (disp), 8)
# define jx86_xorq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_XOR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_xorn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_XOR, (breg), (disp), (simm), JX86_NWS)
#define jx86_xorn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_XOR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_xorn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_XOR, (dreg), (simm), JX86_NWS)
#define jx86_xorn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_XOR, (dreg), (breg), (disp), JX86_NWS)
#define jx86_xorn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_XOR, (dreg), (sreg), JX86_NWS)

#define jx86_cmpb_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_CMP, (breg), (disp), (simm), 1)
#define jx86_cmpb_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_CMP, (breg), (disp), (sreg), 1)
#define jx86_cmpb_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_CMP, (dreg), (simm), 1)
#define jx86_cmpb_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_CMP, (dreg), (breg), (disp), 1)
#define jx86_cmpb_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_CMP, (dreg), (sreg), 1)
#define jx86_cmpw_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_CMP, (breg), (disp), (simm), 2)
#define jx86_cmpw_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_CMP, (breg), (disp), (sreg), 2)
#define jx86_cmpw_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_CMP, (dreg), (simm), 2)
#define jx86_cmpw_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_CMP, (dreg), (breg), (disp), 2)
#define jx86_cmpw_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_CMP, (dreg), (sreg), 2)
#define jx86_cmpl_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_CMP, (breg), (disp), (simm), 4)
#define jx86_cmpl_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_CMP, (breg), (disp), (sreg), 4)
#define jx86_cmpl_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_CMP, (dreg), (simm), 4)
#define jx86_cmpl_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_CMP, (dreg), (breg), (disp), 4)
#define jx86_cmpl_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_CMP, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_cmpq_membase_imm(cp, breg, disp, simm) jx86_alu_membase_imm((cp), JX86_CMP, (breg), (disp), (simm), 8)
# define jx86_cmpq_membase_reg(cp, breg, disp, sreg) jx86_alu_membase_reg((cp), JX86_CMP, (breg), (disp), (sreg), 8)
# define jx86_cmpq_reg_imm(cp, dreg, simm)           jx86_alu_reg_imm((cp), JX86_CMP, (dreg), (simm), 8)
# define jx86_cmpq_reg_membase(cp, dreg, breg, disp) jx86_alu_reg_membase((cp), JX86_CMP, (dreg), (breg), (disp), 8)
# define jx86_cmpq_reg_reg(cp, dreg, sreg)           jx86_alu_reg_reg((cp), JX86_CMP, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_cmpn_membase_imm(cp, breg, disp, simm)  jx86_alu_membase_imm((cp), JX86_CMP, (breg), (disp), (simm), JX86_NWS)
#define jx86_cmpn_membase_reg(cp, breg, disp, sreg)  jx86_alu_membase_reg((cp), JX86_CMP, (breg), (disp), (sreg), JX86_NWS)
#define jx86_cmpn_reg_imm(cp, dreg, simm)            jx86_alu_reg_imm((cp), JX86_CMP, (dreg), (simm), JX86_NWS)
#define jx86_cmpn_reg_membase(cp, dreg, breg, disp)  jx86_alu_reg_membase((cp), JX86_CMP, (dreg), (breg), (disp), JX86_NWS)
#define jx86_cmpn_reg_reg(cp, dreg, sreg)            jx86_alu_reg_reg((cp), JX86_CMP, (dreg), (sreg), JX86_NWS)


/* CALL instruction
 * ----------------
 */

#define jx86_call(cp, addr)                                     \
  do {                                                          \
    const jx86_uint8_t *_addr_ = (const jx86_uint8_t *) (addr); \
    const jx86_intptr_t _rel32_ = _addr_ - (cp) - 5;            \
    if (JX86_IS_IMM32(_rel32_)) {                               \
      jx86_emit_uint8((cp), 0xe8);                              \
      jx86_emit_int32((cp), _rel32_);                           \
    }                                                           \
    else {                                                      \
      jx86_emit_uint32((cp), 0x000215ff);                       \
      jx86_emit_uint32((cp), 0x08eb0000);                       \
      jx86_emit_uint64((cp), (jx86_uintptr_t) _addr_);          \
    }                                                           \
  } while (0)

#define jx86_call_membase(cp, breg, disp, size)               \
  do {                                                        \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);      \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);  \
    const unsigned _size_ = (const unsigned) (size);          \
    jx86_emit_osp((cp), _size_);                              \
    jx86_emit_rex((cp), 0, 0, 0, _breg_);                     \
    jx86_emit_uint8((cp), 0xff);                              \
    jx86_emit_membase((cp), 2, _breg_, _disp_);               \
  } while (0)

#define jx86_call_reg(cp, reg, size)                    \
  do {                                                  \
    const jx86_reg_t _reg_ = (const jx86_reg_t) (reg);  \
    const unsigned _size_ = (const unsigned) (size);    \
    jx86_emit_osp((cp), _size_);                        \
    jx86_emit_rex((cp), 0, 0, 0, _reg_);                \
    jx86_emit_uint8((cp), 0xff);                        \
    jx86_emit_reg((cp), 2, _reg_);                      \
  } while (0)

#ifdef JX86_32
# define jx86_callw_membase(cp, breg, disp) jx86_call_membase((cp), (breg), (disp), 2)
# define jx86_callw_reg(cp, reg)            jx86_call_reg((cp), (reg), 2)
# define jx86_calll_membase(cp, breg, disp) jx86_call_membase((cp), (breg), (disp), 4)
# define jx86_calll_reg(cp, reg)            jx86_call_reg((cp), (reg), 4)
#else /* !JX86_32 */
# define jx86_callq_membase(cp, breg, disp) jx86_call_membase((cp), (breg), (disp), 8)
# define jx86_callq_reg(cp, reg)            jx86_call_reg((cp), (reg), 8)
#endif /* JX86_32 */
#define jx86_calln_membase(cp, breg, disp)  jx86_call_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_calln_reg(cp, reg)             jx86_call_reg((cp), (reg), JX86_NWS)


/* CLD/STD instructions
 * --------------------
 */

#define jx86_cld(cp) jx86_emit_uint8((cp), 0xfc)
#define jx86_std(cp) jx86_emit_uint8((cp), 0xfd)


/* CWD/CDQ/CQO instructions
 * ------------------------
 */

#define jx86_cwd(cp)                            \
  jx86_emit_uint16((cp), 0x9966);

#define jx86_cdq(cp)                            \
  jx86_emit_uint8((cp), 0x99);

#ifdef JX86_64
# define jx86_cqo(cp)                           \
  jx86_emit_uint16((cp), 0x9948);
#endif


/* IDIV instruction
 * ----------------
 */

#define jx86_idiv_membase(cp, breg, disp, size)               \
  do {                                                        \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);      \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);  \
    const unsigned _size_ = (const unsigned) (size);          \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);             \
    jx86_emit_opcode1((cp), _size_, 0xf6);                    \
    jx86_emit_membase((cp), 7, _breg_, _disp_);               \
  } while (0)

#define jx86_idiv_reg(cp, sreg, size)                           \
  do {                                                          \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _sreg_);               \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_reg((cp), 7, _sreg_);                             \
  } while (0)

#define jx86_idivb_membase(cp, breg, disp)  jx86_idiv_membase((cp), (breg), (disp), 1)
#define jx86_idivb_reg(cp, sreg)            jx86_idiv_reg((cp), (sreg), 1)
#define jx86_idivw_membase(cp, breg, disp)  jx86_idiv_membase((cp), (breg), (disp), 2)
#define jx86_idivw_reg(cp, sreg)            jx86_idiv_reg((cp), (sreg), 2)
#define jx86_idivl_membase(cp, breg, disp)  jx86_idiv_membase((cp), (breg), (disp), 4)
#define jx86_idivl_reg(cp, sreg)            jx86_idiv_reg((cp), (sreg), 4)
#ifdef JX86_64
# define jx86_idivq_membase(cp, breg, disp) jx86_idiv_membase((cp), (breg), (disp), 8)
# define jx86_idivq_reg(cp, sreg)           jx86_idiv_reg((cp), (sreg), 8)
#endif
#define jx86_idivn_membase(cp, breg, disp)  jx86_idiv_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_idivn_reg(cp, sreg)            jx86_idiv_reg((cp), (sreg), JX86_NWS)


/* IMUL instruction
 * ----------------
 */

#define jx86_imul_membase(cp, breg, disp, size)                 \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);               \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_membase((cp), 5, _breg_, _disp_);                 \
  } while (0)

#define jx86_imul_reg(cp, sreg, size)                           \
  do {                                                          \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _sreg_);               \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_reg((cp), 5, _sreg_);                             \
  } while (0)

#define jx86_imul_reg_membase(cp, dreg, breg, disp, size)       \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _breg_);          \
    jx86_emit_uint16((cp), 0xaf0f);                             \
    jx86_emit_membase((cp), _dreg_, _breg_, _disp_);            \
  } while (0)

#define jx86_imul_reg_reg(cp, dreg, sreg, size)                 \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _sreg_);          \
    jx86_emit_uint16((cp), 0xaf0f);                             \
    jx86_emit_reg((cp), _dreg_, _sreg_);                        \
  } while (0)

#define jx86_imulb_membase(cp, breg, disp)            jx86_imul_membase((cp), (breg), (disp), 1)
#define jx86_imulb_reg(cp, sreg)                      jx86_imul_reg((cp), (sreg), 1)
#define jx86_imulw_membase(cp, breg, disp)            jx86_imul_membase((cp), (breg), (disp), 2)
#define jx86_imulw_reg(cp, sreg)                      jx86_imul_reg((cp), (sreg), 2)
#define jx86_imulw_reg_membase(cp, dreg, breg, disp)  jx86_imul_reg_membase((cp), (dreg), (breg), (disp), 2)
#define jx86_imulw_reg_reg(cp, dreg, sreg)            jx86_imul_reg_reg((cp), (dreg), (sreg), 2)
#define jx86_imull_membase(cp, breg, disp)            jx86_imul_membase((cp), (breg), (disp), 4)
#define jx86_imull_reg(cp, sreg)                      jx86_imul_reg((cp), (sreg), 4)
#define jx86_imull_reg_membase(cp, dreg, breg, disp)  jx86_imul_reg_membase((cp), (dreg), (breg), (disp), 4)
#define jx86_imull_reg_reg(cp, dreg, sreg)            jx86_imul_reg_reg((cp), (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_imulq_membase(cp, breg, disp)           jx86_imul_membase((cp), (breg), (disp), 8)
# define jx86_imulq_reg(cp, sreg)                     jx86_imul_reg((cp), (sreg), 8)
# define jx86_imulq_reg_membase(cp, dreg, breg, disp) jx86_imul_reg_membase((cp), (dreg), (breg), (disp), 8)
# define jx86_imulq_reg_reg(cp, dreg, sreg)           jx86_imul_reg_reg((cp), (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_imuln_membase(cp, breg, disp)            jx86_imul_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_imuln_reg(cp, sreg)                      jx86_imul_reg((cp), (sreg), JX86_NWS)
#define jx86_imuln_reg_membase(cp, dreg, breg, disp)  jx86_imul_reg_membase((cp), (dreg), (breg), (disp), JX86_NWS)
#define jx86_imuln_reg_reg(cp, dreg, sreg)            jx86_imul_reg_reg((cp), (dreg), (sreg), JX86_NWS)


/* Jcc instructions
 * ----------------
 */

typedef JX86_DECLARE_ENUM_CC(J, 0x70U) jx86_jcc_t;

#define jx86_jcc(cp, jcc, addr)                                         \
  do {                                                                  \
    const jx86_uint8_t *_addr_ = (const jx86_uint8_t *) (addr);         \
    if (JX86_IS_IMM8(_addr_ - (cp) - 2))                                \
      jx86_jcc8((cp), (jcc), _addr_);                                   \
    else                                                                \
      jx86_jcc32((cp), (jcc), _addr_);                                  \
  } while (0)

#define jx86_jcc8(cp, jcc, addr)                                        \
  do {                                                                  \
    const jx86_intptr_t _rel_ = (const jx86_uint8_t *) (addr) - (cp) - 2; \
    jx86_assert(JX86_IS_IMM8(_rel_));                                   \
    jx86_emit_uint8((cp), (jcc));                                       \
    jx86_emit_int8((cp), _rel_);                                        \
  } while (0)

#define jx86_jcc8_forward(cp, jcc)                                      \
  do {                                                                  \
    jx86_emit_uint8((cp), (jcc));                                       \
    (cp) += sizeof(jx86_int8_t);                                        \
  } while (0)

#define jx86_jcc8_patch(cp, jcp)                                        \
  do {                                                                  \
    jx86_uint8_t *_jcp_ = (jx86_uint8_t *) (jcp);                       \
    const jx86_intptr_t _rel_ = (cp) - _jcp_ - 2;                       \
    jx86_assert(JX86_IS_IMM8(_rel_));                                   \
    jx86_assert(_jcp_[0] >= 0x70 && _jcp_[0] <= 0x7f);                  \
    *((jx86_int8_t *) (_jcp_ + 1)) = _rel_;                             \
  } while (0)

#define jx86_jcc32(cp, jcc, addr)                                       \
  do {                                                                  \
    const jx86_intptr_t _rel_ = (const jx86_uint8_t *) (addr) - (cp) - 6; \
    jx86_assert(JX86_IS_IMM32(_rel_));                                  \
    jx86_emit_uint16((cp), (((jcc) + 0x10U) << 8) | 0x0fU);             \
    jx86_emit_int32((cp), _rel_);                                       \
  } while (0)

#define jx86_jcc32_forward(cp, jcc)                                     \
  do {                                                                  \
    jx86_emit_uint16((cp), (((jcc) + 0x10U) << 8) | 0x0fU);             \
    (cp) += sizeof(jx86_int32_t);                                       \
  } while (0)

#define jx86_jcc32_patch(cp, jcp)                                       \
  do {                                                                  \
    jx86_uint8_t *_jcp_ = (jx86_uint8_t *) (jcp);                       \
    const jx86_intptr_t _rel_ = (cp) - _jcp_ - 6;                       \
    jx86_assert(JX86_IS_IMM32(_rel_));                                  \
    jx86_assert(_jcp_[0] == 0x0f &&                                     \
                _jcp_[1] >= 0x80 && _jcp_[1] <= 0x8f);                  \
    *((jx86_int32_t *) (_jcp_ + 2)) = _rel_;                            \
  } while (0)

#define jx86_ja(cp, addr)       jx86_jcc((cp), JX86_JA, (addr))
#define jx86_ja8(cp, addr)      jx86_jcc8((cp), JX86_JA, (addr))
#define jx86_ja8_forward(cp)    jx86_jcc8_forward((cp), JX86_JA)
#define jx86_ja32(cp, addr)     jx86_jcc32((cp), JX86_JA, (addr))
#define jx86_ja32_forward(cp)   jx86_jcc32_forward((cp), JX86_JA)

#define jx86_jae(cp, addr)      jx86_jcc((cp), JX86_JAE, (addr))
#define jx86_jae8(cp, addr)     jx86_jcc8((cp), JX86_JAE, (addr))
#define jx86_jae8_forward(cp)   jx86_jcc8_forward((cp), JX86_JAE)
#define jx86_jae32(cp, addr)    jx86_jcc32((cp), JX86_JAE, (addr))
#define jx86_jae32_forward(cp)  jx86_jcc32_forward((cp), JX86_JAE)

#define jx86_jb(cp, addr)       jx86_jcc((cp), JX86_JB, (addr))
#define jx86_jb8(cp, addr)      jx86_jcc8((cp), JX86_JB, (addr))
#define jx86_jb8_forward(cp)    jx86_jcc8_forward((cp), JX86_JB)
#define jx86_jb32(cp, addr)     jx86_jcc32((cp), JX86_JB, (addr))
#define jx86_jb32_forward(cp)   jx86_jcc32_forward((cp), JX86_JB)

#define jx86_jbe(cp, addr)      jx86_jcc((cp), JX86_JBE, (addr))
#define jx86_jbe8(cp, addr)     jx86_jcc8((cp), JX86_JBE, (addr))
#define jx86_jbe8_forward(cp)   jx86_jcc((cp), JX86_JBE)
#define jx86_jbe32(cp, addr)    jx86_jcc((cp), JX86_JBE, (addr))
#define jx86_jbe32_forward(cp)  jx86_jcc((cp), JX86_JBE)

#define jx86_jc(cp, addr)       jx86_jcc((cp), JX86_JC, (addr))
#define jx86_jc8(cp, addr)      jx86_jcc8((cp), JX86_JC, (addr))
#define jx86_jc8_forward(cp)    jx86_jcc8_forward((cp), JX86_JC)
#define jx86_jc32(cp, addr)     jx86_jcc32((cp), JX86_JC, (addr))
#define jx86_jc32_forward(cp)   jx86_jcc32_forward((cp), JX86_JC)

#define jx86_je(cp, addr)       jx86_jcc((cp), JX86_JE, (addr))
#define jx86_je8(cp, addr)      jx86_jcc8((cp), JX86_JE, (addr))
#define jx86_je8_forward(cp)    jx86_jcc8_forward((cp), JX86_JE)
#define jx86_je32(cp, addr)     jx86_jcc32((cp), JX86_JE, (addr))
#define jx86_je32_forward(cp)   jx86_jcc32_forward((cp), JX86_JE)

#define jx86_jg(cp, addr)       jx86_jcc((cp), JX86_JG, (addr))
#define jx86_jg8(cp, addr)      jx86_jcc8((cp), JX86_JG, (addr))
#define jx86_jg8_forward(cp)    jx86_jcc8_forward((cp), JX86_JG)
#define jx86_jg32(cp, addr)     jx86_jcc32((cp), JX86_JG, (addr))
#define jx86_jg32_forward(cp)   jx86_jcc32_forward((cp), JX86_JG)

#define jx86_jge(cp, addr)      jx86_jcc((cp), JX86_JGE, (addr))
#define jx86_jge8(cp, addr)     jx86_jcc8((cp), JX86_JGE, (addr))
#define jx86_jge8_forward(cp)   jx86_jcc8_forward((cp), JX86_JGE)
#define jx86_jge32(cp, addr)    jx86_jcc32((cp), JX86_JGE, (addr))
#define jx86_jge32_forward(cp)  jx86_jcc32_forward((cp), JX86_JGE)

#define jx86_jl(cp, addr)       jx86_jcc((cp), JX86_JL, (addr))
#define jx86_jl8(cp, addr)      jx86_jcc8((cp), JX86_JL, (addr))
#define jx86_jl8_forward(cp)    jx86_jcc8_forward((cp), JX86_JL)
#define jx86_jl32(cp, addr)     jx86_jcc32((cp), JX86_JL, (addr))
#define jx86_jl32_forward(cp)   jx86_jcc32_forward((cp), JX86_JL)

#define jx86_jle(cp, addr)      jx86_jcc((cp), JX86_JLE, (addr))
#define jx86_jle8(cp, addr)     jx86_jcc8((cp), JX86_JLE, (addr))
#define jx86_jle8_forward(cp)   jx86_jcc8_forward((cp), JX86_JLE)
#define jx86_jle32(cp, addr)    jx86_jcc32((cp), JX86_JLE, (addr))
#define jx86_jle32_forward(cp)  jx86_jcc32_forward((cp), JX86_JLE)

#define jx86_jna(cp, addr)      jx86_jcc((cp), JX86_JNA, (addr))
#define jx86_jna8(cp, addr)     jx86_jcc8((cp), JX86_JNA, (addr))
#define jx86_jna8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNA)
#define jx86_jna32(cp, addr)    jx86_jcc32((cp), JX86_JNA, (addr))
#define jx86_jna32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNA)

#define jx86_jnae(cp, addr)     jx86_jcc((cp), JX86_JNAE, (addr))
#define jx86_jnae8(cp, addr)    jx86_jcc8((cp), JX86_JNAE, (addr))
#define jx86_jnae8_forward(cp)  jx86_jcc8_forward((cp), JX86_JNAE)
#define jx86_jnae32(cp, addr)   jx86_jcc32((cp), JX86_JNAE, (addr))
#define jx86_jnae32_forward(cp) jx86_jcc32_forward((cp), JX86_JNAE)

#define jx86_jnb(cp, addr)      jx86_jcc((cp), JX86_JNB, (addr))
#define jx86_jnb8(cp, addr)     jx86_jcc8((cp), JX86_JNB, (addr))
#define jx86_jnb8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNB)
#define jx86_jnb32(cp, addr)    jx86_jcc32((cp), JX86_JNB, (addr))
#define jx86_jnb32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNB)

#define jx86_jnbe(cp, addr)     jx86_jcc((cp), JX86_JNBE, (addr))
#define jx86_jnbe8(cp, addr)    jx86_jcc8((cp), JX86_JNBE, (addr))
#define jx86_jnbe8_forward(cp)  jx86_jcc8_forward((cp), JX86_JNBE)
#define jx86_jnbe32(cp, addr)   jx86_jcc32((cp), JX86_JNBE, (addr))
#define jx86_jnbe32_forward(cp) jx86_jcc32_forward((cp), JX86_JNBE)

#define jx86_jnc(cp, addr)      jx86_jcc((cp), JX86_JNC, (addr))
#define jx86_jnc8(cp, addr)     jx86_jcc8((cp), JX86_JNC, (addr))
#define jx86_jnc8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNC)
#define jx86_jnc32(cp, addr)    jx86_jcc32((cp), JX86_JNC, (addr))
#define jx86_jnc32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNC)

#define jx86_jne(cp, addr)      jx86_jcc((cp), JX86_JNE, (addr))
#define jx86_jne8(cp, addr)     jx86_jcc8((cp), JX86_JNE, (addr))
#define jx86_jne8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNE)
#define jx86_jne32(cp, addr)    jx86_jcc32((cp), JX86_JNE, (addr))
#define jx86_jne32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNE)

#define jx86_jng(cp, addr)      jx86_jcc((cp), JX86_JNG, (addr))
#define jx86_jng8(cp, addr)     jx86_jcc8((cp), JX86_JNG, (addr))
#define jx86_jng8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNG)
#define jx86_jng32(cp, addr)    jx86_jcc32((cp), JX86_JNG, (addr))
#define jx86_jng32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNG)

#define jx86_jnge(cp, addr)     jx86_jcc((cp), JX86_JNGE, (addr))
#define jx86_jnge8(cp, addr)    jx86_jcc8((cp), JX86_JNGE, (addr))
#define jx86_jnge8_forward(cp)  jx86_jcc8_forward((cp), JX86_JNGE)
#define jx86_jnge32(cp, addr)   jx86_jcc32((cp), JX86_JNGE, (addr))
#define jx86_jnge32_forward(cp) jx86_jcc32_forward((cp), JX86_JNGE)

#define jx86_jnl(cp, addr)      jx86_jcc((cp), JX86_JNL, (addr))
#define jx86_jnl8(cp, addr)     jx86_jcc8((cp), JX86_JNL, (addr))
#define jx86_jnl8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNL)
#define jx86_jnl32(cp, addr)    jx86_jcc32((cp), JX86_JNL, (addr))
#define jx86_jnl32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNL)

#define jx86_jnle(cp, addr)     jx86_jcc((cp), JX86_JNLE, (addr))
#define jx86_jnle8(cp, addr)    jx86_jcc8((cp), JX86_JNLE, (addr))
#define jx86_jnle8_forward(cp)  jx86_jcc8_forward((cp), JX86_JNLE)
#define jx86_jnle32(cp, addr)   jx86_jcc32((cp), JX86_JNLE, (addr))
#define jx86_jnle32_forward(cp) jx86_jcc32_forward((cp), JX86_JNLE)

#define jx86_jno(cp, addr)      jx86_jcc((cp), JX86_JNO, (addr))
#define jx86_jno8(cp, addr)     jx86_jcc8((cp), JX86_JNO, (addr))
#define jx86_jno8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNO)
#define jx86_jno32(cp, addr)    jx86_jcc32((cp), JX86_JNO, (addr))
#define jx86_jno32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNO)

#define jx86_jnp(cp, addr)      jx86_jcc((cp), JX86_JNP, (addr))
#define jx86_jnp8(cp, addr)     jx86_jcc8((cp), JX86_JNP, (addr))
#define jx86_jnp8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNP)
#define jx86_jnp32(cp, addr)    jx86_jcc32((cp), JX86_JNP, (addr))
#define jx86_jnp32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNP)

#define jx86_jns(cp, addr)      jx86_jcc((cp), JX86_JNS, (addr))
#define jx86_jns8(cp, addr)     jx86_jcc8((cp), JX86_JNS, (addr))
#define jx86_jns8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNS)
#define jx86_jns32(cp, addr)    jx86_jcc32((cp), JX86_JNS, (addr))
#define jx86_jns32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNS)

#define jx86_jnz(cp, addr)      jx86_jcc((cp), JX86_JNZ, (addr))
#define jx86_jnz8(cp, addr)     jx86_jcc8((cp), JX86_JNZ, (addr))
#define jx86_jnz8_forward(cp)   jx86_jcc8_forward((cp), JX86_JNZ)
#define jx86_jnz32(cp, addr)    jx86_jcc32((cp), JX86_JNZ, (addr))
#define jx86_jnz32_forward(cp)  jx86_jcc32_forward((cp), JX86_JNZ)

#define jx86_jo(cp, addr)       jx86_jcc((cp), JX86_JO, (addr))
#define jx86_jo8(cp, addr)      jx86_jcc8((cp), JX86_JO, (addr))
#define jx86_jo8_forward(cp)    jx86_jcc8_forward((cp), JX86_JO)
#define jx86_jo32(cp, addr)     jx86_jcc32((cp), JX86_JO, (addr))
#define jx86_jo32_forward(cp)   jx86_jcc32_forward((cp), JX86_JO)

#define jx86_jp(cp, addr)       jx86_jcc((cp), JX86_JP, (addr))
#define jx86_jp8(cp, addr)      jx86_jcc8((cp), JX86_JP, (addr))
#define jx86_jp8_forward(cp)    jx86_jcc8_forward((cp), JX86_JP)
#define jx86_jp32(cp, addr)     jx86_jcc32((cp), JX86_JP, (addr))
#define jx86_jp32_forward(cp)   jx86_jcc32_forward((cp), JX86_JP)

#define jx86_jpe(cp, addr)      jx86_jcc((cp), JX86_JPE, (addr))
#define jx86_jpe8(cp, addr)     jx86_jcc8((cp), JX86_JPE, (addr))
#define jx86_jpe8_forward(cp)   jx86_jcc8_forward((cp), JX86_JPE)
#define jx86_jpe32(cp, addr)    jx86_jcc32((cp), JX86_JPE, (addr))
#define jx86_jpe32_forward(cp)  jx86_jcc32_forward((cp), JX86_JPE)

#define jx86_jpo(cp, addr)      jx86_jcc((cp), JX86_JPO, (addr))
#define jx86_jpo8(cp, addr)     jx86_jcc8((cp), JX86_JPO, (addr))
#define jx86_jpo8_forward(cp)   jx86_jcc8_forward((cp), JX86_JPO)
#define jx86_jpo32(cp, addr)    jx86_jcc32((cp), JX86_JPO, (addr))
#define jx86_jpo32_forward(cp)  jx86_jcc32_forward((cp), JX86_JPO)

#define jx86_js(cp, addr)       jx86_jcc((cp), JX86_JS, (addr))
#define jx86_js8(cp, addr)      jx86_jcc8((cp), JX86_JS, (addr))
#define jx86_js8_forward(cp)    jx86_jcc8_forward((cp), JX86_JS)
#define jx86_js32(cp, addr)     jx86_jcc32((cp), JX86_JS, (addr))
#define jx86_js32_forward(cp)   jx86_jcc32_forward((cp), JX86_JS)

#define jx86_jz(cp, addr)       jx86_jcc((cp), JX86_JZ, (addr))
#define jx86_jz8(cp, addr)      jx86_jcc8((cp), JX86_JZ, (addr))
#define jx86_jz8_forward(cp)    jx86_jcc8_forward((cp), JX86_JZ)
#define jx86_jz32(cp, addr)     jx86_jcc32((cp), JX86_JZ, (addr))
#define jx86_jz32_forward(cp)   jx86_jcc32_forward((cp), JX86_JZ)


/* JMP instruction
 * ---------------
 */

#define jx86_jmp(cp, addr)                                      \
  do {                                                          \
    const jx86_uint8_t *_addr_ = (const jx86_uint8_t *) (addr); \
    const jx86_intptr_t _rel8_ = _addr_ - (cp) - 2;             \
    if (JX86_IS_IMM8(_rel8_)) {                                 \
      jx86_emit_uint16((cp), (_rel8_ << 8) | 0xeb);             \
    }                                                           \
    else {                                                      \
      const jx86_intptr_t _rel32_ = _rel8_ - 3;                 \
      if (JX86_IS_IMM32(_rel32_)) {                             \
        jx86_emit_uint8((cp), 0xe9);                            \
        jx86_emit_int32((cp), _rel32_);                         \
      }                                                         \
      else {                                                    \
        jx86_emit_uint16((cp), 0x25ff);                         \
        jx86_emit_int32((cp), 0);                               \
        jx86_emit_uint64((cp), (jx86_uintptr_t) _addr_);        \
      }                                                         \
    }                                                           \
  } while (0)

#define jx86_jmp_membase(cp, breg, disp, size)                  \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osp((cp), _size_);                                \
    jx86_emit_rex((cp), 0, 0, 0, _breg_);                       \
    jx86_emit_uint8((cp), 0xff);                                \
    jx86_emit_membase((cp), 4, _breg_, _disp_);                 \
  } while (0)

#define jx86_jmp_memindex(cp, breg, disp, ireg, shift, size)            \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osp((cp), _size_);                                        \
    jx86_emit_rex((cp), 0, 0, _ireg_, _breg_);                          \
    jx86_emit_uint8((cp), 0xff);                                        \
    jx86_emit_memindex((cp), 4, _breg_, _disp_, _ireg_, _shift_);       \
  } while (0)

#define jx86_jmp_reg(cp, reg, size)                     \
  do {                                                  \
    const jx86_reg_t _reg_ = (const jx86_reg_t) (reg);  \
    const unsigned _size_ = (const unsigned) (size);    \
    jx86_emit_osp((cp), _size_);                        \
    jx86_emit_rex((cp), 0, 0, 0, _reg_);                \
    jx86_emit_uint8((cp), 0xff);                        \
    jx86_emit_reg((cp), 4, _reg_);                      \
  } while (0)

#define jx86_jmp32_forward(cp)                          \
  do {                                                  \
    jx86_emit_uint8((cp), 0xe9);                        \
    (cp) += sizeof(jx86_int32_t);                       \
  } while (0)

#define jx86_jmp32_patch(cp, jcp)                       \
  do {                                                  \
    jx86_uint8_t *_jcp_ = (jx86_uint8_t *) (jcp);       \
    const jx86_intptr_t _rel_ = (cp) - _jcp_ - 5;       \
    jx86_assert(JX86_IS_IMM32(_rel_));                  \
    jx86_assert(_jcp_[0] == 0xe9);                      \
    *((jx86_int32_t *) (_jcp_ + 1)) = _rel_;            \
  } while (0)

#ifdef JX86_32
# define jx86_jmpw_membase(cp, breg, disp)               jx86_jmp_membase((cp), (breg), (disp), 2)
# define jx86_jmpw_memindex(cp, breg, disp, ireg, shift) jx86_jmp_memindex((cp), (breg), (disp), (ireg), (shift), 2)
# define jx86_jmpw_reg(cp, reg)                          jx86_jmp_reg((cp), (reg), 2)
# define jx86_jmpl_membase(cp, breg, disp)               jx86_jmp_membase((cp), (breg), (disp), 4)
# define jx86_jmpl_memindex(cp, breg, disp, ireg, shift) jx86_jmp_memindex((cp), (breg), (disp), (ireg), (shift), 4)
# define jx86_jmpl_reg(cp, reg)                          jx86_jmp_reg((cp), (reg), 4)
#else /* !JX86_32 */
# define jx86_jmpq_membase(cp, breg, disp)               jx86_jmp_membase((cp), (breg), (disp), 8)
# define jx86_jmpq_memindex(cp, breg, disp, ireg, shift) jx86_jmp_memindex((cp), (breg), (disp), (ireg), (shift), 8)
# define jx86_jmpq_reg(cp, reg)                          jx86_jmp_reg((cp), (reg), 8)
#endif
#define jx86_jmpn_membase(cp, breg, disp)                jx86_jmp_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_jmpn_memindex(cp, breg, disp, ireg, shift)  jx86_jmp_memindex((cp), (breg), (disp), (ireg), (shift), JX86_NWS)
#define jx86_jmpn_reg(cp, reg)                           jx86_jmp_reg((cp), (reg), JX86_NWS)


/* LEA instruction
 * ---------------
 */

#ifdef JX86_64
# define jx86_lean_reg_forward(cp, dreg)                         \
  do {                                                           \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);         \
    jx86_emit_rex((cp), 8, _dreg_, 0, 0);                        \
    jx86_emit_uint8((cp), 0x8d);                                 \
    jx86_emit_address_byte((cp), 0, _dreg_, 5);                  \
    (cp) += sizeof(jx86_int32_t);                                \
  } while (0)

# define jx86_lean_reg_patch(cp, lcp)                            \
  do {                                                           \
    jx86_uint8_t *_lcp_ = (jx86_uint8_t *) (lcp);                \
    const jx86_intptr_t _rel_ = (cp) - _lcp_ - 7;                \
    jx86_assert(JX86_IS_IMM32(_rel_));                           \
    jx86_assert(_lcp_[1] == 0x8d);                               \
    *((jx86_int32_t *) (_lcp_ + 3)) = _rel_;                     \
  } while (0)
#else
# define jx86_lean_reg_forward(cp, dreg)                         \
  do {                                                           \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);         \
    jx86_emit_uint8((cp), 0xb8 + JX86_REG_CODE(_dreg_));         \
    (cp) += sizeof(jx86_int32_t);                                \
  } while (0)

# define jx86_lean_reg_patch(cp, lcp)                            \
  do {                                                           \
    jx86_uint8_t *_lcp_ = (jx86_uint8_t *) (lcp);                \
    jx86_assert(_lcp_[0] >= 0xb8 && _lcp_[0] <= 0xbf);           \
    *((jx86_uint32_t *) (_lcp_ + 1)) = (jx86_uint32_t) (cp);     \
  } while (0)
#endif /* JX86_64 */

#define jx86_lea_reg_membase(cp, dreg, breg, disp, size)         \
  do {                                                           \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);         \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);         \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);     \
    const unsigned _size_ = (const unsigned) (size);             \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _breg_);           \
    jx86_emit_uint8((cp), 0x8d);                                 \
    jx86_emit_membase((cp), _dreg_, _breg_, _disp_);             \
  } while (0)

#define jx86_lea_reg_memindex(cp, dreg, breg, disp, ireg, shift, size)  \
  do {                                                                  \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, _dreg_, _ireg_, _breg_);             \
    jx86_emit_uint8((cp), 0x8d);                                        \
    jx86_emit_memindex((cp), _dreg_, _breg_, _disp_, _ireg_, _shift_);  \
  } while (0)

#define jx86_leaw_reg_membase(cp, dreg, breg, disp)                 jx86_lea_reg_membase((cp), (dreg), (breg), (disp), 2)
#define jx86_leaw_reg_memindex(cp, dreg, breg, disp, ireg, shift)   jx86_lea_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 2)
#define jx86_leal_reg_membase(cp, dreg, breg, disp)                 jx86_lea_reg_membase((cp), (dreg), (breg), (disp), 4)
#define jx86_leal_reg_memindex(cp, dreg, breg, disp, ireg, shift)   jx86_lea_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 4)
#ifdef JX86_64
# define jx86_leaq_reg_membase(cp, dreg, breg, disp)                jx86_lea_reg_membase((cp), (dreg), (breg), (disp), 8)
# define jx86_leaq_reg_memindex(cp, dreg, breg, disp, ireg, shift)  jx86_lea_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 8)
#endif
#define jx86_lean_reg_membase(cp, dreg, breg, disp)                 jx86_lea_reg_membase((cp), (dreg), (breg), (disp), JX86_NWS)
#define jx86_lean_reg_memindex(cp, dreg, breg, disp, ireg, shift)   jx86_lea_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), JX86_NWS)


/* MOV instruction
 * ---------------
 */

#ifdef JX86_32
# define jx86_mov_mem_reg(cp, dmem, sreg, size)                 \
  do {                                                          \
    const jx86_int32_t _dmem_ = (const jx86_int32_t) (dmem);    \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _sreg_);               \
    jx86_emit_opcode1((cp), _size_, 0x88);                      \
    jx86_emit_mem((cp), _sreg_, _dmem_);                        \
  } while (0)
#endif /* JX86_32 */

#define jx86_mov_membase_imm(cp, breg, disp, simm, size)                \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);            \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_ == 1 ? 0 : _size_, 0, 0, _breg_);     \
    jx86_emit_opcode1((cp), _size_, 0xc6);                              \
    jx86_emit_membase((cp), 0, _breg_, _disp_);                         \
    jx86_emit_imm((cp), _size_, _simm_);                                \
  } while (0)

#define jx86_mov_membase_reg(cp, breg, disp, sreg, size)        \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _sreg_, 0, _breg_);          \
    jx86_emit_opcode1((cp), _size_, 0x88);                      \
    jx86_emit_membase((cp), _sreg_, _breg_, _disp_);            \
  } while (0)

#define jx86_mov_memindex_imm(cp, breg, disp, ireg, shift, simm, size)  \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);            \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, 0, _ireg_, _breg_);                  \
    jx86_emit_opcode1((cp), _size_, 0xc6);                              \
    jx86_emit_memindex((cp), 0, _breg_, _disp_, _ireg_, _shift_);       \
    jx86_emit_imm((cp), _size_, _simm_);                                \
  } while (0)

#define jx86_mov_memindex_reg(cp, breg, disp, ireg, shift, sreg, size)  \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);                \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, _sreg_, _ireg_, _breg_);             \
    jx86_emit_opcode1((cp), _size_, 0x88);                              \
    jx86_emit_memindex((cp), _sreg_, _breg_, _disp_, _ireg_, _shift_);  \
  } while (0)

#define jx86_mov_reg_imm(cp, dreg, simm, size)                          \
  do {                                                                  \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);                \
    const jx86_intptr_t _simm_ = (const jx86_intptr_t) (simm);          \
    const unsigned _size_ = (const unsigned) (size);                    \
    if (_size_ == 8 && _simm_ == (_simm_ & 0xffffffffL)) {              \
      /* x86_64 zero-extends results of 32bit operations */             \
      jx86_emit_rex((cp), 4, 0, 0, _dreg_);                             \
      jx86_emit_uint8((cp), 0xb8 + JX86_REG_CODE(_dreg_));              \
      jx86_emit_uint32((cp), _simm_);                                   \
    }                                                                   \
    else {                                                              \
      jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);                     \
      if (_size_ == 1) {                                                \
        jx86_emit_uint8((cp), 0xb0 + JX86_REG_CODE(_dreg_));            \
        jx86_emit_int8((cp), _simm_);                                   \
      }                                                                 \
      else if (_size_ == 2) {                                           \
        jx86_emit_uint8((cp), 0xb8 + JX86_REG_CODE(_dreg_));            \
        jx86_emit_int16((cp), _simm_);                                  \
      }                                                                 \
      else if (_size_ == 4) {                                           \
        jx86_emit_uint8((cp), 0xb8 + JX86_REG_CODE(_dreg_));            \
        jx86_emit_int32((cp), _simm_);                                  \
      }                                                                 \
      else {                                                            \
        jx86_emit_uint8((cp), 0xb8 + JX86_REG_CODE(_dreg_));            \
        jx86_emit_int64((cp), _simm_);                                  \
      }                                                                 \
    }                                                                   \
  } while (0)

#ifdef JX86_32
# define jx86_mov_reg_mem(cp, dreg, smem, size)                 \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_int32_t _smem_ = (const jx86_int32_t) (smem);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);               \
    jx86_emit_opcode1((cp), _size_, 0x8a);                      \
    jx86_emit_mem((cp), _dreg_, _smem_);                        \
  } while (0)
#endif /* JX86_32 */

#define jx86_mov_reg_membase(cp, dreg, breg, disp, size)              \
  do {                                                                \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);              \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);              \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);          \
    const unsigned _size_ = (const unsigned) (size);                  \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _breg_);                \
    jx86_emit_opcode1((cp), _size_, 0x8a);                            \
    jx86_emit_membase((cp), _dreg_, _breg_, _disp_);                  \
  } while (0)

#define jx86_mov_reg_memindex(cp, dreg, breg, disp, ireg, shift, size)  \
  do {                                                                  \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const unsigned _size_ = (const unsigned) (size);                    \
    jx86_emit_osprex((cp), _size_, _dreg_, _ireg_, _breg_);             \
    jx86_emit_opcode1((cp), _size_, 0x8a);                              \
    jx86_emit_memindex((cp), _dreg_, _breg_, _disp_, _ireg_, _shift_);  \
  } while (0)

#define jx86_mov_reg_reg(cp, dreg, sreg, size)                  \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _sreg_);          \
    jx86_emit_opcode1((cp), _size_, 0x8a);                      \
    jx86_emit_reg((cp), _dreg_, _sreg_);                        \
  } while (0)

#ifdef JX86_32
# define jx86_movb_mem_reg(cp, dmem, sreg)                         jx86_mov_mem_reg((cp), (dmem), (sreg), 1)
#endif
#define jx86_movb_membase_imm(cp, breg, disp, simm)                jx86_mov_membase_imm((cp), (breg), (disp), (simm), 1)
#define jx86_movb_membase_reg(cp, breg, disp, sreg)                jx86_mov_membase_reg((cp), (breg), (disp), (sreg), 1)
#define jx86_movb_memindex_imm(cp, breg, disp, ireg, shift, simm)  jx86_mov_memindex_imm((cp), (breg), (disp), (ireg), (shift), (simm), 1)
#define jx86_movb_memindex_reg(cp, breg, disp, ireg, shift, sreg)  jx86_mov_memindex_reg((cp), (breg), (disp), (ireg), (shift), (sreg), 1)
#define jx86_movb_reg_imm(cp, dreg, simm)                          jx86_mov_reg_imm((cp), (dreg), (simm), 1)
#ifdef JX86_32
# define jx86_movb_reg_mem(cp, dreg, smem)                         jx86_mov_reg_mem((cp), (dreg), (smem), 1)
#endif
#define jx86_movb_reg_membase(cp, dreg, breg, disp)                jx86_mov_reg_membase((cp), (dreg), (breg), (disp), 1)
#define jx86_movb_reg_memindex(cp, dreg, breg, disp, ireg, shift)  jx86_mov_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 1)
#define jx86_movb_reg_reg(cp, dreg, sreg)                          jx86_mov_reg_reg((cp), (dreg), (sreg), 1)
#ifdef JX86_32
# define jx86_movw_mem_reg(cp, dmem, sreg)                         jx86_mov_mem_reg((cp), (dmem), (sreg), 2)
#endif
#define jx86_movw_membase_imm(cp, breg, disp, simm)                jx86_mov_membase_imm((cp), (breg), (disp), (simm), 2)
#define jx86_movw_membase_reg(cp, breg, disp, sreg)                jx86_mov_membase_reg((cp), (breg), (disp), (sreg), 2)
#define jx86_movw_memindex_imm(cp, breg, disp, ireg, shift, simm)  jx86_mov_memindex_imm((cp), (breg), (disp), (ireg), (shift), (simm), 2)
#define jx86_movw_memindex_reg(cp, breg, disp, ireg, shift, sreg)  jx86_mov_memindex_reg((cp), (breg), (disp), (ireg), (shift), (sreg), 2)
#define jx86_movw_reg_imm(cp, dreg, simm)                          jx86_mov_reg_imm((cp), (dreg), (simm), 2)
#ifdef JX86_32
# define jx86_movw_reg_mem(cp, dreg, smem)                         jx86_mov_reg_mem((cp), (dreg), (smem), 2)
#endif
#define jx86_movw_reg_membase(cp, dreg, breg, disp)                jx86_mov_reg_membase((cp), (dreg), (breg), (disp), 2)
#define jx86_movw_reg_memindex(cp, dreg, breg, disp, ireg, shift)  jx86_mov_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 2)
#define jx86_movw_reg_reg(cp, dreg, sreg)                          jx86_mov_reg_reg((cp), (dreg), (sreg), 2)
#ifdef JX86_32
# define jx86_movl_mem_reg(cp, dmem, sreg)                         jx86_mov_mem_reg((cp), (dmem), (sreg), 4)
#endif
#define jx86_movl_membase_imm(cp, breg, disp, simm)                jx86_mov_membase_imm((cp), (breg), (disp), (simm), 4)
#define jx86_movl_membase_reg(cp, breg, disp, sreg)                jx86_mov_membase_reg((cp), (breg), (disp), (sreg), 4)
#define jx86_movl_memindex_imm(cp, breg, disp, ireg, shift, simm)  jx86_mov_memindex_imm((cp), (breg), (disp), (ireg), (shift), (simm), 4)
#define jx86_movl_memindex_reg(cp, breg, disp, ireg, shift, sreg)  jx86_mov_memindex_reg((cp), (breg), (disp), (ireg), (shift), (sreg), 4)
#define jx86_movl_reg_imm(cp, dreg, simm)                          jx86_mov_reg_imm((cp), (dreg), (simm), 4)
#ifdef JX86_32
# define jx86_movl_reg_mem(cp, dreg, smem)                         jx86_mov_reg_mem((cp), (dreg), (smem), 4)
#endif
#define jx86_movl_reg_membase(cp, dreg, breg, disp)                jx86_mov_reg_membase((cp), (dreg), (breg), (disp), 4)
#define jx86_movl_reg_memindex(cp, dreg, breg, disp, ireg, shift)  jx86_mov_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 4)
#define jx86_movl_reg_reg(cp, dreg, sreg)                          jx86_mov_reg_reg((cp), (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_movq_membase_imm(cp, breg, disp, simm)               jx86_mov_membase_imm((cp), (breg), (disp), (simm), 8)
# define jx86_movq_membase_reg(cp, breg, disp, sreg)               jx86_mov_membase_reg((cp), (breg), (disp), (sreg), 8)
# define jx86_movq_memindex_imm(cp, breg, disp, ireg, shift, simm) jx86_mov_memindex_imm((cp), (breg), (disp), (ireg), (shift), (simm), 8)
# define jx86_movq_memindex_reg(cp, breg, disp, ireg, shift, sreg) jx86_mov_memindex_reg((cp), (breg), (disp), (ireg), (shift), (sreg), 8)
# define jx86_movq_reg_imm(cp, dreg, simm)                         jx86_mov_reg_imm((cp), (dreg), (simm), 8)
# define jx86_movq_reg_membase(cp, dreg, breg, disp)               jx86_mov_reg_membase((cp), (dreg), (breg), (disp), 8)
# define jx86_movq_reg_memindex(cp, dreg, breg, disp, ireg, shift) jx86_mov_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), 8)
# define jx86_movq_reg_reg(cp, dreg, sreg)                         jx86_mov_reg_reg((cp), (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_movn_membase_imm(cp, breg, disp, simm)                jx86_mov_membase_imm((cp), (breg), (disp), (simm), JX86_NWS)
#define jx86_movn_membase_reg(cp, breg, disp, sreg)                jx86_mov_membase_reg((cp), (breg), (disp), (sreg), JX86_NWS)
#define jx86_movn_memindex_imm(cp, breg, disp, ireg, shift, simm)  jx86_mov_memindex_imm((cp), (breg), (disp), (ireg), (shift), (simm), JX86_NWS)
#define jx86_movn_memindex_reg(cp, breg, disp, ireg, shift, sreg)  jx86_mov_memindex_reg((cp), (breg), (disp), (ireg), (shift), (sreg), JX86_NWS)
#define jx86_movn_reg_imm(cp, dreg, simm)                          jx86_mov_reg_imm((cp), (dreg), (simm), JX86_NWS)
#define jx86_movn_reg_membase(cp, dreg, breg, disp)                jx86_mov_reg_membase((cp), (dreg), (breg), (disp), JX86_NWS)
#define jx86_movn_reg_memindex(cp, dreg, breg, disp, ireg, shift)  jx86_mov_reg_memindex((cp), (dreg), (breg), (disp), (ireg), (shift), JX86_NWS)
#define jx86_movn_reg_reg(cp, dreg, sreg)                          jx86_mov_reg_reg((cp), (dreg), (sreg), JX86_NWS)


/* MOVZX instruction
 * -----------------
 */

#define jx86_movzxlb_reg_membase(cp, dreg, breg, disp)          \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    jx86_emit_osprex((cp), 4, _dreg_, 0, _breg_);               \
    jx86_emit_uint8((cp), 0x0f);                                \
    jx86_emit_uint8((cp), 0xb6);                                \
    jx86_emit_membase((cp), _dreg_, _breg_, _disp_);            \
  } while (0)

#define jx86_movzxlb_reg_reg(cp, dreg, sreg)                    \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    jx86_emit_osprex((cp), 4, _dreg_, 0, _sreg_);               \
    jx86_emit_uint8((cp), 0x0f);                                \
    jx86_emit_uint8((cp), 0xb6);                                \
    jx86_emit_reg((cp), _dreg_, _sreg_);                        \
  } while (0)


/* NEG instruction
 * ---------------
 */

#define jx86_neg_reg(cp, reg, size)                             \
  do {                                                          \
    const jx86_reg_t _reg_ = (const jx86_reg_t) (reg);          \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _reg_);                \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_reg((cp), 3, _reg_);                              \
  } while (0)

#define jx86_negb_reg(cp, reg)  jx86_neg_reg((cp), (reg), 1)
#define jx86_negw_reg(cp, reg)  jx86_neg_reg((cp), (reg), 2)
#define jx86_negl_reg(cp, reg)  jx86_neg_reg((cp), (reg), 4)
#ifdef JX86_64
# define jx86_negq_reg(cp, reg) jx86_neg_reg((cp), (reg), 8)
#endif /* JX86_64 */
#define jx86_negn_reg(cp, reg)  jx86_neg_reg((cp), (reg), JX86_NWS)


/* POP instruction
 * ---------------
 */

#define jx86_pop_membase(cp, breg, disp, size)                  \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osp((cp), _size_);                                \
    jx86_emit_rex((cp), 0, 0, 0, _breg_);                       \
    jx86_emit_uint8((cp), 0x8f);                                \
    jx86_emit_membase((cp), 0, _breg_, _disp_);                 \
  } while (0)

#define jx86_pop_reg(cp, dreg, size)                            \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osp((cp), _size_);                                \
    jx86_emit_rex((cp), 0, 0, 0, _dreg_);                       \
    jx86_emit_uint8((cp), 0x58 + JX86_REG_CODE(_dreg_));        \
  } while (0)

#define jx86_popw_membase(cp, breg, disp)  jx86_pop_membase((cp), (breg), (disp), 2)
#define jx86_popw_reg(cp, dreg)            jx86_pop_reg((cp), (dreg), 2)
#ifndef JX86_64
# define jx86_popl_membase(cp, breg, disp) jx86_pop_membase((cp), (breg), (disp), 4)
# define jx86_popl_reg(cp, dreg)           jx86_pop_reg((cp), (dreg), 4)
#else /* JX86_64 */
# define jx86_popq_membase(cp, breg, disp) jx86_pop_membase((cp), (breg), (disp), 8)
# define jx86_popq_reg(cp, dreg)           jx86_pop_reg((cp), (dreg), 8)
#endif /* !JX86_64 */
#define jx86_popn_membase(cp, breg, disp)  jx86_pop_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_popn_reg(cp, dreg)            jx86_pop_reg((cp), (dreg), JX86_NWS)


/* PUSH instruction
 * ----------------
 */

#define jx86_push_imm(cp, simm)                                 \
  do {                                                          \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);    \
    if (JX86_IS_IMM8(_simm_)) {                                 \
      jx86_emit_uint8((cp), 0x6a);                              \
      jx86_emit_int8((cp), _simm_);                             \
    }                                                           \
    else {                                                      \
      jx86_emit_uint8((cp), 0x68);                              \
      jx86_emit_int32((cp), _simm_);                            \
    }                                                           \
  } while (0)

#define jx86_push_membase(cp, breg, disp, size)                 \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osp((cp), _size_);                                \
    jx86_emit_rex((cp), 0, 0, 0, _breg_);                       \
    jx86_emit_uint8((cp), 0xff);                                \
    jx86_emit_membase((cp), 6, _breg_, _disp_);                 \
  } while (0)

#define jx86_push_reg(cp, sreg, size)                           \
  do {                                                          \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osp((cp), _size_);                                \
    jx86_emit_rex((cp), 0, 0, 0, _sreg_);                       \
    jx86_emit_uint8((cp), 0x50 + JX86_REG_CODE(_sreg_));        \
  } while (0)

#define jx86_pushw_membase(cp, breg, disp)  jx86_push_membase((cp), (breg), (disp), 2)
#define jx86_pushw_reg(cp, sreg)            jx86_push_reg((cp), (sreg), 2)
#ifndef JX86_64
# define jx86_pushl_membase(cp, breg, disp) jx86_push_membase((cp), (breg), (disp), 4)
# define jx86_pushl_reg(cp, sreg)           jx86_push_reg((cp), (sreg), 4)
#else /* JX86_64 */
# define jx86_pushq_membase(cp, breg, disp) jx86_push_membase((cp), (breg), (disp), 8)
# define jx86_pushq_reg(cp, sreg)           jx86_push_reg((cp), (sreg), 8)
#endif /* !JX86_64 */
#define jx86_pushn_membase(cp, breg, disp)  jx86_push_membase((cp), (breg), (disp), JX86_NWS)
#define jx86_pushn_reg(cp, sreg)            jx86_push_reg((cp), (sreg), JX86_NWS)


/* REP MOVS instructions
 * ---------------------
 */

#define jx86_rep_movsl(cp)                      \
  do {                                          \
    jx86_emit_uint8((cp), 0xf3);                \
    jx86_emit_uint8((cp), 0xa5);                \
  } while (0)
#ifdef JX86_64
# define jx86_rep_movsq(cp)                     \
  do {                                          \
    jx86_emit_uint8((cp), 0xf3);                \
    jx86_emit_rex((cp), 8, 0, 0, 0);            \
    jx86_emit_uint8((cp), 0xa5);                \
  } while (0)
# define jx86_rep_movsn(cp)                     \
  jx86_rep_movsq((cp))
#else
# define jx86_rep_movsn(cp)                     \
  jx86_rep_movsl((cp))
#endif


/* RET instruction
 * ---------------
 */

#define jx86_ret(cp) jx86_emit_uint8((cp), 0xc3)


/* SET instructions
 * ----------------
 */

typedef JX86_DECLARE_ENUM_CC(SET, 0x90U) jx86_setcc_t;

#define jx86_setcc_membase(cp, setcc, breg, disp)               \
  do {                                                          \
    const jx86_setcc_t _setcc_ = (const jx86_setcc_t) (setcc);  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    jx86_emit_rex((cp), 0, 0, 0, _breg_);                       \
    jx86_emit_uint8((cp), 0x0f);                                \
    jx86_emit_uint8((cp), _setcc_);                             \
    jx86_emit_membase((cp), 0, _breg_, _disp_);                 \
  } while (0)

#define jx86_setcc_reg(cp, setcc, dreg)                         \
  do {                                                          \
    const jx86_setcc_t _setcc_ = (const jx86_setcc_t) (setcc);  \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    jx86_emit_rex((cp), 1, 0, 0, _dreg_);                       \
    jx86_emit_uint8((cp), 0x0f);                                \
    jx86_emit_uint8((cp), _setcc_);                             \
    jx86_emit_reg((cp), 0, _dreg_);                             \
  } while (0)

#define jx86_seta_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETA, (breg), (disp))
#define jx86_seta_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETA, (dreg))
#define jx86_setae_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETAE, (breg), (disp))
#define jx86_setae_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETAE, (dreg))
#define jx86_setb_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETB, (breg), (disp))
#define jx86_setb_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETB, (dreg))
#define jx86_setbe_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETBE, (breg), (disp))
#define jx86_setbe_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETBE, (dreg))
#define jx86_setc_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETC, (breg), (disp))
#define jx86_setc_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETC, (dreg))
#define jx86_sete_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETE, (breg), (disp))
#define jx86_sete_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETE, (dreg))
#define jx86_setg_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETG, (breg), (disp))
#define jx86_setg_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETG, (dreg))
#define jx86_setge_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETGE, (breg), (disp))
#define jx86_setge_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETGE, (dreg))
#define jx86_setl_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETL, (breg), (disp))
#define jx86_setl_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETL, (dreg))
#define jx86_setle_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETLE, (breg), (disp))
#define jx86_setle_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETLE, (dreg))
#define jx86_setna_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNA, (breg), (disp))
#define jx86_setna_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNA, (dreg))
#define jx86_setnae_membase(cp, breg, disp) jx86_setcc_membase((cp), JX86_SETNAE, (breg), (disp))
#define jx86_setnae_reg(cp, dreg)           jx86_setcc_reg((cp), JX86_SETNAE, (dreg))
#define jx86_setnb_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNB, (breg), (disp))
#define jx86_setnb_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNB, (dreg))
#define jx86_setnbe_membase(cp, breg, disp) jx86_setcc_membase((cp), JX86_SETNBE, (breg), (disp))
#define jx86_setnbe_reg(cp, dreg)           jx86_setcc_reg((cp), JX86_SETNBE, (dreg))
#define jx86_setnc_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNC, (breg), (disp))
#define jx86_setnc_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNC, (dreg))
#define jx86_setne_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNE, (breg), (disp))
#define jx86_setne_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNE, (dreg))
#define jx86_setng_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNG, (breg), (disp))
#define jx86_setng_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNG, (dreg))
#define jx86_setnge_membase(cp, breg, disp) jx86_setcc_membase((cp), JX86_SETNGE, (breg), (disp))
#define jx86_setnge_reg(cp, dreg)           jx86_setcc_reg((cp), JX86_SETNGE, (dreg))
#define jx86_setnl_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNL, (breg), (disp))
#define jx86_setnl_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNL, (dreg))
#define jx86_setnle_membase(cp, breg, disp) jx86_setcc_membase((cp), JX86_SETNLE, (breg), (disp))
#define jx86_setnle_reg(cp, dreg)           jx86_setcc_reg((cp), JX86_SETNLE, (dreg))
#define jx86_setno_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNO, (breg), (disp))
#define jx86_setno_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNO, (dreg))
#define jx86_setnp_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNP, (breg), (disp))
#define jx86_setnp_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNP, (dreg))
#define jx86_setns_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNS, (breg), (disp))
#define jx86_setns_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNS, (dreg))
#define jx86_setnz_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETNZ, (breg), (disp))
#define jx86_setnz_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETNZ, (dreg))
#define jx86_seto_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETO, (breg), (disp))
#define jx86_seto_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETO, (dreg))
#define jx86_setp_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETP, (breg), (disp))
#define jx86_setp_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETP, (dreg))
#define jx86_setpe_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETPE, (breg), (disp))
#define jx86_setpe_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETPE, (dreg))
#define jx86_setpo_membase(cp, breg, disp)  jx86_setcc_membase((cp), JX86_SETPO, (breg), (disp))
#define jx86_setpo_reg(cp, dreg)            jx86_setcc_reg((cp), JX86_SETPO, (dreg))
#define jx86_sets_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETS, (breg), (disp))
#define jx86_sets_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETS, (dreg))
#define jx86_setz_membase(cp, breg, disp)   jx86_setcc_membase((cp), JX86_SETZ, (breg), (disp))
#define jx86_setz_reg(cp, dreg)             jx86_setcc_reg((cp), JX86_SETZ, (dreg))


/* SHIFT instructions
 * ------------------
 */

typedef enum
{
  JX86_ROL = 0,
  JX86_ROR = 1,
  JX86_RCL = 2,
  JX86_RCR = 3,
  JX86_SHL = 4,
  JX86_SHR = 5,
  JX86_SAR = 7
} jx86_shf_t;

#define jx86_shf_membase_imm(cp, shf, breg, disp, simm, size)   \
  do {                                                          \
    const jx86_shf_t _shf_ = (const jx86_shf_t) (shf);          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const unsigned _simm_ = (const unsigned) (simm);            \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);               \
    jx86_emit_opcode1((cp), _size_, _simm_ == 1 ? 0xd0 : 0xc0); \
    jx86_emit_membase((cp), _shf_, _breg_, _disp_);             \
    if (_simm_ != 1)                                            \
      jx86_emit_uint8((cp), _simm_);                            \
  } while (0)

#define jx86_shf_membase_reg(cp, shf, breg, disp, sreg, size)   \
  do {                                                          \
    const jx86_shf_t _shf_ = (const jx86_shf_t) (shf);          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_assert(_sreg_ == JX86_CL);                             \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);               \
    jx86_emit_opcode1((cp), _size_, 0xd2);                      \
    jx86_emit_membase((cp), _shf_, _breg_, _disp_);             \
  } while (0)

#define jx86_shf_reg_imm(cp, shf, dreg, simm, size)             \
  do {                                                          \
    const jx86_shf_t _shf_ = (const jx86_shf_t) (shf);          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const unsigned _simm_ = (const unsigned) (simm);            \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);               \
    jx86_emit_opcode1((cp), _size_, _simm_ == 1 ? 0xd0 : 0xc0); \
    jx86_emit_reg((cp), _shf_, _dreg_);                         \
    if (_simm_ != 1)                                            \
      jx86_emit_uint8((cp), _simm_);                            \
  } while (0)

#define jx86_shf_reg_reg(cp, shf, dreg, sreg, size)             \
  do {                                                          \
    const jx86_shf_t _shf_ = (const jx86_shf_t) (shf);          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_assert(_sreg_ == JX86_CL);                             \
    jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);               \
    jx86_emit_opcode1((cp), _size_, 0xd2);                      \
    jx86_emit_reg((cp), _shf_, _dreg_);                         \
  } while (0)

#define jx86_shfb_membase_imm(cp, shf, breg, disp, simm)  jx86_shf_membase_imm((cp), (shf), (breg), (disp), (simm), 1)
#define jx86_shfb_membase_reg(cp, shf, breg, disp, simm)  jx86_shf_membase_reg((cp), (shf), (breg), (disp), (sreg), 1)
#define jx86_shfb_reg_imm(cp, shf, dreg, simm)            jx86_shf_reg_imm((cp), (shf), (dreg), (simm), 1)
#define jx86_shfb_reg_reg(cp, shf, dreg, sreg)            jx86_shf_reg_reg((cp), (shf), (dreg), (sreg), 1)
#define jx86_shfw_membase_imm(cp, shf, breg, disp, simm)  jx86_shf_membase_imm((cp), (shf), (breg), (disp), (simm), 2)
#define jx86_shfw_membase_reg(cp, shf, breg, disp, simm)  jx86_shf_membase_reg((cp), (shf), (breg), (disp), (sreg), 2)
#define jx86_shfw_reg_imm(cp, shf, dreg, simm)            jx86_shf_reg_imm((cp), (shf), (dreg), (simm), 2)
#define jx86_shfw_reg_reg(cp, shf, dreg, sreg)            jx86_shf_reg_reg((cp), (shf), (dreg), (sreg), 2)
#define jx86_shfl_membase_imm(cp, shf, breg, disp, simm)  jx86_shf_membase_imm((cp), (shf), (breg), (disp), (simm), 4)
#define jx86_shfl_membase_reg(cp, shf, breg, disp, simm)  jx86_shf_membase_reg((cp), (shf), (breg), (disp), (sreg), 4)
#define jx86_shfl_reg_imm(cp, shf, dreg, simm)            jx86_shf_reg_imm((cp), (shf), (dreg), (simm), 4)
#define jx86_shfl_reg_reg(cp, shf, dreg, sreg)            jx86_shf_reg_reg((cp), (shf), (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_shfq_membase_imm(cp, shf, breg, disp, simm) jx86_shf_membase_imm((cp), (shf), (breg), (disp), (simm), 8)
# define jx86_shfq_membase_reg(cp, shf, breg, disp, simm) jx86_shf_membase_reg((cp), (shf), (breg), (disp), (sreg), 8)
# define jx86_shfq_reg_imm(cp, shf, dreg, simm)           jx86_shf_reg_imm((cp), (shf), (dreg), (simm), 8)
# define jx86_shfq_reg_reg(cp, shf, dreg, sreg)           jx86_shf_reg_reg((cp), (shf), (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_shfn_membase_imm(cp, shf, breg, disp, simm)  jx86_shf_membase_imm((cp), (shf), (breg), (disp), (simm), JX86_NWS)
#define jx86_shfn_membase_reg(cp, shf, breg, disp, simm)  jx86_shf_membase_reg((cp), (shf), (breg), (disp), (sreg), JX86_NWS)
#define jx86_shfn_reg_imm(cp, shf, dreg, simm)            jx86_shf_reg_imm((cp), (shf), (dreg), (simm), JX86_NWS)
#define jx86_shfn_reg_reg(cp, shf, dreg, sreg)            jx86_shf_reg_reg((cp), (shf), (dreg), (sreg), JX86_NWS)

#define jx86_rolb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROL, (breg), (disp), (simm), 1)
#define jx86_rolb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROL, (breg), (disp), (sreg), 1)
#define jx86_rolb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROL, (dreg), (simm), 1)
#define jx86_rolb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROL, (dreg), (sreg), 1)
#define jx86_rolw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROL, (breg), (disp), (simm), 2)
#define jx86_rolw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROL, (breg), (disp), (sreg), 2)
#define jx86_rolw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROL, (dreg), (simm), 2)
#define jx86_rolw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROL, (dreg), (sreg), 2)
#define jx86_roll_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROL, (breg), (disp), (simm), 4)
#define jx86_roll_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROL, (breg), (disp), (sreg), 4)
#define jx86_roll_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROL, (dreg), (simm), 4)
#define jx86_roll_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROL, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_rolq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_ROL, (breg), (disp), (simm), 8)
# define jx86_rolq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_ROL, (breg), (disp), (sreg), 8)
# define jx86_rolq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_ROL, (dreg), (simm), 8)
# define jx86_rolq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_ROL, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_roln_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROL, (breg), (disp), (simm), JX86_NWS)
#define jx86_roln_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROL, (breg), (disp), (sreg), JX86_NWS)
#define jx86_roln_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROL, (dreg), (simm), JX86_NWS)
#define jx86_roln_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROL, (dreg), (sreg), JX86_NWS)

#define jx86_rorb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROR, (breg), (disp), (simm), 1)
#define jx86_rorb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROR, (breg), (disp), (sreg), 1)
#define jx86_rorb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROR, (dreg), (simm), 1)
#define jx86_rorb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROR, (dreg), (sreg), 1)
#define jx86_rorw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROR, (breg), (disp), (simm), 2)
#define jx86_rorw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROR, (breg), (disp), (sreg), 2)
#define jx86_rorw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROR, (dreg), (simm), 2)
#define jx86_rorw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROR, (dreg), (sreg), 2)
#define jx86_rorl_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROR, (breg), (disp), (simm), 4)
#define jx86_rorl_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROR, (breg), (disp), (sreg), 4)
#define jx86_rorl_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROR, (dreg), (simm), 4)
#define jx86_rorl_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_rorq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_ROR, (breg), (disp), (simm), 8)
# define jx86_rorq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_ROR, (breg), (disp), (sreg), 8)
# define jx86_rorq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_ROR, (dreg), (simm), 8)
# define jx86_rorq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_ROR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_rorn_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_ROR, (breg), (disp), (simm), JX86_NWS)
#define jx86_rorn_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_ROR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_rorn_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_ROR, (dreg), (simm), JX86_NWS)
#define jx86_rorn_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_ROR, (dreg), (sreg), JX86_NWS)

#define jx86_rclb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCL, (breg), (disp), (simm), 1)
#define jx86_rclb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCL, (breg), (disp), (sreg), 1)
#define jx86_rclb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCL, (dreg), (simm), 1)
#define jx86_rclb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCL, (dreg), (sreg), 1)
#define jx86_rclw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCL, (breg), (disp), (simm), 2)
#define jx86_rclw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCL, (breg), (disp), (sreg), 2)
#define jx86_rclw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCL, (dreg), (simm), 2)
#define jx86_rclw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCL, (dreg), (sreg), 2)
#define jx86_rcll_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCL, (breg), (disp), (simm), 4)
#define jx86_rcll_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCL, (breg), (disp), (sreg), 4)
#define jx86_rcll_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCL, (dreg), (simm), 4)
#define jx86_rcll_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCL, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_rclq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_RCL, (breg), (disp), (simm), 8)
# define jx86_rclq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_RCL, (breg), (disp), (sreg), 8)
# define jx86_rclq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_RCL, (dreg), (simm), 8)
# define jx86_rclq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_RCL, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_rcln_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCL, (breg), (disp), (simm), JX86_NWS)
#define jx86_rcln_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCL, (breg), (disp), (sreg), JX86_NWS)
#define jx86_rcln_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCL, (dreg), (simm), JX86_NWS)
#define jx86_rcln_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCL, (dreg), (sreg), JX86_NWS)

#define jx86_rcrb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCR, (breg), (disp), (simm), 1)
#define jx86_rcrb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCR, (breg), (disp), (sreg), 1)
#define jx86_rcrb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCR, (dreg), (simm), 1)
#define jx86_rcrb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCR, (dreg), (sreg), 1)
#define jx86_rcrw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCR, (breg), (disp), (simm), 2)
#define jx86_rcrw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCR, (breg), (disp), (sreg), 2)
#define jx86_rcrw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCR, (dreg), (simm), 2)
#define jx86_rcrw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCR, (dreg), (sreg), 2)
#define jx86_rcrl_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCR, (breg), (disp), (simm), 4)
#define jx86_rcrl_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCR, (breg), (disp), (sreg), 4)
#define jx86_rcrl_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCR, (dreg), (simm), 4)
#define jx86_rcrl_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_rcrq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_RCR, (breg), (disp), (simm), 8)
# define jx86_rcrq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_RCR, (breg), (disp), (sreg), 8)
# define jx86_rcrq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_RCR, (dreg), (simm), 8)
# define jx86_rcrq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_RCR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_rcrn_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_RCR, (breg), (disp), (simm), JX86_NWS)
#define jx86_rcrn_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_RCR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_rcrn_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_RCR, (dreg), (simm), JX86_NWS)
#define jx86_rcrn_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_RCR, (dreg), (sreg), JX86_NWS)

#define jx86_shlb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHL, (breg), (disp), (simm), 1)
#define jx86_shlb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHL, (breg), (disp), (sreg), 1)
#define jx86_shlb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHL, (dreg), (simm), 1)
#define jx86_shlb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHL, (dreg), (sreg), 1)
#define jx86_shlw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHL, (breg), (disp), (simm), 2)
#define jx86_shlw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHL, (breg), (disp), (sreg), 2)
#define jx86_shlw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHL, (dreg), (simm), 2)
#define jx86_shlw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHL, (dreg), (sreg), 2)
#define jx86_shll_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHL, (breg), (disp), (simm), 4)
#define jx86_shll_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHL, (breg), (disp), (sreg), 4)
#define jx86_shll_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHL, (dreg), (simm), 4)
#define jx86_shll_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHL, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_shlq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_SHL, (breg), (disp), (simm), 8)
# define jx86_shlq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_SHL, (breg), (disp), (sreg), 8)
# define jx86_shlq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_SHL, (dreg), (simm), 8)
# define jx86_shlq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_SHL, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_shln_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHL, (breg), (disp), (simm), JX86_NWS)
#define jx86_shln_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHL, (breg), (disp), (sreg), JX86_NWS)
#define jx86_shln_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHL, (dreg), (simm), JX86_NWS)
#define jx86_shln_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHL, (dreg), (sreg), JX86_NWS)

#define jx86_shrb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHR, (breg), (disp), (simm), 1)
#define jx86_shrb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHR, (breg), (disp), (sreg), 1)
#define jx86_shrb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHR, (dreg), (simm), 1)
#define jx86_shrb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHR, (dreg), (sreg), 1)
#define jx86_shrw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHR, (breg), (disp), (simm), 2)
#define jx86_shrw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHR, (breg), (disp), (sreg), 2)
#define jx86_shrw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHR, (dreg), (simm), 2)
#define jx86_shrw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHR, (dreg), (sreg), 2)
#define jx86_shrl_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHR, (breg), (disp), (simm), 4)
#define jx86_shrl_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHR, (breg), (disp), (sreg), 4)
#define jx86_shrl_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHR, (dreg), (simm), 4)
#define jx86_shrl_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_shrq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_SHR, (breg), (disp), (simm), 8)
# define jx86_shrq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_SHR, (breg), (disp), (sreg), 8)
# define jx86_shrq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_SHR, (dreg), (simm), 8)
# define jx86_shrq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_SHR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_shrn_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SHR, (breg), (disp), (simm), JX86_NWS)
#define jx86_shrn_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SHR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_shrn_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SHR, (dreg), (simm), JX86_NWS)
#define jx86_shrn_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SHR, (dreg), (sreg), JX86_NWS)

#define jx86_sarb_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SAR, (breg), (disp), (simm), 1)
#define jx86_sarb_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SAR, (breg), (disp), (sreg), 1)
#define jx86_sarb_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SAR, (dreg), (simm), 1)
#define jx86_sarb_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SAR, (dreg), (sreg), 1)
#define jx86_sarw_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SAR, (breg), (disp), (simm), 2)
#define jx86_sarw_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SAR, (breg), (disp), (sreg), 2)
#define jx86_sarw_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SAR, (dreg), (simm), 2)
#define jx86_sarw_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SAR, (dreg), (sreg), 2)
#define jx86_sarl_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SAR, (breg), (disp), (simm), 4)
#define jx86_sarl_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SAR, (breg), (disp), (sreg), 4)
#define jx86_sarl_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SAR, (dreg), (simm), 4)
#define jx86_sarl_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SAR, (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_sarq_membase_imm(cp, breg, disp, simm) jx86_shf_membase_imm((cp), JX86_SAR, (breg), (disp), (simm), 8)
# define jx86_sarq_membase_reg(cp, breg, disp, sreg) jx86_shf_membase_reg((cp), JX86_SAR, (breg), (disp), (sreg), 8)
# define jx86_sarq_reg_imm(cp, dreg, simm)           jx86_shf_reg_imm((cp), JX86_SAR, (dreg), (simm), 8)
# define jx86_sarq_reg_reg(cp, dreg, sreg)           jx86_shf_reg_reg((cp), JX86_SAR, (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_sarn_membase_imm(cp, breg, disp, simm)  jx86_shf_membase_imm((cp), JX86_SAR, (breg), (disp), (simm), JX86_NWS)
#define jx86_sarn_membase_reg(cp, breg, disp, sreg)  jx86_shf_membase_reg((cp), JX86_SAR, (breg), (disp), (sreg), JX86_NWS)
#define jx86_sarn_reg_imm(cp, dreg, simm)            jx86_shf_reg_imm((cp), JX86_SAR, (dreg), (simm), JX86_NWS)
#define jx86_sarn_reg_reg(cp, dreg, sreg)            jx86_shf_reg_reg((cp), JX86_SAR, (dreg), (sreg), JX86_NWS)


/* TEST instruction
 * ----------------
 */

#ifdef JX86_32
# define jx86_test_mem_imm(cp, dmem, simm, size)                \
  do {                                                          \
    const jx86_int32_t _dmem_ = (const jx86_int32_t) (dmem);    \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, 0);                    \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_mem((cp), 0, _dmem_);                             \
    jx86_emit_imm((cp), _size_, _simm_);                        \
  } while (0)
#endif /* JX86_32 */

#define jx86_test_membase_imm(cp, breg, disp, simm, size)       \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _breg_);               \
    jx86_emit_opcode1((cp), _size_, 0xf6);                      \
    jx86_emit_membase((cp), 0, _breg_, _disp_);                 \
    jx86_emit_imm((cp), _size_, _simm_);                        \
  } while (0)

#define jx86_test_membase_reg(cp, breg, disp, sreg, size)       \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _sreg_, 0, _breg_);          \
    jx86_emit_opcode1((cp), _size_, 0x84);                      \
    jx86_emit_membase((cp), _sreg_, _breg_, _disp_);            \
  } while (0)

#define jx86_test_reg_imm(cp, dreg, simm, size)                 \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_int32_t _simm_ = (const jx86_int32_t) (simm);    \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);               \
    if (_dreg_ == JX86_EAX)                                     \
      jx86_emit_opcode1((cp), _size_, 0xa8);                    \
    else {                                                      \
      jx86_emit_opcode1((cp), _size_, 0xf6);                    \
      jx86_emit_reg((cp), 0, _dreg_);                           \
    }                                                           \
    jx86_emit_imm((cp), _size_, _simm_);                        \
  } while (0)

#define jx86_test_reg_reg(cp, dreg, sreg, size)                 \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _dreg_, 0, _sreg_);          \
    jx86_emit_opcode1((cp), _size_, 0x84);                      \
    jx86_emit_reg((cp), _sreg_, _dreg_);                        \
  } while (0)

#ifdef JX86_32
# define jx86_testb_mem_imm(cp, dmem, simm)           jx86_test_mem_imm((cp), (dmem), (simm), 1)
#endif
#define jx86_testb_membase_imm(cp, breg, disp, simm)  jx86_test_membase_imm((cp), (breg), (disp), (simm), 1)
#define jx86_testb_membase_reg(cp, breg, disp, sreg)  jx86_test_membase_reg((cp), (breg), (disp), (sreg), 1)
#define jx86_testb_reg_imm(cp, dreg, simm)            jx86_test_reg_imm((cp), (dreg), (simm), 1)
#define jx86_testb_reg_reg(cp, dreg, sreg)            jx86_test_reg_reg((cp), (dreg), (sreg), 1)
#ifdef JX86_32
# define jx86_testw_mem_imm(cp, dmem, simm)           jx86_test_mem_imm((cp), (dmem), (simm), 2)
#endif
#define jx86_testw_membase_imm(cp, breg, disp, simm)  jx86_test_membase_imm((cp), (breg), (disp), (simm), 2)
#define jx86_testw_membase_reg(cp, breg, disp, sreg)  jx86_test_membase_reg((cp), (breg), (disp), (sreg), 2)
#define jx86_testw_reg_imm(cp, dreg, simm)            jx86_test_reg_imm((cp), (dreg), (simm), 2)
#define jx86_testw_reg_reg(cp, dreg, sreg)            jx86_test_reg_reg((cp), (dreg), (sreg), 2)
#ifdef JX86_32
# define jx86_testl_mem_imm(cp, dmem, simm)           jx86_test_mem_imm((cp), (dmem), (simm), 4)
#endif
#define jx86_testl_membase_imm(cp, breg, disp, simm)  jx86_test_membase_imm((cp), (breg), (disp), (simm), 4)
#define jx86_testl_membase_reg(cp, breg, disp, sreg)  jx86_test_membase_reg((cp), (breg), (disp), (sreg), 4)
#define jx86_testl_reg_imm(cp, dreg, simm)            jx86_test_reg_imm((cp), (dreg), (simm), 4)
#define jx86_testl_reg_reg(cp, dreg, sreg)            jx86_test_reg_reg((cp), (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_testq_membase_imm(cp, breg, disp, simm) jx86_test_membase_imm((cp), (breg), (disp), (simm), 8)
# define jx86_testq_membase_reg(cp, breg, disp, sreg) jx86_test_membase_reg((cp), (breg), (disp), (sreg), 8)
# define jx86_testq_reg_imm(cp, dreg, simm)           jx86_test_reg_imm((cp), (dreg), (simm), 8)
# define jx86_testq_reg_reg(cp, dreg, sreg)           jx86_test_reg_reg((cp), (dreg), (sreg), 8)
#endif /* JX86_64 */
#define jx86_testn_membase_imm(cp, breg, disp, simm)  jx86_test_membase_imm((cp), (breg), (disp), (simm), JX86_NWS)
#define jx86_testn_membase_reg(cp, breg, disp, sreg)  jx86_test_membase_reg((cp), (breg), (disp), (sreg), JX86_NWS)
#define jx86_testn_reg_imm(cp, dreg, simm)            jx86_test_reg_imm((cp), (dreg), (simm), JX86_NWS)
#define jx86_testn_reg_reg(cp, dreg, sreg)            jx86_test_reg_reg((cp), (dreg), (sreg), JX86_NWS)


/* UD2 instruction
 * ---------------
 */

#define jx86_ud2(cp)                            \
  do {                                          \
    jx86_emit_uint16((cp), 0x0b0f);             \
  } while (0)


/* XCHG instruction
 * ----------------
 */

#define jx86_xchg_membase_reg(cp, breg, disp, sreg, size)       \
  do {                                                          \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);        \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);    \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    jx86_emit_osprex((cp), _size_, _sreg_, 0, _breg_);          \
    jx86_emit_opcode1((cp), _size_, 0x86);                      \
    jx86_emit_membase((cp), _sreg_, _breg_, _disp_);            \
  } while (0)

#define jx86_xchg_reg_membase(cp, dreg, breg, disp, size)       \
  jx86_xchg_membase_reg((cp), (breg), (disp), (dreg), (size))

#define jx86_xchg_reg_reg(cp, dreg, sreg, size)                 \
  do {                                                          \
    const jx86_reg_t _dreg_ = (const jx86_reg_t) (dreg);        \
    const jx86_reg_t _sreg_ = (const jx86_reg_t) (sreg);        \
    const unsigned _size_ = (const unsigned) (size);            \
    if (_dreg_ == JX86_EAX && _size_ != 1) {                    \
      jx86_emit_osprex((cp), _size_, 0, 0, _sreg_);             \
      jx86_emit_uint8((cp), 0x90 + JX86_REG_CODE(_sreg_));      \
    }                                                           \
    else if (_sreg_ == JX86_EAX && _size_ != 1) {               \
      jx86_emit_osprex((cp), _size_, 0, 0, _dreg_);             \
      jx86_emit_uint8((cp), 0x90 + JX86_REG_CODE(_dreg_));      \
    }                                                           \
    else {                                                      \
      jx86_emit_osprex((cp), _size_, _dreg_, 0, _sreg_);        \
      jx86_emit_opcode1((cp), _size_, 0x86);                    \
      jx86_emit_reg((cp), _sreg_, _dreg_);                      \
    }                                                           \
  } while (0)

#define jx86_xchgb_membase_reg(cp, breg, disp, sreg)  jx86_xchg_membase_reg((cp), (breg), (disp), (sreg), 1)
#define jx86_xchgb_reg_membase(cp, dreg, breg, disp)  jx86_xchg_reg_membase((cp), (dreg), (breg), (disp), 1)
#define jx86_xchgb_reg_reg(cp, dreg, sreg)            jx86_xchg_reg_reg((cp), (dreg), (sreg), 1)
#define jx86_xchgw_membase_reg(cp, breg, disp, sreg)  jx86_xchg_membase_reg((cp), (breg), (disp), (sreg), 2)
#define jx86_xchgw_reg_membase(cp, dreg, breg, disp)  jx86_xchg_reg_membase((cp), (dreg), (breg), (disp), 2)
#define jx86_xchgw_reg_reg(cp, dreg, sreg)            jx86_xchg_reg_reg((cp), (dreg), (sreg), 2)
#define jx86_xchgl_membase_reg(cp, breg, disp, sreg)  jx86_xchg_membase_reg((cp), (breg), (disp), (sreg), 4)
#define jx86_xchgl_reg_membase(cp, dreg, breg, disp)  jx86_xchg_reg_membase((cp), (dreg), (breg), (disp), 4)
#define jx86_xchgl_reg_reg(cp, dreg, sreg)            jx86_xchg_reg_reg((cp), (dreg), (sreg), 4)
#ifdef JX86_64
# define jx86_xchgq_membase_reg(cp, breg, disp, sreg) jx86_xchg_membase_reg((cp), (breg), (disp), (sreg), 8)
# define jx86_xchgq_reg_membase(cp, dreg, breg, disp) jx86_xchg_reg_membase((cp), (dreg), (breg), (disp), 8)
# define jx86_xchgq_reg_reg(cp, dreg, sreg)           jx86_xchg_reg_reg((cp), (dreg), (sreg), 8)
#endif
#define jx86_xchgn_membase_reg(cp, breg, disp, sreg)  jx86_xchg_membase_reg((cp), (breg), (disp), (sreg), JX86_NWS)
#define jx86_xchgn_reg_membase(cp, dreg, breg, disp)  jx86_xchg_reg_membase((cp), (dreg), (breg), (disp), JX86_NWS)
#define jx86_xchgn_reg_reg(cp, dreg, sreg)            jx86_xchg_reg_reg((cp), (dreg), (sreg), JX86_NWS)


/* SSE instructions
 * ----------------
 */

#define jx86_sse_op2_membase_xmm(cp, op1, op2, breg, disp, sxmm)        \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_xmm_t _sxmm_ = (const jx86_xmm_t) (sxmm);                \
    jx86_emit_rex((cp), 0, _sxmm_, 0, _breg_);                          \
    jx86_emit_uint16((cp), ((op2) << 8) | (op1));                       \
    jx86_emit_membase((cp), _sxmm_, _breg_, _disp_);                    \
  } while (0)

#define jx86_sse_op2_xmm_membase(cp, op1, op2, dxmm, breg, disp)        \
  do {                                                                  \
    const jx86_xmm_t _dxmm_ = (const jx86_xmm_t) (dxmm);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    jx86_emit_rex((cp), 0, _dxmm_, 0, _breg_);                          \
    jx86_emit_uint16((cp), ((op2) << 8) | (op1));                       \
    jx86_emit_membase((cp), _dxmm_, _breg_, _disp_);                    \
  } while (0)

#define jx86_sse_op2_xmm_xmm(cp, op1, op2, dxmm, sxmm)              \
  do {                                                              \
    const jx86_xmm_t _dxmm_ = (const jx86_xmm_t) (dxmm);            \
    const jx86_xmm_t _sxmm_ = (const jx86_xmm_t) (sxmm);            \
    jx86_emit_rex((cp), 0, _dxmm_, 0, _sxmm_);                      \
    jx86_emit_uint16((cp), ((op2) << 8) | (op1));                   \
    jx86_emit_reg((cp), _dxmm_, _sxmm_);                            \
  } while (0)

#define jx86_sse_op3_membase_xmm(cp, op1, op2, op3, breg, disp, sxmm)   \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_xmm_t _sxmm_ = (const jx86_xmm_t) (sxmm);                \
    jx86_emit_uint8((cp), (op1));                                       \
    jx86_emit_rex((cp), 0, _sxmm_, 0, _breg_);                          \
    jx86_emit_uint16((cp), ((op3) << 8) | (op2));                       \
    jx86_emit_membase((cp), _sxmm_, _breg_, _disp_);                    \
  } while (0)

#define jx86_sse_op3_memindex_xmm(cp, op1, op2, op3, breg, disp, ireg, shift, sxmm) \
  do {                                                                  \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    const jx86_xmm_t _sxmm_ = (const jx86_xmm_t) (sxmm);                \
    jx86_emit_uint8((cp), (op1));                                       \
    jx86_emit_rex((cp), 0, _sxmm_, _ireg_, _breg_);                     \
    jx86_emit_uint16((cp), ((op3) << 8) | (op2));                       \
    jx86_emit_memindex((cp), _sxmm_, _breg_, _disp_, _ireg_, _shift_);  \
  } while (0)

#define jx86_sse_op3_xmm_membase(cp, op1, op2, op3, dxmm, breg, disp)   \
  do {                                                                  \
    const jx86_xmm_t _dxmm_ = (const jx86_xmm_t) (dxmm);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    jx86_emit_uint8((cp), (op1));                                       \
    jx86_emit_rex((cp), 0, _dxmm_, 0, _breg_);                          \
    jx86_emit_uint16((cp), ((op3) << 8) | (op2));                       \
    jx86_emit_membase((cp), _dxmm_, _breg_, _disp_);                    \
  } while (0)

#define jx86_sse_op3_xmm_memindex(cp, op1, op2, op3, dxmm, breg, disp, ireg, shift) \
  do {                                                                  \
    const jx86_xmm_t _dxmm_ = (const jx86_xmm_t) (dxmm);                \
    const jx86_reg_t _breg_ = (const jx86_reg_t) (breg);                \
    const jx86_int32_t _disp_ = (const jx86_int32_t) (disp);            \
    const jx86_reg_t _ireg_ = (const jx86_reg_t) (ireg);                \
    const unsigned _shift_ = (const unsigned) (shift);                  \
    jx86_emit_uint8((cp), (op1));                                       \
    jx86_emit_rex((cp), 0, _dxmm_, _ireg_, _breg_);                     \
    jx86_emit_uint16((cp), ((op3) << 8) | (op2));                       \
    jx86_emit_memindex((cp), _dxmm_, _breg_, _disp_, _ireg_, _shift_);  \
  } while (0)

#define jx86_sse_op3_xmm_xmm(cp, op1, op2, op3, dxmm, sxmm)     \
  do {                                                          \
    const jx86_xmm_t _dxmm_ = (const jx86_xmm_t) (dxmm);        \
    const jx86_xmm_t _sxmm_ = (const jx86_xmm_t) (sxmm);        \
    jx86_emit_uint8((cp), (op1));                               \
    jx86_emit_rex((cp), 0, _dxmm_, 0, _sxmm_);                  \
    jx86_emit_uint16((cp), ((op3) << 8) | (op2));               \
    jx86_emit_reg((cp), _dxmm_, _sxmm_);                        \
  } while (0)

#define jx86_addsd_xmm_membase(cp, dxmm, breg, disp)                    \
  jx86_sse_op3_xmm_membase((cp), 0xf2, 0x0f, 0x58, (dxmm), (breg), (disp))
#define jx86_addsd_xmm_xmm(cp, dxmm, sxmm)                              \
  jx86_sse_op3_xmm_xmm((cp), 0xf2, 0x0f, 0x58, (dxmm), (sxmm))
#define jx86_divsd_xmm_membase(cp, dxmm, breg, disp)                    \
  jx86_sse_op3_xmm_membase((cp), 0xf2, 0x0f, 0x5e, (dxmm), (breg), (disp))
#define jx86_divsd_xmm_xmm(cp, dxmm, sxmm)                              \
  jx86_sse_op3_xmm_xmm((cp), 0xf2, 0x0f, 0x5e, (dxmm), (sxmm))
#define jx86_mulsd_xmm_membase(cp, dxmm, breg, disp)                    \
  jx86_sse_op3_xmm_membase((cp), 0xf2, 0x0f, 0x59, (dxmm), (breg), (disp))
#define jx86_mulsd_xmm_xmm(cp, dxmm, sxmm)                              \
  jx86_sse_op3_xmm_xmm((cp), 0xf2, 0x0f, 0x59, (dxmm), (sxmm))
#define jx86_sqrtsd_xmm_membase(cp, dxmm, breg, disp)                   \
  jx86_sse_op3_xmm_membase((cp), 0xf2, 0x0f, 0x51, (dxmm), (breg), (disp))
#define jx86_sqrtsd_xmm_xmm(cp, dxmm, sxmm)                             \
  jx86_sse_op3_xmm_xmm((cp), 0xf2, 0x0f, 0x51, (dxmm), (sxmm))
#define jx86_subsd_xmm_membase(cp, dxmm, breg, disp)                    \
  jx86_sse_op3_xmm_membase((cp), 0xf2, 0x0f, 0x5c, (dxmm), (breg), (disp))
#define jx86_subsd_xmm_xmm(cp, dxmm, sxmm)                              \
  jx86_sse_op3_xmm_xmm((cp), 0xf2, 0x0f, 0x5c, (dxmm), (sxmm))

#define jx86_movhpd_membase_xmm(cp, breg, disp, sxmm)                   \
  jx86_sse_op3_membase_xmm((cp), 0x66, 0x0f, 0x17, (breg), (disp), (sxmm))
#define jx86_movhpd_xmm_membase(cp, dxmm, breg, disp)                   \
  jx86_sse_op3_xmm_membase((cp), 0x66, 0x0f, 0x16, (dxmm), (breg), (disp))

#define jx86_movlpd_membase_xmm(cp, breg, disp, sxmm)                   \
  jx86_sse_op3_membase_xmm((cp), 0x66, 0x0f, 0x13, (breg), (disp), (sxmm))
#define jx86_movlpd_memindex_xmm(cp, breg, disp, ireg, shift, sxmm)     \
  jx86_sse_op3_memindex_xmm((cp), 0x66, 0x0f, 0x13, (breg), (disp), (ireg), (shift), (sxmm))
#define jx86_movlpd_xmm_membase(cp, dxmm, breg, disp)                   \
  jx86_sse_op3_xmm_membase((cp), 0x66, 0x0f, 0x12, (dxmm), (breg), (disp))
#define jx86_movlpd_xmm_memindex(cp, dxmm, breg, disp, ireg, shift)     \
  jx86_sse_op3_xmm_memindex((cp), 0x66, 0x0f, 0x12, (dxmm), (breg), (disp), (ireg), (shift))

#define jx86_ucomisd_xmm_membase(cp, dxmm, breg, disp)                  \
  jx86_sse_op3_xmm_membase((cp), 0x66, 0x0f, 0x2e, (dxmm), (breg), (disp))
#define jx86_ucomisd_xmm_xmm(cp, dxmm, sxmm)                            \
  jx86_sse_op3_xmm_xmm((cp), 0x66, 0x0f, 0x2e, (dxmm), (sxmm))

#endif /* TARGET_amd64 || TARGET_i386 */

#endif /* !CAML_JX86_H */
