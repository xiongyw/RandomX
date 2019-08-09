#ifndef __RV64_INSN_SIM__
#define __RV64_INSN_SIM__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
//#define NDEBUG
#include <assert.h>

// this is to simulate risc-v instructions
#if defined(__cplusplus)
extern "C" {
#endif



#define DEBUG 0

#if DEBUG
#define PRI32(msg, name, value) printf("%s: "#name"=%"PRId32"(0x%08"PRIx32")\n", msg, value, value);
#define PRI64(msg, name, value) printf("%s: "#name"=%"PRId64"(0x%016"PRIx64")\n", msg, value, value);
#else
#define PRI32(msg, name, value)
#define PRI64(msg, name, value)
#endif

/* gcc extension */
typedef unsigned __int128 uint128_t;
typedef __int128 int128_t;


/*******************************************************************************
 *
 * utilities
 *
 ******************************************************************************/
 

/*
 * this routine is to check whether or not the value stored in int32_t can be
 * represented by n-bit.
 */
int is_nbit_imm(int32_t imm, uint8_t n);

// return 64-bit sign-extended int; `n` is the sign-bit idx (start from 0)
int64_t sext(int64_t imm, int n);

// return high 20-bit of imm, as signed int
int32_t hi20(int32_t imm);
// return low 12-bit of imm, as signed int
int32_t low12(int32_t imm);
int32_t low5(int32_t imm); // shamt

/*******************************************************************************
 *
 * instructions
 *
 ******************************************************************************/


int64_t rv_add(int64_t rs1, int64_t rs2);

// x[rd]=x[rs1]+sext(imm)
int64_t rv_addi(int64_t rs1, int32_t imm12);

// x[rd]=sext(x[rs1]+sext(imm12))[31:0])
int64_t rv_addiw(int64_t rs1, int32_t imm12);

int64_t rv_and(int64_t rs1, int64_t rs2);
int64_t rv_andi(int64_t rs1, int32_t imm12);

// psedu-insn `li`. only simulatr `li imm32`, as in randomx imm is always 32-bit
int64_t rv_li_imm32(int32_t imm32);

int64_t rv_ld(int64_t rs1, int32_t imm12);

// x[rd]=sext(imm[31:12] << 12)
int64_t rv_lui(int32_t imm20);

int64_t rv_mul(int64_t rs1, int64_t rs2);
int64_t rv_mulh(int64_t rs1, int64_t rs2); // signed
uint64_t rv_mulhu(uint64_t rs1, uint64_t rs2); // unsigned

int64_t rv_or(int64_t rs1, int64_t rs2);

int64_t rv_sll(int64_t rs1, int64_t rs2);
int64_t rv_slli(int64_t rs1, uint8_t shamt);
int64_t rv_srl(int64_t rs1, int64_t rs2);
int64_t rv_srli(int64_t rs1, uint8_t shamt);

int64_t rv_sub(int64_t rs1, int64_t rs2);

int64_t rv_xor(int64_t rs1, int64_t rs2);


#if defined(__cplusplus)
}
#endif


#endif // __RV64_INSN_SIM__
