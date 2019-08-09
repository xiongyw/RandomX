#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
//#define NDEBUG
#include <assert.h>

#include "rv64_insn_sim.h"

int is_nbit_imm(int32_t imm, uint8_t n)
{
    uint8_t N = sizeof(imm) * 8;

    if (n == 0 || n > N)
        return 0;

    uint8_t bitx;
    uint8_t sign_bit = (imm >> (n - 1)) & 0x1;

    for (int i = n; i < N; ++i) {
        bitx = (imm >> i) & 0x1;
        if (sign_bit != bitx) {
            return 0;
        }
    }
    return 1;
}

int64_t sext(int64_t imm, int n)
{
    assert (n < 64);
    uint64_t hi_mask = 0;
    for (int i = n + 1; i < 64; ++i) {
        hi_mask |= (1ul << i);
    }
    //assert((imm & hi_mask) == 0);

    if (imm & (1ul << n)) {
        return (int64_t)(imm | hi_mask);
    } else {
        return (int64_t)imm;
    }
}

int32_t hi20(int32_t imm)
{
    return sext(imm >> 12, 19);
}

int32_t low12(int32_t imm)
{
    return sext(imm & 0xfff, 11);
}

int32_t low5(int32_t imm)
{
    return sext(imm & 0x1f, 4);
}


////////////////////////////////////////////////////////////////

int64_t add(int64_t rs1, int64_t rs2)
{
    return rs1 + rs2;
}

// x[rd]=x[rs1]+sext(imm)
int64_t addi(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    return rs1 + sext(imm12, 11);
}

int64_t li_imm32(int32_t imm32)
{
    int64_t dst = 0;

    if (imm32 >= -2048 && imm32 <= 2047) {
        return addi(0, low12(imm32));   // addi dst, x0, imm;
    }

    if (low12(imm32) >= 0) {
        dst = lui(hi20(imm32));  // lui dst, imm20;
    } else {
        // now `+1` does not overflow the low 20-bit of hi20(imm32),
        // and lui only uses the low 20-bit of the sum
        dst = lui(hi20(imm32) + 1); // lui dst, imm20+1;
    }

    dst = addiw(dst, low12(imm32));  // addiw dst, dst, imm12

    return dst;
}


int64_t slli(int64_t rs1, uint8_t shamt)
{
    return rs1 << shamt;
}


// x[rd]=sext(imm[31:12] << 12)
int64_t lui(int32_t imm20)
{
    // don't care the top 12-bit of imm20, which will be discarded anyway
    //assert(is_nbit_imm(imm20, 20));

    PRI32("lui()", imm20, imm20);
    int64_t tmp = sext(((uint64_t)imm20) << 12, 31);
    PRI64("lui()", tmp, tmp);
    return tmp;
}

// x[rd]=sext(x[rs1]+sext(imm12))[31:0])
int64_t addiw(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    PRI64("addiw()", rs1, rs1);
    PRI32("addiw()", imm12, imm12);
    int64_t sext_imm12 = sext(imm12, 11);
    PRI64("addiw()", sext_imm12, sext_imm12);
    sext_imm12 += rs1;
    PRI64("addiw()", sext_imm12, sext_imm12);
    int32_t trunc32 = (int32_t)(sext_imm12 & 0xffffffff);
    PRI32("addiw()", trunc32, trunc32);
    return sext(trunc32, 31);
}

