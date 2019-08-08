#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <inttypes.h>

// note: this is a test program to check behavior or risc-v instructions

#define DEBUG 1

#if DEBUG
#define PRI32(msg, name, value) printf("%s: "#name"=%"PRId32"(0x%08"PRIx32")\n", msg, value, value);
#define PRI64(msg, name, value) printf("%s: "#name"=%"PRId64"(0x%016"PRIx64")\n", msg, value, value);
#else
#define PRI32(msg, name, value)
#define PRI64(msg, name, value)
#endif

// return 64-bit sign-extended int; `n` is the sign-bit idx (start from 0)
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

// return high 20-bit of imm, as signed int
int32_t hi20(int32_t imm)
{
    return sext(imm >> 12, 19);
}

// return low 12-bit of imm, as signed int
int32_t low12(int32_t imm)
{
    return sext(imm & 0xfff, 11);
}

// x[rd]=sext(imm[31:12] << 12)
int64_t lui(int32_t imm20)
{
    PRI32("lui()", imm20, imm20);
    int64_t tmp = sext(((uint64_t)imm20) << 12, 31);
    PRI64("lui()", tmp, tmp);
    return tmp;
}

// x[rd]=sext(x[rs1]+sext(imm12))[31:0]
int32_t addiw(int64_t rs1, int32_t imm12)
{
    PRI64("addiw()", rs1, rs1);
    PRI32("addiw()", imm12, imm12);
    int64_t sext_imm12 = sext(imm12, 11);
    PRI64("addiw()", sext_imm12, sext_imm12);
    sext_imm12 += rs1;
    PRI64("addiw()", sext_imm12, sext_imm12);
    int32_t trunc32 = (int32_t)(sext_imm12 & 0xffffffff);
    PRI32("addiw()", trunc32, trunc32);
    return trunc32;
}

// simulate `li imm32`, as in randomx imm is only 32-bit
int64_t li_imm32(int32_t imm32)
{
    int64_t dst = 0;

    if (imm32 < 0) {  // negative
        if (low12(imm32) < 0) {
            dst = lui(hi20(imm32) + 1);
        } else {
            dst = lui(hi20(imm32));
        }
        PRI64("li_imm32()", dst, dst);
        dst = addiw(dst, low12(imm32));
        PRI64("li_imm32()", dst, dst);
    }
#if (0)
    else {  // non-negative
        if (imm32 <= 0x7ff) {
            dst = imm32;                                // addi dst, x0, imm;
        } else {
            if (low12(imm32) >= 0) {
                dst = (int32_t)(hi20(imm32) << 12);    // lui dst, imm20;
            } else {
                dst = (int32_t)((hi20(imm32) + 1) << 12);  // lui dst, imm20+1;
            }
            printf("dst= %"PRId64"(%"PRIx64")\n", dst, dst);
            dst += low12(imm32);                    // addiw dst, imm12;
            printf("dst= %"PRId64"(%"PRIx64")\n", dst, dst);
        }
    }
#endif

    return dst;
}

int main(void)
{
    int64_t dst;
    int32_t imm32;

#if defined(__riscv)
    // the following code is to check the `li` behavior on riscv-64
    // compile: riscv64-unknown-linux-gnu-gcc -march=rv64imfd -g -static -O0 -c t.c -o t.o
    // dasm: riscv64-unknown-linux-gnu-objdump -S -Mno-aliases,numeric t.o
    // run: spike pk t
//#define IMM   0x80000801
#define IMM   0x80000000
    asm volatile("li %[dst], %[imm]" : [dst]"=r"(dst) : [imm]"i"(IMM));
    printf("dst= %"PRId64"(%"PRIx64")\n", dst, dst);
    printf("sext=%"PRId64"(%"PRIx64")\n", sext(IMM, 31), sext(IMM, 31));
#else
    for (imm32 = (int32_t)(0x80000000); imm32 < (int32_t)0/*x7fffffff*/; ++ imm32) {
        PRI32("main()", imm32, imm32);
        dst = li_imm32(imm32);
        if (imm32 != dst) {
            printf("failure: imm32=%d(%08x), li_imm32(imm32)=%"PRId64"(%"PRIx64")\n", imm32, imm32, dst, dst);
            printf("hi20=%d(%05x), low12=%d(%03x)\n", hi20(imm32), hi20(imm32), low12(imm32), low12(imm32));
            exit(1);
        };
    }
#endif

    return 0;
}
