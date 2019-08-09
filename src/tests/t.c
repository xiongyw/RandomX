#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

//#define NDEBUG
#include <assert.h>

// note: this is a test program to check behavior or risc-v instructions

#define DEBUG 0

#if DEBUG
#define PRI32(msg, name, value) printf("%s: "#name"=%"PRId32"(0x%08"PRIx32")\n", msg, value, value);
#define PRI64(msg, name, value) printf("%s: "#name"=%"PRId64"(0x%016"PRIx64")\n", msg, value, value);
#else
#define PRI32(msg, name, value)
#define PRI64(msg, name, value)
#endif

/*
 * this routine is to check whether or not the value stored in int32_t can be
 * represented by n-bit:
 *  - if it's no-negative, then bit 31~(n-1) are all 0
 *  - if it's negative, then bit 31~(n-1) are 1
 * to summarize: bit 31~(n-1) should be all the same.
 *
 * simple tests for the routine:
 *
 *  for (int32_t i = -130; i <= 130; ++i) {
 *      if(!is_nbit_imm(i, 8)) {
 *          printf("fail: i=%d\n", i);
 *      }
 *  }
 *
 * it should print the following failure cases:
 *
 *  fail: i=-130
 *  fail: i=-129
 *  fail: i=128
 *  fail: i=129
 *  fail: i=130
 */
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

// x[rd]=x[rs1]+sext(imm)
int64_t addi(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    return rs1 + sext(imm12, 11);
}

// simulate `li imm32`, as in randomx imm is only 32-bit
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
#define IMM   0xfffff800
    asm volatile("li %[dst], %[imm]" : [dst]"=r"(dst) : [imm]"i"(IMM));
    printf("dst= %"PRId64"(%"PRIx64")\n", dst, dst);
    printf("sext=%"PRId64"(%"PRIx64")\n", sext(IMM, 31), sext(IMM, 31));
#else
    for (imm32 = 0x80000000; imm32 <= 0x7fffffff; ++ imm32) {
        PRI32("main()", imm32, imm32);
        dst = li_imm32(imm32);
        if (imm32 != dst) {
            printf("failure: imm32=%d(%08x), li_imm32(imm32)=%"PRId64"(%"PRIx64")\n", imm32, imm32, dst, dst);
            printf("hi20=%d(%05x), low12=%d(%03x)\n", hi20(imm32), hi20(imm32), low12(imm32), low12(imm32));
            exit(1);
        };
    }
    printf("ok\n");
#endif

    return 0;
}

