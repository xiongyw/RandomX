#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>   // sqrt()
//#define NDEBUG
#include <assert.h>

// <https://stackoverflow.com/questions/6867693/change-floating-point-rounding-mode>
#include <fenv.h>


#include "rv64_insn_sim.h"

static uint8_t s_frm = 0; // floating-point rounding mode

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

int64_t rv64_add(int64_t rs1, int64_t rs2)
{
    return rs1 + rs2;
}

// x[rd]=x[rs1]+sext(imm)
int64_t rv64_addi(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    return rs1 + sext(imm12, 11);
}

// x[rd]=sext(x[rs1]+sext(imm12))[31:0])
int64_t rv64_addiw(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t sext_imm12 = sext(imm12, 11);
    sext_imm12 += rs1;
    int32_t trunc32 = (int32_t)(sext_imm12 & 0xffffffff);
    return sext(trunc32, 31);
}


int64_t rv64_and(int64_t rs1, int64_t rs2)
{
    return rs1 & rs2;
}

// x[rd]=x[rs1] & sext(imm)
int64_t rv64_andi(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t sext_imm12 = sext(imm12, 11);
    return rs1 & sext_imm12;
}



double rv64_fadd_d(double rs1, double rs2)
{
#pragma STDC FENV_ACCESS ON

    // store the original rounding mode
    const int originalRounding = fegetround();

    // set the rounding mode
    switch(s_frm) {
        case RV_FRM_RNE: fesetround(FE_TONEAREST); break;
        case RV_FRM_RTZ: fesetround(FE_TOWARDZERO); break;
        case RV_FRM_RDN: fesetround(FE_DOWNWARD); break;
        case RV_FRM_RUP: fesetround(FE_UPWARD); break;
        default:
            __builtin_unreachable();
    }

    // do calculations
    double dst = rs1 + rs2;

    // restore the original mode afterwards
    fesetround(originalRounding);

#pragma STDC FENV_ACCESS OFF

    return dst;
}

double rv64_fcvt_d_l(int64_t rs1)
{
    return (double)rs1;
}

#if (0)
// f[rd]=M[x[rs1]+sext(offset)][63:0]
double rv64_fld(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t addr = rs1 + sext(imm12, 11);
    assert(addr > 0);
    double dst;
	memcpy(&dst, (void*)addr, sizeof(dst));

    return dst;
}
#endif

double rv64_fdiv_d(double rs1, double rs2)
{
#pragma STDC FENV_ACCESS ON

    // store the original rounding mode
    const int originalRounding = fegetround();

    // set the rounding mode
    switch(s_frm) {
        case RV_FRM_RNE: fesetround(FE_TONEAREST); break;
        case RV_FRM_RTZ: fesetround(FE_TOWARDZERO); break;
        case RV_FRM_RDN: fesetround(FE_DOWNWARD); break;
        case RV_FRM_RUP: fesetround(FE_UPWARD); break;
        default:
            __builtin_unreachable();
    }

    // do calculations
    double dst = rs1 / rs2;

    // restore the original mode afterwards
    fesetround(originalRounding);

#pragma STDC FENV_ACCESS OFF

    return dst;
}


double rv64_fmul_d(double rs1, double rs2)
{
#pragma STDC FENV_ACCESS ON

    // store the original rounding mode
    const int originalRounding = fegetround();

    // set the rounding mode
    switch(s_frm) {
        case RV_FRM_RNE: fesetround(FE_TONEAREST); break;
        case RV_FRM_RTZ: fesetround(FE_TOWARDZERO); break;
        case RV_FRM_RDN: fesetround(FE_DOWNWARD); break;
        case RV_FRM_RUP: fesetround(FE_UPWARD); break;
        default:
            __builtin_unreachable();
    }

    // do calculations
    double dst = rs1 * rs2;

    // restore the original mode afterwards
    fesetround(originalRounding);

#pragma STDC FENV_ACCESS OFF

    return dst;
}



double rv64_fmv_d_x(int64_t rs1)
{
    double d;
	memcpy((void*)(&d), (void*)(&rs1), sizeof(rs1));
    return d;
}

int64_t rv64_fmv_x_d(double rs1)
{
    int64_t i;
	memcpy((void*)(&i), (void*)(&rs1), sizeof(rs1));
    return i;
}

// pseudo-instruction: read round mode, expand to `csrrs rd, frm, x0`
int64_t rv64p_frrm(void)
{
    return (int64_t)s_frm;
}

double rv64_fsqrt_d(double rs)
{
#pragma STDC FENV_ACCESS ON

    // store the original rounding mode
    const int originalRounding = fegetround();

    // set the rounding mode
    switch(s_frm) {
        case RV_FRM_RNE: fesetround(FE_TONEAREST); break;
        case RV_FRM_RTZ: fesetround(FE_TOWARDZERO); break;
        case RV_FRM_RDN: fesetround(FE_DOWNWARD); break;
        case RV_FRM_RUP: fesetround(FE_UPWARD); break;
        default:
            __builtin_unreachable();
    }

    // do calculations
    double dst = sqrt(rs);

    // restore the original mode afterwards
    fesetround(originalRounding);

#pragma STDC FENV_ACCESS OFF

    return dst;
}


// pseudo-instruction: set round mode, expand to `csrrw rd, frm, rs1`
void rv64p_fsrm(int64_t rs)
{
    //printf("rv64p_fsrm(): rs=%ld\n", rs); fflush(stdout);
    s_frm = (uint8_t)rs;
}

double rv64_fsub_d(double rs1, double rs2)
{
#pragma STDC FENV_ACCESS ON

    // store the original rounding mode
    const int originalRounding = fegetround();

    // set the rounding mode
    switch(s_frm) {
        case RV_FRM_RNE: fesetround(FE_TONEAREST); break;
        case RV_FRM_RTZ: fesetround(FE_TOWARDZERO); break;
        case RV_FRM_RDN: fesetround(FE_DOWNWARD); break;
        case RV_FRM_RUP: fesetround(FE_UPWARD); break;
        default:
            __builtin_unreachable();
    }

    // do calculations
    double dst = rs1 - rs2;

    // restore the original mode afterwards
    fesetround(originalRounding);

#pragma STDC FENV_ACCESS OFF

    return dst;
}



// x[rd]=M[x[rs1]+sext(offset)][63:0]
int64_t rv64_ld(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t addr = rs1 + sext(imm12, 11);
    assert(addr > 0);
    int64_t dst;
    // x86-64 and rv64 are all native little-endian
	memcpy(&dst, (void*)addr, sizeof(dst));

    return dst;
}

// x[rd]=sext(M[x[rs1]+sext(offset)][31:0])
int64_t rv64_lw(int64_t rs1, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t addr = rs1 + sext(imm12, 11);
    assert(addr > 0);
    // load 4-byte
    uint32_t dst0;
	memcpy(&dst0, (void*)addr, sizeof(dst0));
    // sext() to 8-byte
    int64_t dst = sext(dst0, 31);

    return dst;
}



// p means 'pseudo'
int64_t rv64p_li_imm32(int32_t imm32)
{
    int64_t dst = 0;

    if (imm32 >= -2048 && imm32 <= 2047) {
        return rv64_addi(0, low12(imm32));
    }

    if (low12(imm32) >= 0) {
        dst = rv64_lui(hi20(imm32));
    } else {
        // now `+1` does not overflow the low 20-bit of hi20(imm32),
        // and lui only uses the low 20-bit of the sum
        dst = rv64_lui(hi20(imm32) + 1);
    }

    dst = rv64_addiw(dst, low12(imm32));

    return dst;
}



// x[rd]=sext(imm[31:12] << 12)
int64_t rv64_lui(int32_t imm20)
{
    // don't care the top 12-bit of imm20, which will be discarded anyway
    //assert(is_nbit_imm(imm20, 20));

    PRI32("lui()", imm20, imm20);
    int64_t tmp = sext(((uint64_t)imm20) << 12, 31);
    PRI64("lui()", tmp, tmp);
    return tmp;
}

int64_t rv64_mul(int64_t rs1, int64_t rs2)
{
    return rs1 * rs2;
}

int64_t rv64_mulh(int64_t rs1, int64_t rs2)
{
    return ((int128_t)rs1 * rs2) >> 64;
}

uint64_t rv64_mulhu(uint64_t rs1, uint64_t rs2)
{
    return ((uint128_t)rs1 * rs2) >> 64;
}

int64_t rv64_or(int64_t rs1, int64_t rs2)
{
    return rs1 | rs2;
}

int64_t rv64_sll(int64_t rs1, int64_t rs2)
{
    return (int64_t)(((uint64_t)rs1) << (rs2 & 63));
}

int64_t rv64_slli(int64_t rs1, uint8_t shamt)
{
    return (int64_t)(((uint64_t)rs1) << shamt);
}

int64_t rv64_srl(int64_t rs1, int64_t rs2)
{
    return (int64_t)(((uint64_t)rs1) >> (rs2 & 63));
}

int64_t rv64_srli(int64_t rs1, uint8_t shamt)
{
    return (int64_t)(((uint64_t)rs1) >> shamt);
}

// M[x[rs1]+sext(offset)]=rs2[63:0]
void rv64_sd(int64_t rs1, int64_t rs2, int32_t imm12)
{
    assert(is_nbit_imm(imm12, 12));
    int64_t addr = rs1 + sext(imm12, 11);
    assert(addr > 0);
    // x86-64 and rv64 are all native little-endian
	memcpy((void*)addr, (void*)(&rs2), sizeof(rs2));
}


int64_t rv64_sub(int64_t rs1, int64_t rs2)
{
    return rs1 - rs2;
}

int64_t rv64_xor(int64_t rs1, int64_t rs2)
{
    return rs1 ^ rs2;
}
