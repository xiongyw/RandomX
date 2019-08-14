/*
Copyright (c) 2018-2019, tevador <tevador@gmail.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <climits>
#include "assembly_generator_rv64.hpp"
#include "common.hpp"
#include "reciprocal.h"
#include "program.hpp"
#include "superscalar.hpp"

#include "instruction.hpp"

namespace randomx {

    // integer register (x?) index for r0~7
    static uint8_t s_ireg_map[RegistersCount] = { 10, 11, 12, 13, 14, 15, 16, 17 };

    // float register (f?) index for f0~3, e0~3, a0~3
    static uint8_t s_freg_map[RegisterCountFlt * 3 * 2] = {
        10, 11, 12, 13, 14, 15, 16, 17, // f0l, f0h, f1l, f1h, f2l, f2h, f3l, f3h
         0,  1,  2,  3,  4,  5,  6,  7, // e0l, e0h, e1l, e1h, e2l, e2h, e3l, e3h
         8,  9, 18, 19, 20, 21, 22, 23  // a0l, a0h, a1l, a1h, a2l, a2h, a3l, a3h
    };

    static uint8_t x_zero = 0;
    static uint8_t x_scratchpad = 28;
    static uint8_t x_tmp1 = 29;
    static uint8_t x_tmp2 = 30;
    static uint8_t x_n64 = 31;
    static uint8_t x_L1_mask = 9;
    static uint8_t x_L2_mask = 18;
    static uint8_t x_L3_mask = 19;
    static uint8_t x_scale_mask = 20;
    static uint8_t x_E_and_mask = 21;
    static uint8_t x_E_or_mask_l = 22;
    static uint8_t x_E_or_mask_h = 23;
    


    uint8_t AssemblyGeneratorRV64::getIRegIdx(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr, bool is_src) {

        // handle special case when `ibc.isrc=&BytecodeMachine::zero` for *_M instructions
        if ((instr.src % RegistersCount) == (instr.dst % RegistersCount) && 
            (ibc.type == InstructionType::IADD_M ||
             ibc.type == InstructionType::ISUB_M ||
             ibc.type == InstructionType::IMUL_M ||
             ibc.type == InstructionType::IMULH_M ||
             ibc.type == InstructionType::ISMULH_M ||
             ibc.type == InstructionType::IXOR_M)) {
            return x_zero;
        }

        int offset = (uint8_t*)(is_src? ibc.isrc : ibc.idst) - (uint8_t*)nreg;
        offset /= 8;
        
        //if (is_src && (offset < 0 || offset >= RegistersCount))  return x_zero;
        
        assert(offset < RegistersCount);
        return s_ireg_map[offset];
    }

    uint8_t AssemblyGeneratorRV64::getFRegIdx(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr, bool is_src, bool is_low) {
        int offset = (uint8_t*)(is_src? ibc.fsrc : ibc.fdst) - (uint8_t*)nreg - 8 * RegistersCount;
        offset /= 8;
        assert(offset < RegisterCountFlt * 3 * 2 && (offset % 2) == 0);
        return s_freg_map[is_low? offset : offset + 1];
    }

    void AssemblyGeneratorRV64::generateProgram(Program& prog) {

        NativeRegisterFile reg;
        BytecodeMachine decoder;
        InstructionByteCode bytecode[RANDOMX_PROGRAM_SIZE];

        // compile all instructions first
        decoder.beginCompilation(reg);
        for (unsigned i = 0; i < prog.getSize(); ++i) {
            auto& instr = prog(i);
            auto& ibc = bytecode[i];
            decoder.compileInstruction(instr, i, ibc);
        }

        for (unsigned i = 0; i < prog.getSize(); ++i) {
            // VM instruction
            printf("L%03d:                           # ", i);
            std::cout << prog(i);

            // native instructions
            generateCode(bytecode[i], &reg, prog, i);
        }
    }

    void AssemblyGeneratorRV64::generateCode(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        switch(ibc.type) {
            case InstructionType::IADD_RS:  h_IADD_RS(ibc, nreg, prog, i);  break;
            case InstructionType::IADD_M:   h_IADD_M(ibc, nreg, prog, i);   break;
            case InstructionType::ISUB_R:   h_ISUB_R(ibc, nreg, prog, i);   break;
            case InstructionType::ISUB_M:   h_ISUB_M(ibc, nreg, prog, i);   break;
            case InstructionType::IMUL_R:   h_IMUL_R(ibc, nreg, prog, i);   break;
            case InstructionType::IMUL_M:   h_IMUL_M(ibc, nreg, prog, i);   break;
            case InstructionType::IMULH_R:  h_IMULH_R(ibc, nreg, prog, i);  break;
            case InstructionType::IMULH_M:  h_IMULH_M(ibc, nreg, prog, i);  break;
            case InstructionType::ISMULH_R: h_ISMULH_R(ibc, nreg, prog, i); break;
            case InstructionType::ISMULH_M: h_ISMULH_M(ibc, nreg, prog, i); break;
            case InstructionType::IMUL_RCP: h_IMUL_RCP(ibc, nreg, prog, i); break;
            case InstructionType::INEG_R:   h_INEG_R(ibc, nreg, prog, i);   break;
            case InstructionType::IXOR_R:   h_IXOR_R(ibc, nreg, prog, i);   break;
            case InstructionType::IXOR_M:   h_IXOR_M(ibc, nreg, prog, i);   break;
            case InstructionType::IROR_R:   h_IROR_R(ibc, nreg, prog, i);   break;
            case InstructionType::IROL_R:   h_IROL_R(ibc, nreg, prog, i);   break;
            case InstructionType::ISWAP_R:  h_ISWAP_R(ibc, nreg, prog, i);  break;
            case InstructionType::FSWAP_R:  h_FSWAP_R(ibc, nreg, prog, i);  break;
            case InstructionType::FADD_R:   h_FADD_R(ibc, nreg, prog, i);   break;
            case InstructionType::FADD_M:   h_FADD_M(ibc, nreg, prog, i);   break;
            case InstructionType::FSUB_R:   h_FSUB_R(ibc, nreg, prog, i);   break;
            case InstructionType::FSUB_M:   h_FSUB_M(ibc, nreg, prog, i);   break;
            case InstructionType::FSCAL_R:  h_FSCAL_R(ibc, nreg, prog, i);  break;
            case InstructionType::FMUL_R:   h_FMUL_R(ibc, nreg, prog, i);   break;
            case InstructionType::FDIV_M:   h_FDIV_M(ibc, nreg, prog, i);   break;
            case InstructionType::FSQRT_R:  h_FSQRT_R(ibc, nreg, prog, i);  break;
            case InstructionType::CBRANCH:  h_CBRANCH(ibc, nreg, prog, i);  break;
            case InstructionType::CFROUND:  h_CFROUND(ibc, nreg, prog, i);  break;
            case InstructionType::ISTORE:   h_ISTORE(ibc, nreg, prog, i);   break;
            case InstructionType::NOP:                                break;
            default:
                __builtin_unreachable();
        }
    }

    void AssemblyGeneratorRV64::traceint(InstructionByteCode& ibc) {
        if (trace) {
            //printf("    sub %s, 8\n", regSP);
            //printf("    sd %s, 0(%s)\n", regR[ibc.dst], regSP);
        }
    }

    void AssemblyGeneratorRV64::traceflt(InstructionByteCode& ibc) {
        if (trace) {
            //printf("    sub %s, 8\n", regSP);
            //printf("    sd %s, 0(%s)\n", regZero, regSP);
        }
    }

    void AssemblyGeneratorRV64::tracenop(InstructionByteCode& ibc) {
        if (trace) {
            //printf("    sub %s, 8\n", regSP);
            //printf("    sd %s, 0(%s)\n", regZero, regSP);
        }
    }

    // x_tmp1 holds the offset in scratchpad, either for load or store
    void AssemblyGeneratorRV64::get_offset(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr, bool is_load) {
        uint8_t reg = getIRegIdx(ibc, nreg, instr, is_load);
    
        //x_tmp1 = *ibc.isrc + ibc.imm
        if ((int32_t)ibc.imm >= -2048 && ibc.imm <= 2047) {
            printf("    addi x%d, x%d, %d      ; load imm32\n", x_tmp1, reg, (int32_t)ibc.simm);
        } else {
            // x_tmp1 = li_imm32(ibc.imm)
            if (low12(ibc.simm) >= 0) {
                printf("    lui x%d, %d            ; load imm32\n", x_tmp1, hi20(ibc.imm));
            } else {
                printf("    lui x%d, %d            ; load imm32\n", x_tmp1, hi20(ibc.imm) + 1);
            }
            printf("    addiw x%d, x%d, %d\n", x_tmp1, x_tmp1, low12(ibc.imm));

            // x_tmp1 = x_tmp1 + isrc
            if (reg != x_zero)
                printf("    add x%d, x%d, x%d          ; offset = src + imm32\n", x_tmp1, x_tmp1, reg);
        }

        // x_tmp1 = x_tmp1 & mask: offset in scratchpad
        if ((instr.src % RegistersCount == instr.dst % RegistersCount) || instr.getModCond() >= StoreL3Condition){ 
            printf("    and x%d, x%d, x%d           ; offset & scratchpad_mask\n", x_tmp1, x_tmp1, x_L3_mask);
        } else {
            printf("    and x%d, x%d, x%d           ; offset & scratchpad_mask\n", x_tmp1, x_tmp1, instr.getModMem() ? x_L1_mask : x_L2_mask);
        }
    }

    // x_tmp1 holds 64-bit value from scratchpad
    void AssemblyGeneratorRV64::load64(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr) {
        get_offset(ibc, nreg, instr, true);
        printf("    add x%d, x%d, x%d           ; offset + scratchpad\n", x_tmp1, x_tmp1, x_scratchpad);
        printf("    ld x%d, 0(x%d)\n", x_tmp1, x_tmp1);
    }

    // read 8-byte to x_tmp1/x_tmp2, 4-byte each
    void AssemblyGeneratorRV64::load32_x2(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr) {
        get_offset(ibc, nreg, instr, true);
        printf("    add x%d, x%d, x%d\n", x_tmp2, x_tmp1, x_scratchpad);
        printf("    lw x%d, 0(x%d)\n", x_tmp1, x_tmp2);
        printf("    lw x%d, 4(x%d)\n", x_tmp2, x_tmp2);
    }
    

    void AssemblyGeneratorRV64::h_IADD_RS(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
        
        printf("    slli x%d, x%d, %d\n", x_tmp1, src, ibc.shift);

        if (ibc.imm != 0) {
            if ((int32_t)ibc.imm >= -2048 && ibc.imm <= 2047) {
                printf("    addi x%d, x%d, %d\n", x_tmp1, x_tmp1, (int32_t)ibc.simm);
            } else {
                // x_tmp2 = li_imm32(ibc.imm)
                if (low12(ibc.simm) >= 0) {
                    printf("    lui x%d, %d\n", x_tmp2, hi20(ibc.imm));
                } else {
                    printf("    lui x%d, %d\n", x_tmp2, hi20(ibc.imm) + 1);
                }
                printf("    addiw x%d, x%d, %d\n", x_tmp2, x_tmp2, low12(ibc.imm));

                // x_tmp1 = x_tmp1 + x_tmp2
                printf("    add x%d, x%d, x%d\n", x_tmp1, x_tmp1, x_tmp2);
            }
        }

        printf("    add x%d, x%d, x%d\n", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_IADD_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        
        load64(ibc, nreg, prog(i));
        printf("    add x%d, x%d, x%d\n", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_ISUB_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISUB_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IMUL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IMUL_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IMULH_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IMULH_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISMULH_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISMULH_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_INEG_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IXOR_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IXOR_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IROR_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IROL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_IMUL_RCP(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISWAP_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FSWAP_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FADD_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        printf("    fadd.d f%d, f%d, f%d\n", 
            getFRegIdx(ibc, nreg, prog(i), false, true),
            getFRegIdx(ibc, nreg, prog(i), false, true),
            getFRegIdx(ibc, nreg, prog(i), true,  true));
        printf("    fadd.d f%d, f%d, f%d\n", 
            getFRegIdx(ibc, nreg, prog(i), false, false),
            getFRegIdx(ibc, nreg, prog(i), false, false),
            getFRegIdx(ibc, nreg, prog(i), true,  false));
    }

    void AssemblyGeneratorRV64::h_FADD_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FSUB_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FSUB_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FSCAL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FMUL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FDIV_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_FSQRT_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_CFROUND(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_CBRANCH(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISTORE(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_NOP(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }





}
