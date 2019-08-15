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
#include "stdarg.h"

namespace randomx {

#define INDENT_SIZE      4
#define COMMENT_COL     30

    void print_insn(const char* comment, const char *format,...) {
        int r;
        va_list ap;

        // indent spaces
        for (int i = 0; i < INDENT_SIZE; i ++)
            printf(" ");

        // instruction proper
        va_start(ap,format);
        r = vprintf(format,ap);
        va_end(ap);

        if (!comment) {
            printf("\n");
        } else {
            // space between instruction and comments
            for (int i = 0; i < (COMMENT_COL - INDENT_SIZE - r); ++i)
                printf(" ");

            // comment and newline
            printf("; %s\n", comment);
        }
    }

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

    static uint8_t fl_tmp = 28;
    static uint8_t fh_tmp = 29;



    uint8_t AssemblyGeneratorRV64::getIRegIdx(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr, bool is_src) {

        // handle special case when `ibc.isrc=&BytecodeMachine::zero` for I*_M instructions
        if (is_src && (instr.src % RegistersCount) == (instr.dst % RegistersCount) &&
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
            printf("L%03d:", i);
            for (int i = 0; i < COMMENT_COL - 5; i ++) printf(" ");
            printf("# ");
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

    // x_tmp1 holds the offset in scratchpad, either for load or store of integer instructions (I*)
    void AssemblyGeneratorRV64::getIOffset(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr, bool is_load) {
        uint8_t reg = getIRegIdx(ibc, nreg, instr, is_load);

        // reg==0 means "src=0": the address is imm only, so we can mask it with L3 mask at compile time.
        // fixme: if scratchpad base addr is fixed, load operations can be further optimized by adding the base addr at compile time.
        if (is_load && reg == 0) { 
            InstructionByteCode _ibc = ibc;
            _ibc.imm &= ScratchpadL3Mask;
            load_ibc_imm(_ibc);
            return;
        }

        //x_tmp1 = *ibc.isrc + ibc.imm
        if ((int32_t)ibc.imm >= -2048 && ibc.imm <= 2047) {
            print_insn("load imm32", "addi x%d, x%d, %d", x_tmp1, reg, (int32_t)ibc.simm);
        } else {

            // x_tmp1 = li_imm32(ibc.imm)
            if (low12(ibc.simm) >= 0) {
                print_insn("load imm32", "lui x%d, %d", x_tmp1, hi20(ibc.imm));
            } else {
                print_insn("load imm32", "lui x%d, %d", x_tmp1, hi20(ibc.imm) + 1);
            }
            print_insn(nullptr, "addiw x%d, x%d, %d", x_tmp1, x_tmp1, low12(ibc.imm));

            // x_tmp1 = x_tmp1 + isrc
            if (reg != x_zero)
                print_insn("offset=src+imm32", "add x%d, x%d, x%d", x_tmp1, x_tmp1, reg);
        }

        // x_tmp1 = x_tmp1 & mask: offset in scratchpad
        if ((instr.src % RegistersCount == instr.dst % RegistersCount) || instr.getModCond() >= StoreL3Condition){
            print_insn("offset & L3 mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_L3_mask);
        } else {
            if (instr.getModMem())
                print_insn("offset & L1 mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_L1_mask);
            else
                print_insn("offset & L2 mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_L2_mask);
        }
    }

    // x_tmp1 holds the offset in scratchpad for load of float instructions (F*)
    void AssemblyGeneratorRV64::getFOffset(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr) {
        uint8_t src = getIRegIdx(ibc, nreg, instr, true/*is_src*/);

        //x_tmp1 = *ibc.isrc + ibc.imm
        if ((int32_t)ibc.imm >= -2048 && ibc.imm <= 2047) {
            print_insn("load imm32", "addi x%d, x%d, %d", x_tmp1, src, (int32_t)ibc.simm);
        } else {
            // x_tmp1 = li_imm32(ibc.imm)
            if (low12(ibc.simm) >= 0) {
                print_insn("load imm32", "lui x%d, %d", x_tmp1, hi20(ibc.imm));
            } else {
                print_insn("load imm32", "lui x%d, %d", x_tmp1, hi20(ibc.imm) + 1);
            }
            print_insn(nullptr, "addiw x%d, x%d, %d", x_tmp1, x_tmp1, low12(ibc.imm));

            // x_tmp1 = x_tmp1 + isrc
            print_insn("offset=src+imm32", "add x%d, x%d, x%d", x_tmp1, x_tmp1, src);
        }

        // x_tmp1 = x_tmp1 & LxMask: offset in scratchpad
        if (instr.getModMem())
            print_insn("offset & L1 mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_L1_mask);
        else
            print_insn("offset & L2 mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_L2_mask);
    }

    // x_tmp1 holds 64-bit value from scratchpad
    void AssemblyGeneratorRV64::load64(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr) {
        getIOffset(ibc, nreg, instr, true);
        print_insn("offset+scratchpad", "add x%d, x%d, x%d", x_tmp1, x_tmp1, x_scratchpad);
        print_insn(nullptr, "ld x%d, 0(x%d)", x_tmp1, x_tmp1);
    }

    // read 8-byte to x_tmp1/x_tmp2, 4-byte each
    void AssemblyGeneratorRV64::load32_x2(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction& instr) {
        getFOffset(ibc, nreg, instr);
        print_insn("offset+scratchpad", "add x%d, x%d, x%d", x_tmp2, x_tmp1, x_scratchpad);
        print_insn(nullptr, "lw x%d, 0(x%d)", x_tmp1, x_tmp2);
        print_insn(nullptr, "lw x%d, 4(x%d)", x_tmp2, x_tmp2);
    }

    // load ibc.imm to x_tmp1
    void AssemblyGeneratorRV64::load_ibc_imm(InstructionByteCode& ibc) {
    
        
        if (ibc.simm >= -2048 && ibc.imm <= 2047) {
            print_insn(nullptr, "add x%d, x%d, %d", x_tmp1, x_zero, low12(ibc.imm));
        } else {
            // x_tmp1 = li_imm32(ibc.imm)
            if (low12(ibc.simm) >= 0) {
                print_insn(nullptr, "lui x%d, %d", x_tmp1, hi20(ibc.imm));
            } else {
                print_insn(nullptr, "lui x%d, %d", x_tmp1, hi20(ibc.imm) + 1);
            }
            print_insn(nullptr, "addiw x%d, x%d, %d", x_tmp1, x_tmp1, low12(ibc.imm));
        }
    }
    

    void AssemblyGeneratorRV64::h_IADD_RS(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);

        // *ibc.isrc << ibc.shift
        if (ibc.shift == 0) {
            print_insn("dst+=src", "add x%d, x%d, x%d", dst, dst, src);
        } else {
            print_insn("src<<shamt", "slli x%d, x%d, %d", x_tmp1, src, ibc.shift);
            print_insn("dst+=(src<<shamt)", "add x%d, x%d, x%d", dst, dst, x_tmp1);
        }

        // ibc.imm
        if (ibc.imm != 0) {
            if (ibc.simm >= -2048 && ibc.imm <= 2047) {
                print_insn("dst+=imm", "addi x%d, x%d, %d", dst, dst, low12(ibc.imm));
            } else {
                // x_tmp1 = li_imm32(ibc.imm)
                if (low12(ibc.simm) >= 0) {
                    print_insn(nullptr, "lui x%d, %d", x_tmp1, hi20(ibc.imm));
                } else {
                    print_insn(nullptr, "lui x%d, %d", x_tmp1, hi20(ibc.imm) + 1);
                }
                print_insn(nullptr, "addiw x%d, x%d, %d", x_tmp1, x_tmp1, low12(ibc.imm));

                // dst = dst + x_tmp1
                print_insn("dst+=imm", "add x%d, x%d, x%d", dst, dst, x_tmp1);
            }
        }
    }

    void AssemblyGeneratorRV64::h_IADD_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "add x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_ISUB_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        
        if ((prog(i).src % RegistersCount == prog(i).dst % RegistersCount)) { // src=imm32
            if (ibc.imm != 0) {
                load_ibc_imm(ibc);
                print_insn("dst-=imm", "sub x%d, x%d, x%d", dst, dst, x_tmp1);
            }
        } else {
            uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
            print_insn(nullptr, "sub x%d, x%d, x%d", dst, dst, src);
        }
    }

    void AssemblyGeneratorRV64::h_ISUB_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 13 has a instr case where src==dst, which leads to src=0
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "sub x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_IMUL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        
        // it could be a IMUL_RCP. in such cases: dst*=ibc.imm
        if (prog(i).opcode < ceil_IMUL_RCP && prog(i).opcode >= ceil_ISMULH_M) {
            load_ibc_imm(ibc);
            print_insn(nullptr, "mul x%d, x%d, x%d", dst, dst, x_tmp1);
            return;
        }
        
        if ((prog(i).src % RegistersCount == prog(i).dst % RegistersCount)) { // src=imm32
            if (ibc.imm != 0) {
                load_ibc_imm(ibc);
                print_insn("dst*=imm", "mul x%d, x%d, x%d", dst, dst, x_tmp1);
            }
        } else {
            uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
            print_insn(nullptr, "mul x%d, x%d, x%d", dst, dst, src);
        }
    }

    void AssemblyGeneratorRV64::h_IMUL_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 1 has a instr case where src==dst, which leads to src=0
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "mul x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_IMULH_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
        print_insn(nullptr, "mulhu x%d, x%d, x%d", dst, dst, src);
    }

    void AssemblyGeneratorRV64::h_IMULH_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 7 has a instr case where src==dst, which leads to src=0
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "mulhu x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_ISMULH_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
        print_insn(nullptr, "mulh x%d, x%d, x%d", dst, dst, src);
    }

    void AssemblyGeneratorRV64::h_ISMULH_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 17 has a instr case where src==dst, which leads to src=0
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "mulh x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_INEG_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        print_insn(nullptr, "sub x%d, x%d, x%d", dst, x_zero, dst);
    }

    void AssemblyGeneratorRV64::h_IXOR_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        
        if ((prog(i).src % RegistersCount == prog(i).dst % RegistersCount)) { // src=imm32
            load_ibc_imm(ibc);
            print_insn(nullptr, "xor x%d, x%d, x%d", dst, dst, x_tmp1);
        } else {
            uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
            print_insn(nullptr, "xor x%d, x%d, x%d", dst, dst, src);
        }
    }

    void AssemblyGeneratorRV64::h_IXOR_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 3 has a instr case where src==dst, which leads to src=0
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        load64(ibc, nreg, prog(i));
        print_insn(nullptr, "xor x%d, x%d, x%d", dst, dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_IROR_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 5 has cases where src==dst, which leads to src=imm
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        // get right shamt into x_tmp1        
        if ((prog(i).src % RegistersCount == prog(i).dst % RegistersCount)) { // src=imm32
            print_insn("right shamt", "add x%d, x%d, %d", x_tmp1, x_zero, low12(ibc.imm & 63));
        } else {
            uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
            print_insn("right shamt", "andi x%d, x%d, 63", x_tmp1, src);
        }
        
        print_insn("left shamt", "sub x%d, x%d, x%d", x_tmp2, x_n64, x_tmp1);
        print_insn(nullptr, "srl x%d, x%d, x%d", x_tmp1, dst, x_tmp1);
        print_insn(nullptr, "sll x%d, x%d, x%d", x_tmp2, dst, x_tmp2);
        print_insn(nullptr, "or x%d, x%d, x%d", dst, x_tmp1, x_tmp2);
    }

    void AssemblyGeneratorRV64::h_IROL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // NB: nonce 2 has a case where src==dst, which leads to src=imm
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);

        // get left shamt into x_tmp1        
        if ((prog(i).src % RegistersCount == prog(i).dst % RegistersCount)) { // src=imm32
            print_insn("left shamt", "add x%d, x%d, %d", x_tmp1, x_zero, low12(ibc.imm & 63));
        } else {
            uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);
            print_insn("left shamt", "andi x%d, x%d, 63", x_tmp1, src);
        }
        
        print_insn("right shamt", "sub x%d, x%d, x%d", x_tmp2, x_n64, x_tmp1);
        print_insn(nullptr, "sll x%d, x%d, x%d", x_tmp1, dst, x_tmp1);
        print_insn(nullptr, "srl x%d, x%d, x%d", x_tmp2, dst, x_tmp2);
        print_insn(nullptr, "or x%d, x%d, x%d", dst, x_tmp1, x_tmp2);
    }

    void AssemblyGeneratorRV64::h_IMUL_RCP(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        // IMUL_RCP maps to either NOP or IMUL_R
        __builtin_unreachable();
    }

    void AssemblyGeneratorRV64::h_ISWAP_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst = getIRegIdx(ibc, nreg, prog(i), false);
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);

        if (dst == src) { // it's a NOP, and the register is treated as "unmodfied"
            // nonce 2 has a case where dst==src
            return;
        }
        print_insn(nullptr, "addi x%d, x%d, 0", x_tmp1, src);
        print_insn(nullptr, "addi x%d, x%d, 0", src, dst);
        print_insn(nullptr, "addi x%d, x%d, 0", dst, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_FSWAP_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);
        
        print_insn(nullptr, "fmv.x.d x%d, f%d", x_tmp1, dst_l);
        print_insn(nullptr, "fmv.x.d x%d, f%d", x_tmp2, dst_h);
        
        print_insn(nullptr, "fmv.d.x f%d, x%d", dst_l, x_tmp2);
        print_insn(nullptr, "fmv.d.x f%d, x%d", dst_h, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_FADD_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t src_l = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  true/*is_low*/);
        uint8_t src_h = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  false/*is_low*/);
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);
        print_insn(nullptr, "fadd.d f%d, f%d, f%d", dst_l, dst_l, src_l);
        print_insn(nullptr, "fadd.d f%d, f%d, f%d", dst_h, dst_h, src_h);
    }

    void AssemblyGeneratorRV64::h_FADD_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);

        load32_x2(ibc, nreg, prog(i));

        print_insn(nullptr, "fcvt.d.l f%d, x%d", fl_tmp, x_tmp1);
        print_insn(nullptr, "fcvt.d.l f%d, x%d", fh_tmp, x_tmp2);
        print_insn(nullptr, "fadd.d f%d, f%d, f%d", dst_l, dst_l, fl_tmp);
        print_insn(nullptr, "fadd.d f%d, f%d, f%d", dst_h, dst_h, fh_tmp);
    }

    void AssemblyGeneratorRV64::h_FSUB_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t src_l = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  true/*is_low*/);
        uint8_t src_h = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  false/*is_low*/);
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);
        print_insn(nullptr, "fsub.d f%d, f%d, f%d", dst_l, dst_l, src_l);
        print_insn(nullptr, "fsub.d f%d, f%d, f%d", dst_h, dst_h, src_h);
    }

    void AssemblyGeneratorRV64::h_FSUB_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);

        load32_x2(ibc, nreg, prog(i));

        print_insn(nullptr, "fcvt.d.l f%d, x%d", fl_tmp, x_tmp1);
        print_insn(nullptr, "fcvt.d.l f%d, x%d", fh_tmp, x_tmp2);
        print_insn(nullptr, "fsub.d f%d, f%d, f%d", dst_l, dst_l, fl_tmp);
        print_insn(nullptr, "fsub.d f%d, f%d, f%d", dst_h, dst_h, fh_tmp);
    }

    void AssemblyGeneratorRV64::h_FSCAL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);

        print_insn(nullptr, "fmv.x.d x%d, f%d", x_tmp1, dst_l);
        print_insn(nullptr, "fmv.x.d x%d, f%d", x_tmp2, dst_h);
        
        print_insn(nullptr, "xor x%d, x%d, x%d", x_tmp1, x_tmp1, x_scale_mask);
        print_insn(nullptr, "xor x%d, x%d, x%d", x_tmp2, x_tmp2, x_scale_mask);
        
        print_insn(nullptr, "fmv.d.x f%d, x%d", dst_l, x_tmp1);
        print_insn(nullptr, "fmv.d.x f%d, x%d", dst_h, x_tmp2);
    }

    void AssemblyGeneratorRV64::h_FMUL_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t src_l = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  true/*is_low*/);
        uint8_t src_h = getFRegIdx(ibc, nreg, prog(i), true/*is_src*/,  false/*is_low*/);
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);
        print_insn(nullptr, "fmul.d f%d, f%d, f%d", dst_l, dst_l, src_l);
        print_insn(nullptr, "fmul.d f%d, f%d, f%d", dst_h, dst_h, src_h);
    }

    void AssemblyGeneratorRV64::h_FDIV_M(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);

        load32_x2(ibc, nreg, prog(i));
        
        print_insn("cvt to double", "fcvt.d.l f%d, x%d", fl_tmp, x_tmp1);
        print_insn(nullptr,         "fcvt.d.l f%d, x%d", fh_tmp, x_tmp2);

        print_insn("mv double to int", "fmv.x.d x%d, f%d", x_tmp1, fl_tmp);
        print_insn(nullptr,            "fmv.x.d x%d, f%d", x_tmp2, fh_tmp);

        print_insn("`and` with mantissa mask", "and x%d, x%d, x%d", x_tmp1, x_tmp1, x_E_and_mask);
        print_insn(nullptr,                    "and x%d, x%d, x%d", x_tmp2, x_tmp2, x_E_and_mask);
        
        print_insn("`or` with exponent mask (lo & hi)", "or x%d, x%d, x%d", x_tmp1, x_tmp1, x_E_or_mask_l);
        print_insn(nullptr,                             "or x%d, x%d, x%d", x_tmp2, x_tmp2, x_E_or_mask_h);

        print_insn("mv int back to double", "fmv.d.x f%d, x%d", fl_tmp, x_tmp1);
        print_insn(nullptr,                 "fmv.d.x f%d, x%d", fh_tmp, x_tmp2);
        
        print_insn("div",   "fdiv.d f%d, f%d, f%d", dst_l, dst_l, fl_tmp);
        print_insn(nullptr, "fdiv.d f%d, f%d, f%d", dst_h, dst_h, fh_tmp);
    }

    void AssemblyGeneratorRV64::h_FSQRT_R(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
        uint8_t dst_l = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, true/*is_low*/);
        uint8_t dst_h = getFRegIdx(ibc, nreg, prog(i), false/*is_src*/, false/*is_low*/);
        print_insn(nullptr, "fsqrt.d f%d, f%d", dst_l, dst_l);
        print_insn(nullptr, "fsqrt.d f%d, f%d", dst_h, dst_h);
    }

    void AssemblyGeneratorRV64::h_CFROUND(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {

        const char l_fsrm[] = "FSRM";
        const char l_lt3[] = "LT3";
        uint8_t src = getIRegIdx(ibc, nreg, prog(i), true);

        uint8_t right_shamt = ibc.imm & 63;
        uint8_t left_shamt = 64 - right_shamt;
        
        print_insn("ror",   "srli x%d, x%d, %d", x_tmp1, src, right_shamt);
        print_insn(nullptr, "slli x%d, x%d, %d", x_tmp2, src, left_shamt);
        print_insn(nullptr, "or x%d, x%d, x%d", x_tmp1, x_tmp1, x_tmp2);
        
        print_insn("rx round mode",   "andi x%d, x%d, %d", x_tmp1, x_tmp1, 0x3);
        print_insn("0->0(RNE)", "beq x%d, x0, %s", x_tmp1, l_fsrm);

        // x_tmp1 != 0
        print_insn(nullptr, "addi x%d, x0, 3", x_tmp2);
        print_insn(nullptr, "blt x%d, x%d, %s", x_tmp1, x_tmp2, l_lt3);
        // x_tmp1 >= 3
        print_insn("3->1(RTZ)", "addi x%d, x0, %d", x_tmp1, 1);
        print_insn(nullptr, "jal %s", l_fsrm);
        // x_tmp < 3
        print_insn("1->2(RDN), 2->3(RUP)", "%s: addi x%d, x%d, 1", l_lt3, x_tmp1, x_tmp1);

        // do the fsrm via `csrrw`
        print_insn(nullptr, "%s: csrrw x0, frm, x%d", l_fsrm, x_tmp1);
    }

    void AssemblyGeneratorRV64::h_CBRANCH(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_ISTORE(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }

    void AssemblyGeneratorRV64::h_NOP(InstructionByteCode& ibc, NativeRegisterFile* nreg, Program& prog, int i) {
    }





}
