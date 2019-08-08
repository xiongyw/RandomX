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

// note: for JIT later, do not use any risc-v pseudo-instructions (or register abi-names) here.

namespace randomx {

	static const char* regR[]  = { "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17" };

	static const char* regFLo[] = { "f10", "f11", "f12", "f13" };
	static const char* regFHi[] = { "f14", "f15", "f16", "f17" };

	static const char* regELo[] = { "f0", "f1", "f2", "f3" };
	static const char* regEHi[] = { "f4", "f5", "f6", "f7" };

	static const char* regALo[] = {  "f8",  "f9", "f18", "f19" };
	static const char* regAHi[] = { "f20", "f21", "f22", "f23" };

	static const char* regFELo[] = { "f10", "f11", "f12", "f13", "f0", "f1", "f2", "f3" };
	static const char* regFEHi[] = { "f14", "f15", "f16", "f17", "f4", "f5", "f6", "f7" };

	static const char* regScratchpadAddr = "x7";
	static const char* regIC = "x28";  // instruction count
    static const char* regZero = "x0";
    static const char* regRA = "x1";   // return address
    static const char* regSP = "x2";   // stack pointer
    static const char* regTmp1 = "x29";
    static const char* regTmp2 = "x30";

	static const char* tempRegxLo = "f28";
	static const char* tempRegxHi = "f24";
	static const char* mantissaMaskRegLo = "f29";  // E 'and' mask
	static const char* mantissaMaskRegHi = "f25";
	static const char* exponentMaskRegLo = "f30";  // E 'or' mask
	static const char* exponentMaskRegHi = "f26";
	static const char* scaleMaskRegLo = "f31"; // scale mask
	static const char* scaleMaskRegHi = "f27";

    /*
     * this routine is to check whether or not the value stored in int32_t can be
     * represented by 12-bit:
     * - if it's unsigned, then bit 31-12 should be all zero
     * - if it's signed (negative), then bit 31-12 are all the same as bit 11
     */
    bool is_nbit_imm(int32_t imm, uint8_t n)
    {
        if (n > 32) return false;
    
        bool zero_31_n = true; // is bit n~63 are all zero
        for (int i = n; i < 32; i ++) {
            uint8_t bitx = (imm >> i) & 0x1;
            if (bitx) {
                zero_31_n = false;
                break;
            }
        }
        if (zero_31_n)
            return true;
    
        uint8_t bitn = (imm >> (n-1)) & 0x1;
        for (int i = n; i < 32; i ++) {
            uint8_t bitx = (imm >> i) & 0x1;
            if (bitn != bitx)
                return false;
        }
        return true;
    }


	void AssemblyGeneratorRV64::generateProgram(Program& prog) {
		for (unsigned i = 0; i < RegistersCount; ++i) {
			registerUsage[i] = -1;
		}
		asmCode.str(std::string()); //clear
		for (unsigned i = 0; i < prog.getSize(); ++i) {
			asmCode << "randomx_isn_" << i << ":" << std::endl;
			Instruction& instr = prog(i);
			instr.src %= RegistersCount;
			instr.dst %= RegistersCount;
			generateCode(instr, i);
		}
	}

	void AssemblyGeneratorRV64::generateAsm(SuperscalarProgram& prog) {
#if (0)
		asmCode.str(std::string()); //clear
#ifdef RANDOMX_ALIGN
		asmCode << "ALIGN 16" << std::endl;
#endif
		for (unsigned i = 0; i < prog.getSize(); ++i) {
			Instruction& instr = prog(i);
			switch ((SuperscalarInstructionType)instr.opcode)
			{
			case SuperscalarInstructionType::ISUB_R:
				asmCode << "sub " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
				break;
			case SuperscalarInstructionType::IXOR_R:
				asmCode << "xor " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
				break;
			case SuperscalarInstructionType::IADD_RS:
				asmCode << "lea " << regR[instr.dst] << ", [" << regR[instr.dst] << "+" << regR[instr.src] << "*" << (1 << (instr.getModShift())) << "]" << std::endl;
				break;
			case SuperscalarInstructionType::IMUL_R:
				asmCode << "imul " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
				break;
			case SuperscalarInstructionType::IROR_C:
				asmCode << "ror " << regR[instr.dst] << ", " << instr.getImm32() << std::endl;
				break;
			case SuperscalarInstructionType::IADD_C7:
				asmCode << "add " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
				break;
			case SuperscalarInstructionType::IXOR_C7:
				asmCode << "xor " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
				break;
			case SuperscalarInstructionType::IADD_C8:
				asmCode << "add " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
#ifdef RANDOMX_ALIGN
				asmCode << "nop" << std::endl;
#endif
				break;
			case SuperscalarInstructionType::IXOR_C8:
				asmCode << "xor " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
#ifdef RANDOMX_ALIGN
				asmCode << "nop" << std::endl;
#endif
				break;
			case SuperscalarInstructionType::IADD_C9:
				asmCode << "add " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
#ifdef RANDOMX_ALIGN
				asmCode << "xchg ax, ax ;nop" << std::endl;
#endif
				break;
			case SuperscalarInstructionType::IXOR_C9:
				asmCode << "xor " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
#ifdef RANDOMX_ALIGN
				asmCode << "xchg ax, ax ;nop" << std::endl;
#endif
				break;
			case SuperscalarInstructionType::IMULH_R:
				asmCode << "mov rax, " << regR[instr.dst] << std::endl;
				asmCode << "mul " << regR[instr.src] << std::endl;
				asmCode << "mov " << regR[instr.dst] << ", rdx" << std::endl;
				break;
			case SuperscalarInstructionType::ISMULH_R:
				asmCode << "mov rax, " << regR[instr.dst] << std::endl;
				asmCode << "imul " << regR[instr.src] << std::endl;
				asmCode << "mov " << regR[instr.dst] << ", rdx" << std::endl;
				break;
			case SuperscalarInstructionType::IMUL_RCP:
				asmCode << "mov rax, " << (int64_t)randomx_reciprocal(instr.getImm32()) << std::endl;
				asmCode << "imul " << regR[instr.dst] << ", rax" << std::endl;
				break;
			default:
				UNREACHABLE;
			}
		}
#endif
	}

	void AssemblyGeneratorRV64::traceint(Instruction& instr) {
		if (trace) {
			//asmCode << "\tpush " << regR[instr.dst] << std::endl;
            asmCode << "\tsub " << regSP << 8 << std::endl;
            asmCode << "\tsd " << regR[instr.dst] << ", 0(" << regSP << ")" << std::endl;
		}
	}

	void AssemblyGeneratorRV64::traceflt(Instruction& instr) {
		if (trace) {
			//asmCode << "\tpush 0" << std::endl;
            asmCode << "\tsub " << regSP << 8 << std::endl;
            asmCode << "\tsd " << regZero << ", 0(" << regSP << ")" << std::endl;
		}
	}

	void AssemblyGeneratorRV64::tracenop(Instruction& instr) {
		if (trace) {
			//asmCode << "\tpush 0" << std::endl;
            asmCode << "\tsub " << regSP << 8 << std::endl;
            asmCode << "\tsd " << regZero << ", 0(" << regSP << ")" << std::endl;
		}
	}

	void AssemblyGeneratorRV64::generateCode(Instruction& instr, int i) {
		asmCode << "\t; " << instr;
		auto generator = engine[instr.opcode];
		(this->*generator)(instr, i);
	}

	void AssemblyGeneratorRV64::genAddressReg(Instruction& instr, const char* reg = "eax") {
#if (0)
		asmCode << "\tlea " << reg << ", [" << regR32[instr.src] << std::showpos << (int32_t)instr.getImm32() << std::noshowpos << "]" << std::endl;
		asmCode << "\tand " << reg << ", " << ((instr.getModMem()) ? ScratchpadL1Mask : ScratchpadL2Mask) << std::endl;
#endif
	}

	void AssemblyGeneratorRV64::genAddressRegDst(Instruction& instr, int maskAlign = 8) {
#if (0)
		asmCode << "\tlea eax, [" << regR32[instr.dst] << std::showpos << (int32_t)instr.getImm32() << std::noshowpos << "]" << std::endl;
		int mask;
		if (instr.getModCond() < StoreL3Condition) {
			mask = instr.getModMem() ? ScratchpadL1Mask : ScratchpadL2Mask;
		}
		else {
			mask = ScratchpadL3Mask;
		}
		asmCode << "\tand eax" << ", " << (mask & (-maskAlign)) << std::endl;
#endif
	}

	int32_t AssemblyGeneratorRV64::genAddressImm(Instruction& instr) {
		return (int32_t)instr.getImm32() & ScratchpadL3Mask;
	}

	void AssemblyGeneratorRV64::h_IADD_RS(Instruction& instr, int i) {
#if (0)	
		registerUsage[instr.dst] = i;
        asmCode << "\tslli " << regTmp1 << ", " << regR[instr.src] << ", " << instr.getModShift() << std::endl;
        asmCode << "\tadd " << regR[instr.dst] << ", " << regR[instr.dst] << ", " << regTmp1 << std::endl;
		if(instr.dst == RegisterNeedsDisplacement) {
        }
		traceint(instr);
#endif        
	}

	void AssemblyGeneratorRV64::h_IADD_M(Instruction& instr, int i) {
#if (0)	
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr);
			asmCode << "\tadd " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		}
		else {
			asmCode << "\tadd " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		traceint(instr);
#endif        
	}

	void AssemblyGeneratorRV64::h_ISUB_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			asmCode << "\tsub " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
		}
		else {
			asmCode << "\tsub " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
		}
		traceint(instr);
#endif
    }

	void AssemblyGeneratorRV64::h_ISUB_M(Instruction& instr, int i) {
#if (0)
        registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr);
			asmCode << "\tsub " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		}
		else {
			asmCode << "\tsub " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		traceint(instr);
#endif        
	}

	void AssemblyGeneratorRV64::h_IMUL_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			asmCode << "\timul " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
		}
		else {
			asmCode << "\timul " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IMUL_M(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr);
			asmCode << "\timul " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		}
		else {
			asmCode << "\timul " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IMULH_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
		asmCode << "\tmul " << regR[instr.src] << std::endl;
		asmCode << "\tmov " << regR[instr.dst] << ", rdx" << std::endl;
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IMULH_M(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr, "ecx");
			asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
			asmCode << "\tmul qword ptr [" << regScratchpadAddr << "+rcx]" << std::endl;
		}
		else {
			asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
			asmCode << "\tmul qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		asmCode << "\tmov " << regR[instr.dst] << ", rdx" << std::endl;
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_ISMULH_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
		asmCode << "\timul " << regR[instr.src] << std::endl;
		asmCode << "\tmov " << regR[instr.dst] << ", rdx" << std::endl;
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_ISMULH_M(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr, "ecx");
			asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
			asmCode << "\timul qword ptr [" << regScratchpadAddr << "+rcx]" << std::endl;
		}
		else {
			asmCode << "\tmov rax, " << regR[instr.dst] << std::endl;
			asmCode << "\timul qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		asmCode << "\tmov " << regR[instr.dst] << ", rdx" << std::endl;
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_INEG_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		asmCode << "\tneg " << regR[instr.dst] << std::endl;
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IXOR_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			asmCode << "\txor " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
		}
		else {
			asmCode << "\txor " << regR[instr.dst] << ", " << (int32_t)instr.getImm32() << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IXOR_M(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			genAddressReg(instr);
			asmCode << "\txor " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		}
		else {
			asmCode << "\txor " << regR[instr.dst] << ", qword ptr [" << regScratchpadAddr << "+" << genAddressImm(instr) << "]" << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IROR_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			asmCode << "\tmov ecx, " << regR32[instr.src] << std::endl;
			asmCode << "\tror " << regR[instr.dst] << ", cl" << std::endl;
		}
		else {
			asmCode << "\tror " << regR[instr.dst] << ", " << (instr.getImm32() & 63) << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IROL_R(Instruction& instr, int i) {
#if (0)
		registerUsage[instr.dst] = i;
		if (instr.src != instr.dst) {
			asmCode << "\tmov ecx, " << regR32[instr.src] << std::endl;
			asmCode << "\trol " << regR[instr.dst] << ", cl" << std::endl;
		}
		else {
			asmCode << "\trol " << regR[instr.dst] << ", " << (instr.getImm32() & 63) << std::endl;
		}
		traceint(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_IMUL_RCP(Instruction& instr, int i) {
#if (0)
		uint64_t divisor = instr.getImm32();
		if (!isZeroOrPowerOf2(divisor)) {
			registerUsage[instr.dst] = i;
			asmCode << "\tmov rax, " << randomx_reciprocal(divisor) << std::endl;
			asmCode << "\timul " << regR[instr.dst] << ", rax" << std::endl;
			traceint(instr);
		}
		else {
			tracenop(instr);
		}
#endif
	}

	void AssemblyGeneratorRV64::h_ISWAP_R(Instruction& instr, int i) {
#if (0)
		if (instr.src != instr.dst) {
			registerUsage[instr.dst] = i;
			registerUsage[instr.src] = i;
			asmCode << "\txchg " << regR[instr.dst] << ", " << regR[instr.src] << std::endl;
			traceint(instr);
		}
		else {
			tracenop(instr);
		}
#endif
	}

	void AssemblyGeneratorRV64::h_FSWAP_R(Instruction& instr, int i) {
#if (0)
		asmCode << "\tshufpd " << regFE[instr.dst] << ", " << regFE[instr.dst] << ", 1" << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FADD_R(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		instr.src %= RegisterCountFlt;
		asmCode << "\taddpd " << regF[instr.dst] << ", " << regA[instr.src] << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FADD_M(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		genAddressReg(instr);
		asmCode << "\tcvtdq2pd " << tempRegx << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		asmCode << "\taddpd " << regF[instr.dst] << ", " << tempRegx << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FSUB_R(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		instr.src %= RegisterCountFlt;
		asmCode << "\tsubpd " << regF[instr.dst] << ", " << regA[instr.src] << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FSUB_M(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		genAddressReg(instr);
		asmCode << "\tcvtdq2pd " << tempRegx << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		asmCode << "\tsubpd " << regF[instr.dst] << ", " << tempRegx << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FSCAL_R(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		asmCode << "\txorps " << regF[instr.dst] << ", " << scaleMaskReg << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FMUL_R(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		instr.src %= RegisterCountFlt;
		asmCode << "\tmulpd " << regE[instr.dst] << ", " << regA[instr.src] << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FDIV_M(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		genAddressReg(instr);
		asmCode << "\tcvtdq2pd " << tempRegx << ", qword ptr [" << regScratchpadAddr << "+rax]" << std::endl;
		asmCode << "\tandps " << tempRegx << ", " << mantissaMaskReg << std::endl;
		asmCode << "\torps " << tempRegx << ", " << exponentMaskReg << std::endl;
		asmCode << "\tdivpd " << regE[instr.dst] << ", " << tempRegx << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_FSQRT_R(Instruction& instr, int i) {
#if (0)
		instr.dst %= RegisterCountFlt;
		asmCode << "\tsqrtpd " << regE[instr.dst] << ", " << regE[instr.dst] << std::endl;
		traceflt(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_CFROUND(Instruction& instr, int i) {
#if (0)
		asmCode << "\tmov rax, " << regR[instr.src] << std::endl;
		int rotate = (13 - (instr.getImm32() & 63)) & 63;
		if (rotate != 0)
			asmCode << "\trol rax, " << rotate << std::endl;
		asmCode << "\tand eax, 24576" << std::endl;
		asmCode << "\tor eax, 40896" << std::endl;
		asmCode << "\tpush rax" << std::endl;
		asmCode << "\tldmxcsr dword ptr [rsp]" << std::endl;
		asmCode << "\tpop rax" << std::endl;
		tracenop(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_CBRANCH(Instruction& instr, int i) {
#if (0)
		int reg = instr.dst;
		int target = registerUsage[reg] + 1;
		int shift = instr.getModCond() + ConditionOffset;
		int32_t imm = instr.getImm32() | (1L << shift);
		if (ConditionOffset > 0 || shift > 0)
			imm &= ~(1L << (shift - 1));
		asmCode << "\tadd " << regR[reg] << ", " << imm << std::endl;
		asmCode << "\ttest " << regR[reg] << ", " << (ConditionMask << shift) << std::endl;
		asmCode << "\tjz randomx_isn_" << target << std::endl;
		//mark all registers as used
		for (unsigned j = 0; j < RegistersCount; ++j) {
			registerUsage[j] = i;
		}
#endif
	}

	void AssemblyGeneratorRV64::h_ISTORE(Instruction& instr, int i) {
#if (0)
		genAddressRegDst(instr);
		asmCode << "\tmov qword ptr [" << regScratchpadAddr << "+rax], " << regR[instr.src] << std::endl;
		tracenop(instr);
#endif
	}

	void AssemblyGeneratorRV64::h_NOP(Instruction& instr, int i) {
#if (0)
		asmCode << "\tnop" << std::endl;
		tracenop(instr);
#endif
	}

#include "instruction_weights.hpp"
#define INST_HANDLE(x) REPN(&AssemblyGeneratorRV64::h_##x, WT(x))

	InstructionGeneratorRV64 AssemblyGeneratorRV64::engine[256] = {
		INST_HANDLE(IADD_RS)
		INST_HANDLE(IADD_M)
		INST_HANDLE(ISUB_R)
		INST_HANDLE(ISUB_M)
		INST_HANDLE(IMUL_R)
		INST_HANDLE(IMUL_M)
		INST_HANDLE(IMULH_R)
		INST_HANDLE(IMULH_M)
		INST_HANDLE(ISMULH_R)
		INST_HANDLE(ISMULH_M)
		INST_HANDLE(IMUL_RCP)
		INST_HANDLE(INEG_R)
		INST_HANDLE(IXOR_R)
		INST_HANDLE(IXOR_M)
		INST_HANDLE(IROR_R)
		INST_HANDLE(IROL_R)
		INST_HANDLE(ISWAP_R)
		INST_HANDLE(FSWAP_R)
		INST_HANDLE(FADD_R)
		INST_HANDLE(FADD_M)
		INST_HANDLE(FSUB_R)
		INST_HANDLE(FSUB_M)
		INST_HANDLE(FSCAL_R)
		INST_HANDLE(FMUL_R)
		INST_HANDLE(FDIV_M)
		INST_HANDLE(FSQRT_R)
		INST_HANDLE(CBRANCH)
		INST_HANDLE(CFROUND)
		INST_HANDLE(ISTORE)
		INST_HANDLE(NOP)
	};
}
