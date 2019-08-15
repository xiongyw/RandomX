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

#pragma once

#include "common.hpp"
#include <sstream>

#include "bytecode_machine.hpp"

namespace randomx {

	class Program;
	class AssemblyGeneratorRV64;
	class Instruction;
    class BytecodeMachine;

	class AssemblyGeneratorRV64 {
	public:
		void generateProgram(Program& prog);

	private:
		void traceint(InstructionByteCode&);
		void traceflt(InstructionByteCode&);
		void tracenop(InstructionByteCode&);

		void generateCode(InstructionByteCode& ibc, NativeRegisterFile*, Program&, int);
        
        uint8_t getIRegIdx(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&, bool is_src);
        uint8_t getFRegIdx(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&, bool is_src, bool is_low);
        
        void getIOffset(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&, bool is_load);
        void getFOffset(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&);

        void load64(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&);
        void load32_x2(InstructionByteCode& ibc, NativeRegisterFile* nreg, Instruction&);
        void load_ibc_imm(InstructionByteCode& ibc);
        
		void h_IADD_RS(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IADD_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISUB_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISUB_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IMUL_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IMUL_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IMULH_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IMULH_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISMULH_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISMULH_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IMUL_RCP(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_INEG_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IXOR_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IXOR_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IROR_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_IROL_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISWAP_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FSWAP_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FADD_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FADD_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FSUB_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FSUB_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FSCAL_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FMUL_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FDIV_M(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_FSQRT_R(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_CBRANCH(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_CFROUND(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_ISTORE(InstructionByteCode&, NativeRegisterFile*, Program&, int);
		void h_NOP(InstructionByteCode&, NativeRegisterFile*, Program&, int);

	};
}
