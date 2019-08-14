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

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <cfloat>
#include "vm_interpreted.hpp"
#include "dataset.hpp"
#include "intrin_portable.h"
#include "reciprocal.h"

namespace randomx {

	template<class Allocator, bool softAes>
	void InterpretedVm<Allocator, softAes>::setDataset(randomx_dataset* dataset) {
		datasetPtr = dataset;
		mem.memory = dataset->memory;
	}

	template<class Allocator, bool softAes>
	void InterpretedVm<Allocator, softAes>::run(void* seed) {
		VmBase<Allocator, softAes>::generateProgram(seed);
		randomx_vm::initialize();
		execute();
	}

	template<class Allocator, bool softAes>
	void InterpretedVm<Allocator, softAes>::execute() {

		NativeRegisterFile nreg;  // r[] are initialized to 0

        // load a[] from config (entropy)
		for(unsigned i = 0; i < RegisterCountFlt; ++i)
			nreg.a[i] = rx_load_vec_f128(&reg.a[i].lo);

		compileProgram(program, bytecode, nreg);

		uint32_t spAddr0 = mem.mx;
		uint32_t spAddr1 = mem.ma;

        // $4.6.2 Loop execution
		for(unsigned ic = 0; ic < RANDOMX_PROGRAM_ITERATIONS; ++ic) {
            // 1. calculate spAddr0/spAddr1: XOR of registers readReg0 and readReg1 is 
            //    calculated; and spAddr0 is XORed with the low 32 bits of the result,
            //    and spAddr1 with the high 32 bits.
			uint64_t spMix = nreg.r[config.readReg0] ^ nreg.r[config.readReg1];
			spAddr0 ^= spMix;
			spAddr0 &= ScratchpadL3Mask64;
			spAddr1 ^= spMix >> 32;
			spAddr1 &= ScratchpadL3Mask64;

            // 2. update r0~r7: spAddr0 is used to perform a 64-byte aligned read from Scratchpad
            //    level 3. The 64 bytes are XORed with all integer registers in order r0-r7.
			for (unsigned i = 0; i < RegistersCount; ++i)
				nreg.r[i] ^= load64(scratchpad + spAddr0 + 8 * i);

            // 3. init f0~f3 and e0~e3: spAddr1 is used to perform a 64-byte aligned read from
            //    Scratchpad level 3. Each floating point register f0-f3 and e0-e3 is initialized
            //    using an 8-byte value according to the conversion rules from chapters 4.3.1 and 4.3.2.
			for (unsigned i = 0; i < RegisterCountFlt; ++i)
				nreg.f[i] = rx_cvt_packed_int_vec_f128(scratchpad + spAddr1 + 8 * i);

			for (unsigned i = 0; i < RegisterCountFlt; ++i)
				nreg.e[i] = maskRegisterExponentMantissa(config, rx_cvt_packed_int_vec_f128(scratchpad + spAddr1 + 8 * (RegisterCountFlt + i)));

            // 4. The 256 instructions stored in the Program Buffer are executed.
			executeBytecode(bytecode, scratchpad, config);

            // 5. The mx register is XORed with the low 32 bits of registers readReg2 and readReg3
			mem.mx ^= nreg.r[config.readReg2] ^ nreg.r[config.readReg3];

            // 6. A 64-byte Dataset item at address `datasetOffset + mx % RANDOMX_DATASET_BASE_SIZE` is 
            //    prefetched from the Dataset (it will be used during the next iteration).
			mem.mx &= CacheLineAlignMask;
			datasetPrefetch(datasetOffset + mem.mx);

            // 7. A 64-byte Dataset item at address `datasetOffset + ma % RANDOMX_DATASET_BASE_SIZE` is
            //    loaded from the Dataset. The 64 bytes are XORed with all integer registers in order r0-r7.
			datasetRead(datasetOffset + mem.ma, nreg.r);

            // 8. The values of registers mx and ma are swapped.
			std::swap(mem.mx, mem.ma);

            // 9. The values of all integer registers r0-r7 are written to the Scratchpad (L3) at 
            //    address spAddr1 (64-byte aligned).
			for (unsigned i = 0; i < RegistersCount; ++i)
				store64(scratchpad + spAddr1 + 8 * i, nreg.r[i]);

            // 10. Register f[] is XORed with register e[] and the result is stored in register f[].
			for (unsigned i = 0; i < RegisterCountFlt; ++i)
				nreg.f[i] = rx_xor_vec_f128(nreg.f[i], nreg.e[i]);

            // 11. f0-f3 are written to the Scratchpad (L3) at address spAddr0 (64-byte aligned).
			for (unsigned i = 0; i < RegisterCountFlt; ++i)
				rx_store_vec_f128((double*)(scratchpad + spAddr0 + 16 * i), nreg.f[i]);

            // 12. spAddr0 and spAddr1 are both set to zero.
			spAddr0 = 0;
			spAddr1 = 0;
		}

		for (unsigned i = 0; i < RegistersCount; ++i)
			store64(&reg.r[i], nreg.r[i]);

		for (unsigned i = 0; i < RegisterCountFlt; ++i)
			rx_store_vec_f128(&reg.f[i].lo, nreg.f[i]);

		for (unsigned i = 0; i < RegisterCountFlt; ++i)
			rx_store_vec_f128(&reg.e[i].lo, nreg.e[i]);
	}

	template<class Allocator, bool softAes>
	void InterpretedVm<Allocator, softAes>::datasetRead(uint64_t address, int_reg_t(&r)[RegistersCount]) {
		uint64_t* datasetLine = (uint64_t*)(mem.memory + address);
		for (int i = 0; i < RegistersCount; ++i)
			r[i] ^= datasetLine[i];
	}

	template<class Allocator, bool softAes>
	void InterpretedVm<Allocator, softAes>::datasetPrefetch(uint64_t address) {
		rx_prefetch_nta(mem.memory + address);
	}

	template class InterpretedVm<AlignedAllocator<CacheLineSize>, false>;
	template class InterpretedVm<AlignedAllocator<CacheLineSize>, true>;
	template class InterpretedVm<LargePageAllocator, false>;
	template class InterpretedVm<LargePageAllocator, true>;
}