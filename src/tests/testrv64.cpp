#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <iomanip>
#include "utility.hpp"
#include "../bytecode_machine.hpp"
#include "../dataset.hpp"
#include "../blake2/endian.h"
#include "../blake2/blake2.h"
#include "../blake2_generator.hpp"
#include "../superscalar.hpp"
#include "../reciprocal.h"
#include "../intrin_portable.h"
#include "../jit_compiler.hpp"
#include "../aes_hash.hpp"


void calc_hash(randomx_flags flags, 
               bool is_hex,      // is `input[]` the hex representation of blob
               const void* key, int K, 
               const void* input, int H,
               void* output)
{
    randomx_cache* cache = nullptr;
    randomx_dataset* dataset = nullptr;
    randomx_vm* vm = nullptr;

    // 0. fastmode or lightmode: allocate dataset or not
    bool fastmode =  flags & (RANDOMX_FLAG_FULL_MEM);
    
    // 1. init cache/dataset/vm 
	std::cout << "randomx_alloc_cache()..." << std::endl;
    cache = randomx_alloc_cache(flags);
    assert(cache != nullptr);
    
	std::cout << "randomx_init_cache()..." << std::endl;
	randomx_init_cache(cache, key, K);

    if (fastmode) {
        // fixme: allocat 2GB failed on `spike`
    	std::cout << "randomx_alloc_dataset()..." << std::endl;
        dataset = randomx_alloc_dataset(flags);
        assert(dataset != nullptr);

    	std::cout << "randomx_init_dataset()..." << std::endl;
    	randomx_init_dataset(dataset, cache, 0, randomx_dataset_item_count());
    }

	std::cout << "randomx_create_vm()..." << std::endl;
	vm = randomx_create_vm(flags, cache, fastmode? dataset : nullptr);
    assert(vm != nullptr);

    // 2. calculate hash
	std::cout << "randomx_calculate_hash()..." << std::endl;
    if (is_hex) {
        char blob[H / 2];
        hex2bin((char*)input, H, blob);
	    randomx_calculate_hash(vm, blob, H/2, output);
    } else {
	    randomx_calculate_hash(vm, input, H, output);
    }

    // 3. tear down
	std::cout << "randomx_destroy_vm()..." << std::endl;
	randomx_destroy_vm(vm);
    if (fastmode) {
    	std::cout << "randomx_release_dataset()..." << std::endl;
    	randomx_release_dataset(dataset);
    }
	std::cout << "randomx_release_cache()..." << std::endl;
	randomx_release_cache(cache);
}


int main() {
    randomx_flags flags = RANDOMX_FLAG_DEFAULT;  // InterpretedLightVmDefault()
    const char key0[] = "test key 000";
    const char input0[] = "This is a test";
    const char hashhex0[] = "b33f8d10a8655d6f1925e3754adeb0a6da4c2f48a81cd4c220a412f1ef016a15";

    const char key1[] = "test key 001";
    const char input1[] = "0b0b98bea7e805e0010a2126d287a2a0cc833d312cb786385a7c2f9de69d25537f584a9bc9977b00000000666fd8753bf61a8631f12984e3fd44f4014eca629276817b56f32e9b68bd82f416";
    const char hashhex1[] = "f60caf300917760337e8ce51487484e6a33d4aaa15aa79c985efb4ea00390918"; 
    char hash[RANDOMX_HASH_SIZE];
    
    calc_hash(flags, false, key0, strlen(key0), input0, strlen(input0), hash);
    if (!equalsHex(hash, hashhex0)) {
        throw std::runtime_error("hash 0 check failed!\n");
    }

    calc_hash(flags, true, key1, strlen(key1), input1, strlen(input1), &hash);
    if (!equalsHex(hash, hashhex1)) {
        throw std::runtime_error("hash 1 check failed!\n");
    }

    return 0;
}
