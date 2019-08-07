#include "../randomx.h"
#include <stdio.h>
#include <string.h>

const char myKey[] = "RandomX example key";
const char myInput[] = "RandomX example input";
const char myHashHex[] = "512eeb69bb764df47804d32e4a0911d66005924bfb64a7ef57a9364162b741c1";

int main() {
	char hash[RANDOMX_HASH_SIZE];
    char hashhex[RANDOMX_HASH_SIZE*2 + 1] = {0};

	randomx_cache *myCache = randomx_alloc_cache(RANDOMX_FLAG_DEFAULT);
	randomx_init_cache(myCache, myKey, sizeof(myKey));
	randomx_vm *myMachine = randomx_create_vm(RANDOMX_FLAG_DEFAULT, myCache, NULL);

	randomx_calculate_hash(myMachine, myInput, sizeof(myInput), hash);

	randomx_destroy_vm(myMachine);
	randomx_release_cache(myCache);

	for (unsigned i = 0; i < RANDOMX_HASH_SIZE; ++i) {
		//printf("%02x", hash[i] & 0xff);
        sprintf(hashhex + i * 2, "%02x", hash[i] & 0xff);
    }
	printf("%s\n", hashhex);

    if (strncmp(hashhex, myHashHex, RANDOMX_HASH_SIZE * 2)) {
        printf("hash check failed!\n");
    } else {
        printf("hash check ok!\n");
    }

	return 0;
}
