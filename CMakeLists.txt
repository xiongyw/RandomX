# Copyright (c) 2019, The Monero Project
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, are
# permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list
#    of conditions and the following disclaimer in the documentation and/or other
#    materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be
#    used to endorse or promote products derived from this software without specific
#    prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


cmake_minimum_required(VERSION 2.8.7)



set (randomx_sources
src/aes_hash.cpp
src/argon2_ref.c
src/bytecode_machine.cpp
src/dataset.cpp
src/soft_aes.cpp
src/virtual_memory.cpp
src/vm_interpreted.cpp
src/allocator.cpp
src/assembly_generator_x86.cpp
src/assembly_generator_rv64.cpp
src/instruction.cpp
src/randomx.cpp
src/superscalar.cpp
src/vm_compiled.cpp
src/vm_interpreted_light.cpp
src/argon2_core.c
src/blake2_generator.cpp
src/instructions_portable.cpp
src/reciprocal.c
src/virtual_machine.cpp
src/vm_compiled_light.cpp
src/blake2/blake2b.c
src/rv64_insn_sim.c)


if(NOT ARCH_ID)
  # allow cross compiling
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "")
    set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})
  endif()
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_ID)
endif()

if(NOT ARM_ID)
  set(ARM_ID "${ARCH_ID}")
endif()

if(NOT ARCH)
  set(ARCH "default")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Setting default build type: ${CMAKE_BUILD_TYPE}")
endif()

include(CheckCXXCompilerFlag)
include(CheckCCompilerFlag)

function(add_flag flag)
  string(REPLACE "-" "_" supported_cxx ${flag}_cxx)
  check_cxx_compiler_flag(${flag} ${supported_cxx})
  if(${${supported_cxx}})
    message(STATUS "Setting CXX flag ${flag}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}" PARENT_SCOPE)
  endif()
  string(REPLACE "-" "_" supported_c ${flag}_c)
  check_c_compiler_flag(${flag} ${supported_c})
  if(${${supported_c}})
    message(STATUS "Setting C flag ${flag}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flag}" PARENT_SCOPE)
  endif()
endfunction()

# x86-64
if (ARCH_ID STREQUAL "x86_64" OR ARCH_ID STREQUAL "x86-64" OR ARCH_ID STREQUAL "amd64")

  if(ARCH STREQUAL "native")
    add_flag("-march=native")
    list(APPEND randomx_sources
      src/jit_compiler_x86_static.S
      src/jit_compiler_x86.cpp)
    # cheat because cmake and ccache hate each other
    set_property(SOURCE src/jit_compiler_x86_static.S PROPERTY LANGUAGE C)
  elseif(ARCH STREQUAL "rv64")
    message("ARCH=rv64: cross-compiling on x86-64...")

    #set(CMAKE_C_COMPILER "/opt/riscv64/bin/riscv64-unknown-elf-gcc")
    #set(CMAKE_CXX_COMPILER "/opt/riscv64/bin/riscv64-unknown-elf-g++")
    set(CMAKE_C_COMPILER "/opt/riscv64/bin/riscv64-unknown-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "/opt/riscv64/bin/riscv64-unknown-linux-gnu-g++")
    set(CMAKE_C_FLAGS  "${CMAKE_CXX_FLAGS} -march=rv64imfd -mabi=lp64d -static")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=rv64imfd -mabi=lp64d -static")

    list(APPEND randomx_sources
      src/jit_compiler_rv64_static.S
      src/jit_compiler_rv64.cpp)
    # cheat because cmake and ccache hate each other
    set_property(SOURCE src/jit_compiler_rv64_static.S PROPERTY LANGUAGE C)
  else()
    # default build has hardware AES enabled (software AES can be selected at runtime)
    add_flag("-maes")
    # use fallback methods in `instrin_portal.h`
    add_flag("-U__SSE2__")
    list(APPEND randomx_sources
      src/jit_compiler_x86_static.S
      src/jit_compiler_x86.cpp)
    # cheat because cmake and ccache hate each other
    set_property(SOURCE src/jit_compiler_x86_static.S PROPERTY LANGUAGE C)
  endif()
endif()

# PowerPC
if (ARCH_ID STREQUAL "ppc64" OR ARCH_ID STREQUAL "ppc64le")
  if(ARCH STREQUAL "native")
    add_flag("-mcpu=native")
  endif()
  # PowerPC AES requires ALTIVEC (POWER7+), so it cannot be enabled in the default build
endif()

# ARMv8
if (ARM_ID STREQUAL "aarch64" OR ARM_ID STREQUAL "arm64" OR ARM_ID STREQUAL "armv8-a")
  if(ARCH STREQUAL "native")
    add_flag("-march=native")
  else()
    # default build has hardware AES enabled (software AES can be selected at runtime)
    add_flag("-march=armv8-a+crypto")
  endif()
endif()

set(RANDOMX_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE STRING "RandomX Include path")

add_library(randomx ${randomx_sources})
set_property(TARGET randomx PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET randomx PROPERTY CXX_STANDARD 11)
set_property(TARGET randomx PROPERTY CXX_STANDARD_REQUIRED ON)

# interpreated c-example
add_executable(api1 src/tests/api-example1.c)
target_link_libraries(api1 PRIVATE randomx)
set_property(TARGET api1 PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET api1 PROPERTY CXX_STANDARD 11)  

add_executable(randomx-codegen src/tests/code-generator.cpp)
target_link_libraries(randomx-codegen PRIVATE randomx)
set_property(TARGET randomx-codegen PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET randomx-codegen PROPERTY CXX_STANDARD 11)


if (ARCH STREQUAL "rv64")
  add_executable(testrv64 src/tests/testrv64.cpp)
  target_link_libraries(testrv64 PRIVATE randomx)
  set_property(TARGET testrv64 PROPERTY POSITION_INDEPENDENT_CODE ON)
  set_property(TARGET testrv64 PROPERTY CXX_STANDARD 11)  
else()
  add_executable(randomx-tests src/tests/tests.cpp)
  target_link_libraries(randomx-tests PRIVATE randomx)
  set_property(TARGET randomx-tests PROPERTY POSITION_INDEPENDENT_CODE ON)
  set_property(TARGET randomx-tests PROPERTY CXX_STANDARD 11)


	if (NOT Threads_FOUND AND UNIX AND NOT APPLE)
	  set(THREADS_PREFER_PTHREAD_FLAG ON)
	  find_package(Threads)
	endif()

  add_executable(randomx-benchmark
    src/tests/benchmark.cpp
    src/tests/affinity.cpp)
  target_link_libraries(randomx-benchmark
    PRIVATE randomx
    PRIVATE ${CMAKE_THREAD_LIBS_INIT})

  include(CheckCXXSourceCompiles)
  check_cxx_source_compiles("
  #include <cstdint>
  #include <atomic>
  int main() {
    std::atomic<uint64_t> a;
    a.is_lock_free();
  }" HAVE_CXX_ATOMICS)
  
  if(NOT HAVE_CXX_ATOMICS)
    target_link_libraries(randomx-benchmark
      PRIVATE "atomic")
  endif()
  set_property(TARGET randomx-benchmark PROPERTY POSITION_INDEPENDENT_CODE ON)
  set_property(TARGET randomx-benchmark PROPERTY CXX_STANDARD 11)
endif()
