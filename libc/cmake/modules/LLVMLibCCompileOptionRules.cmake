#
# Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
# See https://llvm.org/LICENSE.txt for license information.
# SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#
# Modifications (c) Copyright 2024 Advanced Micro Devices, Inc. or its affiliates

function(_get_compile_options_from_flags output_var)
  set(compile_options "")

  if(LIBC_TARGET_ARCHITECTURE_IS_RISCV64 OR(LIBC_CPU_FEATURES MATCHES "FMA"))
    check_flag(ADD_FMA_FLAG ${FMA_OPT_FLAG} ${ARGN})
  endif()
  check_flag(ADD_SSE4_2_FLAG ${ROUND_OPT_FLAG} ${ARGN})
  check_flag(ADD_EXPLICIT_SIMD_OPT_FLAG ${EXPLICIT_SIMD_OPT_FLAG} ${ARGN})

  if(LLVM_COMPILER_IS_GCC_COMPATIBLE)
    if(ADD_FMA_FLAG)
      if(LIBC_TARGET_ARCHITECTURE_IS_X86)
        list(APPEND compile_options "-mavx2")
        list(APPEND compile_options "-mfma")
      elseif(LIBC_TARGET_ARCHITECTURE_IS_RISCV64)
        list(APPEND compile_options "-D__LIBC_RISCV_USE_FMA")
      endif()
    endif()
    if(ADD_SSE4_2_FLAG)
      list(APPEND compile_options "-msse4.2")
    endif()
    if(ADD_EXPLICIT_SIMD_OPT_FLAG)
      list(APPEND compile_options "-D__LIBC_EXPLICIT_SIMD_OPT")
    endif()
  elseif(MSVC)
    if(ADD_FMA_FLAG)
      list(APPEND compile_options "/arch:AVX2")
    endif()
    if(ADD_EXPLICIT_SIMD_OPT_FLAG)
      list(APPEND compile_options "/D__LIBC_EXPLICIT_SIMD_OPT")
    endif()
  endif()

  set(${output_var} ${compile_options} PARENT_SCOPE)
endfunction(_get_compile_options_from_flags)

function(_get_common_compile_options output_var flags)
  _get_compile_options_from_flags(compile_flags ${flags})

  set(compile_options ${LIBC_COMPILE_OPTIONS_DEFAULT} ${compile_flags})

  if(LLVM_COMPILER_IS_GCC_COMPATIBLE)
    list(APPEND compile_options "-fpie")

    if(LLVM_LIBC_FULL_BUILD)
      list(APPEND compile_options "-DLIBC_FULL_BUILD")
      # Only add -ffreestanding flag in full build mode.
      list(APPEND compile_options "-ffreestanding")
    endif()

    if(LIBC_COMPILER_HAS_FIXED_POINT)
      list(APPEND compile_options "-ffixed-point")
    endif()

    list(APPEND compile_options "-fno-builtin")
    list(APPEND compile_options "-fno-exceptions")
    if(NOT LIBC_TARGET_ARCHITECTURE_IS_AIE)
      # AIE currently requires lax vector conversions
      list(APPEND compile_options "-fno-lax-vector-conversions")
    endif()
    list(APPEND compile_options "-fno-unwind-tables")
    list(APPEND compile_options "-fno-asynchronous-unwind-tables")
    list(APPEND compile_options "-fno-rtti")
    if (LIBC_CC_SUPPORTS_PATTERN_INIT)
      list(APPEND compile_options "-ftrivial-auto-var-init=pattern")
    endif()
    if (LIBC_CONF_KEEP_FRAME_POINTER)
      list(APPEND compile_options "-fno-omit-frame-pointer")
      if (LIBC_TARGET_ARCHITECTURE_IS_X86)
        list(APPEND compile_options "-mno-omit-leaf-frame-pointer")
      endif()
    endif()
    if (LIBC_CONF_ENABLE_STACK_PROTECTOR)
      list(APPEND compile_options "-fstack-protector-strong")
    endif()
    list(APPEND compile_options "-Wall")
    list(APPEND compile_options "-Wextra")
    # -DLIBC_WNO_ERROR=ON if you can't build cleanly with -Werror.
    if(NOT LIBC_WNO_ERROR)
      list(APPEND compile_options "-Werror")
    endif()
    list(APPEND compile_options "-Wconversion")
    list(APPEND compile_options "-Wno-sign-conversion")
    list(APPEND compile_options "-Wimplicit-fallthrough")
    list(APPEND compile_options "-Wwrite-strings")
    list(APPEND compile_options "-Wextra-semi")
    if(NOT CMAKE_COMPILER_IS_GNUCXX)
      list(APPEND compile_options "-Wnewline-eof")
      list(APPEND compile_options "-Wnonportable-system-include-path")
      list(APPEND compile_options "-Wstrict-prototypes")
      list(APPEND compile_options "-Wthread-safety")
      list(APPEND compile_options "-Wglobal-constructors")
    endif()
  elseif(MSVC)
    list(APPEND compile_options "/EHs-c-")
    list(APPEND compile_options "/GR-")
  endif()
  if (LIBC_TARGET_OS_IS_GPU)
    list(APPEND compile_options "-nogpulib")
    list(APPEND compile_options "-fvisibility=hidden")
    list(APPEND compile_options "-fconvergent-functions")
    list(APPEND compile_options "-flto")
    list(APPEND compile_options "-Wno-multi-gpu")

    if(LIBC_TARGET_ARCHITECTURE_IS_NVPTX)
      list(APPEND compile_options "-Wno-unknown-cuda-version")
      list(APPEND compile_options "SHELL:-mllvm -nvptx-emit-init-fini-kernel=false")
      list(APPEND compile_options "--cuda-feature=+ptx63")
      if(LIBC_CUDA_ROOT)
        list(APPEND compile_options "--cuda-path=${LIBC_CUDA_ROOT}")
      endif()
    elseif(LIBC_TARGET_ARCHITECTURE_IS_AMDGPU)
      list(APPEND compile_options "SHELL:-Xclang -mcode-object-version=none")
    endif()

    # Manually disable all standard include paths and include the resource
    # directory to prevent system headers from being included.
    list(APPEND compile_options "-isystem${COMPILER_RESOURCE_DIR}/include")
    list(APPEND compile_options "-nostdinc")
  endif()
  set(${output_var} ${compile_options} PARENT_SCOPE)
endfunction()

function(_get_common_test_compile_options output_var c_test flags)
  _get_compile_options_from_flags(compile_flags ${flags})

  set(compile_options ${LIBC_COMPILE_OPTIONS_DEFAULT} ${compile_flags})

  if(LLVM_COMPILER_IS_GCC_COMPATIBLE)
    list(APPEND compile_options "-fpie")

    if(LLVM_LIBC_FULL_BUILD)
      list(APPEND compile_options "-DLIBC_FULL_BUILD")
      # Only add -ffreestanding flag in full build mode.
      list(APPEND compile_options "-ffreestanding")
      list(APPEND compile_options "-fno-exceptions")
      list(APPEND compile_options "-fno-unwind-tables")
      list(APPEND compile_options "-fno-asynchronous-unwind-tables")
      if(NOT ${c_test})
        list(APPEND compile_options "-fno-rtti")
      endif()
    endif()

    if(LIBC_COMPILER_HAS_FIXED_POINT)
      list(APPEND compile_options "-ffixed-point")
    endif()

    # list(APPEND compile_options "-Wall")
    # list(APPEND compile_options "-Wextra")
    # -DLIBC_WNO_ERROR=ON if you can't build cleanly with -Werror.
    if(NOT LIBC_WNO_ERROR)
      # list(APPEND compile_options "-Werror")
    endif()
    # list(APPEND compile_options "-Wconversion")
    # list(APPEND compile_options "-Wno-sign-conversion")
    # list(APPEND compile_options "-Wimplicit-fallthrough")
    # list(APPEND compile_options "-Wwrite-strings")
    # list(APPEND compile_options "-Wextra-semi")
    # if(NOT CMAKE_COMPILER_IS_GNUCXX)
    #   list(APPEND compile_options "-Wnewline-eof")
    #   list(APPEND compile_options "-Wnonportable-system-include-path")
    #   list(APPEND compile_options "-Wstrict-prototypes")
    #   list(APPEND compile_options "-Wthread-safety")
    #   list(APPEND compile_options "-Wglobal-constructors")
    # endif()
  endif()
  set(${output_var} ${compile_options} PARENT_SCOPE)
endfunction()

function(_get_hermetic_test_compile_options output_var flags)
  _get_compile_options_from_flags(compile_flags ${flags})
  list(APPEND compile_options ${LIBC_COMPILE_OPTIONS_DEFAULT} ${compile_flags}
       ${flags} -fpie -ffreestanding -fno-exceptions -fno-rtti)

  # The GPU build requires overriding the default CMake triple and architecture.
  if(LIBC_TARGET_ARCHITECTURE_IS_AMDGPU)
    list(APPEND compile_options
         -Wno-multi-gpu -nogpulib -mcpu=${LIBC_GPU_TARGET_ARCHITECTURE} -flto
         -mcode-object-version=${LIBC_GPU_CODE_OBJECT_VERSION})
  elseif(LIBC_TARGET_ARCHITECTURE_IS_NVPTX)
    list(APPEND compile_options
         "SHELL:-mllvm -nvptx-emit-init-fini-kernel=false"
         -Wno-multi-gpu --cuda-path=${LIBC_CUDA_ROOT}
         -nogpulib -march=${LIBC_GPU_TARGET_ARCHITECTURE} -fno-use-cxa-atexit)
  endif()

  if(LLVM_LIBC_FULL_BUILD)
    list(APPEND compile_options "-DLIBC_FULL_BUILD")
  endif()
  
  set(${output_var} ${compile_options} PARENT_SCOPE)
endfunction()
