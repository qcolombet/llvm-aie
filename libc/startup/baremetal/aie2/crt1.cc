//
// This file is licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// (c) Copyright 2024 Advanced Micro Devices, Inc. or its affiliates

extern "C" {
extern int main(int, char **);
void _Exit(int val) {
  (void)val;
  done();
}
void _main_init() { _Exit(main(0, nullptr)); }
}
