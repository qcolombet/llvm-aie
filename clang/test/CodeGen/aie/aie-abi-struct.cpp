//===- aie-abi-struct.cpp ---------------------------------------*- C++ -*-===//
//
// This file is licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
//
//===----------------------------------------------------------------------===//
// RUN: %clang_cc1 -triple aie -emit-llvm %s -o - | FileCheck %s
// RUN: %clang_cc1 -triple aie2 -emit-llvm %s -o - | FileCheck %s

#include <stdint.h>

extern "C" {

// Empty structs get eliminated as return types, but not arguments
struct S_EMPTY {
  int x[0];
};

// CHECK-LABEL: define {{[^@]*}}%struct.S_EMPTY @ret_S_EMPTY
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S_EMPTY ret_S_EMPTY(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S_EMPTY
// CHECK-SAME: ([[STRUCT_S_EMPTY:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S_EMPTY(struct S_EMPTY) {}

struct S2I {
  int x;
  int y;
};

// CHECK-LABEL: define {{[^@]*}}%struct.S2I @ret_S2I
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S2I ret_S2I(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S2I
// CHECK-SAME: ([[STRUCT_S2I:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S2I(struct S2I) {}

// Change standard alignment.
struct S2I_A8 {
  int x __attribute__((aligned(8)));
  int y;
};

// CHECK-LABEL: define {{[^@]*}}%struct.S2I_A8 @ret_S2I_A8
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S2I_A8 ret_S2I_A8(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S2I_A8
// CHECK-SAME: ([[STRUCT_S2I_A8:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S2I_A8(struct S2I_A8) {}

struct S3I {
  int x1, x2, x3;
};

// CHECK-LABEL: define {{[^@]*}}%struct.S3I @ret_S3I
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S3I ret_S3I(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S3I
// CHECK-SAME: ([[STRUCT_S3I:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S3I(struct S3I) {}

struct S4I {
  int x1, x2, x3, x4;
};

// CHECK-LABEL: define {{[^@]*}}%struct.S4I @ret_S4I
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S4I ret_S4I(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S4I
// CHECK-SAME: ([[STRUCT_S4I:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S4I(struct S4I) {}

struct S2LI {
  int64_t x1, x2;
};

// CHECK-LABEL: define {{[^@]*}}%struct.S2LI @ret_S2LI
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S2LI ret_S2LI(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S2LI
// CHECK-SAME: ([[STRUCT_S2LI:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S2LI(struct S2LI) {}

struct S8xSI {
  int16_t x[8];
};

// CHECK-LABEL: define {{[^@]*}}%struct.S8xSI @ret_S8xSI
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S8xSI ret_S8xSI(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S8xSI
// CHECK-SAME: ([[STRUCT_S8XSI:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S8xSI(struct S8xSI) {}

struct S16xC {
  char x[16];
};

// CHECK-LABEL: define {{[^@]*}}%struct.S16xC @ret_S16xC
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S16xC ret_S16xC(void) {
  return {};
}
// CHECK-LABEL: define {{[^@]*}}void @pass_S16xC
// CHECK-SAME: ([[STRUCT_S16XC:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S16xC(struct S16xC) {}

struct S4xI {
  int x[4];
};

// CHECK-LABEL: define {{[^@]*}}%struct.S4xI @ret_S4xI
// CHECK-SAME: () #[[ATTR0:[0-9]+]] {
struct S4xI ret_S4xI(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S4xI
// CHECK-SAME: ([[STRUCT_S4XI:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S4xI(struct S4xI) {}

// Using a bigger alignment stops direct returns.
struct S4I_A8 {
  int x1, x2, x3, x4 __attribute__((aligned(8)));
};

// CHECK-LABEL: define {{[^@]*}}void @ret_S4I_A8
// CHECK-SAME: (ptr dead_on_unwind noalias writable sret([[STRUCT_S4I_A8:%.*]]) align 8 [[AGG_RESULT:%.*]]) #[[ATTR0:[0-9]+]] {
struct S4I_A8 ret_S4I_A8(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S4I_A8
// CHECK-SAME: ([[STRUCT_S4I_A8:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S4I_A8(struct S4I_A8) {}

struct S5I {
  int x1, x2, x3, x4, x5;
};

// CHECK-LABEL: define {{[^@]*}}void @ret_S5I
// CHECK-SAME: (ptr dead_on_unwind noalias writable sret([[STRUCT_S5I:%.*]]) align 4 [[AGG_RESULT:%.*]]) #[[ATTR0:[0-9]+]] {
struct S5I ret_S5I(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S5I
// CHECK-SAME: ([[STRUCT_S5I:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S5I(struct S5I) {}

struct S5xI {
  int x[5];
};

// CHECK-LABEL: define {{[^@]*}}void @ret_S5xI
// CHECK-SAME: (ptr dead_on_unwind noalias writable sret([[STRUCT_S5XI:%.*]]) align 4 [[AGG_RESULT:%.*]]) #[[ATTR0:[0-9]+]] {
struct S5xI ret_S5xI(void) { return {}; }
// CHECK-LABEL: define {{[^@]*}}void @pass_S5xI
// CHECK-SAME: ([[STRUCT_S5XI:%.*]] [[DOTCOERCE:%.*]]) #[[ATTR0:[0-9]+]] {
void pass_S5xI(struct S5xI) {}
}
