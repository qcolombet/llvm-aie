; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
;
; This file is licensed under the Apache License v2.0 with LLVM Exceptions.
; See https://llvm.org/LICENSE.txt for license information.
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
;
; (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
;RUN: llc -O2 -mtriple=aie2 %s -o - | FileCheck %s

define dso_local noundef <8 x i64> @test_sub_acc(<8 x i64> noundef %acc1, <8 x i64> noundef %acc2) local_unnamed_addr #0 {
; CHECK-LABEL: test_sub_acc:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    mova r0, #0; nopb ; nopxm ; nops
; CHECK-NEXT:    vsub.f bml0, bml1, bml2, r0
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <8 x i64> @llvm.aie2.sub.accfloat(<8 x i64> %acc1, <8 x i64> %acc2, i32 0)
  ret <8 x i64> %0
}

declare <8 x i64> @llvm.aie2.sub.accfloat(<8 x i64>, <8 x i64>, i32) #1
