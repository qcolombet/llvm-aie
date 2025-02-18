; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
;
; This file is licensed under the Apache License v2.0 with LLVM Exceptions.
; See https://llvm.org/LICENSE.txt for license information.
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
;
; (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
; RUN: llc -O2 -mtriple=aie2 %s -o - | FileCheck %s
%struct.Data = type { i32, i32, i32 }

@d = dso_local local_unnamed_addr global %struct.Data zeroinitializer, align 4

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(read, argmem: none, inaccessiblemem: none)
define dso_local noundef i32 @_Z4getAv() {
; CHECK-LABEL: _Z4getAv:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    movxm p0, #(d+4)
; CHECK-NEXT:    lda r0, [p0], #4
; CHECK-NEXT:    lda r1, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    add r0, r1, r0 // Delay Slot 1
entry:
  %0 = load i32, ptr getelementptr inbounds (%struct.Data, ptr @d, i20 0, i32 1), align 4
  %1 = load i32, ptr getelementptr inbounds (%struct.Data, ptr @d, i20 0, i32 2), align 4
  %add = add nsw i32 %1, %0
  ret i32 %add
}

%struct.anon = type { i16, i16, i32 }

@X = internal global %struct.anon zeroinitializer, align 4

; Function Attrs: mustprogress noinline nounwind optnone
define dso_local noundef i32 @foo() {
; CHECK-LABEL: foo:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    movxm p0, #(X+4)
; CHECK-NEXT:    lda.s16 r0, [p0, #-2]
; CHECK-NEXT:    lda r1, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    add r0, r0, r1 // Delay Slot 1
entry:
  %0 = load i16, ptr getelementptr inbounds (%struct.anon, ptr @X, i32 0, i32 1), align 2
  %conv = sext i16 %0 to i32
  %1 = load i32, ptr getelementptr inbounds (%struct.anon, ptr @X, i32 0, i32 2), align 4
  %add = add nsw i32 %conv, %1
  ret i32 %add
}

%struct.test = type { i16, i16, i8, i32 }

@Y = dso_local global %struct.test zeroinitializer, align 4

; Function Attrs: mustprogress noinline nounwind optnone
define dso_local noundef i32 @bar() {
; CHECK-LABEL: bar:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; movxm p0, #(Y+4)
; CHECK-NEXT:    lda.s16 r0, [p0, #-2]
; CHECK-NEXT:    lda.s16 r1, [p0, #-4]
; CHECK-NEXT:    lda.s8 r2, [p0], #4
; CHECK-NEXT:    lda r3, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    add r0, r0, r1 // Delay Slot 3
; CHECK-NEXT:    add r0, r0, r2 // Delay Slot 2
; CHECK-NEXT:    add r0, r0, r3 // Delay Slot 1
entry:
  %0 = load i16, ptr getelementptr inbounds (%struct.test, ptr @Y, i32 0, i32 1), align 2
  %conv = sext i16 %0 to i32
  %1 = load i16, ptr @Y, align 4
  %conv1 = sext i16 %1 to i32
  %add = add nsw i32 %conv, %conv1
  %2 = load i8, ptr getelementptr inbounds (%struct.test, ptr @Y, i32 0, i32 2), align 4
  %conv2 = sext i8 %2 to i32
  %add3 = add nsw i32 %add, %conv2
  %3 = load i32, ptr getelementptr inbounds (%struct.test, ptr @Y, i32 0, i32 3), align 4
  %add4 = add nsw i32 %add3, %3
  ret i32 %add4
}
