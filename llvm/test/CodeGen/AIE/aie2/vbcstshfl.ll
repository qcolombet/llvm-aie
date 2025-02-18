; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
;
; This file is licensed under the Apache License v2.0 with LLVM Exceptions.
; See https://llvm.org/LICENSE.txt for license information.
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
;
; (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
; RUN: llc -O2 -mtriple=aie2 %s -o - | FileCheck %s


define dso_local noundef <64 x i8> @_Z15test_shuffle_s8ij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z15test_shuffle_s8ij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.8 x0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle8(i32 %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <64 x i8>
  ret <64 x i8> %1
}


define dso_local noundef <32 x i16> @_Z16test_shuffle_s16ij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z16test_shuffle_s16ij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.16 x0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle16(i32 %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <32 x i16>
  ret <32 x i16> %1
}


define dso_local noundef <16 x i32> @_Z16test_shuffle_s32ij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z16test_shuffle_s32ij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.32 x0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle32(i32 %b, i32 %m)
  ret <16 x i32> %0
}


define dso_local noundef <8 x i64> @_Z19test_shuffle_s8_accij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z19test_shuffle_s8_accij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.8 bml0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle8(i32 %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <8 x i64>
  ret <8 x i64> %1
}


define dso_local noundef <8 x i64> @_Z20test_shuffle_s16_accij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z20test_shuffle_s16_accij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.16 bml0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle16(i32 %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <8 x i64>
  ret <8 x i64> %1
}


define dso_local noundef <8 x i64> @_Z20test_shuffle_s32_accij(i32 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z20test_shuffle_s32_accij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.32 bml0, r0, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle32(i32 %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <8 x i64>
  ret <8 x i64> %1
}

define dso_local noundef <16 x i32> @_Z16test_shuffle_s64Dv2_ij(<2 x i32> noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z16test_shuffle_s64Dv2_ij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r0 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.64 x0, r17:r16, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle64(<2 x i32> %b, i32 %m)
  ret <16 x i32> %0
}

define dso_local noundef <8 x i64> @_Z20test_shuffle_s64_accDv2_ij(<2 x i32> noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z20test_shuffle_s64_accDv2_ij:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; ret lr ; nopm ; nopv
; CHECK-NEXT:    nopx // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    mov r29, r0 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.64 bml0, r17:r16, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle64(<2 x i32> %b, i32 %m)
  %1 = bitcast <16 x i32> %0 to <8 x i64>
  ret <8 x i64> %1
}

define dso_local noundef <16 x i32> @_Z16test_shuffle_s64xj(i64 noundef %b, i32 noundef %m) local_unnamed_addr #0 {
; CHECK-LABEL: _Z16test_shuffle_s64xj:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; nopb ; ret lr ; nopm ; nops
; CHECK-NEXT:    mov r24, r0 // Delay Slot 5
; CHECK-NEXT:    mov r29, r2 // Delay Slot 4
; CHECK-NEXT:    mov r25, r1 // Delay Slot 3
; CHECK-NEXT:    vbcstshfl.64 x0, r25:r24, r29 // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  %0 = bitcast i64 %b to <2 x i32>
  %1 = tail call <16 x i32> @llvm.aie2.vbcst.shuffle64(<2 x i32> %0, i32 %m)
  ret <16 x i32> %1
}


declare <16 x i32> @llvm.aie2.vbcst.shuffle8(i32, i32) #1
declare <16 x i32> @llvm.aie2.vbcst.shuffle16(i32, i32) #1
declare <16 x i32> @llvm.aie2.vbcst.shuffle32(i32, i32) #1
declare <16 x i32> @llvm.aie2.vbcst.shuffle64(<2 x i32>, i32) #1
