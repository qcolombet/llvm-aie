; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
;
; This file is licensed under the Apache License v2.0 with LLVM Exceptions.
; See https://llvm.org/LICENSE.txt for license information.
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
;
; (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
; RUN: llc -O2 -mtriple=aie2 --enable-pipeliner=0 %s -o - | FileCheck %s

@buffer1 = dso_local local_unnamed_addr global [1000 x i32] zeroinitializer, align 4
@buffer2 = dso_local local_unnamed_addr global [1000 x i32] zeroinitializer, align 4

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingWord() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingWord:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    movxm p0, #buffer2
; CHECK-NEXT:    lda r0, [p0], #4
; CHECK-NEXT:    lda r1, [p0], #4
; CHECK-NEXT:    lda r2, [p0], #4
; CHECK-NEXT:    lda r3, [p0], #4
; CHECK-NEXT:    lda r4, [p0], #4
; CHECK-NEXT:    lda r5, [p0, #0]
; CHECK-NEXT:    movxm p0, #buffer1
; CHECK-NEXT:    st r0, [p0], #4
; CHECK-NEXT:    st r1, [p0], #4; ret lr
; CHECK-NEXT:    st r2, [p0], #4 // Delay Slot 5
; CHECK-NEXT:    st r3, [p0], #4 // Delay Slot 4
; CHECK-NEXT:    st r4, [p0], #4 // Delay Slot 3
; CHECK-NEXT:    st r5, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 4 dereferenceable(24) @buffer1, ptr noundef nonnull align 4 dereferenceable(24) @buffer2, i32 24, i1 false)
  ret void
}

; Function Attrs: mustprogress nocallback nofree nounwind willreturn memory(argmem: readwrite)
declare void @llvm.memcpy.p0.p0.i32(ptr noalias nocapture writeonly, ptr noalias nocapture readonly, i32, i1 immarg) #1

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingWordByte() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingWordByte:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; nopa ; nops ; movxm p1, #(buffer2+8); nopv
; CHECK-NEXT:    lda.s8 r0, [p1, #0]; nopb ; movxm p0, #(buffer1+8); nops
; CHECK-NEXT:    st.s8 r0, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    paddb [p1], #-8
; CHECK-NEXT:    lda r0, [p1], #4
; CHECK-NEXT:    lda r1, [p1, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    paddb [p0], #-8 // Delay Slot 4
; CHECK-NEXT:    st r0, [p0], #4 // Delay Slot 3
; CHECK-NEXT:    st r1, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 4 dereferenceable(9) @buffer1, ptr noundef nonnull align 4 dereferenceable(9) @buffer2, i32 9, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingHalfByte() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingHalfByte:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; nopb ; movxm p0, #buffer2; nops
; CHECK-NEXT:    lda.s16 r0, [p0], #2; movxm p1, #buffer1
; CHECK-NEXT:    st.s16 r0, [p1], #2
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    lda.s8 r0, [p0, #0]
; CHECK-NEXT:    st.s8 r0, [p1, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 4 dereferenceable(3) @buffer1, ptr noundef nonnull align 4 dereferenceable(3) @buffer2, i32 3, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingWordHalfByte() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingWordHalfByte:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; nopb ; movxm p1, #(buffer2+8)
; CHECK-NEXT:    movxm p2, #(buffer1+8)
; CHECK-NEXT:    lda.s16 r0, [p1], #2; mov p0, p1
; CHECK-NEXT:    st.s16 r0, [p2], #2; mov p3, p2
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    lda.s8 r0, [p1, #0]
; CHECK-NEXT:    st.s8 r0, [p2, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    paddb [p0], #-8
; CHECK-NEXT:    lda r0, [p0], #4
; CHECK-NEXT:    lda r1, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    paddb [p3], #-8 // Delay Slot 4
; CHECK-NEXT:    st r0, [p3], #4 // Delay Slot 3
; CHECK-NEXT:    st r1, [p3, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 4 dereferenceable(11) @buffer1, ptr noundef nonnull align 4 dereferenceable(11) @buffer2, i32 11, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingVector16() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingVector16:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; nopb ; movxm p0, #buffer2; nops
; CHECK-NEXT:    vlda.128 wh0, [p0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    vmov q0, wh0 // Delay Slot 4
; CHECK-NEXT:    movxm p0, #buffer1 // Delay Slot 3
; CHECK-NEXT:    st q0, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 16 dereferenceable(16) @buffer1, ptr noundef nonnull align 16 dereferenceable(16) @buffer2, i32 16, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingWordVector16() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingWordVector16:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nop ; movxm p0, #buffer2
; CHECK-NEXT:    lda q0, [p0], #16
; CHECK-NEXT:    lda q2, [p0], #16
; CHECK-NEXT:    lda r0, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    movxm p0, #buffer1 // Delay Slot 5
; CHECK-NEXT:    st q0, [p0], #16 // Delay Slot 4
; CHECK-NEXT:    st q2, [p0], #16 // Delay Slot 3
; CHECK-NEXT:    st r0, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 16 dereferenceable(36) @buffer1, ptr noundef nonnull align 16 dereferenceable(36) @buffer2, i32 36, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingWordVector32() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingWordVector32:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopa ; nopb ; movxm p0, #buffer2
; CHECK-NEXT:    vldb wh0, [p0], #32
; CHECK-NEXT:    lda r0, [p0], #4
; CHECK-NEXT:    lda r1, [p0], #4
; CHECK-NEXT:    lda r2, [p0], #4
; CHECK-NEXT:    lda r3, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    movxm p0, #buffer1
; CHECK-NEXT:    vst wh0, [p0], #32; ret lr
; CHECK-NEXT:    st r0, [p0], #4 // Delay Slot 5
; CHECK-NEXT:    st r1, [p0], #4 // Delay Slot 4
; CHECK-NEXT:    st r2, [p0], #4 // Delay Slot 3
; CHECK-NEXT:    st r3, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 32 dereferenceable(48) @buffer1, ptr noundef nonnull align 32 dereferenceable(48) @buffer2, i32 48, i1 false)
  ret void
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemcpyUsingAlignedWordCall() local_unnamed_addr #0 {
; CHECK-LABEL: lowerMemcpyUsingAlignedWordCall:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    jl #memcpy
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    paddb [sp], #32 // Delay Slot 4
; CHECK-NEXT:    st lr, [sp, #-32] // 4-byte Folded Spill Delay Slot 3
; CHECK-NEXT:    movxm p1, #buffer1 // Delay Slot 2
; CHECK-NEXT:    mova r0, #256; movxm p2, #buffer2 // Delay Slot 1
; CHECK-NEXT:    lda lr, [sp, #-32] // 4-byte Folded Reload
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    paddb [sp], #-32 // Delay Slot 1
entry:
  tail call void @llvm.memcpy.p0.p0.i32(ptr noundef nonnull align 4 dereferenceable(256) @buffer1, ptr noundef nonnull align 4 dereferenceable(256) @buffer2, i32 256, i1 false)
  ret void
}


; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(write, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemsetUsingWordVector32() local_unnamed_addr #2 {
; CHECK-LABEL: lowerMemsetUsingWordVector32:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    mova r0, #0; nopb ; nopxm ; nops
; CHECK-NEXT:    vbcst.32 x0, r0
; CHECK-NEXT:    movxm p0, #buffer1
; CHECK-NEXT:    vst wl0, [p0], #32; ret lr
; CHECK-NEXT:    st r0, [p0], #4 // Delay Slot 5
; CHECK-NEXT:    st r0, [p0], #4 // Delay Slot 4
; CHECK-NEXT:    st r0, [p0], #4 // Delay Slot 3
; CHECK-NEXT:    st r0, [p0, #0] // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memset.p0.i32(ptr noundef nonnull align 32 dereferenceable(48) @buffer1, i8 0, i32 48, i1 false)
  ret void
}

; Function Attrs: mustprogress nocallback nofree nounwind willreturn memory(argmem: write)
declare void @llvm.memset.p0.i32(ptr nocapture writeonly, i8, i32, i1 immarg) #3

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(write, argmem: none, inaccessiblemem: none)
define dso_local void @lowerMemsetUsingWordByte() local_unnamed_addr #2 {
; CHECK-LABEL: lowerMemsetUsingWordByte:
; CHECK:         .p2align 4
; CHECK-NEXT:  // %bb.0: // %entry
; CHECK-NEXT:    nopb ; mova r0, #0; nops ; movxm p0, #(buffer1+4); nopv
; CHECK-NEXT:    nopa ; nopb ; nopx ; st r0, [p0, #-4]
; CHECK-NEXT:    st.s8 r0, [p0, #0]
; CHECK-NEXT:    nop
; CHECK-NEXT:    ret lr
; CHECK-NEXT:    nop // Delay Slot 5
; CHECK-NEXT:    nop // Delay Slot 4
; CHECK-NEXT:    nop // Delay Slot 3
; CHECK-NEXT:    nop // Delay Slot 2
; CHECK-NEXT:    nop // Delay Slot 1
entry:
  tail call void @llvm.memset.p0.i32(ptr noundef nonnull align 4 dereferenceable(5) @buffer1, i8 0, i32 5, i1 false)
  ret void
}

attributes #0 = { mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none) "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { mustprogress nocallback nofree nounwind willreturn memory(argmem: readwrite) }
attributes #2 = { mustprogress nofree norecurse nosync nounwind willreturn memory(write, argmem: none, inaccessiblemem: none) "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #3 = { mustprogress nocallback nofree nounwind willreturn memory(argmem: write) }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 18.0.0git (git@github.com:Xilinx/llvm-aie.git 4a8034bbeb163db2ece3e21469114dd7426cb13c)"}
