//===-- llvm/CodeGen/AllocationOrder.cpp - Allocation Order ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// Modifications (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its
// affiliates
//
//===----------------------------------------------------------------------===//
//
// This file implements an allocation order for virtual registers.
//
// The preferred allocation order for a virtual register depends on allocation
// hints and target hooks. The AllocationOrder class encapsulates all of that.
//
//===----------------------------------------------------------------------===//

#include "AllocationOrder.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "regalloc"

// Compare VirtRegMap::getRegAllocPref().
AllocationOrder AllocationOrder::create(unsigned VirtReg, const VirtRegMap &VRM,
                                        const RegisterClassInfo &RegClassInfo,
                                        const LiveRegMatrix *Matrix) {
  if (VRM.hasRequiredPhys(VirtReg)) {
    return AllocationOrder(MCPhysReg(VRM.getRequiredPhys(VirtReg)));
  }
  const MachineFunction &MF = VRM.getMachineFunction();
  const TargetRegisterInfo *TRI = &VRM.getTargetRegInfo();
  auto Order = RegClassInfo.getOrder(MF.getRegInfo().getRegClass(VirtReg));
  SmallVector<MCPhysReg, 16> Hints;
  bool HardHints =
      TRI->getRegAllocationHints(VirtReg, Order, Hints, MF, &VRM, Matrix);

  LLVM_DEBUG({
    if (!Hints.empty()) {
      dbgs() << "hints:";
      for (unsigned I = 0, E = Hints.size(); I != E; ++I)
        dbgs() << ' ' << printReg(Hints[I], TRI);
      dbgs() << '\n';
    }
  });
#ifndef NDEBUG
  for (unsigned I = 0, E = Hints.size(); I != E; ++I)
    assert(is_contained(Order, Hints[I]) &&
           "Target hint is outside allocation order.");
#endif
  return AllocationOrder(std::move(Hints), Order, HardHints);
}

AllocationOrder::AllocationOrder(MCPhysReg RequiredReg) : IterationLimit(1) {
  OrderScratch = {RequiredReg};
  Order = OrderScratch;
}

AllocationOrder::AllocationOrder(const AllocationOrder &A)
    : Hints(A.Hints), IterationLimit(A.IterationLimit) {
  if (A.OrderScratch.empty()) {
    Order = A.Order;
  } else {
    OrderScratch = A.OrderScratch;
    Order = OrderScratch;
  }
}
