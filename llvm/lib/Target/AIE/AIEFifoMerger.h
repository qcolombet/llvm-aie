//===- AIEFifoMerger.h ------------------------------*- C++ -*-===//
//
// This file is licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AIE_AIEFIFOMERGER_H
#define LLVM_LIB_TARGET_AIE_AIEFIFOMERGER_H

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

class AIEFifoMerger : public llvm::MachineFunctionPass {

public:
  static char ID;
  AIEFifoMerger() : MachineFunctionPass(ID) {}
  llvm::StringRef getPassName() const override { return "AIE Fifo Merger"; }
  bool runOnMachineFunction(llvm::MachineFunction &MF) override;
};

} // end namespace llvm

#endif
