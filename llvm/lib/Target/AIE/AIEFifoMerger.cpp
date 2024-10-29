//===- AIEMachineAlignment.cpp -----------------------------*- C++ -*-===//
//
// This file is licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates
//
//===----------------------------------------------------------------------===//

#include "AIE.h"
#include "AIE2.h"
#include "AIE2InstrInfo.h"
#include "AIE2Subtarget.h"
#include "AIE2TargetMachine.h"
#include "AIEBundle.h"
#include "AIESubtarget.h"

#include "AIEBundle.h"
#include "AIEFifoMerger.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineInstrBundle.h"

#include <iostream>
#include <map>
using namespace llvm;

#define DEBUG_TYPE "aie-pipeline-phi-elimination"

static constexpr int kExpectedRegSeqOperands = 5;
static constexpr int kExpectedPhiOperands = 5;

using ReplaceMapTy = DenseMap<Register, SmallVector<Register>>;

bool AIEFifoMerger::runOnMachineFunction(MachineFunction &MF) {
  const AIE2InstrInfo *TII = MF.getSubtarget<AIE2Subtarget>().getInstrInfo();
  bool changed = false;
  // Using std map for ordered iteration.
  std::map<unsigned, SmallVector<MachineInstr *>> ToMerge;
  for (auto &MBB : MF) {
    for (auto &I : MBB) {
      // Only look at *_prep/*_pop instructions.
      if (I.getOpcode() != AIE2::VLDB_pstm_imm_prep &&
          I.getOpcode() != AIE2::VLDB_pstm_imm_pop) {
        continue;
      }
      changed = true;
      unsigned FifoReg;
      if (I.getOpcode() == AIE2::VLDB_pstm_imm_prep) {
        FifoReg = I.getOperand(0).getReg();
      } else {
        FifoReg = I.getOperand(I.getNumOperands() - 1).getReg();
      }
      ToMerge[FifoReg].push_back(&I);
    }
  }
  // Merge instructions in pseudo.
  for (auto &TM : ToMerge) {
    unsigned PopOutput = 0;
    for (auto *I : TM.second) {
      if (I->getOpcode() == AIE2::VLDB_pstm_imm_pop) {
        PopOutput = I->getOperand(0).getReg();
        break;
      }
    }
    for (auto *I : TM.second) {
      if (I->getOpcode() == AIE2::VLDB_pstm_imm_prep) {
        MachineBasicBlock::iterator It(I);
        auto *NewI =
            BuildMI(*I->getParent(), It, I->getDebugLoc(),
                    TII->get(AIE2::VLD_pstm_imm_4x32_pseudo), PopOutput)
                .getInstr();
        for (unsigned i = 1; i < I->getNumOperands(); ++i) {
          NewI->addOperand(I->getOperand(i));
        }
        I->eraseFromParent();
      } else {
        I->eraseFromParent();
      }
    }
  }
  return changed;
}

INITIALIZE_PASS_BEGIN(AIEFifoMerger, DEBUG_TYPE, "AIE Fifo Merger", false,
                      false)
INITIALIZE_PASS_END(AIEFifoMerger, DEBUG_TYPE, "AIE Fifo Merger", false, false)

char AIEFifoMerger::ID = 0;
llvm::FunctionPass *llvm::createAIEFifoMerger() { return new AIEFifoMerger(); }
