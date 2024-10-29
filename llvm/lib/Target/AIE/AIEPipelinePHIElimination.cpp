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
#include "AIEPipelinePHIElimination.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineInstrBundle.h"

#include <iostream>
using namespace llvm;

#define DEBUG_TYPE "aie-pipeline-phi-elimination"

static constexpr int kExpectedRegSeqOperands = 5;

using ReplaceMapTy = DenseMap<Register, SmallVector<Register>>;

// Helper to recursively add PHIs to remove list and update replacement map for
// correct replacement of register.
static void addPhisToRemove(
    const MachineRegisterInfo &MRI, const MachineBasicBlock *InnerLoopBlock,
    ReplaceMapTy &ReplaceMap, SmallVector<MachineInstr *> &ToBeCleanedUp,
    Register Reg, std::optional<Register> ToReplaceWithReg = std::nullopt) {
  auto RegUses = MRI.use_instr_begin(Reg);
  SmallVector<MachineInstr *> Stack;
  for (; RegUses != MRI.use_instr_end(); ++RegUses) {
    Stack.push_back(&*RegUses);
  }
  while (!Stack.empty()) {
    MachineInstr *Instr = Stack.pop_back_val();
    LLVM_DEBUG(dbgs() << "Processing add: ");
    LLVM_DEBUG(Instr->dump());
    assert(Instr->getOpcode() == TargetOpcode::PHI);
    if (Instr->getParent() == InnerLoopBlock) {
      ToBeCleanedUp.push_back(Instr);
      if (ToReplaceWithReg.has_value()) {
        ReplaceMap[*ToReplaceWithReg].push_back(Instr->getOperand(0).getReg());
      }
      RegUses = MRI.use_instr_begin(Instr->getOperand(0).getReg());
      // Recursively process multiple layers of PHIs
      for (; RegUses != MRI.use_instr_end(); ++RegUses) {
        if (RegUses->getOpcode() == TargetOpcode::PHI) {
          Stack.push_back(&*RegUses);
        }
      }
    }
  }
}

// Helper function checking that a load/phi/regsequence instruction maintains
// the proper conditions for processing. Return the candidate instruction
// through which we are going to pipeline through.
MachineInstr *checkConditions(MachineInstr *MI, MachineRegisterInfo &MRI) {
  MachineOperand &MOut = MI->getOperand(0);

  MachineInstr *Candidate = nullptr;
  LLVM_DEBUG(dbgs() << "Search successors for: ");
  LLVM_DEBUG(MI->dump());
  for (auto it = MRI.use_instr_begin(MOut.getReg()); it != MRI.use_instr_end();
       ++it) {
    // Check that uses are either PHI nodes or REG_SEQUENCEs.
    // If the input instruction (MI) is a PHI node itself then
    // if the user is a PHI they both need to be in the same BB (the loop).
    if ((it->getOpcode() != TargetOpcode::PHI ||
         (MI->getOpcode() == TargetOpcode::PHI &&
          it->getParent() != MI->getParent())) &&
        it->getOpcode() != TargetOpcode::REG_SEQUENCE) {
      continue;
    }
    // We are trying to find a single candidate. Multiple candidates
    // is something we don't understand.
    if (Candidate != nullptr) {
      LLVM_DEBUG(dbgs() << "Failing because not unique:");
      LLVM_DEBUG(Candidate->dump());
      LLVM_DEBUG(it->dump());
      return nullptr;
    }
    Candidate = &*it;
  }
  if (Candidate == nullptr) {
    LLVM_DEBUG(dbgs() << "Didn't find successor for:");
    LLVM_DEBUG(MI->dump());
    return nullptr;
  }
  LLVM_DEBUG(dbgs() << "Operands:\n");
  LLVM_DEBUG(for (auto &MO : Candidate->operands()) { MO.dump(); });
  if (Candidate->getNumOperands() != kExpectedRegSeqOperands) {
    return nullptr;
  }
  LLVM_DEBUG(dbgs() << "Found a reg sequence or phi: ");
  LLVM_DEBUG(Candidate->dump());
  return Candidate;
}

bool AIEPipelinePHIElimination::runOnMachineFunction(MachineFunction &MF) {
  SmallVector<MachineInstr *> EliminationCandidates;
  // Collect candidates, for this example only 4x32 vlds.
  for (auto &MB : MF) {
    for (auto &MI : MB) {
      if (MI.getOpcode() == AIE2::VLD_pstm_imm_4x32_pseudo) {
        EliminationCandidates.push_back(&MI);
      }
    }
  }
  if (EliminationCandidates.empty()) {
    return false;
  }
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const AIE2InstrInfo *TII = MF.getSubtarget<AIE2Subtarget>().getInstrInfo();
  SmallSet<MachineInstr *, 8> AlreadyProcessed;

  unsigned LastFifoReg = AIE2::fifo0;

  SmallVector<MachineInstr *> ToBeCleanedUp;
  DenseMap<Register, SmallVector<Register>> ReplaceMap;
  for (auto *MI : EliminationCandidates) {
    SmallVector<MachineInstr *> ToProcess(1, MI);
    SmallVector<MachineInstr *> LoadsInPrehead;
    SmallSet<MachineInstr *, 8> InLoopInstr;
    MachineInstr *LoadInLoop = nullptr;
    if (AlreadyProcessed.count(MI)) {
      continue;
    }
    MachineBasicBlock *InnerLoopBlock = nullptr;
    // Process instructions. We expect loads at the beginning going through
    // REG_SEQUENCES and PHIs as the average sequence looks something like:
    // x = VLD
    // y = VLD
    // ...
    // z = REG_SEQUENCE x, y
    // ...
    // q = PHI z, ..
    // k = PHI q, ..
    //
    // Where there could be multiple levels of PHIs.
    while (!ToProcess.empty()) {
      auto *P = ToProcess.pop_back_val();
      if (P->getOpcode() != TargetOpcode::PHI &&
          P->getOpcode() != TargetOpcode::REG_SEQUENCE &&
          P->getOpcode() != AIE2::VLD_pstm_imm_4x32_pseudo) {
        continue;
      }
      AlreadyProcessed.insert(P);
      if (P->getOpcode() == TargetOpcode::REG_SEQUENCE) {
        // Identify unrecognized condition. We already found a load/regsequence
        // for this chain because InnerLoopBlock is set. We expected only one.
        if (P->getParent() == InnerLoopBlock) {
          if (LoadInLoop != nullptr) {
            LoadsInPrehead.clear();
            LoadInLoop = nullptr;
            LLVM_DEBUG(dbgs() << "Multiple in loop loads:");
            LLVM_DEBUG(P->dump());
            break;
          }
          LLVM_DEBUG(dbgs() << "In loop load: ");
          LLVM_DEBUG(P->dump());
          LoadInLoop = P;
        } else {
          // Regsequence/Load in pre-header or otherwise before the loop.
          LoadsInPrehead.push_back(P);
        }
      }
      // Push users to process for VLD instructions.
      if (P->getOpcode() != AIE2::VLD_pstm_imm_4x32_pseudo) {
        auto it = MRI.def_instr_begin(P->getOperand(1).getReg());
        if (!AlreadyProcessed.count(&*it)) {
          ToProcess.push_back(&*it);
        }
        it = MRI.def_instr_begin(P->getOperand(3).getReg());
        if (!AlreadyProcessed.count(&*it)) {
          ToProcess.push_back(&*it);
        }
      }
      auto *result = checkConditions(P, MRI);
      if (result == nullptr) {
        continue;
      }
      if (P->getOpcode() == TargetOpcode::PHI) {
        // Consider the PHIs inside the loop.
        InLoopInstr.insert(P);
        if (InnerLoopBlock == nullptr) {
          InnerLoopBlock = P->getParent();
        } else {
          if (InnerLoopBlock != P->getParent()) {
            // Do not perform any transformation if condition is broken.
            LoadsInPrehead.clear();
            LoadInLoop = nullptr;
          }
        }
      }
      if (!AlreadyProcessed.count(result)) {
        ToProcess.push_back(result);
      }
    }
    // Quick candidate skip if there's nothing to do.
    if (LoadInLoop == nullptr || LoadsInPrehead.empty()) {
      continue;
    }
    // Helper function to do the actual work for all the sub-instructions
    // to be pipelined. (Made a lambda as its accessing quite some local
    // members)
    auto convert_regseq_to_push = [&](MachineInstr *instr) {
      // Process regsequence (we only expect regsequences here for the
      // instructiosn we support right now).
      if (instr->getOpcode() != TargetOpcode::REG_SEQUENCE) {
        return;
      }
      MachineOperand &MO0 = instr->getOperand(1);
      MachineOperand &MO1 = instr->getOperand(3);
      auto mo0_it = MRI.def_instr_begin(MO0.getReg());
      auto mo1_it = MRI.def_instr_begin(MO1.getReg());
      if (instr->getParent() == InnerLoopBlock) {
        // If its a regsequence inside the loop then we need to add the pop
        // instructions in the loop.
        Register new_mo0_vec_reg =
            MRI.createVirtualRegister(MRI.getRegClass(MO0.getReg()));
        auto insert_it = InnerLoopBlock->getFirstNonPHI();
        BuildMI(*InnerLoopBlock, insert_it, mo0_it->getDebugLoc(),
                TII->get(AIE2::VLDB_pstm_imm_pop), new_mo0_vec_reg)
            .addReg(LastFifoReg);
        Register new_mo1_vec_reg =
            MRI.createVirtualRegister(MRI.getRegClass(MO1.getReg()));
        BuildMI(*InnerLoopBlock, insert_it, mo1_it->getDebugLoc(),
                TII->get(AIE2::VLDB_pstm_imm_pop), new_mo1_vec_reg)
            .addReg(LastFifoReg + 1);
        Register new_rseq_vec_reg = MRI.createVirtualRegister(
            MRI.getRegClass(instr->getOperand(0).getReg()));
        auto RS_mib =
            BuildMI(*InnerLoopBlock, insert_it, mo0_it->getDebugLoc(),
                    TII->get(TargetOpcode::REG_SEQUENCE), new_rseq_vec_reg)
                .addReg(new_mo0_vec_reg)
                .addImm(instr->getOperand(2).getImm())
                .addReg(new_mo1_vec_reg)
                .addImm(instr->getOperand(4).getImm());
        Register rs_reg = instr->getOperand(0).getReg();
        ReplaceMap[RS_mib.getInstr()->getOperand(0).getReg()].push_back(rs_reg);
        // Add users of regsequence (that should be PHIs) to be removed.
        addPhisToRemove(MRI, InnerLoopBlock, ReplaceMap, ToBeCleanedUp, rs_reg,
                        RS_mib.getInstr()->getOperand(0).getReg());
      } else {
        // Add users of regsequence (that should be PHIs) to be removed.
        addPhisToRemove(MRI, InnerLoopBlock, ReplaceMap, ToBeCleanedUp,
                        instr->getOperand(0).getReg());
      }

      ToBeCleanedUp.push_back(&*mo0_it);
      ToBeCleanedUp.push_back(&*mo1_it);
      ToBeCleanedUp.push_back(instr);
      MachineRegisterInfo::def_instr_iterator LoadIts[2] = {mo0_it, mo1_it};
      MachineBasicBlock::iterator MO0InstrIt(&*mo0_it);
      MachineBasicBlock::iterator MO1InstrIt(&*mo1_it);
      if (std::distance(MO0InstrIt, MO1InstrIt) > 0) {
        std::swap(LoadIts[0], LoadIts[1]);
      }
      // Add *_prep instructions at proper location.
      MachineBasicBlock::iterator LoadInsertPoint(instr);
      for (int i = 0; i < 2; ++i) {
        auto LoadMBI =
            BuildMI(*instr->getParent(), LoadInsertPoint, instr->getDebugLoc(),
                    TII->get(AIE2::VLDB_pstm_imm_prep), LastFifoReg + i);
        auto Load = LoadIts[i];
        for (unsigned OpIdx = 1; OpIdx < Load->getNumOperands(); ++OpIdx) {
          auto &Op = Load->getOperand(OpIdx);
          if (Op.isReg()) {
            if (Op.isDef()) {
              Register NewLoadRegDef = MRI.createVirtualRegister(
                  MRI.getRegClass(Load->getOperand(OpIdx).getReg()));
              ReplaceMap[Load->getOperand(OpIdx).getReg()].push_back(
                  NewLoadRegDef);
              LoadMBI.addDef(NewLoadRegDef);
            } else {
              LoadMBI.addReg(Load->getOperand(OpIdx).getReg());
            }
          } else {
            assert(Load->getOperand(OpIdx).isImm() && "Unsupported kind");
            LoadMBI.addImm(Load->getOperand(OpIdx).getImm());
          }
        }
      }
    };
    // Process instructions and convert to FIFOs.
    convert_regseq_to_push(LoadInLoop);
    for (auto *I : LoadsInPrehead) {
      convert_regseq_to_push(I);
    }
    LastFifoReg += 2;
  }
  // Replace virtual registers with new ones.
  for (auto &RMEntry : ReplaceMap) {
    for (Register ToReplace : RMEntry.second) {
      MRI.replaceRegWith(ToReplace, RMEntry.first);
    }
  }
  // Cleanup dead instructions.
  SmallSet<MachineInstr *, 8> Eliminated;
  for (auto *ToCleanup : ToBeCleanedUp) {
    if (!Eliminated.insert(ToCleanup).second) {
      continue;
    }
    ToCleanup->eraseFromParent();
  }
  return true;
}

INITIALIZE_PASS_BEGIN(AIEPipelinePHIElimination, DEBUG_TYPE,
                      "AIE Pipeline PHI Elim", false, false)
INITIALIZE_PASS_END(AIEPipelinePHIElimination, DEBUG_TYPE,
                    "AIE Pipeline PHI Elim", false, false)

char AIEPipelinePHIElimination::ID = 0;
llvm::FunctionPass *llvm::createAIEPipelinePHIElimination() {
  return new AIEPipelinePHIElimination();
}
