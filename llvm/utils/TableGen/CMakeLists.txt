#
# Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
# See https://llvm.org/LICENSE.txt for license information.
# SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
#
# Modifications (c) Copyright 2023-2024 Advanced Micro Devices, Inc. or its affiliates

# Basic utilities which is the strict minimum needed to build
# llvm-min-tblgen.
add_subdirectory(Basic)
# Common utilities are all of the reusable components and helper
# code needed by the backends.
add_subdirectory(Common)

set(LLVM_LINK_COMPONENTS Support)

# llvm-min-tablegen only contains a subset of backends necessary to
# build llvm/include. It must not depend on TableGenCommon, as
# TableGenCommon depends on this already to generate things such as
# ValueType definitions.
add_tablegen(llvm-min-tblgen LLVM_HEADERS
  TableGen.cpp
  Attributes.cpp
  DirectiveEmitter.cpp
  IntrinsicEmitter.cpp
  RISCVTargetDefEmitter.cpp
  VTEmitter.cpp
  $<TARGET_OBJECTS:obj.LLVMTableGenBasic>

  PARTIAL_SOURCES_INTENDED
  )
set_target_properties(llvm-min-tblgen PROPERTIES FOLDER "Tablegenning")

set(LLVM_LINK_COMPONENTS
  CodeGenTypes
  Support
  )

add_tablegen(llvm-tblgen LLVM
  DESTINATION "${LLVM_TOOLS_INSTALL_DIR}"
  EXPORT LLVM
  AIEMemoryCyclesEmitter.cpp
  AIEPreSchedLoweringEmitter.cpp
  AIESplitInstrTablesEmitter.cpp
  AIEVariableInstrItineraryEmitter.cpp
  AsmMatcherEmitter.cpp
  AsmWriterEmitter.cpp
  Attributes.cpp
  CallingConvEmitter.cpp
  CodeEmitterGen.cpp
  CodeGenFormat.cpp
  CodeGenMapTable.cpp
  CompressInstEmitter.cpp
  CTagsEmitter.cpp
  DAGISelEmitter.cpp
  DAGISelMatcherEmitter.cpp
  DAGISelMatcherGen.cpp
  DAGISelMatcherOpt.cpp
  DecoderEmitter.cpp
  DFAEmitter.cpp
  DFAPacketizerEmitter.cpp
  DirectiveEmitter.cpp
  DisassemblerEmitter.cpp
  DXILEmitter.cpp
  ExegesisEmitter.cpp
  FastISelEmitter.cpp
  GlobalISelCombinerEmitter.cpp
  GlobalISelEmitter.cpp
  InstrDocsEmitter.cpp
  InstrInfoEmitter.cpp
  IntrinsicEmitter.cpp
  MacroFusionPredicatorEmitter.cpp
  OptParserEmitter.cpp
  OptRSTEmitter.cpp
  PseudoLoweringEmitter.cpp
  RegisterBankEmitter.cpp
  RegisterInfoEmitter.cpp
  RISCVTargetDefEmitter.cpp
  SearchableTableEmitter.cpp
  SubtargetEmitter.cpp
  TableGen.cpp
  VTEmitter.cpp
  WebAssemblyDisassemblerEmitter.cpp
  X86CompressEVEXTablesEmitter.cpp
  X86DisassemblerTables.cpp
  X86FoldTablesEmitter.cpp
  X86MnemonicTables.cpp
  X86ModRMFilters.cpp
  X86RecognizableInstr.cpp
  $<TARGET_OBJECTS:obj.LLVMTableGenBasic>
  $<TARGET_OBJECTS:obj.LLVMTableGenCommon>

  DEPENDS
  intrinsics_gen # via llvm-min-tablegen
  )
set_target_properties(llvm-tblgen PROPERTIES FOLDER "Tablegenning")
