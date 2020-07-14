// Copyright 2015 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/compiler/backend/instruction-scheduler.h"

namespace v8 {
namespace internal {
namespace compiler {

bool InstructionScheduler::SchedulerSupported() { return true; }

int InstructionScheduler::GetTargetInstructionFlags(
    const Instruction* instr) const {
  switch (instr->arch_opcode()) {
    case kRISCV64AbsD:
    case kRISCV64Add:
    case kRISCV64Add32:
    case kRISCV64AddD:
    case kRISCV64AddS:
    case kRISCV64Sub:
    case kRISCV64Sub32:
    case kRISCV64SubD:
    case kRISCV64SubS:
    case kRISCV64And:
    case kRISCV64Xor:
    case kRISCV64Or:
    case kRISCV64NegD:
    case kRISCV64Cmp:
    case kRISCV64CmpD:
    case kRISCV64CmpS:
    case kRISCV64Mul:
    case kRISCV64Mul32:
    case kRISCV64MulD:
    case kRISCV64Div:
    case kRISCV64Div32:
    case kRISCV64DivD:
    case kRISCV64DivS:
    case kRISCV64DivU:
    case kRISCV64Mod32:
    case kRISCV64Clz:
    case kRISCV64FcvtDUl:
    case kRISCV64FcvtDL:
    case kRISCV64FcvtDW:
    case kRISCV64FcvtDS:
    case kRISCV64FcvtDWu:
    case kRISCV64FcvtWD:
    case kRISCV64FcvtLD:
    case kRISCV64FcvtSD:
    case kRISCV64FcvtSW:
    case kRISCV64FcvtUlD:
    case kRISCV64FcvtUwD:
    case kRISCV64FcvtLS:
    case kRISCV64FcvtUlS:
    case kRISCV64FMVXW:
    case kRISCV64FMVWX:
    case kRISCV64FMVDX:
    case kRISCV64FMVXD:
    case kRISCV64SqrtD:
    case kRISCV64Sll:
    case kRISCV64Srl:
    case kRISCV64Sra:
    case kRISCV64Sll32:
    case kRISCV64Srl32:
    case kRISCV64Sra32:
    case kRISCV64Float64Max:
    case kRISCV64Float64Min:
    case kRISCV64Float64ExtractHighWord32:
    case kRISCV64Float64InsertLowWord32:
    case kRISCV64Float64InsertHighWord32:
    case kRISCV64Float64ExtractLowWord32:
    case kRISCV64Float64RoundDown:
    case kRISCV64Float64SilenceNaN:
    case kRISCV64Float64RoundTiesEven:
    case kRISCV64Float64RoundTruncate:
    case kRISCV64Float64RoundUp:
      return kNoOpcodeFlags;
    case kRISCV64Lb:
    case kRISCV64Lbu:
    case kRISCV64fld:
    case kRISCV64Lh:
    case kRISCV64Lhu:
    case kRISCV64Lw:
    case kRISCV64Lwu:
    case kRISCV64Ld:
    case kRISCV64flw:
    case kRISCV64MsaLd:
    case kRISCV64Peek:
    case kRISCV64Uldc1:
    case kRISCV64Ulh:
    case kRISCV64Ulhu:
    case kRISCV64Ulw:
    case kRISCV64Ulwc1:
    case kRISCV64Word64AtomicLoadUint64:
      return kIsLoadOperation;

    case kRISCV64ModD:
    case kRISCV64ModS:
    case kRISCV64MsaSt:
    case kRISCV64Push:
    case kRISCV64Sb:
    case kRISCV64fsd:
    case kRISCV64Sh:
    case kRISCV64StackClaim:
    case kRISCV64StoreToStackSlot:
    case kRISCV64Sw:
    case kRISCV64Sd:
    case kRISCV64fsw:
    case kRISCV64Usdc1:
    case kRISCV64Ush:
    case kRISCV64Usw:
    case kRISCV64Uswc1:
    case kRISCV64Sync:
    // case kRISCV64Word32AtomicPairStore:
    // case kRISCV64Word32AtomicPairAdd:
    // case kRISCV64Word32AtomicPairSub:
    // case kRISCV64Word32AtomicPairAnd:
    // case kRISCV64Word32AtomicPairOr:
    // case kRISCV64Word32AtomicPairXor:
    // case kRISCV64Word32AtomicPairExchange:
    // case kRISCV64Word32AtomicPairCompareExchange:
    case kRISCV64Word64AtomicAddUint64:
    case kRISCV64Word64AtomicSubUint64:
    case kRISCV64Word64AtomicAndUint64:
    case kRISCV64Word64AtomicOrUint64:
    case kRISCV64Word64AtomicXorUint64:
    case kRISCV64Word64AtomicStoreWord64:
    case kRISCV64Word64AtomicExchangeUint64:
    case kRISCV64Word64AtomicCompareExchangeUint64:
      return kHasSideEffect;

#define CASE(Name) case k##Name:
      COMMON_ARCH_OPCODE_LIST(CASE)
#undef CASE
      // Already covered in architecture independent code.
      UNREACHABLE();
    default:
      UNREACHABLE();
  }
  UNREACHABLE();
}

int InstructionScheduler::GetInstructionLatency(const Instruction* instr) {
  return 1;
}

}  // namespace compiler
}  // namespace internal
}  // namespace v8
