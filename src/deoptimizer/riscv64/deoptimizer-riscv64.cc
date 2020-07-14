// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if V8_TARGET_ARCH_RISCV64

#include "src/codegen/macro-assembler.h"
#include "src/codegen/register-configuration.h"
#include "src/codegen/safepoint-table.h"
#include "src/deoptimizer/deoptimizer.h"
#include "src/objects/objects-inl.h"

namespace v8 {
namespace internal {

const bool Deoptimizer::kSupportsFixedDeoptExitSizes = false;
const int Deoptimizer::kNonLazyDeoptExitSize = 0;
const int Deoptimizer::kLazyDeoptExitSize = 0;

#define __ masm->

void Deoptimizer::GenerateDeoptimizationEntries(MacroAssembler* masm,
                                                Isolate* isolate,
                                                DeoptimizeKind deopt_kind) {}

Float32 RegisterValues::GetFloatRegister(unsigned n) const {
  return Float32::FromBits(
      static_cast<uint32_t>(double_registers_[n].get_bits()));
}

void FrameDescription::SetPc(intptr_t pc) { pc_ = pc; }

void FrameDescription::SetCallerPc(unsigned offset, intptr_t value) {
  if (kPCOnStackSize == 2 * kSystemPointerSize) {
    SetFrameSlot(offset + kSystemPointerSize, 0);
  }
  SetFrameSlot(offset, value);
}

void FrameDescription::SetCallerFp(unsigned offset, intptr_t value) {
  if (kFPOnStackSize == 2 * kSystemPointerSize) {
    SetFrameSlot(offset + kSystemPointerSize, 0);
  }
  SetFrameSlot(offset, value);
}

void FrameDescription::SetCallerConstantPool(unsigned offset, intptr_t value) {
  // No embedded constant pool support.
  UNREACHABLE();
}

#undef __

}  // namespace internal
}  // namespace v8

#endif  // V8_TARGET_ARCH_RISCV64
