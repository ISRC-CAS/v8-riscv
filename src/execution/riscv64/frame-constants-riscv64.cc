// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/execution/frame-constants.h"

#if V8_TARGET_ARCH_RISCV64

#include "src/codegen/assembler.h"
#include "src/codegen/riscv64/assembler-riscv64-inl.h"
#include "src/execution/frames.h"
#include "src/execution/riscv64/frame-constants-riscv64.h"

namespace v8 {
namespace internal {

Register JavaScriptFrame::fp_register() { return v8::internal::fp; }
Register JavaScriptFrame::context_register() { return a0; }
Register JavaScriptFrame::constant_pool_pointer_register() { UNREACHABLE(); }

int InterpreterFrameConstants::RegisterStackSlotCount(int register_count) {
  // Round up to a multiple of two, to make the frame a multiple of 16 bytes.
  return RoundUp(register_count, 2);
}

int BuiltinContinuationFrameConstants::PaddingSlotCount(int register_count) {
  // Round the total slot count up to a multiple of two, to make the frame a
  // multiple of 16 bytes.
  int slot_count = kFixedSlotCount + register_count;
  int rounded_slot_count = RoundUp(slot_count, 2);
  return rounded_slot_count - slot_count;
}

}  // namespace internal
}  // namespace v8

#endif  // V8_TARGET_ARCH_RISCV64
