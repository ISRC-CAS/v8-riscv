// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_EXECUTION_RISCV64_FRAME_CONSTANTS_RISCV64_H_
#define V8_EXECUTION_RISCV64_FRAME_CONSTANTS_RISCV64_H_

#include "src/base/bits.h"
#include "src/base/macros.h"
#include "src/common/globals.h"
#include "src/execution/frame-constants.h"

namespace v8 {
namespace internal {

// The layout of an EntryFrame is as follows:
//
//  slot      Entry frame
//       +---------------------+-----------------------
// -25   |  saved register f27 |
// ...   |        ....         |
// -16   |  saved register f18 |
// -15   |  saved register f9  |
// -14   |  saved register f8  |
// -13   |   saved ra (x2)     |
//       |- - - - - - - - - - -|
// -12   |   saved fp (x8)     |
//       |- - - - - - - - - - -|
// -11   |  saved register x27 |
// ...   |        ...          |
//  -2   |  saved register x18 |
//  -1   |  saved register x9  |
//       |- - - - - - - - - - -|
//   0   |  bad frame pointer  |  <-- frame ptr
//       |   (0xFFF.. FF)      |
//       |- - - - - - - - - - -|
//   1   | stack frame marker  |
//       |      (ENTRY)        |
//       |- - - - - - - - - - -|
//   2   | stack frame marker  |
//       |        (0)          |
//       |- - - - - - - - - - -|
//   3   |     C entry FP      |
//       |- - - - - - - - - - -|
//   4   |   JS entry frame    |
//       |       marker        |
//       |- - - - - - - - - - -|
//   5   |      padding        |  <-- stack ptr
//  -----+---------------------+-----------------------
//
class EntryFrameConstants : public AllStatic {
 public:
  // This is the offset to where JSEntry pushes the current value of
  // Isolate::c_entry_fp onto the stack.
  static constexpr int kCallerFPOffset = -3 * kSystemPointerSize;
  static constexpr int kFixedFrameSize = 6 * kSystemPointerSize;
};

class WasmCompileLazyFrameConstants : public TypedFrameConstants {
 public:
  static constexpr int kNumberOfSavedGpParamRegs = 8;
  static constexpr int kNumberOfSavedFpParamRegs = 8;

  // FP-relative.
  static constexpr int kWasmInstanceOffset = TYPED_FRAME_PUSHED_VALUE_OFFSET(1);
  static constexpr int kFixedFrameSizeFromFp =
      // Header is padded to 16 byte (see {MacroAssembler::EnterFrame}).
      RoundUp<16>(TypedFrameConstants::kFixedFrameSizeFromFp) +
      kNumberOfSavedGpParamRegs * kSystemPointerSize +
      kNumberOfSavedFpParamRegs * kDoubleSize;
};

// Frame constructed by the {WasmDebugBreak} builtin.
// After pushing the frame type marker, the builtin pushes all Liftoff cache
// registers (see liftoff-assembler-defs.h).
class WasmDebugBreakFrameConstants : public TypedFrameConstants {
 public:
  // {x0 .. x28} \ {x16, x17, x18, x26, x27}
  static constexpr uint32_t kPushedGpRegs =
      (1 << 29) - 1 - (1 << 16) - (1 << 17) - (1 << 18) - (1 << 26) - (1 << 27);
  // {d0 .. d29}; {d15} is not used, but we still keep it for alignment reasons
  // (the frame size needs to be a multiple of 16).
  static constexpr uint32_t kPushedFpRegs = (1 << 30) - 1;

  static constexpr int kNumPushedGpRegisters =
      base::bits::CountPopulation(kPushedGpRegs);
  static constexpr int kNumPushedFpRegisters =
      base::bits::CountPopulation(kPushedFpRegs);

  static constexpr int kLastPushedGpRegisterOffset =
      // Header is padded to 16 byte (see {MacroAssembler::EnterFrame}).
      -RoundUp<16>(TypedFrameConstants::kFixedFrameSizeFromFp) -
      kSystemPointerSize * kNumPushedGpRegisters;
  static constexpr int kLastPushedFpRegisterOffset =
      kLastPushedGpRegisterOffset - kDoubleSize * kNumPushedFpRegisters;

  // Offsets are fp-relative.
  static int GetPushedGpRegisterOffset(int reg_code) {
    DCHECK_NE(0, kPushedGpRegs & (1 << reg_code));
    uint32_t lower_regs = kPushedGpRegs & ((uint32_t{1} << reg_code) - 1);
    return kLastPushedGpRegisterOffset +
           base::bits::CountPopulation(lower_regs) * kSystemPointerSize;
  }

  static int GetPushedFpRegisterOffset(int reg_code) {
    DCHECK_NE(0, kPushedFpRegs & (1 << reg_code));
    uint32_t lower_regs = kPushedFpRegs & ((uint32_t{1} << reg_code) - 1);
    return kLastPushedFpRegisterOffset +
           base::bits::CountPopulation(lower_regs) * kDoubleSize;
  }
};
}  // namespace internal
}  // namespace v8
#endif  // V8_EXECUTION_RISCV64_FRAME_CONSTANTS_RISCV64_H_