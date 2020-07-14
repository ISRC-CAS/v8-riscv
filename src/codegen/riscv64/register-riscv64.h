// Copyright 2018 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_CODEGEN_RISCV64_REGISTER_RISCV64_H_
#define V8_CODEGEN_RISCV64_REGISTER_RISCV64_H_

#include "src/codegen/register.h"
#include "src/codegen/reglist.h"

namespace v8 {
namespace internal {

#define GENERAL_REGISTERS(V) \
  V(x0)                      \
  V(x1)                      \
  V(x2)                      \
  V(x3)                      \
  V(x4)                      \
  V(x5)                      \
  V(x6)                      \
  V(x7)                      \
  V(x8)                      \
  V(x9)                      \
  V(x10)                     \
  V(x11)                     \
  V(x12)                     \
  V(x13)                     \
  V(x14)                     \
  V(x15)                     \
  V(x16)                     \
  V(x17)                     \
  V(x18)                     \
  V(x19)                     \
  V(x20)                     \
  V(x21)                     \
  V(x22)                     \
  V(x23)                     \
  V(x24)                     \
  V(x25)                     \
  V(x26)                     \
  V(x27)                     \
  V(x28)                     \
  V(x29)                     \
  V(x30)                     \
  V(x31)

#define ALLOCATABLE_GENERAL_REGISTERS(V) \
  V(x5)                                  \
  V(x6)                                  \
  V(x7)                                  \
  V(x9)                                  \
  V(x10)                                 \
  V(x11)                                 \
  V(x12)                                 \
  V(x13)                                 \
  V(x14)                                 \
  V(x15)                                 \
  V(x16)                                 \
  V(x17)                                 \
  V(x18)                                 \
  V(x19)                                 \
  V(x20)                                 \
  V(x21)                                 \
  V(x22)                                 \
  V(x23)                                 \
  V(x24)                                 \
  V(x25)                                 \
  V(x26)                                 \
  V(x27)                                 \
  V(x28)                                 \
  V(x29)                                 \
  V(x30)                                 \
  V(x31)

enum RegisterCode {
#define REGISTER_CODE(R) kRegCode_##R,
  GENERAL_REGISTERS(REGISTER_CODE)
#undef REGISTER_CODE
      kRegAfterLast
};

class Register : public RegisterBase<Register, kRegAfterLast> {
 public:
  bool is_byte_register() const { return code() <= 3; }
  // Return the high bit of the register code as a 0 or 1.  Used often
  // when constructing the REX prefix byte.
  int high_bit() const { return code() >> 3; }
  // Return the 3 low bits of the register code.  Used when encoding registers
  // in modR/M, SIB, and opcode bytes.
  int low_bits() const { return code() & 0x7; }

 private:
  friend class RegisterBase<Register, kRegAfterLast>;
  explicit constexpr Register(int code) : RegisterBase(code) {}
};

int ToNumber(Register reg);

Register ToRegister(int num);

ASSERT_TRIVIALLY_COPYABLE(Register);
static_assert(sizeof(Register) == sizeof(int),
              "Register can efficiently be passed by value");

#define DECLARE_REGISTER(R) \
  constexpr Register R = Register::from_code(kRegCode_##R);
GENERAL_REGISTERS(DECLARE_REGISTER)
#undef DECLARE_REGISTER

#define ALIAS_REGISTER(register_class, alias, name) \
  constexpr register_class alias = name

// Registers aliases.
// ALIAS_REGISTER(VRegister, v8_, v8);  // Avoid conflicts with namespace v8.
ALIAS_REGISTER(Register, zero_reg, x0);
ALIAS_REGISTER(Register, ra, x1);
ALIAS_REGISTER(Register, sp, x2);
ALIAS_REGISTER(Register, gp, x3);
ALIAS_REGISTER(Register, tp, x4);
ALIAS_REGISTER(Register, t0, x5);
ALIAS_REGISTER(Register, t1, x6);
ALIAS_REGISTER(Register, t2, x7);
ALIAS_REGISTER(Register, s0, x8);
ALIAS_REGISTER(Register, fp, x8);
ALIAS_REGISTER(Register, s1, x9);
ALIAS_REGISTER(Register, a0, x10);
ALIAS_REGISTER(Register, a1, x11);
ALIAS_REGISTER(Register, a2, x12);
ALIAS_REGISTER(Register, a3, x13);
ALIAS_REGISTER(Register, a4, x14);
ALIAS_REGISTER(Register, a5, x15);
ALIAS_REGISTER(Register, a6, x16);
ALIAS_REGISTER(Register, a7, x17);
ALIAS_REGISTER(Register, s2, x18);
ALIAS_REGISTER(Register, s3, x19);
ALIAS_REGISTER(Register, s4, x20);
ALIAS_REGISTER(Register, s5, x21);
ALIAS_REGISTER(Register, s6, x22);
ALIAS_REGISTER(Register, s7, x23);
ALIAS_REGISTER(Register, s8, x24);
ALIAS_REGISTER(Register, s9, x25);
ALIAS_REGISTER(Register, s10, x26);
ALIAS_REGISTER(Register, s11, x27);
ALIAS_REGISTER(Register, t3, x28);
ALIAS_REGISTER(Register, t4, x29);
ALIAS_REGISTER(Register, t5, x30);
ALIAS_REGISTER(Register, t6, x31);

constexpr Register no_reg = Register::no_reg();

constexpr int kNumRegs = 32;

// used as a caller-saved register in JavaScript code callee function
constexpr RegList kJSCallerSaved = Register::ListOf(
    ra, a0, a1, a2, a3, a4, a5, a6, a7, t0, t1, t2, t3, t4, t5, t6);
constexpr int kNumCallerSaved = 16;

constexpr RegList kCalleeSaved =
    Register::ListOf(sp, s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11);
constexpr int kNumCalleeSaved = 13;

// Number of registers for which space is reserved in safepoints.
// to do, redfine  this number
constexpr int kNumSafepointRegisters = 32;

#define DOUBLE_REGISTERS(V) \
  V(f0)                     \
  V(f1)                     \
  V(f2)                     \
  V(f3)                     \
  V(f4)                     \
  V(f5)                     \
  V(f6)                     \
  V(f7)                     \
  V(f8)                     \
  V(f9)                     \
  V(f10)                    \
  V(f11)                    \
  V(f12)                    \
  V(f13)                    \
  V(f14)                    \
  V(f15)                    \
  V(f16)                    \
  V(f17)                    \
  V(f18)                    \
  V(f19)                    \
  V(f20)                    \
  V(f21)                    \
  V(f22)                    \
  V(f23)                    \
  V(f24)                    \
  V(f25)                    \
  V(f26)                    \
  V(f27)                    \
  V(f28)                    \
  V(f29)                    \
  V(f30)                    \
  V(f31)

#define FLOAT_REGISTERS DOUBLE_REGISTERS
#define SIMD128_REGISTERS DOUBLE_REGISTERS

#define ALLOCATABLE_DOUBLE_REGISTERS(V) \
  V(f0)                                 \
  V(f1)                                 \
  V(f2)                                 \
  V(f3)                                 \
  V(f4)                                 \
  V(f5)                                 \
  V(f6)                                 \
  V(f7)                                 \
  V(f8)                                 \
  V(f9)                                 \
  V(f10)                                \
  V(f11)                                \
  V(f12)                                \
  V(f13)                                \
  V(f14)                                \
  V(f15)                                \
  V(f16)                                \
  V(f17)                                \
  V(f18)                                \
  V(f19)                                \
  V(f20)                                \
  V(f21)                                \
  V(f22)                                \
  V(f23)                                \
  V(f24)                                \
  V(f25)                                \
  V(f26)                                \
  V(f27)                                \
  V(f28)                                \
  V(f29)                                \
  V(f30)                                \
  V(f31)

constexpr bool kPadArguments = false;
constexpr bool kSimpleFPAliasing = true;
constexpr bool kSimdMaskRegisters = false;

enum DoubleRegisterCode {
#define REGISTER_CODE(R) kDoubleCode_##R,
  DOUBLE_REGISTERS(REGISTER_CODE)
#undef REGISTER_CODE
      kDoubleAfterLast
};

class FPURegister : public RegisterBase<FPURegister, kDoubleAfterLast> {
 private:
  friend class RegisterBase<FPURegister, kDoubleAfterLast>;
  explicit constexpr FPURegister(int code) : RegisterBase(code) {}
};

ASSERT_TRIVIALLY_COPYABLE(FPURegister);
static_assert(sizeof(FPURegister) == sizeof(int),
              "FPURegister can efficiently be passed by value");

using FloatRegister = FPURegister;

using DoubleRegister = FPURegister;

using Simd128Register = FPURegister;

#define DECLARE_REGISTER(R) \
  constexpr DoubleRegister R = DoubleRegister::from_code(kDoubleCode_##R);
DOUBLE_REGISTERS(DECLARE_REGISTER)
#undef DECLARE_REGISTER

ALIAS_REGISTER(FPURegister, ft0, f0);
ALIAS_REGISTER(FPURegister, ft1, f1);
ALIAS_REGISTER(FPURegister, ft2, f2);
ALIAS_REGISTER(FPURegister, ft3, f3);
ALIAS_REGISTER(FPURegister, ft4, f4);
ALIAS_REGISTER(FPURegister, ft5, f5);
ALIAS_REGISTER(FPURegister, ft6, f6);
ALIAS_REGISTER(FPURegister, ft7, f7);
ALIAS_REGISTER(FPURegister, fs0, f8);
ALIAS_REGISTER(FPURegister, fs1, f9);
ALIAS_REGISTER(FPURegister, fa0, f10);
ALIAS_REGISTER(FPURegister, fa1, f11);
ALIAS_REGISTER(FPURegister, fa2, f12);
ALIAS_REGISTER(FPURegister, fa3, f13);
ALIAS_REGISTER(FPURegister, fa4, f14);
ALIAS_REGISTER(FPURegister, fa5, f15);
ALIAS_REGISTER(FPURegister, fa6, f16);
ALIAS_REGISTER(FPURegister, fa7, f17);
ALIAS_REGISTER(FPURegister, fs2, f18);
ALIAS_REGISTER(FPURegister, fs3, f19);
ALIAS_REGISTER(FPURegister, fs4, f20);
ALIAS_REGISTER(FPURegister, fs5, f21);
ALIAS_REGISTER(FPURegister, fs6, f22);
ALIAS_REGISTER(FPURegister, fs7, f23);
ALIAS_REGISTER(FPURegister, fs8, f24);
ALIAS_REGISTER(FPURegister, fs9, f25);
ALIAS_REGISTER(FPURegister, fs10, f26);
ALIAS_REGISTER(FPURegister, fs11, f27);
ALIAS_REGISTER(FPURegister, ft8, f28);
ALIAS_REGISTER(FPURegister, ft9, f29);
ALIAS_REGISTER(FPURegister, ft10, f30);
ALIAS_REGISTER(FPURegister, ft11, f31);

constexpr DoubleRegister no_dreg = DoubleRegister::no_reg();

// Define {RegisterName} methods for the register types.
DEFINE_REGISTER_NAMES(Register, GENERAL_REGISTERS)
DEFINE_REGISTER_NAMES(FPURegister, DOUBLE_REGISTERS)

FPURegister ToFPURegister(int code);

int ToNumber(FPURegister reg);

constexpr RegList kCallerSavedDoubles =
    Register::ListOf(fa0, fa1, fa2, fa3, fa4, fa5, fa6, fa7, ft0, ft1, ft2, ft3,
                     ft4, ft5, ft6, ft7, ft8, ft9, ft10, ft11);

const int kNumCallerSavedDoubles = 20;

constexpr RegList kCalleeSavedDoubles = Register::ListOf(
    fs0, fs1, fs2, fs3, fs4, fs5, fs6, fs7, fs8, fs9, fs10, fs11);

constexpr int kNumCalleeSavedFPU = 12;

constexpr DoubleRegister kScratchDoubleReg = f30;
constexpr DoubleRegister kDoubleRegZero = f31;

// RISCV has no argument stack slots for a0~a7
constexpr int kCArgSlotCount = 0;

// Give alias names to registers for calling conventions.
constexpr Register kReturnRegister0 = a0;
constexpr Register kReturnRegister1 = a1;
constexpr Register kReturnRegister2 = a2;

// not determinate
constexpr Register kJSFunctionRegister = a1;
// not determinate
constexpr Register kContextRegister = s10;

constexpr Register kAllocateSizeRegister = a1;

// not determinate
constexpr Register kSpeculationPoisonRegister = t3;

constexpr Register kInterpreterAccumulatorRegister = a0;
constexpr Register kInterpreterBytecodeOffsetRegister = t4;
constexpr Register kInterpreterBytecodeArrayRegister = t5;
constexpr Register kInterpreterDispatchTableRegister = t6;

constexpr Register kJavaScriptCallArgCountRegister = a0;
constexpr Register kJavaScriptCallCodeStartRegister = a2;
constexpr Register kJavaScriptCallTargetRegister = kJSFunctionRegister;
constexpr Register kJavaScriptCallNewTargetRegister = a3;
constexpr Register kJavaScriptCallExtraArg1Register = a2;

constexpr Register kRuntimeCallFunctionRegister = a1;
constexpr Register kRuntimeCallArgCountRegister = a0;
constexpr Register kRuntimeCallArgvRegister = a2;

// not determinate
constexpr Register kWasmInstanceRegister = a3;

// Default scratch register used by MacroAssembler (and other code that needs
// a spare register). The register isn't callee save, and not used by the
// function calling convention.

// ScracthRegister should be callee saved, just make it like mips64
constexpr Register kScratchRegister = s3;
constexpr Register kScratchRegister2 = s4;  // just like kScratchRegister
// RootRegister should be callee saved, just make it like mips64
constexpr Register kRootRegister = s6;  // callee save

constexpr Register cp = kContextRegister;
constexpr Register kOffHeapTrampolineRegister = kScratchRegister;
constexpr Register kWasmCompileLazyFuncIndexRegister = x5;
}  // namespace internal
}  // namespace v8

#endif  // V8_CODEGEN_RISCV64_REGISTER_RISCV64_H_
