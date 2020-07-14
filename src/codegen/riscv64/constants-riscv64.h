// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_CODEGEN_RISCV_CONSTANTS_RISCV_H_
#define V8_CODEGEN_RISCV_CONSTANTS_RISCV_H_

#include "src/base/macros.h"
#include "src/common/globals.h"

// Assert that this is an LP64 system, or LLP64 on Windows.
STATIC_ASSERT(sizeof(int) == sizeof(int32_t));
#if 0
#if defined(V8_OS_WIN)
STATIC_ASSERT(sizeof(1L) == sizeof(int32_t));
#else
STATIC_ASSERT(sizeof(long) == sizeof(int64_t));  // NOLINT(runtime/int)
STATIC_ASSERT(sizeof(1L) == sizeof(int64_t));
#endif
STATIC_ASSERT(sizeof(void*) == sizeof(int64_t));
STATIC_ASSERT(sizeof(1) == sizeof(int32_t));
#endif
// Get the standard printf format macros for C99 stdint types.
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

namespace v8 {
namespace internal {

constexpr size_t kMaxPCRelativeCodeRangeInMB = 128;

constexpr uint8_t kInstrSize = 4;
constexpr uint8_t kInstrCompressedSize = 2;
// constexpr uint8_t kLoadLiteralScaleLog2 = 2; //rv64 no need
// constexpr uint8_t kLoadLiteralScale = 1 << kLoadLiteralScaleLog2; //rv64 no
// need constexpr int kMaxLoadLiteralRange = 1 * MB; //rv64 no need

// -----------------------------------------------------------------------------
// Registers and FPURegisters.

// Number of general purpose registers.
const int kNumRegisters = 32;
const int kInvalidRegister = -1;

// Number coprocessor registers.
const int kNumFPURegisters = 32;
const int kInvalidFPURegister = -1;

// Callee saved registers are x2, x8, x9, x18~x27, totally 13 registers
const int kNumberOfCalleeSavedRegisters = 13;
const int kFirstCalleeSavedRegisterIndex = 2;
// Callee saved FP registers are f8, f9, f18~f27, totally 12 registers
const int kNumberOfCalleeSavedVRegisters = 12;
const int kFirstCalleeSavedVRegisterIndex = 2;
// Callee saved registers with no specific purpose in JS are x19-x25.
// const size_t kJSCalleeSavedRegList = 0x03f80000; //qj?

/* qj RV64 has no W* registers
const int kWRegSizeInBits = 32;
const int kWRegSizeInBitsLog2 = 5;
const int kWRegSize = kWRegSizeInBits >> 3;
const int kWRegSizeLog2 = kWRegSizeInBitsLog2 - 3;
*/

const int kXRegSizeInBits = 64;
const int kXRegSizeInBitsLog2 = 6;
const int kXRegSize = kXRegSizeInBits >> 3;
const int kXRegSizeLog2 = kXRegSizeInBitsLog2 - 3;

/* qj RV64 has no S* registers
const int kSRegSizeInBits = 32;
const int kSRegSizeInBitsLog2 = 5;
const int kSRegSize = kSRegSizeInBits >> 3;
const int kSRegSizeLog2 = kSRegSizeInBitsLog2 - 3;
*/
// qj maybe should name as kFReg?
const int kDRegSizeInBits = 64;
const int kDRegSizeInBitsLog2 = 6;
const int kDRegSize = kDRegSizeInBits >> 3;
const int kDRegSizeLog2 = kDRegSizeInBitsLog2 - 3;
const int kDRegSizeInBytesLog2 = kDRegSizeInBitsLog2 - 3;

/*qj RV64 has no B*, H*, Q*, V* registers
const int kBRegSizeInBits = 8;
const int kBRegSize = kBRegSizeInBits >> 3;
const int kHRegSizeInBits = 16;
const int kHRegSize = kHRegSizeInBits >> 3;
const int kQRegSizeInBits = 128;
const int kQRegSizeInBitsLog2 = 7;
const int kQRegSize = kQRegSizeInBits >> 3;
const int kQRegSizeLog2 = kQRegSizeInBitsLog2 - 3;
const int kVRegSizeInBits = kQRegSizeInBits;
const int kVRegSize = kVRegSizeInBits >> 3;
const int64_t kWRegMask = 0x00000000ffffffffL;
const int64_t kXRegMask = 0xffffffffffffffffL;
const int64_t kSRegMask = 0x00000000ffffffffL;
*/

const int64_t kDRegMask = 0xffffffffffffffffL;

// TODO(all) check if the expression below works on all compilers or if it
// triggers an overflow error.
const int64_t kDSignBit = 63;
const int64_t kDSignMask = 0x1LL << kDSignBit;
const int64_t kSSignBit = 31;
const int64_t kSSignMask = 0x1LL << kSSignBit;
const int64_t kXSignBit = 63;
const int64_t kXSignMask = 0x1LL << kXSignBit;
const int64_t kWSignBit = 31;
const int64_t kWSignMask = 0x1LL << kWSignBit;
const int64_t kDQuietNanBit = 51;
const int64_t kDQuietNanMask = 0x1LL << kDQuietNanBit;
const int64_t kSQuietNanBit = 22;
const int64_t kSQuietNanMask = 0x1LL << kSQuietNanBit;
const int64_t kByteMask = 0xffL;
const int64_t kHalfWordMask = 0xffffL;
const int64_t kWordMask = 0xffffffffL;
const uint64_t kXMaxUInt = 0xffffffffffffffffUL;
const uint64_t kWMaxUInt = 0xffffffffUL;
const int64_t kXMaxInt = 0x7fffffffffffffffL;
const int64_t kXMinInt = 0x8000000000000000L;
const int32_t kWMaxInt = 0x7fffffff;
const int32_t kWMinInt = 0x80000000;
/*
//rv64 no
const int kIp0Code = 16;
const int kIp1Code = 17;
*/
const int kFramePointerRegCode = 8;  // x8, Frame Pointer
const int kLinkRegCode = 1;          // x1, Return Address
const int kZeroRegCode = 0;          // x0, Hard-wird Zero
// const int kSPRegInternalCode = 63;//rv64 no
// const unsigned kRegCodeMask = 0x1f;//rv64 no
const unsigned kShiftAmountWRegMask = 0x1f;
const unsigned kShiftAmountXRegMask = 0x3f;

// rv64 no
/*
// Standard machine types defined by AAPCS64.
const unsigned kHalfWordSize = 16;
const unsigned kHalfWordSizeLog2 = 4;
const unsigned kHalfWordSizeInBytes = kHalfWordSize >> 3;
const unsigned kHalfWordSizeInBytesLog2 = kHalfWordSizeLog2 - 3;
const unsigned kWordSize = 32;
const unsigned kWordSizeLog2 = 5;
const unsigned kWordSizeInBytes = kWordSize >> 3;
const unsigned kWordSizeInBytesLog2 = kWordSizeLog2 - 3;
const unsigned kDoubleWordSize = 64;
const unsigned kDoubleWordSizeInBytes = kDoubleWordSize >> 3;
const unsigned kQuadWordSize = 128;
const unsigned kQuadWordSizeInBytes = kQuadWordSize >> 3;
const int kMaxLanesPerVector = 16;
*/
// rv64 hasn't tagged pointer yet
/*
const unsigned kAddressTagOffset = 56;
const unsigned kAddressTagWidth = 8;
const uint64_t kAddressTagMask = ((UINT64_C(1) << kAddressTagWidth) - 1)
                                 << kAddressTagOffset;
static_assert(kAddressTagMask == UINT64_C(0xff00000000000000),
              "AddressTagMask must represent most-significant eight bits.");
*/
// rv64 no
// const uint64_t kTTBRMask = UINT64_C(1) << 55;

// AArch64 floating-point specifics. These match IEEE-754.
const unsigned kDoubleMantissaBits = 52;
const unsigned kDoubleExponentBits = 11;
const unsigned kDoubleExponentBias = 1023;
const unsigned kFloatMantissaBits = 23;
const unsigned kFloatExponentBits = 8;
const unsigned kFloatExponentBias = 127;
// rv64's fp16 is in rvv
/*
const unsigned kFloat16MantissaBits = 10;
const unsigned kFloat16ExponentBits = 5;
const unsigned kFloat16ExponentBias = 15;
*/
// rv64 no? qj: still need for every arch?
// Actual value of root register is offset from the root array's start
// to take advantage of negative displacement values.
// TODO(sigurds): Choose best value.
// TODO(ishell): Choose best value for ptr-compr.
constexpr int kRootRegisterBias = kSystemPointerSize == kTaggedSize ? 256 : 0;
// rv64 no
/*
using float16 = uint16_t;
*/
// rv64 no: not suitable for rv64
#if 0
#define INSTRUCTION_FIELDS_LIST(V_)                     \
  /* Register fields */                                 \
  V_(Rd, 4, 0, Bits)    /* Destination register.     */ \
  V_(Rn, 9, 5, Bits)    /* First source register.    */ \
  V_(Rm, 20, 16, Bits)  /* Second source register.   */ \
  V_(Ra, 14, 10, Bits)  /* Third source register.    */ \
  V_(Rt, 4, 0, Bits)    /* Load dest / store source. */ \
  V_(Rt2, 14, 10, Bits) /* Load second dest /        */ \
                        /* store second source.      */ \
  V_(Rs, 20, 16, Bits)  /* Store-exclusive status    */ \
  V_(PrefetchMode, 4, 0, Bits)                          \
                                                        \
  /* Common bits */                                     \
  V_(SixtyFourBits, 31, 31, Bits)                       \
  V_(FlagsUpdate, 29, 29, Bits)                         \
                                                        \
  /* PC relative addressing */                          \
  V_(ImmPCRelHi, 23, 5, SignedBits)                     \
  V_(ImmPCRelLo, 30, 29, Bits)                          \
                                                        \
  /* Add/subtract/logical shift register */             \
  V_(ShiftDP, 23, 22, Bits)                             \
  V_(ImmDPShift, 15, 10, Bits)                          \
                                                        \
  /* Add/subtract immediate */                          \
  V_(ImmAddSub, 21, 10, Bits)                           \
  V_(ShiftAddSub, 23, 22, Bits)                         \
                                                        \
  /* Add/subtract extend */                             \
  V_(ImmExtendShift, 12, 10, Bits)                      \
  V_(ExtendMode, 15, 13, Bits)                          \
                                                        \
  /* Move wide */                                       \
  V_(ImmMoveWide, 20, 5, Bits)                          \
  V_(ShiftMoveWide, 22, 21, Bits)                       \
                                                        \
  /* Logical immediate, bitfield and extract */         \
  V_(BitN, 22, 22, Bits)                                \
  V_(ImmRotate, 21, 16, Bits)                           \
  V_(ImmSetBits, 15, 10, Bits)                          \
  V_(ImmR, 21, 16, Bits)                                \
  V_(ImmS, 15, 10, Bits)                                \
                                                        \
  /* Test and branch immediate */                       \
  V_(ImmTestBranch, 18, 5, SignedBits)                  \
  V_(ImmTestBranchBit40, 23, 19, Bits)                  \
  V_(ImmTestBranchBit5, 31, 31, Bits)                   \
                                                        \
  /* Conditionals */                                    \
  V_(Condition, 15, 12, Bits)                           \
  V_(ConditionBranch, 3, 0, Bits)                       \
  V_(Nzcv, 3, 0, Bits)                                  \
  V_(ImmCondCmp, 20, 16, Bits)                          \
  V_(ImmCondBranch, 23, 5, SignedBits)                  \
                                                        \
  /* Floating point */                                  \
  V_(FPType, 23, 22, Bits)                              \
  V_(ImmFP, 20, 13, Bits)                               \
  V_(FPScale, 15, 10, Bits)                             \
                                                        \
  /* Load Store */                                      \
  V_(ImmLS, 20, 12, SignedBits)                         \
  V_(ImmLSUnsigned, 21, 10, Bits)                       \
  V_(ImmLSPair, 21, 15, SignedBits)                     \
  V_(ImmShiftLS, 12, 12, Bits)                          \
  V_(LSOpc, 23, 22, Bits)                               \
  V_(LSVector, 26, 26, Bits)                            \
  V_(LSSize, 31, 30, Bits)                              \
                                                        \
  /* NEON generic fields */                             \
  V_(NEONQ, 30, 30, Bits)                               \
  V_(NEONSize, 23, 22, Bits)                            \
  V_(NEONLSSize, 11, 10, Bits)                          \
  V_(NEONS, 12, 12, Bits)                               \
  V_(NEONL, 21, 21, Bits)                               \
  V_(NEONM, 20, 20, Bits)                               \
  V_(NEONH, 11, 11, Bits)                               \
  V_(ImmNEONExt, 14, 11, Bits)                          \
  V_(ImmNEON5, 20, 16, Bits)                            \
  V_(ImmNEON4, 14, 11, Bits)                            \
                                                        \
  /* Other immediates */                                \
  V_(ImmUncondBranch, 25, 0, SignedBits)                \
  V_(ImmCmpBranch, 23, 5, SignedBits)                   \
  V_(ImmLLiteral, 23, 5, SignedBits)                    \
  V_(ImmException, 20, 5, Bits)                         \
  V_(ImmHint, 11, 5, Bits)                              \
  V_(ImmBarrierDomain, 11, 10, Bits)                    \
  V_(ImmBarrierType, 9, 8, Bits)                        \
                                                        \
  /* System (MRS, MSR) */                               \
  V_(ImmSystemRegister, 19, 5, Bits)                    \
  V_(SysO0, 19, 19, Bits)                               \
  V_(SysOp1, 18, 16, Bits)                              \
  V_(SysOp2, 7, 5, Bits)                                \
  V_(CRn, 15, 12, Bits)                                 \
  V_(CRm, 11, 8, Bits)                                  \
                                                        \
  /* Load-/store-exclusive */                           \
  V_(LoadStoreXLoad, 22, 22, Bits)                      \
  V_(LoadStoreXNotExclusive, 23, 23, Bits)              \
  V_(LoadStoreXAcquireRelease, 15, 15, Bits)            \
  V_(LoadStoreXSizeLog2, 31, 30, Bits)                  \
  V_(LoadStoreXPair, 21, 21, Bits)                      \
                                                        \
  /* NEON load/store */                                 \
  V_(NEONLoad, 22, 22, Bits)                            \
                                                        \
  /* NEON Modified Immediate fields */                  \
  V_(ImmNEONabc, 18, 16, Bits)                          \
  V_(ImmNEONdefgh, 9, 5, Bits)                          \
  V_(NEONModImmOp, 29, 29, Bits)                        \
  V_(NEONCmode, 15, 12, Bits)                           \
                                                        \
  /* NEON Shift Immediate fields */                     \
  V_(ImmNEONImmhImmb, 22, 16, Bits)                     \
  V_(ImmNEONImmh, 22, 19, Bits)                         \
  V_(ImmNEONImmb, 18, 16, Bits)

#define SYSTEM_REGISTER_FIELDS_LIST(V_, M_) \
  /* NZCV */                                \
  V_(Flags, 31, 28, Bits, uint32_t)         \
  V_(N, 31, 31, Bits, bool)                 \
  V_(Z, 30, 30, Bits, bool)                 \
  V_(C, 29, 29, Bits, bool)                 \
  V_(V, 28, 28, Bits, bool)                 \
  M_(NZCV, Flags_mask)                      \
                                            \
  /* FPCR */                                \
  V_(AHP, 26, 26, Bits, bool)               \
  V_(DN, 25, 25, Bits, bool)                \
  V_(FZ, 24, 24, Bits, bool)                \
  V_(RMode, 23, 22, Bits, FPRounding)       \
  M_(FPCR, AHP_mask | DN_mask | FZ_mask | RMode_mask)

// rv64 cjy's code
// Fields offsets.
#define DECLARE_FIELDS_OFFSETS(Name, HighBit, LowBit, unused_1, unused_2) \
  const int Name##_offset = LowBit;                                       \
  const int Name##_width = HighBit - LowBit + 1;                          \
  const uint32_t Name##_mask = ((1 << Name##_width) - 1) << LowBit;

#define DECLARE_INSTRUCTION_FIELDS_OFFSETS(Name, HighBit, LowBit, unused_1) \
  DECLARE_FIELDS_OFFSETS(Name, HighBit, LowBit, unused_1, unused_2)

// rv64:qj we need define this LIST? or define directly?
INSTRUCTION_FIELDS_LIST(DECLARE_INSTRUCTION_FIELDS_OFFSETS)

// rv64 no need
// SYSTEM_REGISTER_FIELDS_LIST(DECLARE_FIELDS_OFFSETS, NOTHING)

#undef DECLARE_FIELDS_OFFSETS
#undef DECLARE_INSTRUCTION_FIELDS_OFFSETS

#endif

// ImmPCRel is a compound field (not present in INSTRUCTION_FIELDS_LIST), formed
// from ImmPCRelLo and ImmPCRelHi.
// rv64: no
// const int ImmPCRel_mask = ImmPCRelLo_mask | ImmPCRelHi_mask;

// Condition codes.
enum Condition {
  kNoCondition = -1,
  eq = 0,
  ne = 1,
  hs = 2,
  cs = hs,
  lo = 3,
  cc = lo,
  mi = 4,
  pl = 5,
  vs = 6,
  vc = 7,
  hi = 8,
  ls = 9,
  ge = 10,
  lt = 11,
  gt = 12,
  le = 13,
  al = 14,
  nv = 15  // Behaves as always/al.
};

/* qiuji review start
enum Condition {
  eq = 0,  //beq
  ne = 1,  //bne
  lt = 2,  //blt, signed lower than
  ge = 3,  //bge, signed greater than or equal
  ltu = 4, //bltu, unsigned lower than
  geu = 5, //bgeu, unsigned greater than or equal
  always = 6, //always true
  //alias
  zero = eq,  //beq zero
  not_zero = ne,  //bne zero
  negative = lt,  //blt zero
  positive = lt,  //blt zero
  not_positive = ge,  //blez zero
  not_negative = ge,  //bgez
  gt = lt,  //blt
  le = ge,  //bge
  gtu = ltu,  //bltu
  leu = geu,  //begu
}
*/

// ----- Fields offset and length.
const int kOpcodeShift = 0;
const int kOpcodeBits = 7;
const int kRdBits = 5;
const int kRdShift = 7;
const int kRs1Bits = 5;
const int kRs1Shift = 15;
const int kRs2Bits = 5;
const int kRs2Shift = 20;
const int kRs3Bits = 5;
const int kRs3Shift = 27;
const int kImmIShift = 20;
const int kImmIsShift = 26;  // shift? slli/srli/srai ?
const int kImmS1Shift = 25;
const int kImmS2Shift = 7;
const int kImmB1Shift = 31;
const int kImmB2Shift = 25;
const int kImmB3Shift = 8;
const int kImmB4Shift = 7;
const int kImmUShift = 12;
const int kImmJ1Shift = 31;
const int kImmJ2Shift = 21;
const int kImmJ3Shift = 20;
const int kImmJ4Shift = 12;
const int kFunct2Shift = 25;
const int kFunct3Shift = 12;
const int kFunct6Shift = 26;
const int kFunct7Shift = 25;
const int kImm0Shift = 0;
const int kImm1Shift = 1;
const int kImm2Shift = 2;
const int kImm3Shift = 3;
const int kImm4Shift = 4;
const int kImm5Shift = 5;
const int kImm6Shift = 6;
const int kImm7Shift = 7;
const int kImm8Shift = 8;
const int kImm9Shift = 9;
const int kImm10Shift = 10;
const int kImm11Shift = 11;
const int kImm12Shift = 12;
const int kImm15Shift = 15;
const int kImm19Shift = 19;
const int kImm20Shift = 20;
const int kImm21Shift = 21;
const int kImm24Shift = 24;
const int kImm25Shift = 25;
const int kImm26Shift = 26;
const int kImm30Shift = 30;
const int kImm31Shift = 31;
const int kR_A5Shift = 5;
const int kR_A25Shift = 25;
const int kR_A26Shift = 26;
const int kR_A27Shift = 27;
const int Fence3Shift = 3;
const int Fence4Shift = 4;
const int Fence5Shift = 5;
const int Fence27Shift = 27;
const int Fence28Shift = 28;
const int Fence24Shift = 24;
const int Fence20Shift = 20;
const int kOpcodeMask = ((1 << kOpcodeBits) - 1) << kOpcodeShift;
const int kUnCompressedMask = 3;
const int kRdFieldMask = ((1 << kRdBits) - 1) << kRdShift;
const int kRs1FieldMask = ((1 << kRs1Bits) - 1) << kRs1Shift;
const int kImm3_0Mask = (1 << kImm4Shift) - 1;
const int kImm4_0Mask = (1 << kImm5Shift) - 1;
const int kImm5_0Mask = (1 << kImm6Shift) - 1;
const int kImm8_0Mask = (1 << kImm9Shift) - 1;
const int kImm9_0Mask = (1 << kImm10Shift) - 1;
const int kImm9_4Mask = ((1 << kImm6Shift) - 1) << kImm4Shift;
const int kImm10Mask = (1 << 10);
const int kImm10_1Mask = ((1 << kImm10Shift) - 1) << kImm1Shift;
const int kImm11Mask = (1 << 11);
const int kImm11_0Mask = ((1 << kImm12Shift) - 1);
const int kImm11_5Mask = ((1 << kImm7Shift) - 1) << kImm5Shift;
const int kImm11_8Mask = ((1 << kImm4Shift) - 1) << kImm8Shift;
const int kImm14_12Mask = ((1 << kImm3Shift) - 1) << kImm12Shift;
const int kImm18_11Mask = ((1 << kImm8Shift) - 1) << kImm11Shift;
const int kImm19Mask = 1 << 19;
const int kImm19_0Mask = ((1 << kImm20Shift) - 1);
const int kImm19_12Mask = ((1 << kImm8Shift) - 1) << kImm12Shift;
const int kImm19_15Mask = ((1 << kImm5Shift) - 1) << kImm15Shift;
const int kImm20Mask = 1 << 20;
const int kImm20_0Mask = (1 << kImm20Shift) - 1;
const int kImm23_0Mask = (1 << kImm24Shift) - 1;
const int kImm24_20Mask = ((1 << kImm5Shift) - 1) << kImm20Shift;
const int kImm25_20Mask = ((1 << kImm6Shift) - 1) << kImm20Shift;
const int kImm26_25Mask = ((1 << kImm2Shift) - 1) << kImm25Shift;
const int kImm30_25Mask = ((1 << kImm6Shift) - 1) << kImm25Shift;
const int kImm31Mask = 1 << 31;
const int kImm31_12Mask = ((1 << kImm20Shift) - 1) << kImm12Shift;
const int kImm31_20Mask = ((1 << kImm12Shift) - 1) << kImm20Shift;
const int kImm31_25Mask = ((1 << kImm7Shift) - 1) << kImm25Shift;
const int kImm31_26Mask = ((1 << kImm6Shift) - 1) << kImm26Shift;
const int kBInstr7Mask = 1 << kImm7Shift;
const int kBInstr11_8Mask = ((1 << kImm4Shift) - 1) << kImm8Shift;
const int kBInstr30_25Mask = ((1 << kImm6Shift) - 1) << kImm25Shift;
const int R_A31_27Mask = ((1 << kR_A5Shift) - 1) << kR_A27Shift;
const int R_A25_25Mask = 1 << kR_A25Shift;
const int R_A26_26Mask = 1 << kR_A26Shift;
const int Fence31_27Mask = ((1 << Fence5Shift) - 1) << Fence27Shift;
const int Fence31_28Mask = ((1 << Fence4Shift) - 1) << Fence28Shift;
const int Fence27_24Mask = ((1 << Fence4Shift) - 1) << Fence24Shift;
const int Fence26_24Mask = ((1 << Fence3Shift) - 1) << Fence24Shift;
const int Fence23_20Mask = ((1 << Fence4Shift) - 1) << Fence20Shift;

// FCSR constants.
const uint32_t kFCSRInvalidOpFlagBit = 4;
const uint32_t kFCSRFlagDividebyZeroBit = 3;
const uint32_t kFCSROverflowFlagBit = 2;
const uint32_t kFCSRUnderflowFlagBit = 1;
const uint32_t kFCSRInexactFlagBit = 0;

const uint32_t kFCSRInvalidOpFlagMask = 1 << kFCSRInvalidOpFlagBit;
const uint32_t kFCSRFlagDividebyZeroMask = 1 << kFCSRFlagDividebyZeroBit;
const uint32_t kFCSROverflowFlagMask = 1 << kFCSROverflowFlagBit;
const uint32_t kFCSRUnderflowFlagMask = 1 << kFCSRUnderflowFlagBit;
const uint32_t kFCSRInexactFlagMask = 1 << kFCSRInexactFlagBit;

// postFix encode
const uint32_t kInstrPostFixS = 0;
const uint32_t kInstrPostFixD = 1;
const uint32_t kInstrPostFixQ = 3;

using Instr = int32_t;

enum CSRRegister : uint32_t {
  // Floating-Point Control and Status Registers
  fflags = 0x001,
  frm = 0x002,
  fcsr = 0x003,
  // Counters and Timers
  cycle = 0xC00,
  time = 0xC01,
  instret = 0xC02,
  cycleh = 0xC80,
  timeh = 0xC81,
  instreth = 0xC82
};

enum Opcode : uint32_t {
  // RV32I Base Instruction Set
  LUI = 0x37,
  AUIPC = 0x17,
  JAL = 0x6f,
  JALR = 0x67,
  // qj branch group
  BEQ = 0x63,
  BNE = 0x63,
  BLT = 0x63,
  BGE = 0x63,
  BLTU = 0x63,
  BGEU = 0x63,
  // qj load group
  LB = 0x03,
  LH = 0x03,
  LW = 0x03,
  LBU = 0x03,
  LHU = 0x03,
  // qj store group
  SB = 0x23,
  SH = 0x23,
  SW = 0x23,
  // imm arith
  ADDI = 0x13,
  SLTI = 0x13,
  SLTIU = 0x13,
  XORI = 0x13,
  ORI = 0x13,
  ANDI = 0x13,
  // imm shift 64bit
  SLLI = 0x13,
  SRLI = 0x13,
  SRAI = 0x13,
  // reg arith
  ADD = 0x33,
  SUB = 0x33,
  SLL = 0x33,
  SLT = 0x33,
  SLTU = 0x33,
  XOR = 0x33,
  SRL = 0x33,
  SRA = 0x33,
  OR = 0x33,
  AND = 0x33,
  // fence
  FENCE = 0x0f,
  // fence.i ?
  ECALL = 0x73,
  EBREAK = 0x73,
  // RV64I Base Instruction Set (in addition to RV32I)
  LWU = 0x03,
  LD = 0x03,
  SD = 0x23,
  SLLI64 = 0x13,
  SRLI64 = 0x13,
  SRAI64 = 0x13,
  ADDIW = 0x1b,
  SLLIW = 0x1b,
  SRLIW = 0x1b,
  SRAIW = 0x1b,
  ADDW = 0x3b,
  SUBW = 0x3b,
  SLLW = 0x3b,
  SRLW = 0x3b,
  SRAW = 0x3b,
  //  RV32/RV64 Zifencei Standard Extension
  FENCEi = 0x0f,
  //  RV32/RV64 Zicsr Standard Extension
  CSRRW = 0x73,
  CSRRS = 0x73,
  CSRRC = 0x73,
  CSRRWI = 0x73,
  CSRRSI = 0x73,
  CSRRCI = 0x73,
  // RV32M Standard Extension
  MUL = 0x33,
  MULH = 0x33,
  MULHSU = 0x33,
  MULHU = 0x33,
  DIV = 0x33,
  DIVU = 0x33,
  REM = 0x33,
  REMU = 0x33,
  // RV64M Standard Extension (in addition to RV32M)
  MULW = 0x3b,
  DIVW = 0x3b,
  DIVUW = 0x3b,
  REMW = 0x3b,
  REMUW = 0x3b,
  // RV32A Standard Extension
  LRW = 0x2f,
  SCW = 0x2f,
  AMOSWAPW = 0x2f,
  AMOADDW = 0x2f,
  AMOXORW = 0x2f,
  AMOANDW = 0x2f,
  AMOORW = 0x2f,
  AMOMINW = 0x2f,
  AMOMAXW = 0x2f,
  AMOMINUW = 0x2f,
  AMOMAXUW = 0x2f,
  // RV64A Standard Extension (in addition to RV32A)
  LRD = 0x2f,
  SCD = 0x2f,
  AMOSWAPD = 0x2f,
  AMOADDD = 0x2f,
  AMOXORD = 0x2f,
  AMOANDD = 0x2f,
  AMOORD = 0x2f,
  AMOMIND = 0x2f,
  AMOMAXD = 0x2f,
  AMOMINUD = 0x2f,
  AMOMAXUD = 0x2f,
  // RV32F Standard Extension
  FLW = 0x07,
  FSW = 0x27,
  FMADDS = 0x43,
  FMSUBS = 0x47,
  FNMSUBS = 0x4b,
  FNMADDS = 0x4f,
  FADDS = 0x53,
  FSUBS = 0x53,
  FMULS = 0x53,
  FDIVS = 0x53,
  FSQRTS = 0x53,
  FSGNJS = 0x53,
  FSGNJNS = 0x53,
  FSGNJXS = 0x53,
  FMINS = 0x53,
  FMAXS = 0x53,
  FCVTWS = 0x53,
  FCVTWUS = 0x53,
  FMVXW = 0x53,
  FEQS = 0x53,
  FLTS = 0x53,
  FLES = 0x53,
  FCLASSS = 0x53,
  FCVTSW = 0x53,
  FCVTSWU = 0x53,
  FMVWX = 0x53,
  // RV64F Standard Extension (in addition to RV32F)
  FCVTLS = 0x53,
  FCVTLUS = 0x53,
  FCVTSL = 0x53,
  FCVTSLU = 0x53,
  // RV32D Standard Extension
  FLD = 0x07,
  FSD = 0x27,
  FMADDD = 0x43,
  FMSUBD = 0x47,
  FNMSUBD = 0x4b,
  FNMADDD = 0x4f,
  FADDD = 0x53,
  FSUBD = 0x53,
  FMULD = 0x53,
  FDIVD = 0x53,
  FSQRTD = 0x53,
  FSGNJD = 0x53,
  FSGNJND = 0x53,
  FSGNJXD = 0x53,
  FMIND = 0x53,
  FMAXD = 0x53,
  FCVTSD = 0x53,
  FCVTDS = 0x53,
  FEQD = 0x53,
  FLTD = 0x53,
  FLED = 0x53,
  FCLASSD = 0x53,
  FCVTWD = 0x53,
  FCVTWUD = 0x53,
  FCVTDW = 0x53,
  FCVTDWU = 0x53,
  // RV64D Standard Extension (in addition to RV32D)
  FCVTLD = 0x53,
  FCVTLUD = 0x53,
  FMVXD = 0x53,
  FCVTDL = 0x53,
  FCVTDLU = 0x53,
  FMVDX = 0x53,
// rv64G need not Q ext
#if 0
  // RV32Q Standard Extension
  FLQ = 0x07,
  FSQ = 0x27,
  FMADDQ = 0x43,
  FMSUBQ = 0x47,
  FNMSUBQ = 0x4b,
  FNMADDQ = 0x4f,
  FADDQ = 0x53,
  FSUBQ = 0x53,
  FMULQ = 0x53,
  FDIVQ = 0x53,
  FSQRTQ = 0x53,
  FSGNJQ = 0x53,
  FSGNJNQ = 0x53,
  FSGNJXQ = 0x53,
  FMINQ = 0x53,
  FMAXQ = 0x53,
  FCVTSQ = 0x53,
  FCVTQS = 0x53,
  FCVTDQ = 0x53,
  FCVTQD = 0x53,
  FEQQ = 0x53,
  FLTQ = 0x53,
  FLEQ = 0x53,
  FCLASSQ = 0x53,
  FCVTWQ = 0x53,
  FCVTWUQ = 0x53,
  FCVTQW = 0x53,
  FCVTQWU = 0x53,
  // RV64Q Standard Extension (in addition to RV32Q)
  FCVTLQ = 0x53,
  FCVTLUQ = 0x53,
  FCVTQL = 0x53,
  FCVTQLU = 0x53,
#endif
};

enum Funct5 : uint32_t {
  // The standard atomic-instruction extension
  LR_F5 = 0x02,
  SC_F5 = 0x03,
  AMOSWAP_F5 = 0x01,
  AMOADD_F5 = 0x00,
  AMOXOR_F5 = 0x04,
  AMOAND_F5 = 0x0c,
  AMOOR_F5 = 0x08,
  AMOMIN_F5 = 0x10,
  AMOMAX_F5 = 0x14,
  AMOMINU_F5 = 0x18,
  AMOMAXU_F5 = 0x1c,
};

enum Funct6 : uint32_t {
  //  SPECIAL Encoding of Function Field.
  // RV64I Base Instruction Set
  SLLI64_F6 = ((0U << 4) + 0),
  SRLI64_F6 = ((0U << 4) + 0),
  SRAI64_F6 = ((1U << 4) + 0),
};

enum Funct7 : uint32_t {
  //  SPECIAL Encoding of Function Field.
  // RV32I Base Instruction Set
  SLLI_F7 = ((0U << 3) + 0),
  SRLI_F7 = ((0U << 3) + 0),
  SRAI_F7 = ((4U << 3) + 0),
  ADD_F7 = ((0U << 3) + 0),
  SUB_F7 = ((4U << 3) + 0),
  SLL_F7 = ((0U << 3) + 0),
  SLT_F7 = ((0U << 3) + 0),
  SLTU_F7 = ((0U << 3) + 0),
  XOR_F7 = ((0U << 3) + 0),
  SRL_F7 = ((0U << 3) + 0),
  SRA_F7 = ((4U << 3) + 0),
  OR_F7 = ((0U << 3) + 0),
  AND_F7 = ((0U << 3) + 0),
  // RV64I Base Instruction Set
  SLLIW_F7 = ((0U << 3) + 0),
  SRLIW_F7 = ((0U << 3) + 0),
  SRAIW_F7 = ((4U << 3) + 0),
  ADDW_F7 = ((0U << 3) + 0),
  SUBW_F7 = ((4U << 3) + 0),
  SLLW_F7 = ((0U << 3) + 0),
  SRLW_F7 = ((0U << 3) + 0),
  SRAW_F7 = ((4U << 3) + 0),
  // RV32M Standard Extension
  MUL_F7 = ((0U << 3) + 1),
  MULH_F7 = ((0U << 3) + 1),
  MULHSU_F7 = ((0U << 3) + 1),
  MULHU_F7 = ((0U << 3) + 1),
  DIV_F7 = ((0U << 3) + 1),
  DIVU_F7 = ((0U << 3) + 1),
  REM_F7 = ((0U << 3) + 1),
  REMU_F7 = ((0U << 3) + 1),
  // RV64M Standard Extension (in addition to RV32M)
  MULW_F7 = ((0U << 3) + 1),
  DIVW_F7 = ((0U << 3) + 1),
  DIVUW_F7 = ((0U << 3) + 1),
  REMW_F7 = ((0U << 3) + 1),
  REMUW_F7 = ((0U << 3) + 1),
  // RV32F Standard Extension
  FADDS_F7 = ((0U << 3) + 0),
  FSUBS_F7 = ((0U << 3) + 4),
  FMULS_F7 = ((1U << 3) + 0),
  FDIVS_F7 = ((1U << 3) + 4),
  FSQRTS_F7 = ((5U << 3) + 4),
  FSGNJS_F7 = ((2U << 3) + 0),
  FSGNJNS_F7 = ((2U << 3) + 0),
  FSGNJXS_F7 = ((2U << 3) + 0),
  FMINS_F7 = ((2U << 3) + 4),
  FMAXS_F7 = ((2U << 3) + 4),
  FCVTWS_F7 = ((12U << 3) + 0),
  FCVTWUS_F7 = ((12U << 3) + 0),
  FMVXW_F7 = ((14U << 3) + 0),
  FEQS_F7 = ((10U << 3) + 0),
  FLTS_F7 = ((10U << 3) + 0),
  FLES_F7 = ((10U << 3) + 0),
  FCLASSS_F7 = ((14U << 3) + 0),
  FCVTSW_F7 = ((13U << 3) + 0),
  FCVTSWU_F7 = ((13U << 3) + 0),
  FMVWX_F7 = ((15U << 3) + 0),
  // RV64F Satndard Extension
  FCVTLS_F7 = ((12U << 3) + 0),
  FCVTLUS_F7 = ((12U << 3) + 0),
  FCVTSL_F7 = ((13U << 3) + 0),
  FCVTSLU_F7 = ((13U << 3) + 0),
  // RV32D Standard Extension
  FADDD_F7 = ((0U << 3) + 1),
  FSUBD_F7 = ((0U << 3) + 3),
  FMULD_F7 = ((1U << 3) + 1),
  FDIVD_F7 = ((1U << 3) + 5),
  FSQRTD_F7 = ((5U << 3) + 5),
  FSGNJD_F7 = ((2U << 3) + 1),
  FSGNJND_F7 = ((2U << 3) + 1),
  FSGNJXD_F7 = ((2U << 3) + 1),
  FMIND_F7 = ((2U << 3) + 5),
  FMAXD_F7 = ((2U << 3) + 5),
  FCVTSD_F7 = ((4U << 3) + 0),
  FCVTDS_F7 = ((4U << 3) + 1),
  FEQD_F7 = ((10U << 3) + 1),
  FLTD_F7 = ((10U << 3) + 1),
  FLED_F7 = ((10U << 3) + 1),
  FCLASSD_F7 = ((14U << 3) + 1),
  FCVTWD_F7 = ((12U << 3) + 1),
  FCVTWUD_F7 = ((12U << 3) + 1),
  FCVTDW_F7 = ((13U << 3) + 1),
  FCVTDWU_F7 = ((13U << 3) + 1),
  // RV64D Standard Extension
  FCVTLD_F7 = ((12U << 3) + 1),
  FCVTLUD_F7 = ((12U << 3) + 1),
  FMVXD_F7 = ((14U << 3) + 1),
  FCVTDL_F7 = ((13U << 3) + 1),
  FCVTDLU_F7 = ((13U << 3) + 1),
  FMVDX_F7 = ((15U << 3) + 1),
  // RV32Q Standard Extension
  FADDQ_F7 = ((0U << 3) + 3),
  FSUBQ_F7 = ((0U << 3) + 7),
  FMULQ_F7 = ((1U << 3) + 3),
  FDIVQ_F7 = ((1U << 3) + 7),
  FSQRTQ_F7 = ((5U << 3) + 7),
  FSGNJQ_F7 = ((2U << 3) + 3),
  FSGNJNQ_F7 = ((2U << 3) + 3),
  FSGNJXQ_F7 = ((2U << 3) + 3),
  FMINQ_F7 = ((2U << 3) + 7),
  FMAXQ_F7 = ((2U << 3) + 7),
  FCVTSQ_F7 = ((4U << 3) + 0),
  FCVTQS_F7 = ((4U << 3) + 3),
  FCVTDQ_F7 = ((4U << 3) + 1),
  FCVTQD_F7 = ((4U << 3) + 3),
  FEQQ_F7 = ((10U << 3) + 3),
  FLTQ_F7 = ((10U << 3) + 3),
  FLEQ_F7 = ((10U << 3) + 3),
  FCLASSQ_F7 = ((14U << 3) + 3),
  FCVTWQ_F7 = ((12U << 3) + 3),
  FCVTWUQ_F7 = ((12U << 3) + 3),
  FCVTQW_F7 = ((13U << 3) + 3),
  FCVTQWU_F7 = ((13U << 3) + 3),
  // RV64Q Standard Extension
  FCVTLQ_F7 = ((12U << 3) + 3),
  FCVTLUQ_F7 = ((12U << 3) + 3),
  FCVTQL_F7 = ((13U << 3) + 3),
  FCVTQLU_F7 = ((13U << 3) + 3)
};

enum Funct3 : uint32_t {
  // RV32I Base Instruction Set
  JALR_F3 = 0,
  BEQ_F3 = 0,
  BNE_F3 = 1,
  BLT_F3 = 4,
  BGE_F3 = 5,
  BLTU_F3 = 6,
  BGEU_F3 = 7,
  LB_F3 = 0,
  LH_F3 = 1,
  LW_F3 = 2,
  LBU_F3 = 4,
  LHU_F3 = 5,
  SB_F3 = 0,
  SH_F3 = 1,
  SW_F3 = 2,
  ADDI_F3 = 0,
  SLTI_F3 = 2,
  SLTIU_F3 = 3,
  XORI_F3 = 4,
  ORI_F3 = 6,
  ANDI_F3 = 7,
  SLLI_F3 = 1,
  SRLI_F3 = 5,
  SRAI_F3 = 5,
  ADD_F3 = 0,
  SUB_F3 = 0,
  SLL_F3 = 1,
  SLT_F3 = 2,
  SLTU_F3 = 3,
  XOR_F3 = 4,
  SRL_F3 = 5,
  SRA_F3 = 5,
  OR_F3 = 6,
  AND_F3 = 7,
  FENCE_F3 = 0,
  ECALL_F3 = 0,
  EBREAK_F3 = 0,
  // RV64I Base Instruction Set (in addition to RV32I)
  LWU_F3 = 6,
  LD_F3 = 3,
  SD_F3 = 3,
  SLLI64_F3 = 1,
  SRLI64_F3 = 5,
  SRAI64_F3 = 5,
  ADDIW_F3 = 0,
  SLLIW_F3 = 1,
  SRLIW_F3 = 5,
  SRAIW_F3 = 5,
  ADDW_F3 = 0,
  SUBW_F3 = 0,
  SLLW_F3 = 1,
  SRLW_F3 = 5,
  SRAW_F3 = 5,
  //  RV32/RV64 Zifencei Standard Extension
  FENCEi_F3 = 1,
  //  RV32/RV64 Zicsr Standard Extension
  CSRRW_F3 = 1,
  CSRRS_F3 = 2,
  CSRRC_F3 = 3,
  CSRRWI_F3 = 5,
  CSRRSI_F3 = 6,
  CSRRCI_F3 = 7,
  // RV32M Standard Extension
  MUL_F3 = 0,
  MULH_F3 = 1,
  MULHSU_F3 = 2,
  MULHU_F3 = 3,
  DIV_F3 = 4,
  DIVU_F3 = 5,
  REM_F3 = 6,
  REMU_F3 = 7,
  // RV64M Standard Extension (in addition to RV32M)
  MULW_F3 = 0,
  DIVW_F3 = 4,
  DIVUW_F3 = 5,
  REMW_F3 = 6,
  REMUW_F3 = 7,
  // RV32A Standard Extension
  LRW_F3 = 2,
  SCW_F3 = 2,
  AMOSWAPW_F3 = 2,
  AMOADDW_F3 = 2,
  AMOXORW_F3 = 2,
  AMOANDW_F3 = 2,
  AMOORW_F3 = 2,
  AMOMINW_F3 = 2,
  AMOMAXW_F3 = 2,
  AMOMINUW_F3 = 2,
  AMOMAXUW_F3 = 2,
  // RV64A Standard Extension (in addition to RV32A)
  LRD_F3 = 3,
  SCD_F3 = 3,
  AMOSWAPD_F3 = 3,
  AMOADDD_F3 = 3,
  AMOXORD_F3 = 3,
  AMOANDD_F3 = 3,
  AMOORD_F3 = 3,
  AMOMIND_F3 = 3,
  AMOMAXD_F3 = 3,
  AMOMINUD_F3 = 3,
  AMOMAXUD_F3 = 3,
  // RV32F Standard Extension
  FLW_F3 = 2,
  FSW_F3 = 2,
  FSGNJS_F3 = 0,
  FSGNJNS_F3 = 1,
  FSGNJXS_F3 = 2,
  FMINS_F3 = 0,
  FMAXS_F3 = 1,
  FMVXW_F3 = 0,
  FEQS_F3 = 2,
  FLTS_F3 = 1,
  FLES_F3 = 0,
  FCLASSS_F3 = 1,
  FMVWX_F3 = 0,
  // RV32D Standard Extension
  FLD_F3 = 3,
  FSD_F3 = 3,
  FSGNJD_F3 = 0,
  FSGNJND_F3 = 1,
  FSGNJXD_F3 = 2,
  FMIND_F3 = 0,
  FMAXD_F3 = 1,
  FEQD_F3 = 2,
  FLTD_F3 = 1,
  FLED_F3 = 0,
  FCLASSD_F3 = 1,
  // RV64D Standard Extension (in addition to RV32D)
  FMVXD_F3 = 0,
  FMVDX_F3 = 0,
  // RV32Q Standard Extension
  FLQ_F3 = 4,
  FSQ_F3 = 4,
  FSGNJQ_F3 = 0,
  FSGNJNQ_F3 = 1,
  FSGNJXQ_F3 = 2,
  FMINQ_F3 = 0,
  FMAXQ_F3 = 1,
  FEQQ_F3 = 2,
  FLTQ_F3 = 1,
  FLEQ_F3 = 0,
  FCLASSQ_F3 = 1,
};

enum Funct2 : uint32_t {
  // RV32F Standard Extension
  FMADDS_F2 = 0,
  FMSUBS_F2 = 0,
  FNMSUBS_F2 = 0,
  FNMADDS_F2 = 0,
  // RV32D Standard Extension
  // qiuji need check
  FMADDD_F2 = 1,
  FMSUBD_F2 = 1,
  FNMSUBD_F2 = 1,
  FNMADDD_F2 = 1,
  // RV32Q Standard Extension
  FMADDQ_F2 = 3,
  FMSUBQ_F2 = 3,
  FNMSUBQ_F2 = 3,
  FNMADDQ_F2 = 3,

};

// qj review end
inline Condition NegateCondition(Condition cond) {
  // qj here use "al" first
  DCHECK(cond != al);
  return static_cast<Condition>(cond ^ 1);
}

}  // namespace internal
}  // namespace v8

#endif  // V8_CODEGEN_RISCV_CONSTANTS_RISCV_H_
