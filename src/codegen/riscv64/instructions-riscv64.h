// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_CODEGEN_RISCV64_INSTRUCTIONS_RISCV64_H_
#define V8_CODEGEN_RISCV64_INSTRUCTIONS_RISCV64_H_

#include "src/base/memory.h"
#include "src/codegen/riscv64/constants-riscv64.h"
#include "src/codegen/riscv64/register-riscv64.h"
#include "src/common/globals.h"
#include "src/utils/utils.h"

#define UNSUPPORTED_RISCV() \
  out_buffer_pos_ +=        \
      SNPrintF(out_buffer_ + out_buffer_pos_, "unsupported Instructions");

namespace v8 {
namespace internal {

// ISA constants. --------------------------------------------------------------

#if defined(V8_OS_WIN)
extern "C" {
#endif

#if defined(V8_OS_WIN)
}  // end of extern "C"
#endif

// Instructions. ---------------------------------------------------------------

class InstructionBase {
 private:
  uint32_t SignedExtend32(uint32_t data, uint32_t topBit) const {
    DCHECK(0 <= topBit && topBit <= 31);
    uint32_t topBitMask = 1 << topBit;
    if (topBitMask & data) {
      return ~(topBitMask - 1) | data;
    } else {
      return data;
    }
  }

 public:
  // kCType for compressed instructions
  enum Type {
    kLoadType,
    kLoadFpType,
    kCustom0Type,
    kMiscMemType,
    kOpImmType,
    kAuipcType,
    kOpImm32Type,
    kUnSupport48Type,
    kStoreType,
    kStoreFpType,
    kCustom1Type,
    kAmoType,
    kOpType,
    kLuiType,
    kOp32Type,
    kUnSupport64Type,
    kMaddType,
    kMsubType,
    kNmsubType,
    kNmaddType,
    kOpFpType,
    kReversed0Type,
    kCustom2Type,
    kAnotherUnSupport48Type,
    kBranchType,
    kJalrType,
    kReversed1Type,
    kJalType,
    kSystemType,
    kReversed2Type,
    kCustom3Type,
    kUnSupport80Type,
    kCompressedType = -1,
    kUnsupported = -2
  };

  inline Instr InstructionBits() const {
    return *reinterpret_cast<const Instr*>(this);
  }

  inline void SetInstructionBits(Instr value) {
    *reinterpret_cast<Instr*>(this) = value;
  }

  inline int Bit(int nr) const { return (InstructionBits() >> nr) & 1; }

  inline int Bits(int hi, int lo) const {
    return (InstructionBits() >> lo) & ((2U << (hi - lo)) - 1);
  }

  inline Opcode OpcodeValue() const {
    return static_cast<Opcode>(Bits(kOpcodeBits - 1, kOpcodeShift));
  }

  inline int Rs1Value() const {
    return this->Bits(kRs1Shift + kRs1Bits - 1, kRs1Shift);
  }

  inline int Rs2Value() const {
    return this->Bits(kRs2Shift + kRs2Bits - 1, kRs2Shift);
  }

  inline int Rs3Value() const {
    return this->Bits(kRs3Shift + kRs3Bits - 1, kRs3Shift);
  }

  inline int RdValue() const {
    return this->Bits(kRdShift + kRdBits - 1, kRdShift);
  }

  inline Funct2 Funct2Value() const {
    return Funct2(Bits(kFunct2Shift + kImm2Shift - 1, kFunct2Shift));
  }

  inline Funct3 Funct3Value() const {
    return Funct3(Bits(kFunct3Shift + kImm3Shift - 1, kFunct3Shift));
  }

  inline Funct7 Funct7Value() const {
    return Funct7(Bits(kFunct7Shift + kImm7Shift - 1, kFunct7Shift));
  }

  inline int Funct3FieldRawNoAssert() const {
    return InstructionBits() & kImm14_12Mask;
  }

  inline int Funct7FieldRawNoAssert() const {
    return InstructionBits() & kImm31_25Mask;
  }

  inline int Rs1FieldRawNoAssert() const {
    return InstructionBits() & kImm19_15Mask;
  }

  inline int Rs2FieldRawNoAssert() const {
    return InstructionBits() & kImm24_20Mask;
  }

  inline int RdFieldRawNoAssert() const {
    return InstructionBits() & kRdFieldMask;
  }

  inline bool isCompressed() const {
    return Bits(kImm1Shift, kImm0Shift) != kUnCompressedMask;
  }

  inline uint32_t ImmValueIType() const {
    uint32_t v = Bits(kImm12Shift + kImm20Shift - 1, kImm20Shift);
    return SignedExtend32(v, kImm11Shift);
  }

  inline uint32_t ImmValueSType() const {
    uint32_t v = Bits(kImm11Shift, kImm7Shift) |
                 (Bits(kImm31Shift, kImm25Shift) << kImm5Shift);
    return SignedExtend32(v, kImm11Shift);
  }

  inline uint32_t ImmValueBType() const {
    uint32_t v = (Bit(kImm31Shift) << kImm12Shift) |
                 (Bits(kImm11Shift, kImm8Shift) << kImm1Shift) |
                 (Bit(kImm7Shift) << kImm11Shift) |
                 (Bits(kImm30Shift, kImm25Shift) << kImm5Shift);
    return SignedExtend32(v, kImm12Shift);
  }

  inline uint32_t ImmValueUType() const {
    return Bits(kImm20Shift + kImm12Shift - 1, kImm12Shift);
  }

  inline uint32_t ImmValueJType() const {
    uint32_t v = (Bit(kImm31Shift) << kImm20Shift) |
                 (Bits(kImm30Shift, kImm21Shift) << kImm1Shift) |
                 (Bit(kImm20Shift) << kImm11Shift) |
                 (Bits(kImm19Shift, kImm12Shift) << kImm12Shift);
    return SignedExtend32(v, kImm20Shift) & (-1u);
  }

  inline uint32_t Shamt5() const { return Bits(kImm24Shift, kImm20Shift); }

  inline uint32_t Shamt6() const { return Bits(kImm25Shift, kImm20Shift); }

  inline uint32_t ZImmValue() const { return Bits(kImm19Shift, kRs1Shift); }

  inline Type InstructionType() const;

 protected:
  InstructionBase() {}
};

class Instruction : public InstructionBase {
 public:
  static Instruction* At(byte* pc) {
    return reinterpret_cast<Instruction*>(pc);
  }

 private:
  DISALLOW_IMPLICIT_CONSTRUCTORS(Instruction);
};

InstructionBase::Type InstructionBase::InstructionType() const {
  Opcode opcode = OpcodeValue();
  if (isCompressed()) {
    return kCompressedType;
  } else {
    return InstructionBase::Type(opcode >> 2);
  }
}

}  // namespace internal
}  // namespace v8

#endif  // V8_CODEGEN_RISCV64_INSTRUCTIONS_RISCV64_H_
