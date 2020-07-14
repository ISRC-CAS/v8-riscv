// Copyright 2014 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/objects/objects-inl.h"
#include "test/unittests/compiler/backend/instruction-selector-unittest.h"

namespace v8 {
namespace internal {
namespace compiler {

template <typename T>
struct MachInst {
  T constructor;
  const char* constructor_name;
  ArchOpcode arch_opcode;
  MachineType machine_type;
};

using MachInst1 = MachInst<Node* (RawMachineAssembler::*)(Node*)>;
using MachInst2 = MachInst<Node* (RawMachineAssembler::*)(Node*, Node*)>;

template <typename T>
std::ostream& operator<<(std::ostream& os, const MachInst<T>& mi) {
  return os << mi.constructor_name;
}

// riscv64 type conversion instructions
struct Conversion {
  // The machine_type field in MachInst1 represents the destination type.
  MachInst1 mi;
  MachineType src_machine_type;
};

const Conversion kConversionInstructions[] = {
    {{&RawMachineAssembler::ChangeFloat32ToFloat64, "ChangeFloat32ToFloat64",
      kRISCV64FcvtDS, MachineType::Float64()},
     MachineType::Float32()},
    {{&RawMachineAssembler::ChangeFloat64ToInt32, "ChangeFloat64ToInt32",
      kRISCV64FcvtWD, MachineType::Int32()},
     MachineType::Float64()},
    {{&RawMachineAssembler::ChangeInt64ToFloat64, "ChangeInt64ToFloat64",
      kRISCV64FcvtDL, MachineType::Float64()},
     MachineType::Int64()},
    {{&RawMachineAssembler::ChangeUint32ToFloat64, "ChangeUint32ToFloat64",
      kRISCV64FcvtDWu, MachineType::Float64()},
     MachineType::Uint32()},
    // {{&RawMachineAssembler::TruncateFloat64ToInt64,"TruncateFloat64ToInt64"
    //   ,kRISCV64FcvtLD,MachineType::Int64()},
    //   MachineType::Float64()},
    {{&RawMachineAssembler::TruncateFloat32ToUint32, "TruncateFloat32ToUint32",
      kRISCV64FcvtUwS, MachineType::Uint32()},
     MachineType::Float32()},
    {{&RawMachineAssembler::TruncateFloat64ToUint32, "TruncateFloat64ToUint32",
      kRISCV64FcvtUwD, MachineType::Uint32()},
     MachineType::Float64()},
    // {{&RawMachineAssembler::TruncateInt64ToInt32,"TruncateInt64ToInt32"
    //   ,kRISCV64FcvtWL,MachineType::Int32()},
    //  MachineType::Int64()},
    {{&RawMachineAssembler::TruncateFloat32ToInt32, "TruncateFloat32ToInt32",
      kRISCV64FcvtWS, MachineType::Int32()},
     MachineType::Float32()},
    {{&RawMachineAssembler::TruncateFloat64ToFloat32,
      "TruncateFloat64ToFloat32", kRISCV64FcvtSD, MachineType::Float32()},
     MachineType::Float64()}};

using InstructionSelectorConversionTest =
    InstructionSelectorTestWithParam<Conversion>;

TEST_P(InstructionSelectorConversionTest, Parameter) {
  const Conversion conversion = GetParam();
  StreamBuilder m(this, conversion.mi.machine_type,
                  conversion.src_machine_type);
  m.Return((m.*conversion.mi.constructor)(m.Parameter(0)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(conversion.mi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(1U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorConversionTest,
                         ::testing::ValuesIn(kConversionInstructions));

// riscv64 32bit instruction need clear top 32 bit
const MachInst2 kCanConsiderChangeUint32ToUint64[] = {
    {&RawMachineAssembler::Word32Shl, "Word32Shl", kRISCV64Sll32,
     MachineType::Uint32()},
    {&RawMachineAssembler::Word32Shr, "Word32Shr", kRISCV64Srl32,
     MachineType::Uint32()},
    {&RawMachineAssembler::Word32Sar, "Word32Sar", kRISCV64Sra32,
     MachineType::Uint32()},
    {&RawMachineAssembler::Int32Add, "Int32Add", kRISCV64Add32,
     MachineType::Int32()},
    // {&RawMachineAssembler::Int32AddWithOverflow, "Int32AddWithOverflow",
    //  kRISCV64Add32, MachineType::Int32()},
    // {&RawMachineAssembler::Int32SubWithOverflow, "Int32SubWithOverflow",
    //  kRISCV64Sub32, MachineType::Int32()},
    {&RawMachineAssembler::Int32Sub, "Int32Sub", kRISCV64Sub32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int32Mul, "Int32Mul", kRISCV64Mul32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int32Div, "Int32Div", kRISCV64Div32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int32Mod, "Int32Mod", kRISCV64Mod32,
     MachineType::Int32()}};

const MachInst2 kCanConsiderChangeUint32ToUint64Instruction32[] = {
    {&RawMachineAssembler::Word32Or, "Word32Or", kRISCV64Or,
     MachineType::Uint32()},
    {&RawMachineAssembler::Word32Xor, "Word32Xor", kRISCV64Xor,
     MachineType::Uint32()}};

const MachInst2 kCanConsiderChangeUint32ToUint64CmpInstruction32[] = {
    {&RawMachineAssembler::Word32Equal, "Word32Equal", kRISCV64Cmp,
     MachineType::Uint32()},
    {&RawMachineAssembler::Int32LessThan, "Int32LessThan", kRISCV64Cmp,
     MachineType::Int32()},
    {&RawMachineAssembler::Int32LessThanOrEqual, "Int32LessThanOrEqual",
     kRISCV64Cmp, MachineType::Int32()},
    {&RawMachineAssembler::Uint32LessThan, "Uint32LessThan", kRISCV64Cmp,
     MachineType::Uint32()},
    {&RawMachineAssembler::Uint32LessThanOrEqual, "Uint32LessThanOrEqual",
     kRISCV64Cmp, MachineType::Uint32()}};

using InstructionSelectorConsiderChangeUint32ToUint64Test =
    InstructionSelectorTestWithParam<MachInst2>;

using InstructionSelectorConsiderChangeUint32ToUint64TestInstruction32 =
    InstructionSelectorTestWithParam<MachInst2>;

using InstructionSelectorConsiderChangeUint32ToUint64TestCmpInstruction32 =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorConsiderChangeUint32ToUint64Test, Parameter) {
  const MachInst2 binop = GetParam();
  StreamBuilder m(this, MachineType::Uint64(), binop.machine_type,
                  binop.machine_type);
  m.Return(m.ChangeUint32ToUint64(
      (m.*binop.constructor)(m.Parameter(0), m.Parameter(1))));
  Stream s = m.Build();
  ASSERT_EQ(3U, s.size());
  EXPECT_EQ(binop.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(kRISCV64Sll, s[1]->arch_opcode());  // clear top 32bit
  EXPECT_EQ(kRISCV64Srl, s[2]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[2]->OutputCount());
}

TEST_P(InstructionSelectorConsiderChangeUint32ToUint64TestInstruction32,
       Parameter) {
  const MachInst2 binop = GetParam();
  StreamBuilder m(this, MachineType::Uint64(), binop.machine_type,
                  binop.machine_type);
  m.Return(m.ChangeUint32ToUint64(
      (m.*binop.constructor)(m.Parameter(0), m.Parameter(1))));
  Stream s = m.Build();
  ASSERT_EQ(4U, s.size());
  EXPECT_EQ(binop.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
  EXPECT_EQ(kRISCV64Sll, s[2]->arch_opcode());  // clear top 32bit
  EXPECT_EQ(kRISCV64Srl, s[3]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[3]->OutputCount());
}

TEST_P(InstructionSelectorConsiderChangeUint32ToUint64TestCmpInstruction32,
       Parameter) {
  const MachInst2 binop = GetParam();
  StreamBuilder m(this, MachineType::Uint64(), binop.machine_type,
                  binop.machine_type);
  m.Return(m.ChangeUint32ToUint64(
      (m.*binop.constructor)(m.Parameter(0), m.Parameter(1))));
  Stream s = m.Build();
  ASSERT_EQ(5U, s.size());
  EXPECT_EQ(kRISCV64Sll, s[0]->arch_opcode());  // clear top 32bit
  EXPECT_EQ(kRISCV64Sll, s[1]->arch_opcode());
  EXPECT_EQ(binop.arch_opcode, s[2]->arch_opcode());
  EXPECT_EQ(kRISCV64Sll, s[3]->arch_opcode());  // clear top 32bit
  EXPECT_EQ(kRISCV64Srl, s[4]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[4]->OutputCount());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorConsiderChangeUint32ToUint64Test,
                         ::testing::ValuesIn(kCanConsiderChangeUint32ToUint64));

INSTANTIATE_TEST_SUITE_P(
    InstructionSelectorTest,
    InstructionSelectorConsiderChangeUint32ToUint64TestInstruction32,
    ::testing::ValuesIn(kCanConsiderChangeUint32ToUint64Instruction32));

INSTANTIATE_TEST_SUITE_P(
    InstructionSelectorTest,
    InstructionSelectorConsiderChangeUint32ToUint64TestCmpInstruction32,
    ::testing::ValuesIn(kCanConsiderChangeUint32ToUint64CmpInstruction32));
// riscv64 32bit instruction elided clear top 32 bit
const MachInst2 kCanElidedChangeUint32ToUint64[] = {
    {&RawMachineAssembler::Uint32Div, "Uint32Div", kRISCV64DivU32,
     MachineType::Uint32()},
    {&RawMachineAssembler::Uint32Mod, "Uint32Mod", kRISCV64ModU32,
     MachineType::Uint32()},
    {&RawMachineAssembler::Uint32MulHigh, "Uint32MulHigh", kRISCV64MulHighU32,
     MachineType::Uint32()}};

using InstructionSelectorElidedChangeUint32ToUint64Test =
    InstructionSelectorTestWithParam<MachInst2>;
TEST_P(InstructionSelectorElidedChangeUint32ToUint64Test, Parameter) {
  const MachInst2 binop = GetParam();
  StreamBuilder m(this, MachineType::Uint64(), binop.machine_type,
                  binop.machine_type);
  m.Return(m.ChangeUint32ToUint64(
      (m.*binop.constructor)(m.Parameter(0), m.Parameter(1))));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(binop.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorElidedChangeUint32ToUint64Test,
                         ::testing::ValuesIn(kCanElidedChangeUint32ToUint64));

// Random subset of 64-bit logical immediates.
const uint64_t kLogicalInt12Immediates[] = {
    826,   924,  1428,  1804,  1220,  1281,  650,   1848,  450,   543,
    1282,  740,  2018,  1063,  507,   565,   1476,  403,   966,   1508,
    -1300, -782, -1822, -499,  -1277, -1606, -1302, -1892, -668,  -886,
    -984,  -611, -1,    -1364, -90,   -1448, -296,  -1790, -1954, -329};

const uint64_t kLogicalExceedInt12Immediates[] = {
    2070, 7468, 4998, 6276, 6650, 6667, 5345, 2813, 3857, 6097, -2049};

// Riscv64 logical instructions.
const MachInst2 kLogicalInstructions[] = {
    {&RawMachineAssembler::Word64And, "Word64And", kRISCV64And,
     MachineType::Int64()},
    {&RawMachineAssembler::Word64Or, "Word64Or", kRISCV64Or,
     MachineType::Int64()},
    {&RawMachineAssembler::Word64Xor, "Word64Xor", kRISCV64Xor,
     MachineType::Int64()}};

using InstructionSelectorLogicalTest =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorLogicalTest, Parameter) {
  const MachInst2 dpi = GetParam();
  const MachineType type = dpi.machine_type;
  StreamBuilder m(this, type, type, type);
  m.Return((m.*dpi.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

TEST_P(InstructionSelectorLogicalTest, Immediate) {
  const MachInst2 dpi = GetParam();
  const MachineType type = dpi.machine_type;
  if (type == MachineType::Int32()) {
    // Immediate on the right.
    TRACED_FOREACH(int32_t, imm, kLogicalInt12Immediates) {
      StreamBuilder m(this, type, type);
      m.Return((m.*dpi.constructor)(m.Parameter(0), m.Int32Constant(imm)));
      Stream s = m.Build();
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
      ASSERT_EQ(2U, s[0]->InputCount());
      EXPECT_TRUE(s[0]->InputAt(1)->IsImmediate());
      EXPECT_EQ(imm, s.ToInt32(s[0]->InputAt(1)));
      EXPECT_EQ(1U, s[0]->OutputCount());
    }

    // Immediate on the left; all logical ops should commute.
    TRACED_FOREACH(int32_t, imm, kLogicalInt12Immediates) {
      StreamBuilder m(this, type, type);
      m.Return((m.*dpi.constructor)(m.Int32Constant(imm), m.Parameter(0)));
      Stream s = m.Build();
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
      ASSERT_EQ(2U, s[0]->InputCount());
      EXPECT_TRUE(s[0]->InputAt(1)->IsImmediate());
      EXPECT_EQ(imm, s.ToInt32(s[0]->InputAt(1)));
      EXPECT_EQ(1U, s[0]->OutputCount());
    }

    TRACED_FOREACH(int32_t, imm, kLogicalExceedInt12Immediates) {
      StreamBuilder m(this, type, type);
      m.Return((m.*dpi.constructor)(m.Int32Constant(imm), m.Parameter(0)));
      Stream s = m.Build();
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
      ASSERT_EQ(2U, s[0]->InputCount());
      EXPECT_FALSE(s[0]->InputAt(1)->IsImmediate());
    }
  } else if (type == MachineType::Int64()) {
    // Immediate on the right.
    TRACED_FOREACH(int64_t, imm, kLogicalInt12Immediates) {
      StreamBuilder m(this, type, type);
      m.Return((m.*dpi.constructor)(m.Parameter(0), m.Int64Constant(imm)));
      Stream s = m.Build();
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
      ASSERT_EQ(2U, s[0]->InputCount());
      EXPECT_TRUE(s[0]->InputAt(1)->IsImmediate());
      EXPECT_EQ(imm, s.ToInt64(s[0]->InputAt(1)));
      EXPECT_EQ(1U, s[0]->OutputCount());
    }

    // Immediate on the left; all logical ops should commute.
    TRACED_FOREACH(int64_t, imm, kLogicalInt12Immediates) {
      StreamBuilder m(this, type, type);
      m.Return((m.*dpi.constructor)(m.Int64Constant(imm), m.Parameter(0)));
      Stream s = m.Build();
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
      ASSERT_EQ(2U, s[0]->InputCount());
      EXPECT_TRUE(s[0]->InputAt(1)->IsImmediate());
      EXPECT_EQ(imm, s.ToInt64(s[0]->InputAt(1)));
      EXPECT_EQ(1U, s[0]->OutputCount());
    }
  }
}

// shift instructions.
const MachInst2 kShiftInstructions[] = {
    {&RawMachineAssembler::Word64Shl, "Word64Shl", kRISCV64Sll,
     MachineType::Int64()},
    {&RawMachineAssembler::Word64Shr, "Word64Shr", kRISCV64Srl,
     MachineType::Int64()},
    {&RawMachineAssembler::Word64Sar, "Word64Sar", kRISCV64Sra,
     MachineType::Int64()},
    {&RawMachineAssembler::Word32Shl, "Word32Shl", kRISCV64Sll32,
     MachineType::Int32()},
    {&RawMachineAssembler::Word32Shr, "Word32Shr", kRISCV64Srl32,
     MachineType::Int32()},
    {&RawMachineAssembler::Word32Sar, "Word32Sar", kRISCV64Sra32,
     MachineType::Int32()},
};

using InstructionSelectorShiftTest =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorShiftTest, Parameter) {
  const MachInst2 shift = GetParam();
  const MachineType type = shift.machine_type;
  StreamBuilder m(this, type, type, type);
  m.Return((m.*shift.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(shift.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

TEST_P(InstructionSelectorShiftTest, ShiftByImmediate) {
  const MachInst2 shift = GetParam();
  const MachineType type = shift.machine_type;
  TRACED_FORRANGE(int, imm, 0, ((type == MachineType::Int32()) ? 31 : 63)) {
    StreamBuilder m(this, type, type);
    m.Return((m.*shift.constructor)(m.Parameter(0), m.Int64Constant(imm)));
    Stream s = m.Build();
    ASSERT_EQ(1U, s.size());
    EXPECT_EQ(shift.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
    EXPECT_EQ(imm, s.ToInt64(s[0]->InputAt(1)));
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
  TRACED_FORRANGE(int, imm, 0, ((type == MachineType::Int32()) ? 31 : 63)) {
    StreamBuilder m(this, type, type);
    m.Return((m.*shift.constructor)(m.Int64Constant(imm), m.Parameter(0)));
    Stream s = m.Build();
    ASSERT_EQ(1U, s.size());
    EXPECT_EQ(shift.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorShiftTest,
                         ::testing::ValuesIn(kShiftInstructions));

TEST_F(InstructionSelectorTest, Word64ShlWithChangeInt32ToInt64) {
  TRACED_FORRANGE(int64_t, x, 32, 63) {
    StreamBuilder m(this, MachineType::Int64(), MachineType::Int32());
    Node* const p0 = m.Parameter(0);
    Node* const n = m.Word64Shl(m.ChangeInt32ToInt64(p0), m.Int64Constant(x));
    m.Return(n);
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());  // TODO:ADD can omit?
    EXPECT_EQ(kRISCV64Sll, s[1]->arch_opcode());
    ASSERT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(s.ToVreg(p0), s.ToVreg(s[0]->InputAt(0)));
    EXPECT_EQ(x, s.ToInt64(s[1]->InputAt(1)));
    ASSERT_EQ(1U, s[1]->OutputCount());
    EXPECT_EQ(s.ToVreg(n), s.ToVreg(s[1]->Output()));
  }
}

TEST_F(InstructionSelectorTest, Word64ShlWithChangeUint32ToUint64) {
  TRACED_FORRANGE(int64_t, x, 32, 63) {
    StreamBuilder m(this, MachineType::Int64(), MachineType::Uint32());
    Node* const p0 = m.Parameter(0);
    Node* const n = m.Word64Shl(m.ChangeUint32ToUint64(p0), m.Int64Constant(x));
    m.Return(n);
    Stream s = m.Build();
    ASSERT_EQ(3U, s.size());  // TODO:SLL src 32 src SHL 32 -> AND 0XFF?
    EXPECT_EQ(kRISCV64Sll, s[2]->arch_opcode());
    ASSERT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(s.ToVreg(p0), s.ToVreg(s[0]->InputAt(0)));
    EXPECT_EQ(x, s.ToInt64(s[2]->InputAt(1)));
    ASSERT_EQ(1U, s[2]->OutputCount());
    EXPECT_EQ(s.ToVreg(n), s.ToVreg(s[2]->Output()));
  }
}

// Add/Sub immediates
const int32_t kAddSubImmediates[] = {
    -1,    -10, -45, -569, -981, -1385, -1644, -1892, -2045,
    -2048, 0,   1,   69,   493,  599,   701,   719,   768,
    818,   842, 945, 1246, 1286, 1429,  1669,  2047};

struct AddSub {
  MachInst2 mi;
  ArchOpcode negate_arch_opcode;
};

std::ostream& operator<<(std::ostream& os, const AddSub& op) {
  return os << op.mi;
}

const AddSub kAddSubInstructions[] = {
    {{&RawMachineAssembler::Int32Add, "Int32Add", kRISCV64Add32,
      MachineType::Int32()},
     kRISCV64Add32},
    {{&RawMachineAssembler::Int64Add, "Int64Add", kRISCV64Add,
      MachineType::Int64()},
     kRISCV64Add},
    {{&RawMachineAssembler::Int32Sub, "Int32Sub", kRISCV64Sub32,
      MachineType::Int32()},
     kRISCV64Sub32},
    {{&RawMachineAssembler::Int64Sub, "Int64Sub", kRISCV64Sub,
      MachineType::Int64()},
     kRISCV64Sub}};

using InstructionSelectorAddSubTest = InstructionSelectorTestWithParam<AddSub>;

TEST_P(InstructionSelectorAddSubTest, Parameter) {
  const AddSub dpi = GetParam();
  const MachineType type = dpi.mi.machine_type;
  StreamBuilder m(this, type, type, type);
  m.Return((m.*dpi.mi.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(dpi.mi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

TEST_P(InstructionSelectorAddSubTest, Immediate) {
  const AddSub dpi = GetParam();
  const MachineType type = dpi.mi.machine_type;
  if (dpi.mi.arch_opcode == kRISCV64Sub || dpi.mi.arch_opcode == kRISCV64Sub32)
    return;
  TRACED_FOREACH(int64_t, imm, kAddSubImmediates) {
    StreamBuilder m(this, type, type);
    m.Return((m.*dpi.mi.constructor)(m.Parameter(0), m.Int64Constant(imm)));
    Stream s = m.Build();
    ASSERT_EQ(1U, s.size());
    EXPECT_EQ(dpi.mi.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
    EXPECT_EQ(imm, s.ToInt64(s[0]->InputAt(1)));
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
  TRACED_FOREACH(int32_t, imm, kAddSubImmediates) {
    StreamBuilder m(this, type, type);
    m.Return((m.*dpi.mi.constructor)(m.Parameter(0), m.Int32Constant(imm)));
    Stream s = m.Build();
    ASSERT_EQ(1U, s.size());
    EXPECT_EQ(dpi.mi.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
    EXPECT_EQ(imm, s.ToInt64(s[0]->InputAt(1)));
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorAddSubTest,
                         ::testing::ValuesIn(kAddSubInstructions));

// Riscv64 flag setting data processing instructions.
const MachInst2 kDPFlagSetInstructions[] = {
    {&RawMachineAssembler::Int32Add, "Int32Add", kRISCV64Add32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int32Sub, "Int32Sub", kRISCV64Sub32,
     MachineType::Int32()},
    {&RawMachineAssembler::Word64And, "Word64And", kRISCV64And,
     MachineType::Int64()}};

using InstructionSelectorDPFlagSetTest =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorDPFlagSetTest, BranchWithParameters) {
  const MachInst2 dpi = GetParam();
  const MachineType type = dpi.machine_type;
  StreamBuilder m(this, type, type, type);
  RawMachineLabel a, b;
  m.Branch((m.*dpi.constructor)(m.Parameter(0), m.Parameter(1)), &a, &b);
  m.Bind(&a);
  m.Return(m.Int32Constant(1));
  m.Bind(&b);
  m.Return(m.Int32Constant(0));

  Stream s = m.Build();
  ASSERT_EQ(2U, s.size());
  EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(kRISCV64Cmp, s[1]->arch_opcode());
  EXPECT_EQ(kFlags_branch, s[1]->flags_mode());
  EXPECT_EQ(kNotEqual, s[1]->flags_condition());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorDPFlagSetTest,
                         ::testing::ValuesIn(kDPFlagSetInstructions));

// logical immediates: contiguous set bits, rotated about a power of two
// sized block. The block is then duplicated across the word. Below is a random
// subset of the 32-bit immediates.
const uint32_t kLogical32Immediates[] = {
    0x00000002, 0x00000003, 0x00000070, 0x00000080, 0x00000100, 0x000001C0,
    0x00000300, 0x000007E0, 0x00003FFC, 0x00007FC0, 0x0003C000, 0x0003F000,
    0x0003FFC0, 0x0003FFF8, 0x0007FF00, 0x0007FFE0, 0x000E0000, 0x001E0000,
    0x001FFFFC, 0x003F0000, 0x003F8000, 0x00780000, 0x007FC000, 0x00FF0000,
    0x01800000, 0x01800180, 0x01F801F8, 0x03FE0000, 0x03FFFFC0, 0x03FFFFFC,
    0x06000000, 0x07FC0000, 0x07FFC000, 0x07FFFFC0, 0x07FFFFE0, 0x0FFE0FFE,
    0x0FFFF800, 0x0FFFFFF0, 0x0FFFFFFF, 0x18001800, 0x1F001F00, 0x1F801F80,
    0x30303030, 0x3FF03FF0, 0x3FF83FF8, 0x3FFF0000, 0x3FFF8000, 0x3FFFFFC0,
    0x70007000, 0x7F7F7F7F, 0x7FC00000, 0x7FFFFFC0, 0x8000001F, 0x800001FF,
    0x81818181, 0x9FFF9FFF, 0xC00007FF, 0xC0FFFFFF, 0xDDDDDDDD, 0xE00001FF,
    0xE00003FF, 0xE007FFFF, 0xEFFFEFFF, 0xF000003F, 0xF001F001, 0xF3FFF3FF,
    0xF800001F, 0xF80FFFFF, 0xF87FF87F, 0xFBFBFBFB, 0xFC00001F, 0xFC0000FF,
    0xFC0001FF, 0xFC03FC03, 0xFE0001FF, 0xFF000001, 0xFF03FF03, 0xFF800000,
    0xFF800FFF, 0xFF801FFF, 0xFF87FFFF, 0xFFC0003F, 0xFFC007FF, 0xFFCFFFCF,
    0xFFE00003, 0xFFE1FFFF, 0xFFF0001F, 0xFFF07FFF, 0xFFF80007, 0xFFF87FFF,
    0xFFFC00FF, 0xFFFE07FF, 0xFFFF00FF, 0xFFFFC001, 0xFFFFF007, 0xFFFFF3FF,
    0xFFFFF807, 0xFFFFF9FF, 0xFFFFFC0F, 0xFFFFFEFF};

const int INT12_MAX = 2047;
const int INT12_MIN = -2048;

TEST_F(InstructionSelectorTest, Word32AndBranchWithImmediateOnRight) {
  TRACED_FOREACH(int32_t, imm, kLogical32Immediates) {
    StreamBuilder m(this, MachineType::Int32(), MachineType::Int32());
    RawMachineLabel a, b;
    m.Branch(m.Word32And(m.Parameter(0), m.Int32Constant(imm)), &a, &b);
    m.Bind(&a);
    m.Return(m.Int32Constant(1));
    m.Bind(&b);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build();
    ASSERT_EQ(3U, s.size());
    EXPECT_EQ(kRISCV64And, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
    EXPECT_EQ(kRISCV64Cmp, s[2]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    if ((imm <= INT12_MAX) && (INT12_MIN <= imm)) {
      EXPECT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
    } else {
      EXPECT_EQ(InstructionOperand::UNALLOCATED, s[0]->InputAt(1)->kind());
    }
    EXPECT_EQ(kFlags_branch, s[2]->flags_mode());
    EXPECT_EQ(kNotEqual, s[2]->flags_condition());
  }
}

TEST_F(InstructionSelectorTest, Word64AndBranchWithImmediateOnRight) {
  TRACED_FOREACH(int64_t, imm, kLogical32Immediates) {
    StreamBuilder m(this, MachineType::Int64(), MachineType::Int64());
    RawMachineLabel a, b;
    m.Branch(m.Word64And(m.Parameter(0), m.Int64Constant(imm)), &a, &b);
    m.Bind(&a);
    m.Return(m.Int32Constant(1));
    m.Bind(&b);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(kRISCV64And, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Cmp, s[1]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    if ((imm <= INT12_MAX) && (INT12_MIN <= imm)) {
      EXPECT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
    } else {
      EXPECT_EQ(InstructionOperand::UNALLOCATED, s[0]->InputAt(1)->kind());
    }
    ASSERT_LE(1U, s[0]->InputCount());
    EXPECT_EQ(kFlags_branch, s[1]->flags_mode());
    EXPECT_EQ(kNotEqual, s[1]->flags_condition());
  }
}

TEST_F(InstructionSelectorTest, AddBranchWithImmediateOnRight) {
  TRACED_FOREACH(int32_t, imm, kAddSubImmediates) {
    StreamBuilder m(this, MachineType::Int32(), MachineType::Int32());
    RawMachineLabel a, b;
    m.Branch(m.Int32Add(m.Parameter(0), m.Int32Constant(imm)), &a, &b);
    m.Bind(&a);
    m.Return(m.Int32Constant(1));
    m.Bind(&b);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Cmp, s[1]->arch_opcode());
    EXPECT_EQ(kFlags_branch, s[1]->flags_mode());
    EXPECT_EQ(kNotEqual, s[1]->flags_condition());
  }
}

TEST_F(InstructionSelectorTest, SubBranchWithImmediateOnRight) {
  TRACED_FOREACH(int32_t, imm, kAddSubImmediates) {
    StreamBuilder m(this, MachineType::Int32(), MachineType::Int32());
    RawMachineLabel a, b;
    m.Branch(m.Int32Sub(m.Parameter(0), m.Int32Constant(imm)), &a, &b);
    m.Bind(&a);
    m.Return(m.Int32Constant(1));
    m.Bind(&b);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(kRISCV64Sub32, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Cmp, s[1]->arch_opcode());
    EXPECT_EQ(kFlags_branch, s[1]->flags_mode());
    EXPECT_EQ(kNotEqual, s[1]->flags_condition());
  }
}

struct TestAndBranch {
  MachInst<std::function<Node*(InstructionSelectorTest::StreamBuilder&, Node*,
                               uint64_t mask)>>
      mi;
  FlagsCondition cond;
};

std::ostream& operator<<(std::ostream& os, const TestAndBranch& tb) {
  return os << tb.mi;
}

const TestAndBranch kTestAndBranchMatchers64[] = {
    // Branch on the result of Word64And directly.
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x, uint64_t mask)
          -> Node* { return m.Word64And(x, m.Int64Constant(mask)); },
      "if (x and mask)", kRISCV64And, MachineType::Int64()},
     kNotEqual},
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Word64And(x, m.Int64Constant(mask)),
                             m.Int64Constant(0));
      },
      "if not (x and mask)", kRISCV64And, MachineType::Int64()},
     kEqual},
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x, uint64_t mask)
          -> Node* { return m.Word64And(m.Int64Constant(mask), x); },
      "if (mask and x)", kRISCV64And, MachineType::Int64()},
     kNotEqual},
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Word64And(m.Int64Constant(mask), x),
                             m.Int64Constant(0));
      },
      "if not (mask and x)", kRISCV64And, MachineType::Int64()},
     kEqual},
    // Branch on the result of '(x and mask) == mask'. This tests that a bit
    // is
    // set rather than cleared which is why conditions are inverted.
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Word64And(x, m.Int64Constant(mask)),
                             m.Int64Constant(mask));
      },
      "if ((x and mask) == mask)", kRISCV64And, MachineType::Int64()},
     kEqual},
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Int64Constant(mask),
                             m.Word64And(x, m.Int64Constant(mask)));
      },
      "if (mask == (x and mask))", kRISCV64And, MachineType::Int64()},
     kEqual},
    // Same as above but swap 'mask' and 'x'.
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Word64And(m.Int64Constant(mask), x),
                             m.Int64Constant(mask));
      },
      "if ((mask and x) == mask)", kRISCV64And, MachineType::Int64()},
     kEqual},
    {{[](InstructionSelectorTest::StreamBuilder& m, Node* x,
         uint64_t mask) -> Node* {
        return m.Word64Equal(m.Int64Constant(mask),
                             m.Word64And(m.Int64Constant(mask), x));
      },
      "if (mask == (mask and x))", kRISCV64And, MachineType::Int64()},
     kEqual}};

using InstructionSelectorTestAndBranchTest64 =
    InstructionSelectorTestWithParam<TestAndBranch>;

TEST_P(InstructionSelectorTestAndBranchTest64, TestAndBranch64) {
  const TestAndBranch inst = GetParam();
  TRACED_FORRANGE(int, bit, 0, 10) {
    uint64_t mask = uint64_t{1} << bit;
    StreamBuilder m(this, MachineType::Int64(), MachineType::Int64());
    RawMachineLabel a, b;
    m.Branch(inst.mi.constructor(m, m.Parameter(0), mask), &a, &b);
    m.Bind(&a);
    m.Return(m.Int64Constant(1));
    m.Bind(&b);
    m.Return(m.Int64Constant(0));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());

    EXPECT_EQ(inst.mi.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Cmp, s[1]->arch_opcode());
    EXPECT_EQ(inst.cond, s[1]->flags_condition());
    EXPECT_EQ(2U, s[0]->InputCount());
    ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorTestAndBranchTest64,
                         ::testing::ValuesIn(kTestAndBranchMatchers64));

// RISCV64 Mul/Div instructions.
const MachInst2 kMulDivInstructions[] = {
    {&RawMachineAssembler::Int32Mul, "Int32Mul", kRISCV64Mul32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int64Mul, "Int64Mul", kRISCV64Mul,
     MachineType::Int64()},
    {&RawMachineAssembler::Int32Div, "Int32Div", kRISCV64Div32,
     MachineType::Int32()},
    {&RawMachineAssembler::Int64Div, "Int64Div", kRISCV64Div,
     MachineType::Int64()},
    {&RawMachineAssembler::Uint32Div, "Uint32Div", kRISCV64DivU32,
     MachineType::Int32()},
    {&RawMachineAssembler::Uint64Div, "Uint64Div", kRISCV64DivU,
     MachineType::Int64()}};
using InstructionSelectorMulDivTest =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorMulDivTest, Parameter) {
  const MachInst2 dpi = GetParam();
  const MachineType type = dpi.machine_type;
  StreamBuilder m(this, type, type, type);
  m.Return((m.*dpi.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(dpi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorMulDivTest,
                         ::testing::ValuesIn(kMulDivInstructions));

struct MulDPInst {
  const char* mul_constructor_name;
  Node* (RawMachineAssembler::*mul_constructor)(Node*, Node*);
  Node* (RawMachineAssembler::*add_constructor)(Node*, Node*);
  Node* (RawMachineAssembler::*sub_constructor)(Node*, Node*);
  ArchOpcode add_arch_opcode;
  ArchOpcode sub_arch_opcode;
  MachineType machine_type;
};

std::ostream& operator<<(std::ostream& os, const MulDPInst& inst) {
  return os << inst.mul_constructor_name;
}

static const MulDPInst kMulDPInstructions[] = {
    {"Int32Mul", &RawMachineAssembler::Int32Mul, &RawMachineAssembler::Int32Add,
     &RawMachineAssembler::Int32Sub, kRISCV64Add32, kRISCV64Sub32,
     MachineType::Int32()},
    {"Int64Mul", &RawMachineAssembler::Int64Mul, &RawMachineAssembler::Int64Add,
     &RawMachineAssembler::Int64Sub, kRISCV64Add, kRISCV64Sub,
     MachineType::Int64()}};

using InstructionSelectorIntDPWithIntMulTest =
    InstructionSelectorTestWithParam<MulDPInst>;

TEST_P(InstructionSelectorIntDPWithIntMulTest, AddWithMul) {
  const MulDPInst mdpi = GetParam();
  const MachineType type = mdpi.machine_type;
  {
    StreamBuilder m(this, type, type, type, type);
    Node* n = (m.*mdpi.mul_constructor)(m.Parameter(1), m.Parameter(2));
    m.Return((m.*mdpi.add_constructor)(m.Parameter(0), n));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(mdpi.add_arch_opcode, s[1]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
  {
    StreamBuilder m(this, type, type, type, type);
    Node* n = (m.*mdpi.mul_constructor)(m.Parameter(0), m.Parameter(1));
    m.Return((m.*mdpi.add_constructor)(n, m.Parameter(2)));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(mdpi.add_arch_opcode, s[1]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
}

TEST_P(InstructionSelectorIntDPWithIntMulTest, SubWithMul) {
  const MulDPInst mdpi = GetParam();
  const MachineType type = mdpi.machine_type;
  {
    StreamBuilder m(this, type, type, type, type);
    Node* n = (m.*mdpi.mul_constructor)(m.Parameter(1), m.Parameter(2));
    m.Return((m.*mdpi.sub_constructor)(m.Parameter(0), n));
    Stream s = m.Build();
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(mdpi.sub_arch_opcode, s[1]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[0]->OutputCount());
  }
}
// 暂未实现
// TEST_F(InstructionSelectorTest, Int32MulWithImmediate) {
//   // x * (2^k + 1) -> x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32());
//     m.Return(m.Int32Mul(m.Parameter(0), m.Int32Constant((1 << k) + 1)));
//     Stream s = m.Build();
//     ASSERT_EQ(1U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     ASSERT_EQ(2U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(0)), s.ToVreg(s[0]->InputAt(1)));
//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // (2^k + 1) * x -> x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32());
//     m.Return(m.Int32Mul(m.Int32Constant((1 << k) + 1), m.Parameter(0)));
//     Stream s = m.Build();
//     ASSERT_EQ(1U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(0)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // x * (2^k + 1) + c -> x + (x << k) + c
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Add(m.Int32Mul(m.Parameter(0), m.Int32Constant((1 << k) + 1)),
//                    m.Parameter(1)));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(0)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // (2^k + 1) * x + c -> x + (x << k) + c
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Add(m.Int32Mul(m.Int32Constant((1 << k) + 1), m.Parameter(0)),
//                    m.Parameter(1)));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(0)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // c + x * (2^k + 1) -> c + x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Add(m.Parameter(0),
//                    m.Int32Mul(m.Parameter(1), m.Int32Constant((1 << k) +
//                    1))));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(1)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // c + (2^k + 1) * x -> c + x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Add(m.Parameter(0),
//                    m.Int32Mul(m.Int32Constant((1 << k) + 1),
//                    m.Parameter(1))));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Add32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Add32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(1)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // c - x * (2^k + 1) -> c - x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Sub(m.Parameter(0),
//                    m.Int32Mul(m.Parameter(1), m.Int32Constant((1 << k) +
//                    1))));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Sub32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Sub32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(1)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
//   // c - (2^k + 1) * x -> c - x + (x << k)
//   TRACED_FORRANGE(int32_t, k, 1, 30) {
//     StreamBuilder m(this, MachineType::Int32(), MachineType::Int32(),
//                     MachineType::Int32());
//     m.Return(
//         m.Int32Sub(m.Parameter(0),
//                    m.Int32Mul(m.Int32Constant((1 << k) + 1),
//                    m.Parameter(1))));
//     Stream s = m.Build();
//     ASSERT_EQ(2U, s.size());
//     EXPECT_EQ(kRISCV64Sub32, s[0]->arch_opcode());
//     EXPECT_EQ(kRISCV64Sub32, s[1]->arch_opcode());
//     ASSERT_EQ(3U, s[0]->InputCount());
//     EXPECT_EQ(s.ToVreg(s[0]->InputAt(1)), s.ToVreg(s[0]->InputAt(1)));

//     EXPECT_EQ(1U, s[0]->OutputCount());
//   }
// }

// INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
//                          InstructionSelectorIntDPWithIntMulTest,
//                          ::testing::ValuesIn(kMulDPInstructions));

// ARM64 FP arithmetic instructions.
const MachInst2 kFPArithInstructions[] = {
    {&RawMachineAssembler::Float64Add, "Float64Add", kRISCV64AddD,
     MachineType::Float64()},
    {&RawMachineAssembler::Float64Sub, "Float64Sub", kRISCV64SubD,
     MachineType::Float64()},
    {&RawMachineAssembler::Float64Mul, "Float64Mul", kRISCV64MulD,
     MachineType::Float64()},
    {&RawMachineAssembler::Float64Div, "Float64Div", kRISCV64DivD,
     MachineType::Float64()}};

using InstructionSelectorFPArithTest =
    InstructionSelectorTestWithParam<MachInst2>;

TEST_P(InstructionSelectorFPArithTest, Parameter) {
  const MachInst2 fpa = GetParam();
  StreamBuilder m(this, fpa.machine_type, fpa.machine_type, fpa.machine_type);
  m.Return((m.*fpa.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(fpa.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorFPArithTest,
                         ::testing::ValuesIn(kFPArithInstructions));

struct Cmp {
  MachInst2 mi;
  FlagsCondition cond;
};

std::ostream& operator<<(std::ostream& os, const Cmp& cmp) {
  return os << cmp.mi;
}

const Cmp kFPCmpInstructions[] = {
    {{&RawMachineAssembler::Float64Equal, "Float64Equal", kRISCV64CmpD,
      MachineType::Float64()},
     kEqual},
    {{&RawMachineAssembler::Float64LessThan, "Float64LessThan", kRISCV64CmpD,
      MachineType::Float64()},
     kFloatLessThan},
    {{&RawMachineAssembler::Float64LessThanOrEqual, "Float64LessThanOrEqual",
      kRISCV64CmpD, MachineType::Float64()},
     kFloatLessThanOrEqual},
    {{&RawMachineAssembler::Float32Equal, "Float32Equal", kRISCV64CmpS,
      MachineType::Float32()},
     kEqual},
    {{&RawMachineAssembler::Float32LessThan, "Float32LessThan", kRISCV64CmpS,
      MachineType::Float32()},
     kFloatLessThan},
    {{&RawMachineAssembler::Float32LessThanOrEqual, "Float32LessThanOrEqual",
      kRISCV64CmpS, MachineType::Float32()},
     kFloatLessThanOrEqual}};

using InstructionSelectorFPCmpTest = InstructionSelectorTestWithParam<Cmp>;

TEST_P(InstructionSelectorFPCmpTest, Parameter) {
  const Cmp cmp = GetParam();
  StreamBuilder m(this, MachineType::Int32(), cmp.mi.machine_type,
                  cmp.mi.machine_type);
  m.Return((m.*cmp.mi.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(1U, s.size());
  EXPECT_EQ(cmp.mi.arch_opcode, s[0]->arch_opcode());
  EXPECT_EQ(2U, s[0]->InputCount());
  EXPECT_EQ(1U, s[0]->OutputCount());
  EXPECT_EQ(kFlags_set, s[0]->flags_mode());
  EXPECT_EQ(cmp.cond, s[0]->flags_condition());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorFPCmpTest,
                         ::testing::ValuesIn(kFPCmpInstructions));

const Cmp kIPCmpInstructions[] = {
    {{&RawMachineAssembler::Word64Equal, "kWord64Equal", kRISCV64Cmp,
      MachineType::Int64()},
     kEqual},
    {{&RawMachineAssembler::Int64LessThan, "Int64LessThan", kRISCV64Cmp,
      MachineType::Int64()},
     kSignedLessThan},
    {{&RawMachineAssembler::Int64LessThanOrEqual, "Int64LessThanOrEqual",
      kRISCV64Cmp, MachineType::Int64()},
     kSignedLessThanOrEqual},

    {{&RawMachineAssembler::Uint64LessThan, "Uint64LessThan", kRISCV64Cmp,
      MachineType::Uint64()},
     kUnsignedLessThan},
    {{&RawMachineAssembler::Uint64LessThanOrEqual, "Uint64LessThanOrEqual",
      kRISCV64Cmp, MachineType::Uint64()},
     kUnsignedLessThanOrEqual},
    {{&RawMachineAssembler::Word32Equal, "Word32Equal", kRISCV64Cmp,
      MachineType::Int32()},
     kEqual},
    {{&RawMachineAssembler::Int32LessThan, "Int32LessThan", kRISCV64Cmp,
      MachineType::Int32()},
     kSignedLessThan},
    {{&RawMachineAssembler::Int32LessThanOrEqual, "Int32LessThanOrEqual",
      kRISCV64Cmp, MachineType::Int32()},
     kSignedLessThanOrEqual},
    {{&RawMachineAssembler::Uint32LessThan, "Uint32LessThan", kRISCV64Cmp,
      MachineType::Uint32()},
     kUnsignedLessThan},
    {{&RawMachineAssembler::Uint32LessThanOrEqual, "Uint32LessThanOrEqual",
      kRISCV64Cmp, MachineType::Uint32()},
     kUnsignedLessThanOrEqual}};

using InstructionSelectorIPCmpTest = InstructionSelectorTestWithParam<Cmp>;

TEST_P(InstructionSelectorIPCmpTest, Parameter) {
  const Cmp cmp = GetParam();
  StreamBuilder m(this, MachineType::Int32(), cmp.mi.machine_type,
                  cmp.mi.machine_type);
  m.Return((m.*cmp.mi.constructor)(m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  if (cmp.mi.machine_type == MachineType::Int32() ||
      cmp.mi.machine_type == MachineType::Uint32()) {
    ASSERT_EQ(3U, s.size());
    EXPECT_EQ(kRISCV64Sll, s[0]->arch_opcode());
    EXPECT_EQ(kRISCV64Sll, s[1]->arch_opcode());
    EXPECT_EQ(cmp.mi.arch_opcode, s[2]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[2]->OutputCount());
    EXPECT_EQ(kFlags_set, s[2]->flags_mode());
    EXPECT_EQ(cmp.cond, s[2]->flags_condition());
  } else {
    ASSERT_EQ(1U, s.size());
    EXPECT_EQ(cmp.mi.arch_opcode, s[0]->arch_opcode());
    EXPECT_EQ(2U, s[0]->InputCount());
    EXPECT_EQ(1U, s[0]->OutputCount());
    EXPECT_EQ(kFlags_set, s[0]->flags_mode());
    EXPECT_EQ(cmp.cond, s[0]->flags_condition());
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorIPCmpTest,
                         ::testing::ValuesIn(kIPCmpInstructions));

struct MemoryAccess {
  MachineType type;
  ArchOpcode ldr_opcode;
  ArchOpcode str_opcode;
  const int32_t immediates[20];
};

std::ostream& operator<<(std::ostream& os, const MemoryAccess& memacc) {
  return os << memacc.type;
}

static const MemoryAccess kMemoryAccesses[] = {
    {MachineType::Int8(),
     kRISCV64Lb,
     kRISCV64Sb,
     {-256, -255, -3,  -2,   -1,   0,    1,    2,    3,    255,
      256,  257,  258, 1000, 1001, 2121, 2442, 4093, 4094, 4095}},
    {MachineType::Uint8(),
     kRISCV64Lbu,
     kRISCV64Sb,
     {-256, -255, -3,  -2,   -1,   0,    1,    2,    3,    255,
      256,  257,  258, 1000, 1001, 2121, 2442, 4093, 4094, 4095}},
    {MachineType::Int16(),
     kRISCV64Lh,
     kRISCV64Sh,
     {-256, -255, -3,  -2,   -1,   0,    1,    2,    3,    255,
      256,  258,  260, 4096, 4098, 4100, 4242, 6786, 8188, 8190}},
    {MachineType::Uint16(),
     kRISCV64Lhu,
     kRISCV64Sh,
     {-256, -255, -3,  -2,   -1,   0,    1,    2,    3,    255,
      256,  258,  260, 4096, 4098, 4100, 4242, 6786, 8188, 8190}},
    {MachineType::Int32(),
     kRISCV64Lw,
     kRISCV64Sw,
     {-256, -255, -3,   -2,   -1,   0,    1,    2,    3,     255,
      256,  260,  4096, 4100, 8192, 8196, 3276, 3280, 16376, 16380}},
    {MachineType::Uint32(),
     kRISCV64Lwu,
     kRISCV64Sw,
     {-256, -255, -3,   -2,   -1,   0,    1,    2,    3,     255,
      256,  260,  4096, 4100, 8192, 8196, 3276, 3280, 16376, 16380}},
    {MachineType::Int64(),
     kRISCV64Ld,
     kRISCV64Sd,
     {-256, -255, -3,   -2,   -1,   0,    1,     2,     3,     255,
      256,  264,  4096, 4104, 8192, 8200, 16384, 16392, 32752, 32760}},
    {MachineType::Uint64(),
     kRISCV64Ld,
     kRISCV64Sd,
     {-256, -255, -3,   -2,   -1,   0,    1,     2,     3,     255,
      256,  264,  4096, 4104, 8192, 8200, 16384, 16392, 32752, 32760}},
    {MachineType::Float32(),
     kRISCV64flw,
     kRISCV64fsw,
     {-256, -255, -3,   -2,   -1,   0,    1,    2,    3,     255,
      256,  260,  4096, 4100, 8192, 8196, 3276, 3280, 16376, 16380}},
    {MachineType::Float64(),
     kRISCV64fld,
     kRISCV64fsd,
     {-256, -255, -3,   -2,   -1,   0,    1,     2,     3,     255,
      256,  264,  4096, 4104, 8192, 8200, 16384, 16392, 32752, 32760}}};

using InstructionSelectorMemoryAccessTest =
    InstructionSelectorTestWithParam<MemoryAccess>;

TEST_P(InstructionSelectorMemoryAccessTest, LoadWithParameters) {
  const MemoryAccess memacc = GetParam();
  StreamBuilder m(this, memacc.type, MachineType::Pointer(),
                  MachineType::Int32());
  m.Return(m.Load(memacc.type, m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build();
  ASSERT_EQ(2U, s.size());
  EXPECT_EQ(kRISCV64Add, s[0]->arch_opcode());
  EXPECT_EQ(memacc.ldr_opcode, s[1]->arch_opcode());
  EXPECT_EQ(2U, s[1]->InputCount());
  EXPECT_EQ(1U, s[1]->OutputCount());
}

TEST_P(InstructionSelectorMemoryAccessTest, LoadWithImmediateIndex) {
  const MemoryAccess memacc = GetParam();
  TRACED_FOREACH(int32_t, index, memacc.immediates) {
    StreamBuilder m(this, memacc.type, MachineType::Pointer());
    m.Return(m.Load(memacc.type, m.Parameter(0), m.Int32Constant(index)));
    Stream s = m.Build();
    if ((index <= INT12_MAX) && (index >= INT12_MIN)) {
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(memacc.ldr_opcode, s[0]->arch_opcode());
      EXPECT_EQ(kMode_MRI, s[0]->addressing_mode());
      EXPECT_EQ(2U, s[0]->InputCount());
      ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
      EXPECT_EQ(index, s.ToInt32(s[0]->InputAt(1)));
      ASSERT_EQ(1U, s[0]->OutputCount());
    } else {
      ASSERT_EQ(2U, s.size());
      EXPECT_EQ(kRISCV64Add, s[0]->arch_opcode());
      EXPECT_EQ(memacc.ldr_opcode, s[1]->arch_opcode());
      EXPECT_EQ(2U, s[1]->InputCount());
      EXPECT_EQ(1U, s[1]->OutputCount());
    }
  }
}

TEST_P(InstructionSelectorMemoryAccessTest, StoreWithParameters) {
  const MemoryAccess memacc = GetParam();
  StreamBuilder m(this, MachineType::Int32(), MachineType::Pointer(),
                  MachineType::Int32(), memacc.type);
  m.Store(memacc.type.representation(), m.Parameter(0), m.Parameter(1),
          m.Parameter(2), kNoWriteBarrier);
  m.Return(m.Int32Constant(0));
  Stream s = m.Build();
  ASSERT_EQ(2U, s.size());
  EXPECT_EQ(kRISCV64Add, s[0]->arch_opcode());
  EXPECT_EQ(memacc.str_opcode, s[1]->arch_opcode());
  EXPECT_EQ(kMode_MRI, s[1]->addressing_mode());
  EXPECT_EQ(3U, s[1]->InputCount());
  EXPECT_EQ(0U, s[1]->OutputCount());
}

TEST_P(InstructionSelectorMemoryAccessTest, StoreWithImmediateIndex) {
  const MemoryAccess memacc = GetParam();
  TRACED_FOREACH(int32_t, index, memacc.immediates) {
    StreamBuilder m(this, MachineType::Int32(), MachineType::Pointer(),
                    memacc.type);
    m.Store(memacc.type.representation(), m.Parameter(0),
            m.Int32Constant(index), m.Parameter(1), kNoWriteBarrier);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build();
    if ((index <= INT12_MAX) && (index >= INT12_MIN)) {
      ASSERT_EQ(1U, s.size());
      EXPECT_EQ(memacc.str_opcode, s[0]->arch_opcode());
      EXPECT_EQ(kMode_MRI, s[0]->addressing_mode());
      ASSERT_EQ(3U, s[0]->InputCount());
      ASSERT_EQ(InstructionOperand::IMMEDIATE, s[0]->InputAt(1)->kind());
      EXPECT_EQ(index, s.ToInt32(s[0]->InputAt(1)));
      EXPECT_EQ(0U, s[0]->OutputCount());
    } else {
      ASSERT_EQ(2U, s.size());
      EXPECT_EQ(kRISCV64Add, s[0]->arch_opcode());
      EXPECT_EQ(memacc.str_opcode, s[1]->arch_opcode());
      EXPECT_EQ(kMode_MRI, s[1]->addressing_mode());
      EXPECT_EQ(3U, s[1]->InputCount());
      EXPECT_EQ(0U, s[1]->OutputCount());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorMemoryAccessTest,
                         ::testing::ValuesIn(kMemoryAccesses));

static const WriteBarrierKind kWriteBarrierKinds[] = {
    kMapWriteBarrier, kPointerWriteBarrier, kEphemeronKeyWriteBarrier,
    kFullWriteBarrier};

const int32_t kStoreWithBarrierImmediates[] = {
    -256, -255, -3,   -2,   -1,   0,    1,     2,     3,     255,
    256,  264,  4096, 4104, 8192, 8200, 16384, 16392, 32752, 32760};

using InstructionSelectorStoreWithBarrierTest =
    InstructionSelectorTestWithParam<WriteBarrierKind>;

TEST_P(InstructionSelectorStoreWithBarrierTest,
       StoreWithWriteBarrierParameters) {
  const WriteBarrierKind barrier_kind = GetParam();
  StreamBuilder m(this, MachineType::Int32(), MachineType::TaggedPointer(),
                  MachineType::Int32(), MachineType::AnyTagged());
  m.Store(MachineRepresentation::kTagged, m.Parameter(0), m.Parameter(1),
          m.Parameter(2), barrier_kind);
  m.Return(m.Int32Constant(0));
  Stream s = m.Build(kAllExceptNopInstructions);
  // We have two instructions that are not nops: Store and Return.
  ASSERT_EQ(2U, s.size());
  EXPECT_EQ(kArchStoreWithWriteBarrier, s[0]->arch_opcode());
  EXPECT_EQ(3U, s[0]->InputCount());
  EXPECT_EQ(0U, s[1]->OutputCount());
}

TEST_P(InstructionSelectorStoreWithBarrierTest,
       StoreWithWriteBarrierImmediate) {
  const WriteBarrierKind barrier_kind = GetParam();
  TRACED_FOREACH(int32_t, index, kStoreWithBarrierImmediates) {
    StreamBuilder m(this, MachineType::Int32(), MachineType::TaggedPointer(),
                    MachineType::AnyTagged());
    m.Store(MachineRepresentation::kTagged, m.Parameter(0),
            m.Int32Constant(index), m.Parameter(1), barrier_kind);
    m.Return(m.Int32Constant(0));
    Stream s = m.Build(kAllExceptNopInstructions);
    // We have two instructions that are not nops: Store and Return.
    ASSERT_EQ(2U, s.size());
    EXPECT_EQ(kArchStoreWithWriteBarrier, s[0]->arch_opcode());
    EXPECT_EQ(3U, s[0]->InputCount());
    EXPECT_EQ(0U, s[0]->OutputCount());
  }
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorStoreWithBarrierTest,
                         ::testing::ValuesIn(kWriteBarrierKinds));

struct AtomicInst {
  const char* constructor_name;
  ArchOpcode arch_opcode;
  MachineType machine_type;
};

std::ostream& operator<<(std::ostream& os, const AtomicInst& inst) {
  return os << inst.constructor_name;
}

const AtomicInst AtomicStore[] = {
    {"Word32AtomicStoreWord8", kWord32AtomicStoreWord8, MachineType::Int8()},
    {"Word32AtomicStoreWord8", kWord32AtomicStoreWord8, MachineType::Uint8()},
    {"Word32AtomicStoreWord16", kWord32AtomicStoreWord16, MachineType::Int16()},
    {"Word32AtomicStoreWord16", kWord32AtomicStoreWord16,
     MachineType::Uint16()},
    {"Word32AtomicStoreWord32", kWord32AtomicStoreWord32, MachineType::Int32()},
    {"Word32AtomicStoreWord32", kWord32AtomicStoreWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicStoreWord64", kRISCV64Word64AtomicStoreWord64,
     MachineType::Int64()},
    {"RISCV64Word64AtomicStoreWord64", kRISCV64Word64AtomicStoreWord64,
     MachineType::Uint64()}};

using InstructionSelectorAtomicStore =
    InstructionSelectorTestWithParam<AtomicInst>;

TEST_P(InstructionSelectorAtomicStore, AtomicStoreWithParameters) {
  const AtomicInst inst = GetParam();
  const MachineType type = inst.machine_type;
  StreamBuilder m(this, type, MachineType::Pointer());
  m.Return(m.AtomicStore(type.representation(), m.Parameter(0),
                         m.Int32Constant(0), m.Int32Constant(0), NULL));
  Stream s = m.Build(kAllExceptNopInstructions);
  ASSERT_EQ(3U, s.size());
  EXPECT_EQ(inst.arch_opcode, s[1]->arch_opcode());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,
                         InstructionSelectorAtomicStore,
                         ::testing::ValuesIn(AtomicStore));

const AtomicInst AtomicLoad[] = {
    {"Word32AtomicLoadInt8", kWord32AtomicLoadInt8, MachineType::Int8()},
    {"Word32AtomicLoadUint8", kWord32AtomicLoadUint8, MachineType::Uint8()},
    {"Word32AtomicLoadInt16", kWord32AtomicLoadInt16, MachineType::Int16()},
    {"Word32AtomicLoadUint16", kWord32AtomicLoadUint16, MachineType::Uint16()},
    {"Word32AtomicLoadInt32", kWord32AtomicLoadWord32, MachineType::Int32()},
    {"Word32AtomicLoadUint32", kWord32AtomicLoadWord32, MachineType::Uint32()},
    {"RISCV64Word64AtomicLoadUint64", kRISCV64Word64AtomicLoadUint64,
     MachineType::Uint64()},
    //  {"RISCV64Word64AtomicLoadInt64", kRISCV64Word64AtomicLoadUint64,
    //  MachineType::Int64()}   //  TODO
};

using InstructionSelectorAtomicLoad =
    InstructionSelectorTestWithParam<AtomicInst>;

TEST_P(InstructionSelectorAtomicLoad, AtomicStoreWithParameters) {
  const AtomicInst inst = GetParam();
  const MachineType type = inst.machine_type;
  StreamBuilder m(this, type, MachineType::Pointer(), MachineType::Int32());
  m.Return(m.AtomicLoad(type, m.Parameter(0), m.Parameter(1)));
  Stream s = m.Build(kAllExceptNopInstructions);
  ASSERT_EQ(3U, s.size());
  EXPECT_EQ(inst.arch_opcode, s[1]->arch_opcode());
}

INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest, InstructionSelectorAtomicLoad,
                         ::testing::ValuesIn(AtomicLoad));

const AtomicInst AtomicExchange[] = {
    {"Word32AtomicExchangeInt8", kWord32AtomicExchangeInt8,
     MachineType::Int8()},
    {"Word32AtomicExchangeUint8", kWord32AtomicExchangeUint8,
     MachineType::Uint8()},
    {"Word32AtomicExchangeInt16", kWord32AtomicExchangeInt16,
     MachineType::Int16()},
    {"Word32AtomicExchangeUint16", kWord32AtomicExchangeUint16,
     MachineType::Uint16()},
    {"Word32AtomicExchangeWordInt32", kWord32AtomicExchangeWord32,
     MachineType::Int32()},
    {"Word32AtomicExchangeWordUint32", kWord32AtomicExchangeWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicExchangeWord64", kRISCV64Word64AtomicExchangeUint64,
     MachineType::Uint64()}};

const AtomicInst AtomicAdd[] = {
    {"Word32AtomicAddInt8", kWord32AtomicAddInt8, MachineType::Int8()},
    {"Word32AtomicAddUint8", kWord32AtomicAddUint8, MachineType::Uint8()},
    {"Word32AtomicAddInt16", kWord32AtomicAddInt16, MachineType::Int16()},
    {"Word32AtomicAddUint16", kWord32AtomicAddUint16, MachineType::Uint16()},
    {"Word32AtomicAddWordInt32", kWord32AtomicAddWord32, MachineType::Int32()},
    {"Word32AtomicAddWordUint32", kWord32AtomicAddWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicAddWord64", kRISCV64Word64AtomicAddUint64,
     MachineType::Uint64()}};

const AtomicInst AtomicAnd[] = {
    {"Word32AtomicAndInt8", kWord32AtomicAndInt8, MachineType::Int8()},
    {"Word32AtomicAndUint8", kWord32AtomicAndUint8, MachineType::Uint8()},
    {"Word32AtomicAndInt16", kWord32AtomicAndInt16, MachineType::Int16()},
    {"Word32AtomicAndUint16", kWord32AtomicAndUint16, MachineType::Uint16()},
    {"Word32AtomicAndWordInt32", kWord32AtomicAndWord32, MachineType::Int32()},
    {"Word32AtomicAndWordUint32", kWord32AtomicAndWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicAndWord64", kRISCV64Word64AtomicAndUint64,
     MachineType::Uint64()}};

const AtomicInst AtomicOr[] = {
    {"Word32AtomicOrInt8", kWord32AtomicOrInt8, MachineType::Int8()},
    {"Word32AtomicOrUint8", kWord32AtomicOrUint8, MachineType::Uint8()},
    {"Word32AtomicOrInt16", kWord32AtomicOrInt16, MachineType::Int16()},
    {"Word32AtomicOrUint16", kWord32AtomicOrUint16, MachineType::Uint16()},
    {"Word32AtomicOrWordInt32", kWord32AtomicOrWord32, MachineType::Int32()},
    {"Word32AtomicOrWordUint32", kWord32AtomicOrWord32, MachineType::Uint32()},
    {"RISCV64Word64AtomicOrWord64", kRISCV64Word64AtomicOrUint64,
     MachineType::Uint64()}};

const AtomicInst AtomicXor[] = {
    {"Word32AtomicXorInt8", kWord32AtomicXorInt8, MachineType::Int8()},
    {"Word32AtomicXorUint8", kWord32AtomicXorUint8, MachineType::Uint8()},
    {"Word32AtomicXorInt16", kWord32AtomicXorInt16, MachineType::Int16()},
    {"Word32AtomicXorUint16", kWord32AtomicXorUint16, MachineType::Uint16()},
    {"Word32AtomicXorWordInt32", kWord32AtomicXorWord32, MachineType::Int32()},
    {"Word32AtomicXorWordUint32", kWord32AtomicXorWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicXorWord64", kRISCV64Word64AtomicXorUint64,
     MachineType::Uint64()}};

const AtomicInst AtomicSub[] = {
    {"Word32AtomicSubInt8", kWord32AtomicSubInt8, MachineType::Int8()},
    {"Word32AtomicSubUint8", kWord32AtomicSubUint8, MachineType::Uint8()},
    {"Word32AtomicSubInt16", kWord32AtomicSubInt16, MachineType::Int16()},
    {"Word32AtomicSubUint16", kWord32AtomicSubUint16, MachineType::Uint16()},
    {"Word32AtomicSubWordInt32", kWord32AtomicSubWord32, MachineType::Int32()},
    {"Word32AtomicSubWordUint32", kWord32AtomicSubWord32,
     MachineType::Uint32()},
    {"RISCV64Word64AtomicSubWord64", kRISCV64Word64AtomicSubUint64,
     MachineType::Uint64()}};

#define ATOMIC_FUNCTION_UNITTEST(name)                                         \
  using InstructionSelectorAtomic##name =                                      \
      InstructionSelectorTestWithParam<AtomicInst>;                            \
  TEST_P(InstructionSelectorAtomic##name, AtomicWithParameters##name) {        \
    const AtomicInst inst = GetParam();                                        \
    const MachineType type = inst.machine_type;                                \
    StreamBuilder m(this, type, MachineType::Pointer(), MachineType::Int32()); \
    m.Return(m.Atomic##name(type, m.Parameter(0), m.Parameter(1),              \
                            m.Int32Constant(0), NULL));                        \
    Stream s = m.Build(kAllExceptNopInstructions);                             \
    \ 
      ASSERT_EQ(2U, s.size());                                                 \
    \             
      EXPECT_EQ(inst.arch_opcode, s[0]->arch_opcode());                        \
  }                                                                            \
                                                                               \
  INSTANTIATE_TEST_SUITE_P(InstructionSelectorTest,                            \
                           InstructionSelectorAtomic##name,                    \
                           ::testing::ValuesIn(Atomic##name));

ATOMIC_FUNCTION_UNITTEST(Exchange)
ATOMIC_FUNCTION_UNITTEST(Add)
ATOMIC_FUNCTION_UNITTEST(Sub)
ATOMIC_FUNCTION_UNITTEST(And)
ATOMIC_FUNCTION_UNITTEST(Or)
ATOMIC_FUNCTION_UNITTEST(Xor)
#undef ATOMIC_FUNCTION_UNITTEST

}  // namespace compiler
}  // namespace internal
}  // namespace v8