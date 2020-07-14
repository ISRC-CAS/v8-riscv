// Copyright 2014 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/base/iterator.h"
#include "src/compiler/backend/instruction-selector-impl.h"
#include "src/compiler/node-matchers.h"
#include "src/compiler/node-properties.h"

namespace v8 {
namespace internal {
namespace compiler {
#define TRACE_UNIMPL() \
  PrintF("UNIMPLEMENTED instr_sel: %s at line %d\n", __FUNCTION__, __LINE__)

#define TRACE() PrintF("instr_sel: %s at line %d\n", __FUNCTION__, __LINE__)
#if 0
#define UNIMPLEMENTED()                                                       \
  {                                                                           \
    PrintF("RV64-IS UNIMPLEMENTED: %s at line %d\n", __FUNCTION__, __LINE__); \
    RISCV64OperandGenerator g(this);                                          \
    Emit(kArchNop, g.NoOutput());                                             \
  }
#endif
// Adds RISICV-specific methods for generating InstructionOperands.
class RISCV64OperandGenerator final : public OperandGenerator {
 public:
  explicit RISCV64OperandGenerator(InstructionSelector* selector)
      : OperandGenerator(selector) {}

  InstructionOperand UseOperand(Node* node, InstructionCode opcode) {
    // use immediate operand first
    if (CanBeImmediate(node, opcode)) {
      return UseImmediate(node);
    }
    return UseRegister(node);
  }

  // Use the zero register if the node has the immediate value zero, otherwise
  // assign a register.
  InstructionOperand UseRegisterOrImmediateZero(Node* node) {
    if ((IsIntegerConstant(node) && (GetIntegerConstantValue(node) == 0)) ||
        (IsFloatConstant(node) &&
         (bit_cast<int64_t>(GetFloatConstantValue(node)) == 0))) {
      return UseImmediate(node);
    }
    return UseRegister(node);
  }
  // Judge whether the node is an integer constant
  bool IsIntegerConstant(Node* node) {
    return (node->opcode() == IrOpcode::kInt32Constant) ||
           (node->opcode() == IrOpcode::kInt64Constant);
  }

  // Get the integer constant value of the node
  int64_t GetIntegerConstantValue(Node* node) {
    if (node->opcode() == IrOpcode::kInt32Constant) {
      return OpParameter<int32_t>(node->op());
    }
    DCHECK_EQ(IrOpcode::kInt64Constant, node->opcode());
    return OpParameter<int64_t>(node->op());
  }

  // Judge whether the node is an float constant
  bool IsFloatConstant(Node* node) {
    return (node->opcode() == IrOpcode::kFloat32Constant) ||
           (node->opcode() == IrOpcode::kFloat64Constant);
  }

  // Get the float constant value of the node
  double GetFloatConstantValue(Node* node) {
    if (node->opcode() == IrOpcode::kFloat32Constant) {
      return OpParameter<float>(node->op());
    }
    DCHECK_EQ(IrOpcode::kFloat64Constant, node->opcode());
    return OpParameter<double>(node->op());
  }
  // Judge whether the instruction with the specified opcode can has immediate
  // operand.
  bool CanBeImmediate(Node* node, InstructionCode mode) {
    return IsIntegerConstant(node) &&
           CanBeImmediate(GetIntegerConstantValue(node), mode);
  }
  // Judge whether the instruction with the specified opcode can use "value" as
  // immediate operand.
  // need to be fixed ?
  bool CanBeImmediate(int64_t value, InstructionCode opcode) {
    switch (ArchOpcodeField::decode(opcode)) {
      // case kRISCV64Shl:
      case kRISCV64Sll:
      // case kRISCV64Sar:
      case kRISCV64Sra:
      // case kRISCV64Shr:
      case kRISCV64Srl:
        return is_uint6(value);
      case kRISCV64Sra32:  // rv64 only
      case kRISCV64Srl32:  // rv64 only
      case kRISCV64Sll32:  // rv64 only
        return is_uint5(value);
      case kRISCV64Add:
      case kRISCV64And:
      case kRISCV64Or:
      case kRISCV64Xor:
      case kRISCV64Add32:
        return is_int12(value);
      case kRISCV64Lb:
      case kRISCV64Lbu:
      case kRISCV64Sb:
      case kRISCV64Lh:
      case kRISCV64Lhu:
      case kRISCV64Sh:
      case kRISCV64Lw:
      case kRISCV64Lwu:  // rv64 only
      case kRISCV64Ld:   // rv64 only
      case kRISCV64Sw:
      case kRISCV64Sd:  // rv64 only
      case kRISCV64flw:
      case kRISCV64fsw:
      case kRISCV64fld:
      case kRISCV64fsd:
        return is_int12(value);
      default:
        break;
    }
    return false;
  }
};
// for instructions that have one def reg and one src reg.
static void VisitRR(InstructionSelector* selector, ArchOpcode opcode,
                    Node* node) {
  RISCV64OperandGenerator g(selector);
  selector->Emit(opcode, g.DefineAsRegister(node),
                 g.UseRegister(node->InputAt(0)));
}
// for instructions that have one def reg and two src regs.
static void VisitRRR(InstructionSelector* selector, ArchOpcode opcode,
                     Node* node) {
  RISCV64OperandGenerator g(selector);
  selector->Emit(opcode, g.DefineAsRegister(node),
                 g.UseRegister(node->InputAt(0)),
                 g.UseRegister(node->InputAt(1)));
}
#if 0
static void VisitRRI(InstructionSelector* selector, ArchOpcode opcode,
                     Node* node) {
  RISCV64OperandGenerator g(selector);
  int32_t imm = OpParameter<int32_t>(node->op());
  selector->Emit(opcode, g.DefineAsRegister(node),
                 g.UseRegister(node->InputAt(0)), g.UseImmediate(imm));
}
#endif
// for instructions that have a def reg, two regs or reg + immediate src.
static void VisitRRO(InstructionSelector* selector, ArchOpcode opcode,
                     Node* node) {
  RISCV64OperandGenerator g(selector);
  selector->Emit(opcode, g.DefineAsRegister(node),
                 g.UseRegister(node->InputAt(0)),
                 g.UseOperand(node->InputAt(1), opcode));
}
// try to match immediate operand
bool TryMatchImmediate(InstructionSelector* selector,
                       InstructionCode* opcode_return, Node* node,
                       size_t* input_count_return, InstructionOperand* inputs) {
  RISCV64OperandGenerator g(selector);
  if (g.CanBeImmediate(node, *opcode_return)) {
    *opcode_return |= AddressingModeField::encode(kMode_MRI);
    inputs[0] = g.UseImmediate(node);
    *input_count_return = 1;
    return true;
  }
  return false;
}
static void VisitBinop(InstructionSelector* selector, Node* node,
                       InstructionCode opcode, bool has_reverse_opcode,
                       InstructionCode reverse_opcode,
                       FlagsContinuation* cont) {
  RISCV64OperandGenerator g(selector);
  Int32BinopMatcher m(node);
  InstructionOperand inputs[2];
  size_t input_count = 0;
  InstructionOperand outputs[1];
  size_t output_count = 0;
  // if the right operand is immediate, operands are Reg + Imm
  if (TryMatchImmediate(selector, &opcode, m.right().node(), &input_count,
                        &inputs[1])) {
    inputs[0] = g.UseRegister(m.left().node());
    input_count++;
  } else if (has_reverse_opcode &&
             TryMatchImmediate(selector, &reverse_opcode, m.left().node(),
                               &input_count, &inputs[1])) {
    // if the left operand is immediate, operands are Reg + Imm
    inputs[0] = g.UseRegister(m.right().node());
    opcode = reverse_opcode;
    input_count++;
  } else {
    // operands are Reg + Reg
    inputs[input_count++] = g.UseRegister(m.left().node());
    inputs[input_count++] = g.UseOperand(m.right().node(), opcode);
  }

  if (cont->IsDeoptimize()) {
    // If we can deoptimize as a result of the binop, we need to make sure that
    // the deopt inputs are not overwritten by the binop result. One way
    // to achieve that is to declare the output register as same-as-first.
    outputs[output_count++] = g.DefineSameAsFirst(node);
  } else {
    outputs[output_count++] = g.DefineAsRegister(node);
  }

  DCHECK_NE(0u, input_count);
  DCHECK_EQ(1u, output_count);
  DCHECK_GE(arraysize(inputs), input_count);
  DCHECK_GE(arraysize(outputs), output_count);

  selector->EmitWithContinuation(opcode, output_count, outputs, input_count,
                                 inputs, cont);
}

static void VisitBinop(InstructionSelector* selector, Node* node,
                       InstructionCode opcode, bool has_reverse_opcode,
                       InstructionCode reverse_opcode) {
  FlagsContinuation cont;
  VisitBinop(selector, node, opcode, has_reverse_opcode, reverse_opcode, &cont);
}

static void VisitBinop(InstructionSelector* selector, Node* node,
                       InstructionCode opcode, FlagsContinuation* cont) {
  VisitBinop(selector, node, opcode, false, kArchNop, cont);
}

static void VisitBinop(InstructionSelector* selector, Node* node,
                       InstructionCode opcode) {
  VisitBinop(selector, node, opcode, false, kArchNop);
}
// plct origin Visit begin
void InstructionSelector::VisitStackSlot(Node* node) {
  // std::cout<<"PLCT VisitStackSlot"<<std::endl;
  StackSlotRepresentation rep = StackSlotRepresentationOf(node->op());
  int slot = frame_->AllocateSpillSlot(rep.size());
  OperandGenerator g(this);

  Emit(kArchStackSlot, g.DefineAsRegister(node),
       sequence()->AddImmediate(Constant(slot)), 0, nullptr);
}

void InstructionSelector::VisitAbortCSAAssert(Node* node) {
  RISCV64OperandGenerator g(this);
  Emit(kArchAbortCSAAssert, g.NoOutput(), g.UseFixed(node->InputAt(0), a0));
}
void EmitLoad(InstructionSelector* selector, Node* node, InstructionCode opcode,
              Node* output = nullptr) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  // address is (baseReg, index)
  if (g.CanBeImmediate(index, opcode)) {
    selector->Emit(opcode | AddressingModeField::encode(kMode_MRI),
                   g.DefineAsRegister(output == nullptr ? node : output),
                   g.UseRegister(base), g.UseImmediate(index));
  } else {
    // address is (baseReg + index, 0)
    InstructionOperand addr_reg = g.TempRegister();
    selector->Emit(kRISCV64Add | AddressingModeField::encode(kMode_None),
                   addr_reg, g.UseRegister(index), g.UseRegister(base));
    // Emit desired load opcode, using temp addr_reg.
    selector->Emit(opcode | AddressingModeField::encode(kMode_MRI),
                   g.DefineAsRegister(output == nullptr ? node : output),
                   addr_reg, g.TempImmediate(0));
  }
}

void InstructionSelector::VisitLoad(Node* node) {
  // std::cout<<"PLCT VisitLoad"<<std::endl;
  LoadRepresentation load_rep = LoadRepresentationOf(node->op());
  RISCV64OperandGenerator g(this);

  InstructionCode opcode = kArchNop;
  switch (load_rep.representation()) {
    case MachineRepresentation::kBit:  // Fall through.
    case MachineRepresentation::kWord8:
      // Load Byte
      opcode = load_rep.IsUnsigned() ? kRISCV64Lbu : kRISCV64Lb;
      break;
    case MachineRepresentation::kWord16:
      // Load Halfword
      opcode = load_rep.IsUnsigned() ? kRISCV64Lhu : kRISCV64Lh;
      break;
    case MachineRepresentation::kWord32:
      // Load word
      opcode = load_rep.IsUnsigned() ? kRISCV64Lwu : kRISCV64Lw;
      break;
    case MachineRepresentation::kTaggedSigned:   // Fall through.
    case MachineRepresentation::kTaggedPointer:  // Fall through.
    case MachineRepresentation::kTagged:         // Fall through.
    case MachineRepresentation::kWord64:
      // Load Doubleword
      opcode = kRISCV64Ld;
      break;
    case MachineRepresentation::kFloat32:
      // Floating-point Load Word
      opcode = kRISCV64flw;
      break;
      // Floating-point Load Doubleword
    case MachineRepresentation::kFloat64:
      opcode = kRISCV64fld;
      break;
    case MachineRepresentation::kSimd128:
    case MachineRepresentation::kCompressedPointer:  // Fall through.
    case MachineRepresentation::kCompressed:         // Fall through.
    case MachineRepresentation::kNone:
      UNREACHABLE();
  }
  if (node->opcode() == IrOpcode::kPoisonedLoad) {
    CHECK_NE(poisoning_level_, PoisoningMitigationLevel::kDontPoison);
    opcode |= MiscField::encode(kMemoryAccessPoisoned);
  }

  EmitLoad(this, node, opcode);
}

void InstructionSelector::VisitPoisonedLoad(Node* node) { VisitLoad(node); }

void InstructionSelector::VisitProtectedLoad(Node* node) {
  // TODO(eholk)
  // refer to arm/arm64, mips/mips64, ppc etc
  // std::cout<<"PLCT VisitProtectedLoad"<<std::endl;
  UNIMPLEMENTED();
}

void InstructionSelector::VisitStore(Node* node) {
  // std::cout<<"PLCT VisitStore"<<std::endl;
  RISCV64OperandGenerator g(this);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  Node* value = node->InputAt(2);

  StoreRepresentation store_rep = StoreRepresentationOf(node->op());
  WriteBarrierKind write_barrier_kind = store_rep.write_barrier_kind();
  MachineRepresentation rep = store_rep.representation();

  // TODO(riscv): I guess this could be done in a better way.
  if (write_barrier_kind != kNoWriteBarrier &&
      V8_LIKELY(!FLAG_disable_write_barriers)) {
    DCHECK(CanBeTaggedPointer(rep));
    InstructionOperand inputs[3];
    size_t input_count = 0;
    inputs[input_count++] = g.UseUniqueRegister(base);
    inputs[input_count++] = g.UseUniqueRegister(index);
    inputs[input_count++] = g.UseUniqueRegister(value);
    RecordWriteMode record_write_mode =
        WriteBarrierKindToRecordWriteMode(write_barrier_kind);
    InstructionOperand temps[] = {g.TempRegister(), g.TempRegister()};
    size_t const temp_count = arraysize(temps);
    InstructionCode code = kArchStoreWithWriteBarrier;
    code |= MiscField::encode(static_cast<int>(record_write_mode));
    Emit(code, 0, nullptr, input_count, inputs, temp_count, temps);
  } else {
    ArchOpcode opcode = kArchNop;
    switch (rep) {
      case MachineRepresentation::kBit:  // Fall through.
      case MachineRepresentation::kWord8:
        // Store Byte
        opcode = kRISCV64Sb;
        break;
      case MachineRepresentation::kWord16:
        // Store Halfword
        opcode = kRISCV64Sh;
        break;
      case MachineRepresentation::kWord32:
        // Store word
        opcode = kRISCV64Sw;
        break;
      case MachineRepresentation::kTaggedSigned:   // Fall through.
      case MachineRepresentation::kTaggedPointer:  // Fall through.
      case MachineRepresentation::kTagged:         // Fall through.
      case MachineRepresentation::kWord64:
        // Store Doubleword
        opcode = kRISCV64Sd;
        break;
      case MachineRepresentation::kFloat32:
        // fsw, Floating-point Store Word
        opcode = kRISCV64fsw;
        break;
      case MachineRepresentation::kFloat64:
        // fsd, Floating-point Store Doubleword
        opcode = kRISCV64fsd;
        break;

      case MachineRepresentation::kSimd128:
      case MachineRepresentation::kCompressedPointer:  // Fall through.
      case MachineRepresentation::kCompressed:         // Fall through.
      case MachineRepresentation::kNone:
        UNREACHABLE();
        return;
    }

    // memory is (baseReg, index)
    if (g.CanBeImmediate(index, opcode)) {
      Emit(opcode | AddressingModeField::encode(kMode_MRI), g.NoOutput(),
           g.UseRegister(base), g.UseImmediate(index),
           g.UseRegisterOrImmediateZero(value));
    } else {
      // memory is (baseReg + index, 0)
      InstructionOperand addr_reg = g.TempRegister();
      Emit(kRISCV64Add | AddressingModeField::encode(kMode_None), addr_reg,
           g.UseRegister(index), g.UseRegister(base));
      // Emit desired store opcode, using temp addr_reg.
      Emit(opcode | AddressingModeField::encode(kMode_MRI), g.NoOutput(),
           addr_reg, g.TempImmediate(0), g.UseRegisterOrImmediateZero(value));
    }
  }
}

void InstructionSelector::VisitProtectedStore(Node* node) {
  // TODO(eholk)
  // refer to arm/arm64, mips/mips64, ppc etc
  UNIMPLEMENTED();
}

// Architecture supports unaligned access, therefore VisitLoad is used instead
void InstructionSelector::VisitUnalignedLoad(Node* node) { UNIMPLEMENTED(); }

// Architecture supports unaligned access, therefore VisitStore is used instead
void InstructionSelector::VisitUnalignedStore(Node* node) { UNIMPLEMENTED(); }

namespace {
static void VisitCompare(InstructionSelector* selector, InstructionCode opcode,
                         InstructionOperand left, InstructionOperand right,
                         FlagsContinuation* cont) {
  selector->EmitWithContinuation(opcode, left, right, cont);
}

// Shared routine for multiple float32 compare operations.
void VisitFloat32Compare(InstructionSelector* selector, Node* node,
                         FlagsContinuation* cont) {
  RISCV64OperandGenerator g(selector);
  Float32BinopMatcher m(node);
  InstructionOperand lhs, rhs;

  lhs = g.UseRegister(m.left().node());
  rhs = g.UseRegister(m.right().node());
  VisitCompare(selector, kRISCV64CmpS, lhs, rhs, cont);
}

// Shared routine for multiple float64 compare operations.
void VisitFloat64Compare(InstructionSelector* selector, Node* node,
                         FlagsContinuation* cont) {
  RISCV64OperandGenerator g(selector);
  Float64BinopMatcher m(node);
  InstructionOperand lhs, rhs;

  lhs = g.UseRegister(m.left().node());
  rhs = g.UseRegister(m.right().node());
  VisitCompare(selector, kRISCV64CmpD, lhs, rhs, cont);
}
// Shared routine for multiple word compare operations.
void VisitWordCompare(InstructionSelector* selector, Node* node,
                      InstructionCode opcode, FlagsContinuation* cont,
                      bool commutative) {
  RISCV64OperandGenerator g(selector);
  Node* left = node->InputAt(0);
  Node* right = node->InputAt(1);

  VisitCompare(selector, opcode, g.UseRegister(left), g.UseRegister(right),
               cont);
}
// Shared routine for multiple word compare operations.
void VisitFullWord32Compare(InstructionSelector* selector, Node* node,
                            InstructionCode opcode, FlagsContinuation* cont) {
  RISCV64OperandGenerator g(selector);
  InstructionOperand leftOp = g.TempRegister();
  InstructionOperand rightOp = g.TempRegister();

  selector->Emit(kRISCV64Sll, leftOp, g.UseRegister(node->InputAt(0)),
                 g.TempImmediate(32));
  selector->Emit(kRISCV64Sll, rightOp, g.UseRegister(node->InputAt(1)),
                 g.TempImmediate(32));

  VisitCompare(selector, opcode, leftOp, rightOp, cont);
}
void VisitWord32Compare(InstructionSelector* selector, Node* node,
                        FlagsContinuation* cont) {
  // RISCV64 doesn't support Word32 compare instructions. Instead it relies
  // that the values in registers are correctly sign-extended and uses
  // Word64 comparison instead. This behavior is correct in most cases,
  // but doesn't work when comparing signed with unsigned operands.
  // We could simulate full Word32 compare in all cases but this would
  // create an unnecessary overhead since unsigned integers are rarely
  // used in JavaScript.
  // to be Optimized
  VisitFullWord32Compare(selector, node, kRISCV64Cmp, cont);
}

void VisitWord64Compare(InstructionSelector* selector, Node* node,
                        FlagsContinuation* cont) {
  VisitWordCompare(selector, node, kRISCV64Cmp, cont, false);
}

void VisitAtomicLoad(InstructionSelector* selector, Node* node,
                     ArchOpcode opcode) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  InstructionOperand addr_reg = g.TempRegister();
  selector->Emit(kRISCV64Add | AddressingModeField::encode(kMode_None),
                 addr_reg, g.UseRegister(index), g.UseRegister(base));
  // Emit desired load opcode, using temp addr_reg.
  // lr.w rd, (rs1)
  selector->Emit(opcode | AddressingModeField::encode(kMode_MRI),
                 g.DefineAsRegister(node), addr_reg, g.TempImmediate(0));
}

void VisitAtomicStore(InstructionSelector* selector, Node* node,
                      ArchOpcode opcode) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  Node* value = node->InputAt(2);

  InstructionOperand addr_reg = g.TempRegister();
  selector->Emit(kRISCV64Add | AddressingModeField::encode(kMode_None),
                 addr_reg, g.UseRegister(index), g.UseRegister(base));
  // Emit desired store opcode, using temp addr_reg.
  selector->Emit(opcode | AddressingModeField::encode(kMode_MRI), g.NoOutput(),
                 addr_reg, g.TempImmediate(0),
                 g.UseRegisterOrImmediateZero(value));
}
void VisitAtomicExchange(InstructionSelector* selector, Node* node,
                         ArchOpcode opcode) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  Node* value = node->InputAt(2);

  AddressingMode addressing_mode = kMode_MRI;
  InstructionOperand inputs[3];
  size_t input_count = 0;
  inputs[input_count++] = g.UseUniqueRegister(base);
  inputs[input_count++] = g.UseUniqueRegister(index);
  inputs[input_count++] = g.UseUniqueRegister(value);
  InstructionOperand outputs[1];
  outputs[0] = g.UseUniqueRegister(node);
  InstructionOperand temp[3];
  temp[0] = g.TempRegister();
  temp[1] = g.TempRegister();
  temp[2] = g.TempRegister();
  InstructionCode code = opcode | AddressingModeField::encode(addressing_mode);
  selector->Emit(code, 1, outputs, input_count, inputs, 3, temp);
}

void VisitAtomicCompareExchange(InstructionSelector* selector, Node* node,
                                ArchOpcode opcode) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  Node* old_value = node->InputAt(2);
  Node* new_value = node->InputAt(3);

  AddressingMode addressing_mode = kMode_MRI;
  InstructionOperand inputs[4];
  size_t input_count = 0;
  inputs[input_count++] = g.UseUniqueRegister(base);
  inputs[input_count++] = g.UseUniqueRegister(index);
  inputs[input_count++] = g.UseUniqueRegister(old_value);
  inputs[input_count++] = g.UseUniqueRegister(new_value);
  InstructionOperand outputs[1];
  outputs[0] = g.UseUniqueRegister(node);
  InstructionOperand temp[3];
  temp[0] = g.TempRegister();
  temp[1] = g.TempRegister();
  temp[2] = g.TempRegister();
  InstructionCode code = opcode | AddressingModeField::encode(addressing_mode);
  selector->Emit(code, 1, outputs, input_count, inputs, 3, temp);
}

void VisitAtomicBinop(InstructionSelector* selector, Node* node,
                      ArchOpcode opcode) {
  RISCV64OperandGenerator g(selector);
  Node* base = node->InputAt(0);
  Node* index = node->InputAt(1);
  Node* value = node->InputAt(2);

  AddressingMode addressing_mode = kMode_MRI;
  InstructionOperand inputs[3];
  size_t input_count = 0;
  inputs[input_count++] = g.UseUniqueRegister(base);
  inputs[input_count++] = g.UseUniqueRegister(index);
  inputs[input_count++] = g.UseUniqueRegister(value);
  InstructionOperand outputs[1];
  outputs[0] = g.UseUniqueRegister(node);
  InstructionOperand temps[4];
  temps[0] = g.TempRegister();
  temps[1] = g.TempRegister();
  temps[2] = g.TempRegister();
  temps[3] = g.TempRegister();
  InstructionCode code = opcode | AddressingModeField::encode(addressing_mode);
  selector->Emit(code, 1, outputs, input_count, inputs, 4, temps);
}

}  // namespace

void InstructionSelector::VisitWord32And(Node* node) {
  RISCV64OperandGenerator g(this);
  // No andw/andiw instructions, use and/andi instead
  InstructionOperand tmp_reg = g.TempRegister();
  Emit(kRISCV64And, tmp_reg, g.UseRegister(node->InputAt(0)),
       g.UseOperand(node->InputAt(1), kRISCV64And));
  // truncate and signed extend, same with VisitTruncateInt64ToInt32.
  // addiw, rd, rs, 0
  Emit(kRISCV64Add32, g.DefineAsRegister(node), tmp_reg, g.TempImmediate(0));
}

void InstructionSelector::VisitWord32Or(Node* node) {
  RISCV64OperandGenerator g(this);
  // No orw/oriw instructions, use or/ori instead
  InstructionOperand tmp_reg = g.TempRegister();
  Emit(kRISCV64Or, tmp_reg, g.UseRegister(node->InputAt(0)),
       g.UseOperand(node->InputAt(1), kRISCV64Or));
  // truncate and signed extend, same with VisitTruncateInt64ToInt32.
  // use addiw, rd, rs, 0
  Emit(kRISCV64Add32, g.DefineAsRegister(node), tmp_reg, g.TempImmediate(0));
}

void InstructionSelector::VisitWord32Xor(Node* node) {
  RISCV64OperandGenerator g(this);
  // No xorw/xori instructions, use xor/xori instead
  InstructionOperand tmp_reg = g.TempRegister();
  Emit(kRISCV64Xor, tmp_reg, g.UseRegister(node->InputAt(0)),
       g.UseOperand(node->InputAt(1), kRISCV64Xor));
  // truncate and signed extend,  same with VisitTruncateInt64ToInt32.
  // use addiw, rd, rs, 0
  Emit(kRISCV64Add32, g.DefineAsRegister(node), tmp_reg, g.TempImmediate(0));
}

void InstructionSelector::VisitStackPointerGreaterThan(
    Node* node, FlagsContinuation* cont) {
  StackCheckKind kind = StackCheckKindOf(node->op());
  InstructionCode opcode =
      kArchStackPointerGreaterThan | MiscField::encode(static_cast<int>(kind));

  RISCV64OperandGenerator g(this);

  // No outputs.
  InstructionOperand* const outputs = nullptr;
  const int output_count = 0;

  // Applying an offset to this stack check requires a temp register. Offsets
  // are only applied to the first stack check. If applying an offset, we must
  // ensure the input and temp registers do not alias, thus kUniqueRegister.
  InstructionOperand temps[] = {g.TempRegister()};
  const int temp_count = (kind == StackCheckKind::kJSFunctionEntry ? 1 : 0);
  const auto register_mode = (kind == StackCheckKind::kJSFunctionEntry)
                                 ? OperandGenerator::kUniqueRegister
                                 : OperandGenerator::kRegister;

  Node* const value = node->InputAt(0);
  InstructionOperand inputs[] = {g.UseRegisterWithMode(value, register_mode)};
  static constexpr int input_count = arraysize(inputs);

  EmitWithContinuation(opcode, output_count, outputs, input_count, inputs,
                       temp_count, temps, cont);
}

void InstructionSelector::VisitWord32Shl(Node* node) {
  // sllw, slliw
  VisitRRO(this, kRISCV64Sll32, node);
}

void InstructionSelector::VisitWord32Shr(Node* node) {
  // srlw, srliw
  VisitRRO(this, kRISCV64Srl32, node);
}

void InstructionSelector::VisitWord32Sar(Node* node) {
  // sraw, sraiw
  VisitRRO(this, kRISCV64Sra32, node);
}

void InstructionSelector::VisitWord32Ror(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord32Clz(Node* node) {
  // riscv does't hava clz instruction, tbd?
  VisitRR(this, kRISCV64Clz, node);
}
void InstructionSelector::VisitWord32Ctz(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitWord32Popcnt(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitChangeFloat32ToFloat64(Node* node) {
  // fcvt.d.s rd, rs1, rs2
  // Floating-point Convert to Double from Single
  VisitRR(this, kRISCV64FcvtDS, node);
}
void InstructionSelector::VisitRoundInt32ToFloat32(Node* node) {
  // fcvt.s.w rd, rs1, rs2
  // Floating-point Convert to Single from Word
  VisitRR(this, kRISCV64FcvtSW, node);
}
void InstructionSelector::VisitChangeInt32ToFloat64(Node* node) {
  // fcvt.d.w rd, rs1, rs2
  // Floating-point Convert to Double from Word
  VisitRR(this, kRISCV64FcvtDW, node);
}
void InstructionSelector::VisitTruncateFloat32ToInt32(Node* node) {
  // fcvt.w.s rd, rs1, rs2
  // Floating-point Convert to Word from Single
  VisitRR(this, kRISCV64FcvtWS, node);
}
void InstructionSelector::VisitChangeFloat64ToInt32(Node* node) {
  // fcvt.w.d rd, rs1, rs2
  // Floating-point Convert to Word from Double
  VisitRR(this, kRISCV64FcvtWD, node);
}
void InstructionSelector::VisitTruncateFloat64ToFloat32(Node* node) {
  // fcvt.s.d rd, rs1, rs2
  // Floating-point Convert to Single from Double
  VisitRR(this, kRISCV64FcvtSD, node);
}
void InstructionSelector::VisitRoundFloat64ToInt32(Node* node) {
  // same as VisitChangeFloat64ToInt32
  // fcvt.w.d rd, rs1, rs2
  // Floating-point Convert to Word from Double
  VisitRR(this, kRISCV64FcvtWD, node);
}
void InstructionSelector::VisitBitcastFloat32ToInt32(Node* node) {
  // fmv.w.x
  VisitRR(this, kRISCV64FMVWX, node);
}
void InstructionSelector::VisitBitcastInt32ToFloat32(Node* node) {
  // fmv.x.w
  VisitRR(this, kRISCV64FMVXW, node);
}
void InstructionSelector::VisitFloat64ExtractLowWord32(Node* node) {
  VisitRR(this, kRISCV64Float64ExtractLowWord32, node);
}
void InstructionSelector::VisitFloat64ExtractHighWord32(Node* node) {
  VisitRR(this, kRISCV64Float64ExtractHighWord32, node);
}
void InstructionSelector::VisitSignExtendWord8ToInt32(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitSignExtendWord16ToInt32(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitChangeUint32ToFloat64(Node* node) {
  // fcvt.d.wu rd, rs1, rs2
  //  Floating-point Convert to Double from Unsigned Word
  VisitRR(this, kRISCV64FcvtDWu, node);
}
void InstructionSelector::VisitTruncateFloat64ToWord32(Node* node) {
  // common instruction opcode, refer to arm64, mips64
  VisitRR(this, kArchTruncateDoubleToI, node);
}

void InstructionSelector::VisitWord32ReverseBits(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitWord64ReverseBytes(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitWord32ReverseBytes(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitSimd128ReverseBytes(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitInt32Add(Node* node) {
  // addw, addiw
  VisitBinop(this, node, kRISCV64Add32, true, kRISCV64Add32);
}

void InstructionSelector::VisitInt32Sub(Node* node) {
  // subw
  VisitBinop(this, node, kRISCV64Sub32);
}

void InstructionSelector::VisitInt32Mul(Node* node) {
  // mulw
  VisitRRR(this, kRISCV64Mul32, node);
}

void InstructionSelector::VisitInt32MulHigh(Node* node) {
  // No mulhw, use mulh instead?
  VisitRRR(this, kRISCV64MulHigh32, node);
}

void InstructionSelector::VisitUint32MulHigh(Node* node) {
  // No mulhuw, use mulhu instead?
  VisitRRR(this, kRISCV64MulHighU32, node);
}

void InstructionSelector::VisitInt32Div(Node* node) {
  RISCV64OperandGenerator g(this);
  Int32BinopMatcher m(node);
  // divw
  Emit(kRISCV64Div32, g.DefineSameAsFirst(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitUint32Div(Node* node) {
  // divuw
  RISCV64OperandGenerator g(this);
  Int32BinopMatcher m(node);
  Emit(kRISCV64DivU32, g.DefineSameAsFirst(node),
       g.UseRegister(m.left().node()), g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitInt32Mod(Node* node) {
  // remw
  VisitRRR(this, kRISCV64Mod32, node);
}

void InstructionSelector::VisitUint32Mod(Node* node) {
  // remuw
  VisitRRR(this, kRISCV64ModU32, node);
}

void InstructionSelector::VisitRoundUint32ToFloat32(Node* node) {
  // fcvt.s.wu
  // Floating-point Convert to Single from Unsigned Word
  VisitRR(this, kRISCV64FcvtSUw, node);
}

void InstructionSelector::VisitFloat32Add(Node* node) {
  // fadd.s
  VisitRRR(this, kRISCV64AddS, node);
}
void InstructionSelector::VisitFloat64Add(Node* node) {
  // fadd.d
  VisitRRR(this, kRISCV64AddD, node);
}
void InstructionSelector::VisitFloat32Sub(Node* node) {
  // fsub.s
  VisitRRR(this, kRISCV64SubS, node);
}
void InstructionSelector::VisitFloat64Sub(Node* node) {
  // fsub.d
  VisitRRR(this, kRISCV64SubD, node);
}
void InstructionSelector::VisitFloat32Mul(Node* node) {
  // fmul.s
  VisitRRR(this, kRISCV64MulS, node);
}
void InstructionSelector::VisitFloat64Mul(Node* node) {
  // fmul.d
  VisitRRR(this, kRISCV64MulD, node);
}
void InstructionSelector::VisitFloat32Div(Node* node) {
  // fdiv.s
  VisitRRR(this, kRISCV64DivS, node);
}
void InstructionSelector::VisitFloat64Div(Node* node) {
  // fdiv.d
  VisitRRR(this, kRISCV64DivD, node);
}

void InstructionSelector::VisitFloat64Mod(Node* node) {
  RISCV64OperandGenerator g(this);
  // f10 / fa0, FP Function argument, return value
  // f11 / fa1, FP Function argument, return value
  // f12  ~ f17, FP Function argument
  Emit(kRISCV64ModD, g.DefineAsFixed(node, fa0),
       g.UseFixed(node->InputAt(0), fa0), g.UseFixed(node->InputAt(1), fa1))
      ->MarkAsCall();
}

void InstructionSelector::VisitFloat32Max(Node* node) {
  // fmax.s rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Emit(kRISCV64Float32Max, g.DefineAsRegister(node),
       g.UseRegister(node->InputAt(0)), g.UseRegister(node->InputAt(1)));
}

void InstructionSelector::VisitFloat64Max(Node* node) {
  // fmax.d rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Emit(kRISCV64Float64Max, g.DefineAsRegister(node),
       g.UseRegister(node->InputAt(0)), g.UseRegister(node->InputAt(1)));
}

void InstructionSelector::VisitFloat32Min(Node* node) {
  // fmin.s rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Emit(kRISCV64Float32Min, g.DefineAsRegister(node),
       g.UseRegister(node->InputAt(0)), g.UseRegister(node->InputAt(1)));
}

void InstructionSelector::VisitFloat64Min(Node* node) {
  // fmin.d rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Emit(kRISCV64Float64Min, g.DefineAsRegister(node),
       g.UseRegister(node->InputAt(0)), g.UseRegister(node->InputAt(1)));
}
void InstructionSelector::VisitFloat32Abs(Node* node) {
  // fabs.s rd, rs1
  // Pesudo instruction
  VisitRR(this, kRISCV64AbsS, node);
}

void InstructionSelector::VisitFloat64Abs(Node* node) {
  // fabs.d rd, rs1
  // Pesudo instruction
  VisitRR(this, kRISCV64AbsD, node);
}

void InstructionSelector::VisitFloat32Sqrt(Node* node) {
  // fsqrt.s rd, rs1, rs2
  VisitRR(this, kRISCV64SqrtS, node);
}

void InstructionSelector::VisitFloat64Sqrt(Node* node) {
  // fsqrt.d rd, rs1, rs2
  VisitRR(this, kRISCV64SqrtD, node);
}

void InstructionSelector::VisitFloat32RoundDown(Node* node) {
  VisitRR(this, kRISCV64Float32RoundDown, node);
}

void InstructionSelector::VisitFloat64RoundDown(Node* node) {
  VisitRR(this, kRISCV64Float64RoundDown, node);
}

void InstructionSelector::VisitFloat32RoundUp(Node* node) {
  VisitRR(this, kRISCV64Float32RoundUp, node);
}

void InstructionSelector::VisitFloat64RoundUp(Node* node) {
  VisitRR(this, kRISCV64Float64RoundUp, node);
}

void InstructionSelector::VisitFloat32RoundTruncate(Node* node) {
  VisitRR(this, kRISCV64Float32RoundTruncate, node);
}

void InstructionSelector::VisitFloat64RoundTruncate(Node* node) {
  VisitRR(this, kRISCV64Float64RoundTruncate, node);
}

void InstructionSelector::VisitFloat64RoundTiesAway(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitFloat32RoundTiesEven(Node* node) {
  VisitRR(this, kRISCV64Float32RoundTiesEven, node);
}

void InstructionSelector::VisitFloat64RoundTiesEven(Node* node) {
  VisitRR(this, kRISCV64Float64RoundTiesEven, node);
}

void InstructionSelector::VisitFloat32Neg(Node* node) {
  // fneg.s rd, rs1
  // Pesudo instrucpion
  VisitRR(this, kRISCV64NegS, node);
}

void InstructionSelector::VisitFloat64Neg(Node* node) {
  // fneg.d rd, rs1
  // Pesudo instruction
  VisitRR(this, kRISCV64NegD, node);
}
void InstructionSelector::VisitFloat64Ieee754Binop(Node* node,
                                                   InstructionCode opcode) {
  RISCV64OperandGenerator g(this);
  // f10 / fa0, FP Function argument, return value
  // f11 / fa1, FP Function argument, return value
  // f12  ~ f17, FP Function argument
  Emit(opcode, g.DefineAsFixed(node, fa0), g.UseFixed(node->InputAt(0), fa0),
       g.UseFixed(node->InputAt(1), fa1))
      ->MarkAsCall();
}

void InstructionSelector::VisitFloat64Ieee754Unop(Node* node,
                                                  InstructionCode opcode) {
  RISCV64OperandGenerator g(this);
  Emit(opcode, g.DefineAsFixed(node, fa0), g.UseFixed(node->InputAt(0), fa0))
      ->MarkAsCall();
}

void InstructionSelector::EmitPrepareArguments(
    ZoneVector<PushParameter>* arguments, const CallDescriptor* call_descriptor,
    Node* node) {
  RISCV64OperandGenerator g(this);

  // Prepare for C function call.
  if (call_descriptor->IsCFunctionCall()) {
    Emit(kArchPrepareCallCFunction | MiscField::encode(static_cast<int>(
                                         call_descriptor->ParameterCount())),
         0, nullptr, 0, nullptr);

    // Poke any stack arguments.
    // no slot alloc for the a0~a7 arguments
    int slot = kCArgSlotCount;
    for (PushParameter input : (*arguments)) {
      if (input.node) {
        Emit(kRISCV64StoreToStackSlot, g.NoOutput(), g.UseRegister(input.node),
             g.TempImmediate(slot << kSystemPointerSizeLog2));
        ++slot;
      }
    }
  } else {
    // Possibly align stack here for functions.
    int push_count = static_cast<int>(call_descriptor->StackParameterCount());
    if (push_count > 0) {
      // Calculate needed space
      int stack_size = 0;
      for (size_t n = 0; n < arguments->size(); ++n) {
        PushParameter input = (*arguments)[n];
        if (input.node) {
          stack_size += input.location.GetSizeInPointers();
        }
      }
      Emit(kRISCV64StackClaim, g.NoOutput(),
           g.TempImmediate(stack_size << kSystemPointerSizeLog2));
    }
    for (size_t n = 0; n < arguments->size(); ++n) {
      PushParameter input = (*arguments)[n];
      if (input.node) {
        Emit(kRISCV64StoreToStackSlot, g.NoOutput(), g.UseRegister(input.node),
             g.TempImmediate(n << kSystemPointerSizeLog2));
      }
    }
  }
}

void InstructionSelector::EmitPrepareResults(
    ZoneVector<PushParameter>* results, const CallDescriptor* call_descriptor,
    Node* node) {
  RISCV64OperandGenerator g(this);

  int reverse_slot = 0;
  for (PushParameter output : *results) {
    if (!output.location.IsCallerFrameSlot()) continue;
    // Skip any alignment holes in nodes.
    if (output.node != nullptr) {
      DCHECK(!call_descriptor->IsCFunctionCall());
      if (output.location.GetType() == MachineType::Float32()) {
        MarkAsFloat32(output.node);
      } else if (output.location.GetType() == MachineType::Float64()) {
        MarkAsFloat64(output.node);
      }
      Emit(kRISCV64Peek, g.DefineAsRegister(output.node),
           g.UseImmediate(reverse_slot));
    }
    reverse_slot += output.location.GetSizeInPointers();
  }
}

bool InstructionSelector::IsTailCallAddressImmediate() { return true; }

int InstructionSelector::GetTempsCountForTailCallFromJSFunction() { return 3; }

// Shared routine for word comparison with zero.
void InstructionSelector::VisitWordCompareZero(Node* user, Node* value,
                                               FlagsContinuation* cont) {
  // Try to combine with comparisons against 0 by simply inverting the branch.
  while (CanCover(user, value)) {
    if (value->opcode() == IrOpcode::kWord32Equal) {
      Int32BinopMatcher m(value);
      if (!m.right().Is(0)) break;
      user = value;
      value = m.left().node();
    } else if (value->opcode() == IrOpcode::kWord64Equal) {
      Int64BinopMatcher m(value);
      if (!m.right().Is(0)) break;
      user = value;
      value = m.left().node();
    } else {
      break;
    }

    cont->Negate();
  }

  if (CanCover(user, value)) {
    switch (value->opcode()) {
      case IrOpcode::kWord32Equal:
        cont->OverwriteAndNegateIfEqual(kEqual);
        return VisitWord32Compare(this, value, cont);
      case IrOpcode::kInt32LessThan:
        cont->OverwriteAndNegateIfEqual(kSignedLessThan);
        return VisitWord32Compare(this, value, cont);
      case IrOpcode::kInt32LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kSignedLessThanOrEqual);
        return VisitWord32Compare(this, value, cont);
      case IrOpcode::kUint32LessThan:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThan);
        return VisitWord32Compare(this, value, cont);
      case IrOpcode::kUint32LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThanOrEqual);
        return VisitWord32Compare(this, value, cont);
      case IrOpcode::kWord64Equal:
        cont->OverwriteAndNegateIfEqual(kEqual);
        return VisitWord64Compare(this, value, cont);
      case IrOpcode::kInt64LessThan:
        cont->OverwriteAndNegateIfEqual(kSignedLessThan);
        return VisitWord64Compare(this, value, cont);
      case IrOpcode::kInt64LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kSignedLessThanOrEqual);
        return VisitWord64Compare(this, value, cont);
      case IrOpcode::kUint64LessThan:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThan);
        return VisitWord64Compare(this, value, cont);
      case IrOpcode::kUint64LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThanOrEqual);
        return VisitWord64Compare(this, value, cont);
      case IrOpcode::kFloat32Equal:
        cont->OverwriteAndNegateIfEqual(kEqual);
        return VisitFloat32Compare(this, value, cont);
      case IrOpcode::kFloat32LessThan:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThan);
        return VisitFloat32Compare(this, value, cont);
      case IrOpcode::kFloat32LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThanOrEqual);
        return VisitFloat32Compare(this, value, cont);
      case IrOpcode::kFloat64Equal:
        cont->OverwriteAndNegateIfEqual(kEqual);
        return VisitFloat64Compare(this, value, cont);
      case IrOpcode::kFloat64LessThan:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThan);
        return VisitFloat64Compare(this, value, cont);
      case IrOpcode::kFloat64LessThanOrEqual:
        cont->OverwriteAndNegateIfEqual(kUnsignedLessThanOrEqual);
        return VisitFloat64Compare(this, value, cont);
      case IrOpcode::kProjection:
        // Check if this is the overflow output projection of an
        // <Operation>WithOverflow node.
        if (ProjectionIndexOf(value->op()) == 1u) {
          // We cannot combine the <Operation>WithOverflow with this branch
          // unless the 0th projection (the use of the actual value of the
          // <Operation> is either nullptr, which means there's no use of the
          // actual value, or was already defined, which means it is scheduled
          // *AFTER* this branch).
          Node* const node = value->InputAt(0);
          Node* const result = NodeProperties::FindProjection(node, 0);
          if (!result || IsDefined(result)) {
            switch (node->opcode()) {
              case IrOpcode::kInt32AddWithOverflow:
                cont->OverwriteAndNegateIfEqual(kOverflow);
                return VisitBinop(this, node, kRISCV64AddOvf, cont);
              case IrOpcode::kInt32SubWithOverflow:
                cont->OverwriteAndNegateIfEqual(kOverflow);
                return VisitBinop(this, node, kRISCV64SubOvf, cont);
              case IrOpcode::kInt32MulWithOverflow:
                cont->OverwriteAndNegateIfEqual(kOverflow);
                return VisitBinop(this, node, kRISCV64MulOvf, cont);
              default:
                break;
            }
          }
        }
        break;
      case IrOpcode::kStackPointerGreaterThan:
        cont->OverwriteAndNegateIfEqual(kStackPointerGreaterThanCondition);
        return VisitStackPointerGreaterThan(value, cont);
      default:
        break;
    }
  }

  // Continuation could not be combined with a compare, emit compare against 0.
  RISCV64OperandGenerator g(this);
  InstructionOperand const value_operand = g.UseRegister(value);
  EmitWithContinuation(kRISCV64Cmp, value_operand, g.TempImmediate(0), cont);
}

void InstructionSelector::VisitSwitch(Node* node, const SwitchInfo& sw) {
  RISCV64OperandGenerator g(this);
  InstructionOperand value_operand = g.UseRegister(node->InputAt(0));

  // Emit either ArchTableSwitch or ArchBinarySearchSwitch.
  if (enable_switch_jump_table_ == kEnableSwitchJumpTable) {
    static const size_t kMaxTableSwitchValueRange = 2 << 16;
    size_t table_space_cost = 4 + sw.value_range();
    size_t table_time_cost = 3;
    size_t lookup_space_cost = 3 + 2 * sw.case_count();
    size_t lookup_time_cost = sw.case_count();
    if (sw.case_count() > 4 &&
        table_space_cost + 3 * table_time_cost <=
            lookup_space_cost + 3 * lookup_time_cost &&
        sw.min_value() > std::numeric_limits<int32_t>::min() &&
        sw.value_range() <= kMaxTableSwitchValueRange) {
      InstructionOperand index_operand = value_operand;
      if (sw.min_value()) {
        index_operand = g.TempRegister();
        Emit(kRISCV64Sub32, index_operand, value_operand,
             g.TempImmediate(sw.min_value()));
      }
      // Generate a table lookup.
      return EmitTableSwitch(sw, index_operand);
    }
  }

  // Generate a tree of conditional jumps.
  return EmitBinarySearchSwitch(sw, value_operand);
}

void InstructionSelector::VisitWord32Equal(Node* const node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kEqual, node);
  Int32BinopMatcher m(node);
  if (m.right().Is(0)) {
    return VisitWordCompareZero(m.node(), m.left().node(), &cont);
  }

  VisitWord32Compare(this, node, &cont);
}

void InstructionSelector::VisitInt32LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kSignedLessThan, node);
  VisitWord32Compare(this, node, &cont);
}

void InstructionSelector::VisitInt32LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kSignedLessThanOrEqual, node);
  VisitWord32Compare(this, node, &cont);
}

void InstructionSelector::VisitUint32LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kUnsignedLessThan, node);
  VisitWord32Compare(this, node, &cont);
}

void InstructionSelector::VisitUint32LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kUnsignedLessThanOrEqual, node);
  VisitWord32Compare(this, node, &cont);
}

void InstructionSelector::VisitInt32AddWithOverflow(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitInt32SubWithOverflow(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitInt32MulWithOverflow(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitFloat32Equal(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kEqual, node);
  VisitFloat32Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat32LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kFloatLessThan, node);
  VisitFloat32Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat32LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kFloatLessThanOrEqual, node);
  VisitFloat32Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat64Equal(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kEqual, node);
  VisitFloat64Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat64LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kFloatLessThan, node);
  VisitFloat64Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat64LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kFloatLessThanOrEqual, node);
  VisitFloat64Compare(this, node, &cont);
}

void InstructionSelector::VisitFloat64InsertLowWord32(Node* node) {
  RISCV64OperandGenerator g(this);
  Node* left = node->InputAt(0);
  Node* right = node->InputAt(1);
  Emit(kRISCV64Float64InsertLowWord32, g.DefineSameAsFirst(node),
       g.UseRegister(left), g.UseRegister(right));
}

void InstructionSelector::VisitFloat64InsertHighWord32(Node* node) {
  RISCV64OperandGenerator g(this);
  Node* left = node->InputAt(0);
  Node* right = node->InputAt(1);
  Emit(kRISCV64Float64InsertHighWord32, g.DefineSameAsFirst(node),
       g.UseRegister(left), g.UseRegister(right));
}

void InstructionSelector::VisitFloat64SilenceNaN(Node* node) {
  VisitRR(this, kRISCV64Float64SilenceNaN, node);
}

void InstructionSelector::VisitMemoryBarrier(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord32AtomicLoad(Node* node) {
  LoadRepresentation load_rep = LoadRepresentationOf(node->op());
  ArchOpcode opcode = kArchNop;
  switch (load_rep.representation()) {
    case MachineRepresentation::kWord8:
      opcode =
          load_rep.IsSigned() ? kWord32AtomicLoadInt8 : kWord32AtomicLoadUint8;
      break;
    case MachineRepresentation::kWord16:
      opcode = load_rep.IsSigned() ? kWord32AtomicLoadInt16
                                   : kWord32AtomicLoadUint16;
      break;
    case MachineRepresentation::kWord32:
      opcode = kWord32AtomicLoadWord32;
      break;
    default:
      UNREACHABLE();
  }
  VisitAtomicLoad(this, node, opcode);
}

void InstructionSelector::VisitWord32AtomicStore(Node* node) {
  MachineRepresentation rep = AtomicStoreRepresentationOf(node->op());
  ArchOpcode opcode = kArchNop;
  switch (rep) {
    case MachineRepresentation::kWord8:
      opcode = kWord32AtomicStoreWord8;
      break;
    case MachineRepresentation::kWord16:
      opcode = kWord32AtomicStoreWord16;
      break;
    case MachineRepresentation::kWord32:
      opcode = kWord32AtomicStoreWord32;
      break;
    default:
      UNREACHABLE();
  }

  VisitAtomicStore(this, node, opcode);
}

void InstructionSelector::VisitWord32AtomicExchange(Node* node) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Int8()) {
    opcode = kWord32AtomicExchangeInt8;
  } else if (type == MachineType::Uint8()) {
    opcode = kWord32AtomicExchangeUint8;
  } else if (type == MachineType::Int16()) {
    opcode = kWord32AtomicExchangeInt16;
  } else if (type == MachineType::Uint16()) {
    opcode = kWord32AtomicExchangeUint16;
  } else if (type == MachineType::Int32() || type == MachineType::Uint32()) {
    opcode = kWord32AtomicExchangeWord32;
  } else {
    UNREACHABLE();
    return;
  }

  VisitAtomicExchange(this, node, opcode);
}

void InstructionSelector::VisitWord32AtomicCompareExchange(Node* node) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Int8()) {
    opcode = kWord32AtomicCompareExchangeInt8;
  } else if (type == MachineType::Uint8()) {
    opcode = kWord32AtomicCompareExchangeUint8;
  } else if (type == MachineType::Int16()) {
    opcode = kWord32AtomicCompareExchangeInt16;
  } else if (type == MachineType::Uint16()) {
    opcode = kWord32AtomicCompareExchangeUint16;
  } else if (type == MachineType::Int32() || type == MachineType::Uint32()) {
    opcode = kWord32AtomicCompareExchangeWord32;
  } else {
    UNREACHABLE();
    return;
  }

  VisitAtomicCompareExchange(this, node, opcode);
}

void InstructionSelector::VisitWord32AtomicBinaryOperation(
    Node* node, ArchOpcode int8_op, ArchOpcode uint8_op, ArchOpcode int16_op,
    ArchOpcode uint16_op, ArchOpcode word32_op) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Int8()) {
    opcode = int8_op;
  } else if (type == MachineType::Uint8()) {
    opcode = uint8_op;
  } else if (type == MachineType::Int16()) {
    opcode = int16_op;
  } else if (type == MachineType::Uint16()) {
    opcode = uint16_op;
  } else if (type == MachineType::Int32() || type == MachineType::Uint32()) {
    opcode = word32_op;
  } else {
    UNREACHABLE();
    return;
  }

  VisitAtomicBinop(this, node, opcode);
}

#define VISIT_ATOMIC_BINOP(op)                                   \
  void InstructionSelector::VisitWord32Atomic##op(Node* node) {  \
    VisitWord32AtomicBinaryOperation(                            \
        node, kWord32Atomic##op##Int8, kWord32Atomic##op##Uint8, \
        kWord32Atomic##op##Int16, kWord32Atomic##op##Uint16,     \
        kWord32Atomic##op##Word32);                              \
  }
VISIT_ATOMIC_BINOP(Add)
VISIT_ATOMIC_BINOP(Sub)
VISIT_ATOMIC_BINOP(And)
VISIT_ATOMIC_BINOP(Or)
VISIT_ATOMIC_BINOP(Xor)
#undef VISIT_ATOMIC_BINOP

void InstructionSelector::VisitWord64AtomicExchange(Node* node) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Uint8()) {
    opcode = kRISCV64Word64AtomicExchangeUint8;
  } else if (type == MachineType::Uint16()) {
    opcode = kRISCV64Word64AtomicExchangeUint16;
  } else if (type == MachineType::Uint32()) {
    opcode = kRISCV64Word64AtomicExchangeUint32;
  } else if (type == MachineType::Uint64()) {
    opcode = kRISCV64Word64AtomicExchangeUint64;
  } else {
    UNREACHABLE();
    return;
  }
  VisitAtomicExchange(this, node, opcode);
}

void InstructionSelector::VisitWord64AtomicCompareExchange(Node* node) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Uint8()) {
    opcode = kRISCV64Word64AtomicCompareExchangeUint8;
  } else if (type == MachineType::Uint16()) {
    opcode = kRISCV64Word64AtomicCompareExchangeUint16;
  } else if (type == MachineType::Uint32()) {
    opcode = kRISCV64Word64AtomicCompareExchangeUint32;
  } else if (type == MachineType::Uint64()) {
    opcode = kRISCV64Word64AtomicCompareExchangeUint64;
  } else {
    UNREACHABLE();
    return;
  }
  VisitAtomicCompareExchange(this, node, opcode);
}
void InstructionSelector::VisitWord64AtomicBinaryOperation(
    Node* node, ArchOpcode uint8_op, ArchOpcode uint16_op, ArchOpcode uint32_op,
    ArchOpcode uint64_op) {
  ArchOpcode opcode = kArchNop;
  MachineType type = AtomicOpType(node->op());
  if (type == MachineType::Uint8()) {
    opcode = uint8_op;
  } else if (type == MachineType::Uint16()) {
    opcode = uint16_op;
  } else if (type == MachineType::Uint32()) {
    opcode = uint32_op;
  } else if (type == MachineType::Uint64()) {
    opcode = uint64_op;
  } else {
    UNREACHABLE();
    return;
  }
  VisitAtomicBinop(this, node, opcode);
}
void InstructionSelector::VisitWord64AtomicAdd(Node* node) {
  VisitWord64AtomicBinaryOperation(
      node, kRISCV64Word64AtomicAddUint8, kRISCV64Word64AtomicAddUint16,
      kRISCV64Word64AtomicAddUint32, kRISCV64Word64AtomicAddUint64);
}
void InstructionSelector::VisitWord64AtomicSub(Node* node) {
  VisitWord64AtomicBinaryOperation(
      node, kRISCV64Word64AtomicSubUint8, kRISCV64Word64AtomicSubUint16,
      kRISCV64Word64AtomicSubUint32, kRISCV64Word64AtomicSubUint64);
}
void InstructionSelector::VisitWord64AtomicAnd(Node* node) {
  VisitWord64AtomicBinaryOperation(
      node, kRISCV64Word64AtomicAndUint8, kRISCV64Word64AtomicAndUint16,
      kRISCV64Word64AtomicAndUint32, kRISCV64Word64AtomicAndUint64);
}
void InstructionSelector::VisitWord64AtomicOr(Node* node) {
  VisitWord64AtomicBinaryOperation(
      node, kRISCV64Word64AtomicOrUint8, kRISCV64Word64AtomicOrUint16,
      kRISCV64Word64AtomicOrUint32, kRISCV64Word64AtomicOrUint64);
}
void InstructionSelector::VisitWord64AtomicXor(Node* node) {
  VisitWord64AtomicBinaryOperation(
      node, kRISCV64Word64AtomicXorUint8, kRISCV64Word64AtomicXorUint16,
      kRISCV64Word64AtomicXorUint32, kRISCV64Word64AtomicXorUint64);
}
void InstructionSelector::VisitWord64AtomicLoad(Node* node) {
  LoadRepresentation load_rep = LoadRepresentationOf(node->op());
  ArchOpcode opcode = kArchNop;
  switch (load_rep.representation()) {
    case MachineRepresentation::kWord8:
      opcode = kRISCV64Word64AtomicLoadUint8;
      break;
    case MachineRepresentation::kWord16:
      opcode = kRISCV64Word64AtomicLoadUint16;
      break;
    case MachineRepresentation::kWord32:
      opcode = kRISCV64Word64AtomicLoadUint32;
      break;
    case MachineRepresentation::kWord64:
      opcode = kRISCV64Word64AtomicLoadUint64;
      break;
    default:
      UNREACHABLE();
  }
  VisitAtomicLoad(this, node, opcode);
}
void InstructionSelector::VisitWord64AtomicStore(Node* node) {
  MachineRepresentation rep = AtomicStoreRepresentationOf(node->op());
  ArchOpcode opcode = kArchNop;
  switch (rep) {
    case MachineRepresentation::kWord8:
      opcode = kRISCV64Word64AtomicStoreWord8;
      break;
    case MachineRepresentation::kWord16:
      opcode = kRISCV64Word64AtomicStoreWord16;
      break;
    case MachineRepresentation::kWord32:
      opcode = kRISCV64Word64AtomicStoreWord32;
      break;
    case MachineRepresentation::kWord64:
      opcode = kRISCV64Word64AtomicStoreWord64;
      break;
    default:
      UNREACHABLE();
  }

  VisitAtomicStore(this, node, opcode);
}
// plct add V8_TARGET_ARCH_64_BIT
void InstructionSelector::VisitWord64And(Node* node) {
  // and, andi
  VisitBinop(this, node, kRISCV64And, true, kRISCV64And);
}

void InstructionSelector::VisitWord64Or(Node* node) {
  // or, ori
  VisitBinop(this, node, kRISCV64Or, true, kRISCV64Or);
}

void InstructionSelector::VisitWord64Xor(Node* node) {
  // xor, xori
  VisitBinop(this, node, kRISCV64Xor, true, kRISCV64Xor);
}

void InstructionSelector::VisitWord64Shl(Node* node) {
  // sll, slli
  VisitRRO(this, kRISCV64Sll, node);
}

void InstructionSelector::VisitWord64Shr(Node* node) {
  // srl, srli
  VisitRRO(this, kRISCV64Srl, node);
}
void InstructionSelector::VisitWord64Sar(Node* node) {
  // sra, srai
  VisitRRO(this, kRISCV64Sra, node);
}

void InstructionSelector::VisitWord64Ror(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord64Clz(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord64Ctz(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord64ReverseBits(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitWord64Popcnt(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitWord64Equal(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kEqual, node);
  Int64BinopMatcher m(node);
  if (m.right().Is(0)) {
    return VisitWordCompareZero(m.node(), m.left().node(), &cont);
  }
  VisitWord64Compare(this, node, &cont);
}

void InstructionSelector::VisitInt64Add(Node* node) {
  // add, addi
  VisitBinop(this, node, kRISCV64Add, true, kRISCV64Add);
}

void InstructionSelector::VisitInt64AddWithOverflow(Node* node) {
  if (Node* ovf = NodeProperties::FindProjection(node, 1)) {
    FlagsContinuation cont = FlagsContinuation::ForSet(kOverflow, ovf);
    return VisitBinop(this, node, kRISCV64Add, &cont);
  }
  FlagsContinuation cont;
  VisitBinop(this, node, kRISCV64Add, &cont);
}

void InstructionSelector::VisitInt64Sub(Node* node) {
  // sub
  VisitBinop(this, node, kRISCV64Sub);
}

void InstructionSelector::VisitInt64SubWithOverflow(Node* node) {
  if (Node* ovf = NodeProperties::FindProjection(node, 1)) {
    FlagsContinuation cont = FlagsContinuation::ForSet(kOverflow, ovf);
    return VisitBinop(this, node, kRISCV64Sub, &cont);
  }
  FlagsContinuation cont;
  VisitBinop(this, node, kRISCV64Sub, &cont);
}

void InstructionSelector::VisitInt64Mul(Node* node) {
  // mul rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Int64BinopMatcher m(node);
  Emit(kRISCV64Mul, g.DefineAsRegister(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitInt64Div(Node* node) {
  // div rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Int64BinopMatcher m(node);
  Emit(kRISCV64Div, g.DefineSameAsFirst(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitInt64LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kSignedLessThan, node);
  VisitWord64Compare(this, node, &cont);
}

void InstructionSelector::VisitInt64LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kSignedLessThanOrEqual, node);
  VisitWord64Compare(this, node, &cont);
}

void InstructionSelector::VisitUint64Div(Node* node) {
  // divu rd, rs1, rs2
  RISCV64OperandGenerator g(this);
  Int64BinopMatcher m(node);
  Emit(kRISCV64DivU, g.DefineSameAsFirst(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitInt64Mod(Node* node) {
  // rem
  RISCV64OperandGenerator g(this);
  Int64BinopMatcher m(node);
  Emit(kRISCV64Mod, g.DefineAsRegister(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitUint64LessThan(Node* node) {
  FlagsContinuation cont = FlagsContinuation::ForSet(kUnsignedLessThan, node);
  VisitWord64Compare(this, node, &cont);
}

void InstructionSelector::VisitUint64LessThanOrEqual(Node* node) {
  FlagsContinuation cont =
      FlagsContinuation::ForSet(kUnsignedLessThanOrEqual, node);
  VisitWord64Compare(this, node, &cont);
}

void InstructionSelector::VisitUint64Mod(Node* node) {
  // remu
  RISCV64OperandGenerator g(this);
  Int64BinopMatcher m(node);
  Emit(kRISCV64ModU, g.DefineAsRegister(node), g.UseRegister(m.left().node()),
       g.UseRegister(m.right().node()));
}

void InstructionSelector::VisitBitcastWord32ToWord64(Node* node) {
  // refer to arm64
  // mips64 UNIMPLEMENTED VisitBitcastWord32ToWord64
  DCHECK(SmiValuesAre31Bits());
  DCHECK(COMPRESS_POINTERS_BOOL);
  EmitIdentity(node);
}

void InstructionSelector::VisitChangeInt32ToInt64(Node* node) {
  Node* value = node->InputAt(0);
  if (value->opcode() == IrOpcode::kLoad && CanCover(node, value)) {
    // Generate sign-extending load.
    LoadRepresentation load_rep = LoadRepresentationOf(value->op());
    InstructionCode opcode = kArchNop;
    switch (load_rep.representation()) {
      case MachineRepresentation::kBit:  // Fall through.
      case MachineRepresentation::kWord8:
        opcode = load_rep.IsUnsigned() ? kRISCV64Lbu : kRISCV64Lb;
        break;
      case MachineRepresentation::kWord16:
        opcode = load_rep.IsUnsigned() ? kRISCV64Lhu : kRISCV64Lh;
        break;
      case MachineRepresentation::kWord32:
        opcode = kRISCV64Lw;
        break;
      default:
        UNREACHABLE();
        return;
    }
    EmitLoad(this, value, opcode, node);
  } else {
    RISCV64OperandGenerator g(this);
    // addiw
    Emit(kRISCV64Add32, g.DefineAsRegister(node),
         g.UseRegister(node->InputAt(0)), g.TempImmediate(0));
  }
}
void InstructionSelector::VisitChangeInt64ToFloat64(Node* node) {
  // fcvt.d.l rd, rs1, rs2
  VisitRR(this, kRISCV64FcvtDL, node);
}

void InstructionSelector::VisitChangeUint32ToUint64(Node* node) {
  RISCV64OperandGenerator g(this);
  Node* value = node->InputAt(0);
  switch (value->opcode()) {
    // 32-bit operations will write their result in a 64 bit register,
    // clearing the top 32 bits of the destination register.
    case IrOpcode::kUint32Div:
    case IrOpcode::kUint32Mod:
    case IrOpcode::kUint32MulHigh: {
      Emit(kArchNop, g.DefineSameAsFirst(node), g.Use(value));
      return;
    }
    case IrOpcode::kLoad: {
      LoadRepresentation load_rep = LoadRepresentationOf(value->op());
      if (load_rep.IsUnsigned()) {
        switch (load_rep.representation()) {
          case MachineRepresentation::kWord8:
          case MachineRepresentation::kWord16:
          case MachineRepresentation::kWord32:
            Emit(kArchNop, g.DefineSameAsFirst(node), g.Use(value));
            return;
          default:
            break;
        }
      }
      break;
    }
    default:
      break;
  }
  InstructionOperand tmp_reg = g.TempRegister();
  Emit(kRISCV64Sll, tmp_reg, g.UseRegister(node->InputAt(0)),
       g.TempImmediate(32));
  Emit(kRISCV64Srl, g.DefineAsRegister(node), tmp_reg, g.TempImmediate(32));
}

void InstructionSelector::VisitChangeFloat64ToInt64(Node* node) {
  // fcvt.l.d rd, rs1, rs2
  VisitRR(this, kRISCV64FcvtLD, node);
}

void InstructionSelector::VisitChangeFloat64ToUint64(Node* node) {
  // fcvt.lu.d rd, rs1, rs2
  VisitRR(this, kRISCV64FcvtUlD, node);
}

void InstructionSelector::VisitTruncateFloat64ToInt64(Node* node) {
  // fcvt.l.d rd, rs1, rs2
  // same as VisitChangeFloat64ToInt64
  VisitRR(this, kRISCV64FcvtLD, node);
}

void InstructionSelector::VisitTruncateFloat32ToUint32(Node* node) {
  // fcvt.wu.s rd, rs1, rs2
  // Floating-point Convert to Unsigned Word from Single
  VisitRR(this, kRISCV64FcvtUwS, node);
}

void InstructionSelector::VisitChangeFloat64ToUint32(Node* node) {
  // fcvt.wu.d rd, rs1, rs2
  VisitRR(this, kRISCV64FcvtUwD, node);
}
void InstructionSelector::VisitTruncateFloat64ToUint32(Node* node) {
  // fcvt.wu.d rd, rs1, rs2
  VisitRR(this, kRISCV64FcvtUwD, node);
}

void InstructionSelector::VisitTryTruncateFloat32ToInt64(Node* node) {
  RISCV64OperandGenerator g(this);
  InstructionOperand inputs[] = {g.UseRegister(node->InputAt(0))};
  InstructionOperand outputs[2];
  size_t output_count = 0;
  outputs[output_count++] = g.DefineAsRegister(node);

  Node* success_output = NodeProperties::FindProjection(node, 1);
  if (success_output) {
    outputs[output_count++] = g.DefineAsRegister(success_output);
  }
  // fcvt.l.s rd, rs1, rs2
  // Floating-point Convert to Long from Single
  Emit(kRISCV64FcvtLS, output_count, outputs, 1, inputs);
}

void InstructionSelector::VisitTryTruncateFloat64ToInt64(Node* node) {
  RISCV64OperandGenerator g(this);
  InstructionOperand inputs[] = {g.UseRegister(node->InputAt(0))};
  InstructionOperand outputs[2];
  size_t output_count = 0;
  outputs[output_count++] = g.DefineAsRegister(node);

  Node* success_output = NodeProperties::FindProjection(node, 1);
  if (success_output) {
    outputs[output_count++] = g.DefineAsRegister(success_output);
  }

  // fcvt.l.d rd, rs1, rs2
  Emit(kRISCV64FcvtLD, output_count, outputs, 1, inputs);
}

void InstructionSelector::VisitTryTruncateFloat32ToUint64(Node* node) {
  RISCV64OperandGenerator g(this);
  InstructionOperand inputs[] = {g.UseRegister(node->InputAt(0))};
  InstructionOperand outputs[2];
  size_t output_count = 0;
  outputs[output_count++] = g.DefineAsRegister(node);

  Node* success_output = NodeProperties::FindProjection(node, 1);
  if (success_output) {
    outputs[output_count++] = g.DefineAsRegister(success_output);
  }
  // fcvt.lu.s rd, rs1, rs2
  // Floating-point Convert to Unsigned Long from Single
  Emit(kRISCV64FcvtUlS, output_count, outputs, 1, inputs);
}

void InstructionSelector::VisitTryTruncateFloat64ToUint64(Node* node) {
  RISCV64OperandGenerator g(this);
  InstructionOperand inputs[] = {g.UseRegister(node->InputAt(0))};
  InstructionOperand outputs[2];
  size_t output_count = 0;
  outputs[output_count++] = g.DefineAsRegister(node);

  Node* success_output = NodeProperties::FindProjection(node, 1);
  if (success_output) {
    outputs[output_count++] = g.DefineAsRegister(success_output);
  }

  // fcvt.lu.d rd, rs1, rs2
  Emit(kRISCV64FcvtUlD, output_count, outputs, 1, inputs);
}

void InstructionSelector::VisitTruncateInt64ToInt32(Node* node) {
  // to be optimized? just EmitIdentity(node)?

  // RV64I widens the integer registers and supported user address space to 64
  // bits.
  // Additional instruction variants are provided to manipulate 32-bit values in
  // RV64I, indicated by a 'W' suffix to the opcode. These *W instructions
  // ignore the upper 32 bits of their inputs and always produce 32-bit signed
  // values.
  // Note, ADDIW rd, rs1, 0 writes the sign-extension of the lower 32 bits of
  // register rs1 into register rd (assembler pseudoinstruction SEXT.W rd, rs).
  // So it can use addiw, rd, rs1, 0 to truncate int64 to int32.
  RISCV64OperandGenerator g(this);
  // addiw, rd, rs1, 0
  Emit(kRISCV64Add32, g.DefineAsRegister(node), g.UseRegister(node->InputAt(0)),
       g.TempImmediate(0));
}

void InstructionSelector::VisitRoundInt64ToFloat32(Node* node) {
  // Floating-point Convert to Single from Long
  VisitRR(this, kRISCV64FcvtSL, node);
}

void InstructionSelector::VisitRoundInt64ToFloat64(Node* node) {
  // Floating-point Convert to Double from Long
  VisitRR(this, kRISCV64FcvtDL, node);
}

void InstructionSelector::VisitRoundUint64ToFloat32(Node* node) {
  // Floating-point Convert to Single from Unsigned Long
  VisitRR(this, kRISCV64FcvtSUl, node);
}

void InstructionSelector::VisitRoundUint64ToFloat64(Node* node) {
  // Floating-point Convert to Double from Unsigned Long
  VisitRR(this, kRISCV64FcvtDUl, node);
}

void InstructionSelector::VisitBitcastFloat64ToInt64(Node* node) {
  // fmv.x.d rd, rs1, rs2
  // Floating-point Move Doubleword to Integer
  VisitRR(this, kRISCV64FMVDX, node);
}

void InstructionSelector::VisitBitcastInt64ToFloat64(Node* node) {
  // fmv.d.x rd, rs1, rs2
  // Floating-point Move Doubleword from Integer
  VisitRR(this, kRISCV64FMVXD, node);
}

void InstructionSelector::VisitSignExtendWord8ToInt64(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitSignExtendWord16ToInt64(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitSignExtendWord32ToInt64(Node* node) {
  UNIMPLEMENTED();
}
// plct add V8_TARGET_ARCH_64_BIT end

void InstructionSelector::VisitInt64AbsWithOverflow(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitInt32AbsWithOverflow(Node* node) {
  UNIMPLEMENTED();
}
// plct add the remain 20 undefine symbol

void InstructionSelector::VisitF32x4Neg(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Abs(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Sqrt(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4ExtractLane(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF64x2ExtractLane(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF64x2Max(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF64x2Min(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF64x2ReplaceLane(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI16x8ExtractLaneU(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitI16x8ShrS(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4Add(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4GeU(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4MaxS(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4ShrU(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4Splat(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI32x4UConvertI16x8High(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitI64x2ExtractLane(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI64x2GeS(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI64x2GeU(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI64x2GtS(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Div(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Eq(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Lt(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Min(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4RecipSqrtApprox(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitI16x8AddHoriz(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI16x8MinU(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI16x8Shl(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitI16x8UConvertI32x4(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitI32x4GtS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4GtU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4SConvertF32x4(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI64x2GtU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2ShrU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Splat(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Add(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16ExtractLaneS(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitF32x4Max(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Ne(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Splat(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8AddSaturateS(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8GeU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8GtS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8GtU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8ReplaceLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8SConvertI32x4(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8UConvertI8x16Low(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4MinU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Shl(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16AddSaturateS(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16ExtractLaneU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16GeS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16SubSaturateS(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128Not(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128Or(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4RecipApprox(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Qfms(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Eq(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8MaxU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8SubSaturateS(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4MaxU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4MinS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16AddSaturateU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16Neg(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16SubSaturateU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitLoadTransform(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128AndNot(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128And(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128Select(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x16AllTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x16AnyTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x2AllTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x8AllTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x8AnyTrue(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4Le(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8ExtractLaneS(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8MaxS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8MinS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Shl(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Eq(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16GtU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16MaxS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16MaxU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16MinS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16RoundingAverageU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16SConvertI16x8(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI8x16Splat(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128Xor(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS128Zero(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x2AnyTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x4AllTrue(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitS1x4AnyTrue(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4ReplaceLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8GeS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Ne(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8ShrU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4UConvertF32x4(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4UConvertI16x8Low(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI64x2Add(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Eq(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2Neg(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2ReplaceLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI64x2ShrS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16GeU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16GtS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16MinU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Ne(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16ReplaceLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Shl(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16ShrS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16ShrU(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16UConvertI16x8(Node* node) {
  UNIMPLEMENTED();
}

void InstructionSelector::VisitF32x4Add(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4SConvertI32x4(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitF64x2Qfma(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Splat(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8AddSaturateU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8Add(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Neg(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8RoundingAverageU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8SConvertI8x16High(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8SConvertI8x16Low(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8Splat(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8SubSaturateU(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI16x8UConvertI8x16High(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4ExtractLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4GeS(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4ReplaceLane(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4SConvertI16x8Low(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI64x2Ne(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitF32x4AddHoriz(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Qfma(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Qfms(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF32x4UConvertI32x4(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4AddHoriz(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Eq(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Neg(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4Ne(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI32x4SConvertI16x8High(Node* node) {
  UNIMPLEMENTED();
}
void InstructionSelector::VisitI32x4ShrS(Node* node) { UNIMPLEMENTED(); }
// plct add the remain 20 undefined symbol end

void InstructionSelector::VisitI32x4Abs(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI16x8Abs(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitI8x16Abs(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitS8x16Shuffle(Node* node) { UNIMPLEMENTED(); }

void InstructionSelector::VisitS8x16Swizzle(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Add(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Sub(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Mul(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Div(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Eq(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Ne(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Lt(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Le(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Abs(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Neg(Node* node) { UNIMPLEMENTED(); }
void InstructionSelector::VisitF64x2Sqrt(Node* node) { UNIMPLEMENTED(); }
// static
MachineOperatorBuilder::Flags
InstructionSelector::SupportedMachineOperatorFlags() {
  MachineOperatorBuilder::Flags flags =
      MachineOperatorBuilder::kInt32DivIsSafe |
      MachineOperatorBuilder::kUint32DivIsSafe |
      MachineOperatorBuilder::kWord32ShiftIsSafe |
      MachineOperatorBuilder::kFloat32RoundDown |
      MachineOperatorBuilder::kFloat64RoundDown |
      MachineOperatorBuilder::kFloat32RoundUp |
      MachineOperatorBuilder::kFloat64RoundUp |
      MachineOperatorBuilder::kFloat32RoundTruncate |
      MachineOperatorBuilder::kFloat64RoundTruncate |
      MachineOperatorBuilder::kFloat32RoundTiesEven |
      MachineOperatorBuilder::kFloat64RoundTiesEven;
  return flags;
}

// static
MachineOperatorBuilder::AlignmentRequirements
InstructionSelector::AlignmentRequirements() {
  return MachineOperatorBuilder::AlignmentRequirements::
      FullUnalignedAccessSupport();
}
// #undef UNIMPLEMENTED()
}  // namespace compiler
}  // namespace internal
}  // namespace v8
