// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <limits>

#include "src/base/overflowing-math.h"
#include "src/codegen/assembler-inl.h"
#include "src/codegen/macro-assembler.h"
#include "src/codegen/optimized-compilation-info.h"
#include "src/codegen/riscv64/constants-riscv64.h"
#include "src/compiler/backend/code-generator-impl.h"
#include "src/compiler/backend/gap-resolver.h"
#include "src/compiler/node-matchers.h"
#include "src/compiler/osr.h"
#include "src/heap/heap-inl.h"  // crbug.com/v8/8499
#include "src/objects/smi.h"
#include "src/wasm/wasm-code-manager.h"
#include "src/wasm/wasm-objects.h"

namespace v8 {
namespace internal {
namespace compiler {

#define __ tasm()->

// Adds RISCV64 specific methods for decoding operands.
class RISCV64OperandConverter : public InstructionOperandConverter {
 public:
  RISCV64OperandConverter(CodeGenerator* gen, Instruction* instr)
      : InstructionOperandConverter(gen, instr) {}

  Operand InputImmediate(size_t index) {
    Constant constant = ToConstant(instr_->InputAt(index));
    switch (constant.type()) {
      case Constant::kInt32:
        return Operand(constant.ToInt32());
      case Constant::kInt64:
        return Operand(constant.ToInt64());
      case Constant::kFloat32:
        return Operand::EmbeddedNumber(constant.ToFloat32());
      case Constant::kFloat64:
        return Operand::EmbeddedNumber(constant.ToFloat64().value());
      case Constant::kExternalReference:
      case Constant::kCompressedHeapObject:
      case Constant::kHeapObject:
        // TODO(plind): Maybe we should handle ExtRef & HeapObj here?
        //    maybe not done on arm due to const pool ??
        break;
      case Constant::kDelayedStringConstant:
        return Operand::EmbeddedStringConstant(
            constant.ToDelayedStringConstant());
      case Constant::kRpoNumber:
        UNREACHABLE();  // TODO(titzer): RPO immediates on mips?
        break;
    }
    UNREACHABLE();
  }

  FloatRegister OutputSingleRegister(size_t index = 0) {
    return ToSingleRegister(instr_->OutputAt(index));
  }

  FloatRegister InputSingleRegister(size_t index) {
    return ToSingleRegister(instr_->InputAt(index));
  }

  FloatRegister ToSingleRegister(InstructionOperand* op) {
    // Single (Float) and Double register namespace is same on MIPS,
    // both are typedefs of FPURegister.
    return ToDoubleRegister(op);
  }

  Operand InputOperand(size_t index) {
    InstructionOperand* op = instr_->InputAt(index);
    if (op->IsRegister()) {
      return Operand(ToRegister(op));
    }
    return InputImmediate(index);
  }

  Register InputOrZeroRegister(size_t index) {
    if (instr_->InputAt(index)->IsImmediate()) {
      DCHECK_EQ(0, InputInt32(index));
      return zero_reg;
    }
    return InputRegister(index);
  }

  DoubleRegister InputOrZeroDoubleRegister(size_t index) {
    if (instr_->InputAt(index)->IsImmediate()) return kDoubleRegZero;

    return InputDoubleRegister(index);
  }

  DoubleRegister InputOrZeroSingleRegister(size_t index) {
    if (instr_->InputAt(index)->IsImmediate()) return kDoubleRegZero;

    return InputSingleRegister(index);
  }

  MemOperand MemoryOperand(size_t* first_index) {
    const size_t index = *first_index;
    switch (AddressingModeField::decode(instr_->opcode())) {
      case kMode_None:
        break;
      case kMode_MRI:
        *first_index += 2;
        return MemOperand(InputRegister(index + 0), InputInt32(index + 1));
      default:
        UNREACHABLE();
    }
    UNREACHABLE();
  }

  MemOperand MemoryOperand(size_t index = 0) { return MemoryOperand(&index); }

  MemOperand ToMemOperand(InstructionOperand* op) const {
    DCHECK_NOT_NULL(op);
    DCHECK(op->IsStackSlot() || op->IsFPStackSlot());
    return SlotToMemOperand(AllocatedOperand::cast(op)->index());
  }

  MemOperand SlotToMemOperand(int slot) const {
    FrameOffset offset = frame_access_state()->GetFrameOffset(slot);
    return MemOperand(offset.from_stack_pointer() ? sp : fp, offset.offset());
  }
};

static inline bool HasRegisterInput(Instruction* instr, size_t index) {
  return instr->InputAt(index)->IsRegister();
}

namespace {
void EmitWordLoadPoisoningIfNeeded(CodeGenerator* codegen,
                                   InstructionCode opcode, Instruction* instr,
                                   RISCV64OperandConverter const& i) {
  const MemoryAccessMode access_mode =
      static_cast<MemoryAccessMode>(MiscField::decode(opcode));
  if (access_mode == kMemoryAccessPoisoned) {
    Register value = i.OutputRegister();
    codegen->tasm()->And(value, value, kSpeculationPoisonRegister);
  }
}
class OutOfLineRecordWrite final : public OutOfLineCode {
 public:
  OutOfLineRecordWrite(CodeGenerator* gen, Register object, Register index,
                       Register value, Register scratch0, Register scratch1,
                       RecordWriteMode mode, StubCallMode stub_mode)
      : OutOfLineCode(gen),
        object_(object),
        index_(index),
        value_(value),
        scratch0_(scratch0),
        scratch1_(scratch1),
        mode_(mode),
        stub_mode_(stub_mode),
        must_save_lr_(!gen->frame_access_state()->has_frame()),
        zone_(gen->zone()) {}

  void Generate() final {
    if (mode_ > RecordWriteMode::kValueIsPointer) {
      __ RecordComment("[  JumpIfSmi(value_, exit());");
      __ JumpIfSmi(value_, exit());
      __ RecordComment("]");
    }
    __ CheckPageFlag(value_, scratch0_,
                     MemoryChunk::kPointersToHereAreInterestingMask, eq,
                     exit());
    __ RecordComment("[  Dadd(scratch1_, object_, index_);");
    __ Dadd(scratch1_, object_, index_);
    __ RecordComment("]");
    RememberedSetAction const remembered_set_action =
        mode_ > RecordWriteMode::kValueIsMap ? EMIT_REMEMBERED_SET
                                             : OMIT_REMEMBERED_SET;
    SaveFPRegsMode const save_fp_mode =
        frame()->DidAllocateDoubleRegisters() ? kSaveFPRegs : kDontSaveFPRegs;
    if (must_save_lr_) {
      // We need to save and restore ra if the frame was elided.
      __ RecordComment("[  Push(ra);");
      __ Push(ra);
      __ RecordComment("]");
    }
    if (mode_ == RecordWriteMode::kValueIsEphemeronKey) {
      __ RecordComment(
          "[  CallEphemeronKeyBarrier(object_, scratch1_, save_fp_mode);");
      __ CallEphemeronKeyBarrier(object_, scratch1_, save_fp_mode);
      __ RecordComment("]");
    } else if (stub_mode_ == StubCallMode::kCallWasmRuntimeStub) {
      // A direct call to a wasm runtime stub defined in this module.
      // Just encode the stub index. This will be patched when the code
      // is added to the native module and copied into wasm code space.
      // CallRecordWriteStub second argument is Operand type
      __ CallRecordWriteStub(object_, Operand(scratch1_), remembered_set_action,
                             save_fp_mode, wasm::WasmCode::kRecordWrite);
    } else {
      __ CallRecordWriteStub(object_, Operand(scratch1_), remembered_set_action,
                             save_fp_mode);
    }
    if (must_save_lr_) {
      __ RecordComment("[  Pop(ra);");
      __ Pop(ra);
      __ RecordComment("]");
    }
  }

 private:
  Register const object_;
  Register const index_;
  Register const value_;
  Register const scratch0_;
  Register const scratch1_;
  RecordWriteMode const mode_;
  StubCallMode const stub_mode_;
  bool must_save_lr_;
  Zone* zone_;
};

Condition FlagsConditionToConditionCmp(FlagsCondition condition) {
  switch (condition) {
    case kEqual:
      return eq;
    case kNotEqual:
      return ne;
    case kSignedLessThan:
      return lt;
    case kSignedGreaterThanOrEqual:
      return ge;
    case kSignedLessThanOrEqual:
      return le;
    case kSignedGreaterThan:
      return gt;
    case kUnsignedLessThan:
      return lo;
    case kUnsignedGreaterThanOrEqual:
      return hs;
    case kUnsignedLessThanOrEqual:
      return ls;
    case kUnsignedGreaterThan:
      return hi;
    case kUnorderedEqual:
    case kUnorderedNotEqual:
      break;
    default:
      break;
  }
  UNREACHABLE();
}

Condition FlagsConditionToConditionOvf(FlagsCondition condition) {
  switch (condition) {
    case kOverflow:
      return ne;
    case kNotOverflow:
      return eq;
    default:
      break;
  }
  UNREACHABLE();
}

}  // namespace

#define ASSEMBLE_UNOP(asm_instr)                            \
  do {                                                      \
    if (instr->Output()->IsRegister()) {                    \
      __ RecordComment("[ asm_instr(i.OutputRegister());"); \
      __ asm_instr(i.OutputRegister());                     \
      __ RecordComment("]");                                \
    } else {                                                \
      __ RecordComment("[ asm_instr(i.OutputOperand());");  \
      __ asm_instr(i.OutputOperand());                      \
      __ RecordComment("]");                                \
    }                                                       \
  } while (false)

#define ASSEMBLE_BINOP(asm_instr)                                       \
  do {                                                                  \
    if (HasAddressingMode(instr)) {                                     \
      size_t index = 1;                                                 \
      Operand right = i.MemoryOperand(&index);                          \
      __ RecordComment("[ asm_instr(i.InputRegister(0), right);");      \
      __ asm_instr(i.InputRegister(0), right);                          \
      __ RecordComment("]");                                            \
    } else {                                                            \
      if (HasImmediateInput(instr, 1)) {                                \
        if (HasRegisterInput(instr, 0)) {                               \
          __ RecordComment(                                             \
              "[ asm_instr(i.InputRegister(0), i.InputImmediate(1));"); \
          __ asm_instr(i.InputRegister(0), i.InputImmediate(1));        \
          __ RecordComment("]");                                        \
        } else {                                                        \
          __ RecordComment(                                             \
              "[ asm_instr(i.InputOperand(0), i.InputImmediate(1));");  \
          __ asm_instr(i.InputOperand(0), i.InputImmediate(1));         \
          __ RecordComment("]");                                        \
        }                                                               \
      } else {                                                          \
        if (HasRegisterInput(instr, 1)) {                               \
          __ RecordComment(                                             \
              "[ asm_instr(i.InputRegister(0), i.InputRegister(1));");  \
          __ asm_instr(i.InputRegister(0), i.InputRegister(1));         \
          __ RecordComment("]");                                        \
        } else {                                                        \
          __ RecordComment(                                             \
              "[ asm_instr(i.InputRegister(0), i.InputOperand(1));");   \
          __ asm_instr(i.InputRegister(0), i.InputOperand(1));          \
          __ RecordComment("]");                                        \
        }                                                               \
      }                                                                 \
    }                                                                   \
  } while (false)

#define ASSEMBLE_COMPARE(asm_instr)                                      \
  do {                                                                   \
    if (HasAddressingMode(instr)) {                                      \
      size_t index = 0;                                                  \
      Operand left = i.MemoryOperand(&index);                            \
      if (HasImmediateInput(instr, index)) {                             \
        __ RecordComment("[ asm_instr(left, i.InputImmediate(index));"); \
        __ asm_instr(left, i.InputImmediate(index));                     \
        __ RecordComment("]");                                           \
      } else {                                                           \
        __ RecordComment("[ asm_instr(left, i.InputRegister(index));");  \
        __ asm_instr(left, i.InputRegister(index));                      \
        __ RecordComment("]");                                           \
      }                                                                  \
    } else {                                                             \
      if (HasImmediateInput(instr, 1)) {                                 \
        if (HasRegisterInput(instr, 0)) {                                \
          __ RecordComment(                                              \
              "[ asm_instr(i.InputRegister(0), i.InputImmediate(1));");  \
          __ asm_instr(i.InputRegister(0), i.InputImmediate(1));         \
          __ RecordComment("]");                                         \
        } else {                                                         \
          __ RecordComment(                                              \
              "[ asm_instr(i.InputOperand(0), i.InputImmediate(1));");   \
          __ asm_instr(i.InputOperand(0), i.InputImmediate(1));          \
          __ RecordComment("]");                                         \
        }                                                                \
      } else {                                                           \
        if (HasRegisterInput(instr, 1)) {                                \
          __ RecordComment(                                              \
              "[ asm_instr(i.InputRegister(0), i.InputRegister(1));");   \
          __ asm_instr(i.InputRegister(0), i.InputRegister(1));          \
          __ RecordComment("]");                                         \
        } else {                                                         \
          __ RecordComment(                                              \
              "[ asm_instr(i.InputRegister(0), i.InputOperand(1));");    \
          __ asm_instr(i.InputRegister(0), i.InputOperand(1));           \
          __ RecordComment("]");                                         \
        }                                                                \
      }                                                                  \
    }                                                                    \
  } while (false)

#define ASSEMBLE_MULT(asm_instr)                                     \
  do {                                                               \
    if (HasImmediateInput(instr, 1)) {                               \
      if (HasRegisterInput(instr, 0)) {                              \
        __ asm_instr(i.OutputRegister(), i.InputRegister(0),         \
                     i.InputImmediate(1));                           \
      } else {                                                       \
        __ asm_instr(i.OutputRegister(), i.InputOperand(0),          \
                     i.InputImmediate(1));                           \
      }                                                              \
    } else {                                                         \
      if (HasRegisterInput(instr, 1)) {                              \
        __ RecordComment(                                            \
            "[ asm_instr(i.OutputRegister(), i.InputRegister(1));"); \
        __ asm_instr(i.OutputRegister(), i.InputRegister(1));        \
        __ RecordComment("]");                                       \
      } else {                                                       \
        __ RecordComment(                                            \
            "[ asm_instr(i.OutputRegister(), i.InputOperand(1));");  \
        __ asm_instr(i.OutputRegister(), i.InputOperand(1));         \
        __ RecordComment("]");                                       \
      }                                                              \
    }                                                                \
  } while (false)

#define ASSEMBLE_SHIFT(asm_instr, width)                                   \
  do {                                                                     \
    if (HasImmediateInput(instr, 1)) {                                     \
      if (instr->Output()->IsRegister()) {                                 \
        __ RecordComment(                                                  \
            "[ asm_instr(i.OutputRegister(), "                             \
            "Immediate(i.InputInt##width(1)));");                          \
        __ asm_instr(i.OutputRegister(), Immediate(i.InputInt##width(1))); \
        __ RecordComment("]");                                             \
      } else {                                                             \
        __ RecordComment(                                                  \
            "[ asm_instr(i.OutputOperand(), "                              \
            "Immediate(i.InputInt##width(1)));");                          \
        __ asm_instr(i.OutputOperand(), Immediate(i.InputInt##width(1)));  \
        __ RecordComment("]");                                             \
      }                                                                    \
    } else {                                                               \
      if (instr->Output()->IsRegister()) {                                 \
        __ RecordComment("[ asm_instr##_cl(i.OutputRegister());");         \
        __ asm_instr##_cl(i.OutputRegister());                             \
        __ RecordComment("]");                                             \
      } else {                                                             \
        __ RecordComment("[ asm_instr##_cl(i.OutputOperand());");          \
        __ asm_instr##_cl(i.OutputOperand());                              \
        __ RecordComment("]");                                             \
      }                                                                    \
    }                                                                      \
  } while (false)

#define ASSEMBLE_MOVX(asm_instr)                                               \
  do {                                                                         \
    if (HasAddressingMode(instr)) {                                            \
      __ RecordComment("[ asm_instr(i.OutputRegister(), i.MemoryOperand());"); \
      __ asm_instr(i.OutputRegister(), i.MemoryOperand());                     \
      __ RecordComment("]");                                                   \
    } else if (HasRegisterInput(instr, 0)) {                                   \
      __ RecordComment(                                                        \
          "[ asm_instr(i.OutputRegister(), i.InputRegister(0));");             \
      __ asm_instr(i.OutputRegister(), i.InputRegister(0));                    \
      __ RecordComment("]");                                                   \
    } else {                                                                   \
      __ RecordComment("[ asm_instr(i.OutputRegister(), i.InputOperand(0));"); \
      __ asm_instr(i.OutputRegister(), i.InputOperand(0));                     \
      __ RecordComment("]");                                                   \
    }                                                                          \
  } while (false)

#define ASSEMBLE_SSE_BINOP(asm_instr)                                          \
  do {                                                                         \
    if (instr->InputAt(1)->IsFPRegister()) {                                   \
      __ RecordComment(                                                        \
          "[ asm_instr(i.InputDoubleRegister(0), i.InputDoubleRegister(1));"); \
      __ asm_instr(i.InputDoubleRegister(0), i.InputDoubleRegister(1));        \
      __ RecordComment("]");                                                   \
    } else {                                                                   \
      __ RecordComment(                                                        \
          "[ asm_instr(i.InputDoubleRegister(0), i.InputOperand(1));");        \
      __ asm_instr(i.InputDoubleRegister(0), i.InputOperand(1));               \
      __ RecordComment("]");                                                   \
    }                                                                          \
  } while (false)

#define ASSEMBLE_SSE_UNOP(asm_instr)                                           \
  do {                                                                         \
    if (instr->InputAt(0)->IsFPRegister()) {                                   \
      __ RecordComment(                                                        \
          "[ asm_instr(i.OutputDoubleRegister(), i.InputDoubleRegister(0));"); \
      __ asm_instr(i.OutputDoubleRegister(), i.InputDoubleRegister(0));        \
      __ RecordComment("]");                                                   \
    } else {                                                                   \
      __ RecordComment(                                                        \
          "[ asm_instr(i.OutputDoubleRegister(), i.InputOperand(0));");        \
      __ asm_instr(i.OutputDoubleRegister(), i.InputOperand(0));               \
      __ RecordComment("]");                                                   \
    }                                                                          \
  } while (false)

#define ASSEMBLE_AVX_BINOP(asm_instr)                                  \
  do {                                                                 \
    CpuFeatureScope avx_scope(tasm(), AVX);                            \
    if (instr->InputAt(1)->IsFPRegister()) {                           \
      __ asm_instr(i.OutputDoubleRegister(), i.InputDoubleRegister(0), \
                   i.InputDoubleRegister(1));                          \
    } else {                                                           \
      __ asm_instr(i.OutputDoubleRegister(), i.InputDoubleRegister(0), \
                   i.InputOperand(1));                                 \
    }                                                                  \
  } while (false)

#define ASSEMBLE_IEEE754_BINOP(name)                                       \
  do {                                                                     \
    __ RecordComment("[ PrepareCallCFunction(2);");                        \
    __ PrepareCallCFunction(2);                                            \
    __ RecordComment("]");                                                 \
    __ RecordComment(                                                      \
        "[ CallCFunction(ExternalReference::ieee754_##name##_function(), " \
        "2);");                                                            \
    __ CallCFunction(ExternalReference::ieee754_##name##_function(), 2);   \
    __ RecordComment("]");                                                 \
  } while (false)

#define ASSEMBLE_IEEE754_UNOP(name)                                        \
  do {                                                                     \
    __ RecordComment("[ PrepareCallCFunction(1);");                        \
    __ PrepareCallCFunction(1);                                            \
    __ RecordComment("]");                                                 \
    __ RecordComment(                                                      \
        "[ CallCFunction(ExternalReference::ieee754_##name##_function(), " \
        "1);");                                                            \
    __ CallCFunction(ExternalReference::ieee754_##name##_function(), 1);   \
    __ RecordComment("]");                                                 \
  } while (false)

#define ASSEMBLE_ATOMIC_BINOP(bin_inst, mov_inst, cmpxchg_inst)             \
  do {                                                                      \
    Label binop;                                                            \
    __ RecordComment("[ bind(&binop);");                                    \
    __ bind(&binop);                                                        \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ mov_inst(rax, i.MemoryOperand(1));");               \
    __ mov_inst(rax, i.MemoryOperand(1));                                   \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ movl(i.TempRegister(0), rax);");                    \
    __ movl(i.TempRegister(0), rax);                                        \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ bin_inst(i.TempRegister(0), i.InputRegister(0));"); \
    __ bin_inst(i.TempRegister(0), i.InputRegister(0));                     \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ lock();");                                          \
    __ lock();                                                              \
    __ RecordComment("]");                                                  \
    __ RecordComment(                                                       \
        "[ cmpxchg_inst(i.MemoryOperand(1), i.TempRegister(0));");          \
    __ cmpxchg_inst(i.MemoryOperand(1), i.TempRegister(0));                 \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ j(not_equal, &binop);");                            \
    __ j(not_equal, &binop);                                                \
    __ RecordComment("]");                                                  \
  } while (false)

#define ASSEMBLE_ATOMIC64_BINOP(bin_inst, mov_inst, cmpxchg_inst)           \
  do {                                                                      \
    Label binop;                                                            \
    __ RecordComment("[ bind(&binop);");                                    \
    __ bind(&binop);                                                        \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ mov_inst(rax, i.MemoryOperand(1));");               \
    __ mov_inst(rax, i.MemoryOperand(1));                                   \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ movq(i.TempRegister(0), rax);");                    \
    __ movq(i.TempRegister(0), rax);                                        \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ bin_inst(i.TempRegister(0), i.InputRegister(0));"); \
    __ bin_inst(i.TempRegister(0), i.InputRegister(0));                     \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ lock();");                                          \
    __ lock();                                                              \
    __ RecordComment("]");                                                  \
    __ RecordComment(                                                       \
        "[ cmpxchg_inst(i.MemoryOperand(1), i.TempRegister(0));");          \
    __ cmpxchg_inst(i.MemoryOperand(1), i.TempRegister(0));                 \
    __ RecordComment("]");                                                  \
    __ RecordComment("[ j(not_equal, &binop);");                            \
    __ j(not_equal, &binop);                                                \
    __ RecordComment("]");                                                  \
  } while (false)

#define ASSEMBLE_SIMD_INSTR(opcode, dst_operand, index)                  \
  do {                                                                   \
    if (instr->InputAt(index)->IsSimd128Register()) {                    \
      __ RecordComment(                                                  \
          "[ opcode(dst_operand, i.InputSimd128Register(index));");      \
      __ opcode(dst_operand, i.InputSimd128Register(index));             \
      __ RecordComment("]");                                             \
    } else {                                                             \
      __ RecordComment("[ opcode(dst_operand, i.InputOperand(index));"); \
      __ opcode(dst_operand, i.InputOperand(index));                     \
      __ RecordComment("]");                                             \
    }                                                                    \
  } while (false)

#define ASSEMBLE_SIMD_IMM_INSTR(opcode, dst_operand, index, imm)              \
  do {                                                                        \
    if (instr->InputAt(index)->IsSimd128Register()) {                         \
      __ RecordComment(                                                       \
          "[ opcode(dst_operand, i.InputSimd128Register(index), imm);");      \
      __ opcode(dst_operand, i.InputSimd128Register(index), imm);             \
      __ RecordComment("]");                                                  \
    } else {                                                                  \
      __ RecordComment("[ opcode(dst_operand, i.InputOperand(index), imm);"); \
      __ opcode(dst_operand, i.InputOperand(index), imm);                     \
      __ RecordComment("]");                                                  \
    }                                                                         \
  } while (false)

#define ASSEMBLE_SIMD_PUNPCK_SHUFFLE(opcode)             \
  do {                                                   \
    XMMRegister dst = i.OutputSimd128Register();         \
    DCHECK_EQ(dst, i.InputSimd128Register(0));           \
    byte input_index = instr->InputCount() == 2 ? 1 : 0; \
    ASSEMBLE_SIMD_INSTR(opcode, dst, input_index);       \
  } while (false)

#define ASSEMBLE_SIMD_IMM_SHUFFLE(opcode, SSELevel, imm)                  \
  do {                                                                    \
    CpuFeatureScope sse_scope(tasm(), SSELevel);                          \
    DCHECK_EQ(i.OutputSimd128Register(), i.InputSimd128Register(0));      \
    __ RecordComment(                                                     \
        "[ opcode(i.OutputSimd128Register(), i.InputSimd128Register(1), " \
        "imm);");                                                         \
    __ opcode(i.OutputSimd128Register(), i.InputSimd128Register(1), imm); \
    __ RecordComment("]");                                                \
  } while (false)

#define ASSEMBLE_SIMD_ALL_TRUE(opcode)                              \
  do {                                                              \
    CpuFeatureScope sse_scope(tasm(), SSE4_1);                      \
    Register dst = i.OutputRegister();                              \
    Register tmp1 = i.TempRegister(0);                              \
    XMMRegister tmp2 = i.TempSimd128Register(1);                    \
    __ RecordComment("[ movq(tmp1, Immediate(1));");                \
    __ movq(tmp1, Immediate(1));                                    \
    __ RecordComment("]");                                          \
    __ RecordComment("[ xorq(dst, dst);");                          \
    __ xorq(dst, dst);                                              \
    __ RecordComment("]");                                          \
    __ RecordComment("[ pxor(tmp2, tmp2);");                        \
    __ pxor(tmp2, tmp2);                                            \
    __ RecordComment("]");                                          \
    __ RecordComment("[ opcode(tmp2, i.InputSimd128Register(0));"); \
    __ opcode(tmp2, i.InputSimd128Register(0));                     \
    __ RecordComment("]");                                          \
    __ RecordComment("[ ptest(tmp2, tmp2);");                       \
    __ ptest(tmp2, tmp2);                                           \
    __ RecordComment("]");                                          \
    __ RecordComment("[ cmovq(zero, dst, tmp1);");                  \
    __ cmovq(zero, dst, tmp1);                                      \
    __ RecordComment("]");                                          \
  } while (false)

void CodeGenerator::AssembleDeconstructFrame() {
  __ RecordComment("[  mov(sp, fp);");
  __ mov(sp, fp);
  __ RecordComment("]");
  __ RecordComment("[  Pop(ra, fp);");
  __ Pop(ra, fp);
  __ RecordComment("]");
}

void CodeGenerator::AssemblePrepareTailCall() {
  if (frame_access_state()->has_frame()) {
    __ RecordComment(
        "[  Ld(ra, MemOperand(fp, StandardFrameConstants::kCallerPCOffset));");
    __ Ld(ra, MemOperand(fp, StandardFrameConstants::kCallerPCOffset));
    __ RecordComment("]");
    __ RecordComment(
        "[  Ld(fp, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));");
    __ Ld(fp, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
    __ RecordComment("]");
  }
  frame_access_state()->SetFrameAccessToSP();
}

void CodeGenerator::AssemblePopArgumentsAdaptorFrame(Register args_reg,
                                                     Register scratch1,
                                                     Register scratch2,
                                                     Register scratch3) {
  DCHECK(!AreAliased(args_reg, scratch1, scratch2, scratch3));
  Label done;

  // Check if current frame is an arguments adaptor frame.
  __ RecordComment(
      "[  Ld(scratch3, MemOperand(fp, "
      "StandardFrameConstants::kContextOffset));");
  __ Ld(scratch3, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ RecordComment("]");
  __ Branch(&done, ne, scratch3,
            Operand(StackFrame::TypeToMarker(StackFrame::ARGUMENTS_ADAPTOR)));

  // Load arguments count from current arguments adaptor frame (note, it
  // does not include receiver).
  Register caller_args_count_reg = scratch1;
  __ Ld(caller_args_count_reg,
        MemOperand(fp, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ RecordComment("[  SmiUntag(caller_args_count_reg);");
  __ SmiUntag(caller_args_count_reg);
  __ RecordComment("]");

  __ RecordComment(
      "[  PrepareForTailCall(args_reg, caller_args_count_reg, scratch2, "
      "scratch3);");
  __ PrepareForTailCall(args_reg, caller_args_count_reg, scratch2, scratch3);
  __ RecordComment("]");
  __ RecordComment("[  bind(&done);");
  __ bind(&done);
  __ RecordComment("]");
}

namespace {

void AdjustStackPointerForTailCall(TurboAssembler* tasm,
                                   FrameAccessState* state,
                                   int new_slot_above_sp,
                                   bool allow_shrinkage = true) {
  int current_sp_offset = state->GetSPToFPSlotCount() +
                          StandardFrameConstants::kFixedSlotCountAboveFp;
  int stack_slot_delta = new_slot_above_sp - current_sp_offset;
  if (stack_slot_delta > 0) {
    tasm->Dsub(sp, sp, stack_slot_delta * kSystemPointerSize);
    state->IncreaseSPDelta(stack_slot_delta);
  } else if (allow_shrinkage && stack_slot_delta < 0) {
    tasm->Dadd(sp, sp, -stack_slot_delta * kSystemPointerSize);
    state->IncreaseSPDelta(stack_slot_delta);
  }
}

}  // namespace

void CodeGenerator::AssembleTailCallBeforeGap(Instruction* instr,
                                              int first_unused_stack_slot) {
  AdjustStackPointerForTailCall(tasm(), frame_access_state(),
                                first_unused_stack_slot, false);
}

void CodeGenerator::AssembleTailCallAfterGap(Instruction* instr,
                                             int first_unused_stack_slot) {
  AdjustStackPointerForTailCall(tasm(), frame_access_state(),
                                first_unused_stack_slot);
}

// Check that {kJavaScriptCallCodeStartRegister} is correct.
void CodeGenerator::AssembleCodeStartRegisterCheck() {
  __ RecordComment("[  ComputeCodeStartAddress(kScratchRegister);");
  __ ComputeCodeStartAddress(kScratchRegister);
  __ RecordComment("]");
  __ Assert(eq, AbortReason::kWrongFunctionCodeStart,
            kJavaScriptCallCodeStartRegister, Operand(kScratchRegister));
}

// Check if the code object is marked for deoptimization. If it is, then it
// jumps to the CompileLazyDeoptimizedCode builtin. In order to do this we need
// to:
//    1. read from memory the word that contains that bit, which can be found in
//       the flags in the referenced {CodeDataContainer} object;
//    2. test kMarkedForDeoptimizationBit in those flags; and
//    3. if it is not zero then it jumps to the builtin.
void CodeGenerator::BailoutIfDeoptimized() { UNIMPLEMENTED(); }

void CodeGenerator::GenerateSpeculationPoisonFromCodeStartRegister() {
  // Calculate a mask which has all bits set in the normal case, but has all
  // bits cleared if we are speculatively executing the wrong PC.
  //    difference = (current - expected) | (expected - current)
  //    poison = ~(difference >> (kBitsPerSystemPointer - 1))
  __ RecordComment("[  ComputeCodeStartAddress(kScratchRegister);");
  __ ComputeCodeStartAddress(kScratchRegister);
  __ RecordComment("]");
  __ RecordComment("[  Move(kSpeculationPoisonRegister, kScratchRegister);");
  __ Move(kSpeculationPoisonRegister, kScratchRegister);
  __ RecordComment("]");
  __ sub(kSpeculationPoisonRegister, kSpeculationPoisonRegister,
         kJavaScriptCallCodeStartRegister);
  __ sub(kJavaScriptCallCodeStartRegister, kJavaScriptCallCodeStartRegister,
         kScratchRegister);
  __ or_(kSpeculationPoisonRegister, kSpeculationPoisonRegister,
         kJavaScriptCallCodeStartRegister);
  __ srai(kSpeculationPoisonRegister, kSpeculationPoisonRegister,
          kBitsPerSystemPointer - 1);
  __ RecordComment(
      "[  not_(kSpeculationPoisonRegister, kSpeculationPoisonRegister);");
  __ not_(kSpeculationPoisonRegister, kSpeculationPoisonRegister);
  __ RecordComment("]");
}

void CodeGenerator::AssembleRegisterArgumentPoisoning() {
  __ RecordComment(
      "[  And(kJSFunctionRegister, kJSFunctionRegister, "
      "kSpeculationPoisonRegister);");
  __ And(kJSFunctionRegister, kJSFunctionRegister, kSpeculationPoisonRegister);
  __ RecordComment("]");
  __ RecordComment(
      "[  And(kContextRegister, kContextRegister, "
      "kSpeculationPoisonRegister);");
  __ And(kContextRegister, kContextRegister, kSpeculationPoisonRegister);
  __ RecordComment("]");
  __ RecordComment("[  And(sp, sp, kSpeculationPoisonRegister);");
  __ And(sp, sp, kSpeculationPoisonRegister);
  __ RecordComment("]");
}

// Assembles an instruction after register allocation, producing machine code.
CodeGenerator::CodeGenResult CodeGenerator::AssembleArchInstruction(
    Instruction* instr) {
  RISCV64OperandConverter i(this, instr);
  InstructionCode opcode = instr->opcode();
  ArchOpcode arch_opcode = ArchOpcodeField::decode(opcode);
  switch (arch_opcode) {
    case kArchNop: {
      // std::cout<<"CG: kArchNop: "<<std::endl;
      __ RecordComment("[  nop();");
      __ nop();
      __ RecordComment("]");
      break;
    }
    case kArchCallCodeObject: {
      if (instr->InputAt(0)->IsImmediate()) {
        __ RecordComment("[  Call(i.InputCode(0), RelocInfo::CODE_TARGET);");
        __ Call(i.InputCode(0), RelocInfo::CODE_TARGET);
        __ RecordComment("]");
      } else {
        Register reg = i.InputRegister(0);
        DCHECK_IMPLIES(
            HasCallDescriptorFlag(instr, CallDescriptor::kFixedTargetRegister),
            reg == kJavaScriptCallCodeStartRegister);
        __ RecordComment(
            "[  addi(reg, reg, Code::kHeaderSize - kHeapObjectTag);");
        __ addi(reg, reg, Code::kHeaderSize - kHeapObjectTag);
        __ RecordComment("]");
        __ RecordComment("[  Call(reg);");
        __ Call(reg);
        __ RecordComment("]");
      }
      RecordCallPosition(instr);
      frame_access_state()->ClearSPDelta();
      break;
    }
    case kArchPrepareTailCall:
      AssemblePrepareTailCall();
      break;
#if 1
    case kArchCallCFunction: {
      int const num_parameters = MiscField::decode(instr->opcode());
      Label return_location;
      if (linkage()->GetIncomingDescriptor()->IsWasmCapiFunction()) {
        // Put the return address in a stack slot.
        __ RecordComment("[  lla(x1, &return_location);");
        __ lla(x1, &return_location);
        __ RecordComment("]");
        __ RecordComment(
            "[  sd(ra, MemOperand(fp, "
            "WasmExitFrameConstants::kCallingPCOffset));");
        __ sd(ra, MemOperand(fp, WasmExitFrameConstants::kCallingPCOffset));
        __ RecordComment("]");
      }

      if (instr->InputAt(0)->IsImmediate()) {
        ExternalReference ref = i.InputExternalReference(0);
        __ RecordComment("[  CallCFunction(ref, num_parameters, 0);");
        __ CallCFunction(ref, num_parameters, 0);
        __ RecordComment("]");
      } else {
        Register func = i.InputRegister(0);
        __ RecordComment("[  CallCFunction(func, num_parameters, 0);");
        __ CallCFunction(func, num_parameters, 0);
        __ RecordComment("]");
      }
      __ RecordComment("[  bind(&return_location);");
      __ bind(&return_location);
      __ RecordComment("]");
      if (linkage()->GetIncomingDescriptor()->IsWasmCapiFunction()) {
        RecordSafepoint(instr->reference_map(), Safepoint::kNoLazyDeopt);
      }
      frame_access_state()->SetFrameAccessToDefault();
      // Ideally, we should decrement SP delta to match the change of stack
      // pointer in CallCFunction. However, for certain architectures (e.g.
      // ARM), there may be more strict alignment requirement, causing old SP
      // to be saved on the stack. In those cases, we can not calculate the SP
      // delta statically.
      frame_access_state()->ClearSPDelta();
      if (caller_registers_saved_) {
        // Need to re-sync SP delta introduced in kArchSaveCallerRegisters.
        // Here, we assume the sequence to be:
        //   kArchSaveCallerRegisters;
        //   kArchCallCFunction;
        //   kArchRestoreCallerRegisters;
        int bytes =
            __ RequiredStackSizeForCallerSaved(fp_mode_, kReturnRegister0);
        frame_access_state()->IncreaseSPDelta(bytes / kSystemPointerSize);
      }
      break;
    }
#endif
    case kArchJmp:
      AssembleArchJump(i.InputRpo(0));
      break;
    case kArchRet:
      AssembleReturn(instr->InputAt(0));
      break;

    case kArchTailCallCodeObjectFromJSFunction:
    case kArchTailCallCodeObject: {
      if (arch_opcode == kArchTailCallCodeObjectFromJSFunction) {
        AssemblePopArgumentsAdaptorFrame(kJavaScriptCallArgCountRegister,
                                         i.TempRegister(0), i.TempRegister(1),
                                         i.TempRegister(2));
      }
      if (instr->InputAt(0)->IsImmediate()) {
        __ RecordComment("[  Jump(i.InputCode(0), RelocInfo::CODE_TARGET);");
        __ Jump(i.InputCode(0), RelocInfo::CODE_TARGET);
        __ RecordComment("]");
      } else {
        Register reg = i.InputRegister(0);
        DCHECK_IMPLIES(
            HasCallDescriptorFlag(instr, CallDescriptor::kFixedTargetRegister),
            reg == kJavaScriptCallCodeStartRegister);
        __ RecordComment(
            "[  addi(reg, reg, Code::kHeaderSize - kHeapObjectTag);");
        __ addi(reg, reg, Code::kHeaderSize - kHeapObjectTag);
        __ RecordComment("]");
        __ RecordComment("[  Jump(reg);");
        __ Jump(reg);
        __ RecordComment("]");
      }
      frame_access_state()->ClearSPDelta();
      frame_access_state()->SetFrameAccessToDefault();
      break;
    }
    case kArchTailCallAddress: {
      CHECK(!instr->InputAt(0)->IsImmediate());
      Register reg = i.InputRegister(0);
      DCHECK_IMPLIES(
          HasCallDescriptorFlag(instr, CallDescriptor::kFixedTargetRegister),
          reg == kJavaScriptCallCodeStartRegister);
      __ RecordComment("[  Jump(reg);");
      __ Jump(reg);
      __ RecordComment("]");
      frame_access_state()->ClearSPDelta();
      frame_access_state()->SetFrameAccessToDefault();
      break;
    }
    case kArchStackCheckOffset:
      __ RecordComment(
          "[  Move(i.OutputRegister(), Smi::FromInt(GetStackCheckOffset()));");
      __ Move(i.OutputRegister(), Smi::FromInt(GetStackCheckOffset()));
      __ RecordComment("]");
      break;
    case kArchFramePointer:
      __ RecordComment("[  mov(i.OutputRegister(), fp);");
      __ mov(i.OutputRegister(), fp);
      __ RecordComment("]");
      break;
    case kArchParentFramePointer:
      if (frame_access_state()->has_frame()) {
        __ RecordComment("[  Ld(i.OutputRegister(), MemOperand(fp, 0));");
        __ Ld(i.OutputRegister(), MemOperand(fp, 0));
        __ RecordComment("]");
      } else {
        __ RecordComment("[  mov(i.OutputRegister(), fp);");
        __ mov(i.OutputRegister(), fp);
        __ RecordComment("]");
      }
      break;

    case kRISCV64StackClaim: {
      __ RecordComment("[  Dadd(sp, sp, Operand(-i.InputInt32(0)));");
      __ Dadd(sp, sp, Operand(-i.InputInt32(0)));
      __ RecordComment("]");
      frame_access_state()->IncreaseSPDelta(i.InputInt32(0) /
                                            kSystemPointerSize);
      break;
    }
    case kRISCV64StoreToStackSlot: {
      if (instr->InputAt(0)->IsFPRegister()) {
        if (instr->InputAt(0)->IsSimd128Register()) {
          UNREACHABLE();
        } else {
          // store Floating-point Doubleword
          __ RecordComment(
              "[  Fsd(i.InputDoubleRegister(0), MemOperand(sp, "
              "i.InputInt32(1)));");
          __ Fsd(i.InputDoubleRegister(0), MemOperand(sp, i.InputInt32(1)));
          __ RecordComment("]");
        }
      } else {
        // store double word
        __ RecordComment(
            "[  Sd(i.InputRegister(0), MemOperand(sp, i.InputInt32(1)));");
        __ Sd(i.InputRegister(0), MemOperand(sp, i.InputInt32(1)));
        __ RecordComment("]");
      }
      break;
    }
    case kArchStoreWithWriteBarrier: {
      RecordWriteMode mode =
          static_cast<RecordWriteMode>(MiscField::decode(instr->opcode()));
      Register object = i.InputRegister(0);
      Register index = i.InputRegister(1);
      Register value = i.InputRegister(2);
      Register scratch0 = i.TempRegister(0);
      Register scratch1 = i.TempRegister(1);
      auto ool = new (zone())
          OutOfLineRecordWrite(this, object, index, value, scratch0, scratch1,
                               mode, DetermineStubCallMode());
      __ RecordComment("[  Dadd(kScratchRegister, object, index);");
      __ Dadd(kScratchRegister, object, index);
      __ RecordComment("]");
      __ RecordComment("[  Sd(value, MemOperand(kScratchRegister));");
      __ Sd(value, MemOperand(kScratchRegister));
      __ RecordComment("]");
      __ CheckPageFlag(object, scratch0,
                       MemoryChunk::kPointersFromHereAreInterestingMask, ne,
                       ool->entry());
      __ RecordComment("[  bind(ool->exit());");
      __ bind(ool->exit());
      __ RecordComment("]");
      break;
    }

    case kArchThrowTerminator:
      break;
    case kArchDebugBreak:
      __ RecordComment("[  DebugBreak();");
      __ DebugBreak();
      __ RecordComment("]");
      break;
    case kArchComment:
      __ RecordComment(reinterpret_cast<const char*>(i.InputInt64(0)));
      break;
    case kArchAbortCSAAssert:
      // std::cout<<"CG: kArchAbortCSAAssert: "<<std::endl;
      DCHECK(i.InputRegister(0) == a0);
      {
        // We don't actually want to generate a pile of code for this, so just
        // claim there is a stack frame, without generating one.
        FrameScope scope(tasm(), StackFrame::NONE);
        __ Call(
            isolate()->builtins()->builtin_handle(Builtins::kAbortCSAAssert),
            RelocInfo::CODE_TARGET);
      }
      __ RecordComment("[  ebreak();");
      __ ebreak();
      __ RecordComment("]");
      break;
    case kArchPrepareCallCFunction: {
      // std::cout<<"CG: kArchPrepareCallCFunction: "<<std::endl;
      int const num_parameters = MiscField::decode(instr->opcode());
      __ RecordComment(
          "[  PrepareCallCFunction(num_parameters, kScratchRegister);");
      __ PrepareCallCFunction(num_parameters, kScratchRegister);
      __ RecordComment("]");
      // Frame alignment requires using FP-relative frame addressing.
      frame_access_state()->SetFrameAccessToFP();
      break;
    }
    case kArchWordPoisonOnSpeculation:
      __ And(i.OutputRegister(), i.InputRegister(0),
             kSpeculationPoisonRegister);
      break;

    case kRISCV64Add:
      __ RecordComment(
          "[  Dadd(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dadd(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Add32:
      __ RecordComment(
          "[  Add(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Add(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64AddD:
      __ faddd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1), 0);
      break;
    case kRISCV64AddS:
      __ fadds(i.OutputFloatRegister(), i.InputFloatRegister(0),
               i.InputFloatRegister(1), 0);
      break;
    case kRISCV64Sub:
      __ RecordComment(
          "[  Dsub(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dsub(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Sub32:
      __ RecordComment(
          "[  Sub(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Sub(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64SubD:
      __ fsubd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1), 0);
      break;
    case kRISCV64SubS:
      __ fsubs(i.OutputFloatRegister(), i.InputFloatRegister(0),
               i.InputFloatRegister(1), 0);
      break;
    case kRISCV64Mul:
      __ RecordComment(
          "[  Dmul(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dmul(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Mul32:
      __ RecordComment(
          "[  Mul(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Mul(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64MulHigh:
      __ RecordComment(
          "[  Dmulh(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dmulh(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64MulHighU:
      __ RecordComment(
          "[  Dmulhu(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dmulhu(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64MulD:
      __ fmuld(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1), 0);
      break;
    case kRISCV64MulS:
      __ fmuls(i.OutputFloatRegister(), i.InputFloatRegister(0),
               i.InputFloatRegister(1), 0);
      break;
    case kRISCV64Div32:
      __ RecordComment(
          "[  Div(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Div(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      __ RecordComment(
          "[  Movz(i.OutputRegister(), i.InputRegister(1), "
          "i.InputRegister(1));");
      __ Movz(i.OutputRegister(), i.InputRegister(1), i.InputRegister(1));
      __ RecordComment("]");
      break;
    case kRISCV64DivU32:
      __ RecordComment(
          "[  Divu(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Divu(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      __ RecordComment(
          "[  Movz(i.OutputRegister(), i.InputRegister(1), "
          "i.InputRegister(1));");
      __ Movz(i.OutputRegister(), i.InputRegister(1), i.InputRegister(1));
      __ RecordComment("]");
      break;
    case kRISCV64Div:
      __ RecordComment(
          "[  Ddiv(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Ddiv(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      __ RecordComment(
          "[  Movz(i.OutputRegister(), i.InputRegister(1), "
          "i.InputRegister(1));");
      __ Movz(i.OutputRegister(), i.InputRegister(1), i.InputRegister(1));
      __ RecordComment("]");
      break;
    case kRISCV64DivU:
      __ RecordComment(
          "[  Ddivu(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Ddivu(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      __ RecordComment(
          "[  Movz(i.OutputRegister(), i.InputRegister(1), "
          "i.InputRegister(1));");
      __ Movz(i.OutputRegister(), i.InputRegister(1), i.InputRegister(1));
      __ RecordComment("]");
      break;
    case kRISCV64Mod32:
      __ RecordComment(
          "[  Mod(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Mod(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64ModU32:
      __ RecordComment(
          "[  Modu(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Modu(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Mod:
      __ RecordComment(
          "[  Dmod(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dmod(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64ModU:
      __ RecordComment(
          "[  Dmodu(i.OutputRegister(), i.InputRegister(0), "
          "i.InputOperand(1));");
      __ Dmodu(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64And:
      __ RecordComment(
          "[  And(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ And(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Or:
      __ RecordComment(
          "[  Or(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Or(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Nor:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  Nor(i.OutputRegister(), i.InputRegister(0), "
            "i.InputOperand(1));");
        __ Nor(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
        __ RecordComment("]");
      } else {
        // DCHECK_EQ(0, i.InputOperand(1).immediate());
        __ RecordComment(
            "[  Nor(i.OutputRegister(), i.InputRegister(0), zero_reg);");
        __ Nor(i.OutputRegister(), i.InputRegister(0), zero_reg);
        __ RecordComment("]");
      }
      break;
    case kRISCV64Xor:
      __ RecordComment(
          "[  Xor(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Xor(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Clz:
      // tbd
      UNIMPLEMENTED();
      break;
    case kRISCV64Ctz: {
      UNIMPLEMENTED();
    } break;
    case kRISCV64Popcnt: {
      UNIMPLEMENTED();
    } break;
    case kRISCV64Sll32:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sllw(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ sllw(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ slliw(i.OutputRegister(), i.InputRegister(0),
                 static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Srl32:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sllw(i.InputRegister(0), i.InputRegister(0), 0x0);");
        //__ sllw(i.InputRegister(0), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ RecordComment(
            "[  srlw(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ srlw(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ RecordComment(
            "[  sllw(i.OutputRegister(), i.InputRegister(0), 0x0);");
        //__ sllw(i.OutputRegister(), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ srliw(i.OutputRegister(), i.OutputRegister(),
                 static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Sra32:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sllw(i.InputRegister(0), i.InputRegister(0), 0x0);");
        //__ sllw(i.InputRegister(0), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ RecordComment(
            "[  sraw(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ sraw(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ RecordComment(
            "[  sllw(i.OutputRegister(), i.InputRegister(0), 0x0);");
        //__ sllw(i.OutputRegister(), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ sraiw(i.OutputRegister(), i.OutputRegister(),
                 static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Ext:
      // Extract Bit Field
      UNIMPLEMENTED();
      break;
    case kRISCV64Ins:
      // Insetrt Bit Field
      UNIMPLEMENTED();
      break;
    // Need to review
    case kRISCV64Sll:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sll(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ sll(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ slli(i.OutputRegister(), i.InputRegister(0),
                static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Srl:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sll(i.InputRegister(0), i.InputRegister(0), 0x0);");
        // __ sll(i.InputRegister(0), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ RecordComment(
            "[  srl(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ srl(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ RecordComment(
            "[  sllw(i.OutputRegister(), i.InputRegister(0), 0x0);");
        //__ sllw(i.OutputRegister(), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ srliw(i.OutputRegister(), i.OutputRegister(),
                 static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Sra:
      if (instr->InputAt(1)->IsRegister()) {
        __ RecordComment(
            "[  sll(i.InputRegister(0), i.InputRegister(0), 0x0);");
        //__ sll(i.InputRegister(0), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ RecordComment(
            "[  sra(i.OutputRegister(), i.InputRegister(0), "
            "i.InputRegister(1));");
        __ sra(i.OutputRegister(), i.InputRegister(0), i.InputRegister(1));
        __ RecordComment("]");
      } else {
        int64_t imm = i.InputOperand(1).immediate();
        __ RecordComment(
            "[  sllw(i.OutputRegister(), i.InputRegister(0), 0x0);");
        //__ sllw(i.OutputRegister(), i.InputRegister(0), 0x0);
        __ RecordComment("]");
        __ sraiw(i.OutputRegister(), i.OutputRegister(),
                 static_cast<uint16_t>(imm));
      }
      break;
    case kRISCV64Ror:
      __ RecordComment(
          "[  Ror(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));");
      __ Ror(i.OutputRegister(), i.InputRegister(0), i.InputOperand(1));
      __ RecordComment("]");
      break;
    case kRISCV64Cmp:
      // Pseudo-instruction used for cmp/branch. No opcode emitted here.
      break;
    case kRISCV64Mov:
      // TODO(plind): Should we combine mov/li like this, or use separate instr?
      //    - Also see x64 ASSEMBLE_BINOP & RegisterOrOperandType
      if (HasRegisterInput(instr, 0)) {
        __ RecordComment(
            "[  addi(i.OutputRegister(), i.InputRegister(0), 0x0);");
        __ addi(i.OutputRegister(), i.InputRegister(0), 0x0);
        __ RecordComment("]");
      } else {
        __ RecordComment("[  li(i.OutputRegister(), i.InputOperand(0));");
        __ li(i.OutputRegister(), i.InputOperand(0));
        __ RecordComment("]");
      }
      break;

    case kRISCV64CmpS: {
      UNIMPLEMENTED();
    } break;
    case kRISCV64DivS:
      __ fdivs(i.OutputSingleRegister(), i.InputSingleRegister(0),
               i.InputSingleRegister(1), 0);
      break;
    case kRISCV64ModS: {
      UNIMPLEMENTED();
      break;
    }
    case kRISCV64AbsS:
      __ fsgnjxs(i.OutputSingleRegister(), i.InputSingleRegister(0),
                 i.InputSingleRegister(0));
      break;
    case kRISCV64NegS:
      __ fsgnjns(i.OutputSingleRegister(), i.InputSingleRegister(0),
                 i.InputSingleRegister(0));
      break;
    case kRISCV64SqrtS: {
      __ fsqrts(i.OutputSingleRegister(), i.InputFloatRegister(0),
                i.InputUint32(0));
      break;
    }
    case kRISCV64MaxS:
      __ fmaxs(i.OutputSingleRegister(), i.InputFloatRegister(0),
               i.InputFloatRegister(1));
      break;
    case kRISCV64MinS:
      __ fmins(i.OutputSingleRegister(), i.InputFloatRegister(0),
               i.InputFloatRegister(1));
      break;
    case kRISCV64CmpD: {
      // tbd
      UNIMPLEMENTED();
      break;
    }
    case kRISCV64DivD:
      __ fdivd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1), 0);
      break;
    case kRISCV64ModD: {
      // tbd
      UNIMPLEMENTED();
      break;
    }
    case kRISCV64AbsD:
      __ fsgnjxd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
                 i.InputDoubleRegister(0));
      break;
    case kRISCV64NegD:
      __ fsgnjnd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
                 i.InputDoubleRegister(0));
      break;
    case kRISCV64SqrtD: {
      __ fsqrtd(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
                0 /*i.InputUint32(0)*/);
      break;
    }
    case kRISCV64MaxD:
      __ fmaxD(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1));
      break;
    case kRISCV64MinD:
      __ fmind(i.OutputDoubleRegister(), i.InputDoubleRegister(0),
               i.InputDoubleRegister(1));
      break;
    case kRISCV64Lbu:
      __ RecordComment("[  Lbu(i.OutputRegister(), i.MemoryOperand());");
      __ Lbu(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Lb:
      __ RecordComment("[  Lb(i.OutputRegister(), i.MemoryOperand());");
      __ Lb(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Sb:
      __ RecordComment("[  Sb(i.InputOrZeroRegister(2), i.MemoryOperand());");
      __ Sb(i.InputOrZeroRegister(2), i.MemoryOperand());
      __ RecordComment("]");
      break;
    case kRISCV64Lhu:
      __ RecordComment("[  Lhu(i.OutputRegister(), i.MemoryOperand());");
      __ Lhu(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Lh:
      __ RecordComment("[  Lh(i.OutputRegister(), i.MemoryOperand());");
      __ Lh(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
      break;
    case kRISCV64Sh:
      __ RecordComment("[  Sh(i.InputOrZeroRegister(2), i.MemoryOperand());");
      __ Sh(i.InputOrZeroRegister(2), i.MemoryOperand());
      __ RecordComment("]");
      break;
    case kRISCV64Lw:
      __ RecordComment("[  Lw(i.OutputRegister(), i.MemoryOperand());");
      __ Lw(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Lwu:
      __ RecordComment("[  Lwu(i.OutputRegister(), i.MemoryOperand());");
      __ Lwu(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Ld:
      __ RecordComment("[  Ld(i.OutputRegister(), i.MemoryOperand());");
      __ Ld(i.OutputRegister(), i.MemoryOperand());
      __ RecordComment("]");
      EmitWordLoadPoisoningIfNeeded(this, opcode, instr, i);
      break;
    case kRISCV64Sw:
      __ RecordComment("[  Sw(i.InputOrZeroRegister(2), i.MemoryOperand());");
      __ Sw(i.InputOrZeroRegister(2), i.MemoryOperand());
      __ RecordComment("]");
      break;
    case kRISCV64Sd:
      __ RecordComment("[  Sd(i.InputOrZeroRegister(2), i.MemoryOperand());");
      __ Sd(i.InputOrZeroRegister(2), i.MemoryOperand());
      __ RecordComment("]");
      break;
    case kRISCV64flw: {
      __ RecordComment("[  Flw(i.OutputSingleRegister(), i.MemoryOperand());");
      __ Flw(i.OutputSingleRegister(), i.MemoryOperand());
      __ RecordComment("]");
      break;
    }
    case kRISCV64fld:
      __ RecordComment("[  Fld(i.OutputDoubleRegister(), i.MemoryOperand());");
      __ Fld(i.OutputDoubleRegister(), i.MemoryOperand());
      __ RecordComment("]");
      break;
    case kRISCV64fsw: {
      size_t index = 0;
      MemOperand operand = i.MemoryOperand(&index);
      FPURegister ft = i.InputOrZeroSingleRegister(index);
      if (ft == kDoubleRegZero && !__ IsDoubleZeroRegSet()) {
        __ RecordComment("[  Fmove(kDoubleRegZero, 0.0);");
        __ Fmove(kDoubleRegZero, 0.0);
        __ RecordComment("]");
      }
      __ RecordComment("[  Fsw(i.InputSingleRegister(index), operand);");
      __ Fsw(i.InputSingleRegister(index), operand);
      __ RecordComment("]");
      break;
    }
    case kRISCV64fsd: {
      FPURegister ft = i.InputOrZeroDoubleRegister(2);
      if (ft == kDoubleRegZero && !__ IsDoubleZeroRegSet()) {
        __ RecordComment("[  Fmove(kDoubleRegZero, 0.0);");
        __ Fmove(kDoubleRegZero, 0.0);
        __ RecordComment("]");
      }
      __ RecordComment("[  Fsd(i.InputSingleRegister(2), i.MemoryOperand());");
      __ Fsd(i.InputSingleRegister(2), i.MemoryOperand());
      __ RecordComment("]");
      break;
    }
    default:
      break;
  }
  return kSuccess;

}  // NOLadability/fn_size)

#undef ASSEMBLE_UNOP
#undef ASSEMBLE_BINOP
#undef ASSEMBLE_COMPARE
#undef ASSEMBLE_MULT
#undef ASSEMBLE_SHIFT
#undef ASSEMBLE_MOVX
#undef ASSEMBLE_SSE_BINOP
#undef ASSEMBLE_SSE_UNOP
#undef ASSEMBLE_AVX_BINOP
#undef ASSEMBLE_IEEE754_BINOP
#undef ASSEMBLE_IEEE754_UNOP
#undef ASSEMBLE_ATOMIC_BINOP
#undef ASSEMBLE_ATOMIC64_BINOP
#undef ASSEMBLE_SIMD_INSTR
#undef ASSEMBLE_SIMD_IMM_INSTR
#undef ASSEMBLE_SIMD_PUNPCK_SHUFFLE
#undef ASSEMBLE_SIMD_IMM_SHUFFLE
#undef ASSEMBLE_SIMD_ALL_TRUE

void AssembleBranchToLabels(CodeGenerator* gen, TurboAssembler* tasm,
                            Instruction* instr, FlagsCondition condition,
                            Label* tlabel, Label* flabel, bool fallthru) {
#undef __
#define __ tasm->
  RISCV64OperandConverter i(gen, instr);

  Condition cc = kNoCondition;
  // RISCV does not have condition code flags, so compare and branch are
  // implemented differently than on the other arch's. The compare operations
  // emit mips pseudo-instructions, which are handled here by branch
  // instructions that do the actual comparison. Essential that the input
  // registers to compare pseudo-op are not modified before this branch op, as
  // they are tested here.

  if (instr->arch_opcode() == kRISCV64Cmp) {
    cc = FlagsConditionToConditionCmp(condition);
    __ RecordComment(
        "[  Branch(tlabel, cc, i.InputRegister(0), i.InputOperand(1));");
    __ Branch(tlabel, cc, i.InputRegister(0), i.InputOperand(1));
    __ RecordComment("]");
  } else if (instr->arch_opcode() == kArchStackPointerGreaterThan) {
    cc = FlagsConditionToConditionCmp(condition);
    Register lhs_register = sp;
    uint32_t offset;
    if (gen->ShouldApplyOffsetToStackCheck(instr, &offset)) {
      lhs_register = i.TempRegister(0);
      __ RecordComment("[  Dsub(lhs_register, sp, offset);");
      __ Dsub(lhs_register, sp, offset);
      __ RecordComment("]");
    }
    __ RecordComment(
        "[  Branch(tlabel, cc, lhs_register, Operand(i.InputRegister(0)));");
    __ Branch(tlabel, cc, lhs_register, Operand(i.InputRegister(0)));
    __ RecordComment("]");
  } else {
#if 0
    // tbd
    PrintF("AssembleArchBranch Unimplemented arch_opcode: %d\n",
           instr->arch_opcode());
    UNIMPLEMENTED();
#endif
  }
  __ RecordComment("[  Branch(flabel);  // no fallthru to flabel.");
  if (!fallthru) __ Branch(flabel);  // no fallthru to flabel.
  __ RecordComment("]");
#undef __
#define __ tasm()->
}

// Assembles branches after this instruction.
void CodeGenerator::AssembleArchBranch(Instruction* instr, BranchInfo* branch) {
  Label* tlabel = branch->true_label;
  Label* flabel = branch->false_label;

  AssembleBranchToLabels(this, tasm(), instr, branch->condition, tlabel, flabel,
                         branch->fallthru);
}

void CodeGenerator::AssembleBranchPoisoning(FlagsCondition condition,
                                            Instruction* instr) {
  UNIMPLEMENTED();
}

void CodeGenerator::AssembleArchDeoptBranch(Instruction* instr,
                                            BranchInfo* branch) {
  AssembleArchBranch(instr, branch);
}

void CodeGenerator::AssembleArchJump(RpoNumber target) {
  if (!IsNextInAssemblyOrder(target)) {
    __ RecordComment("[  Branch(GetLabel(target));");
    __ Branch(GetLabel(target));
    __ RecordComment("]");
  }
}

void CodeGenerator::AssembleArchTrap(Instruction* instr,
                                     FlagsCondition condition) {
  UNIMPLEMENTED();
}

// Assembles boolean materializations after this instruction.
void CodeGenerator::AssembleArchBoolean(Instruction* instr,
                                        FlagsCondition condition) {
  RISCV64OperandConverter i(this, instr);

  // Materialize a full 32-bit 1 or 0 value. The result register is always the
  // last output of the instruction.
  DCHECK_NE(0u, instr->OutputCount());
  Register result = i.OutputRegister(instr->OutputCount() - 1);
  Condition cc = kNoCondition;
  // RISCV does not have condition code flags, so compare and branch are
  // implemented differently than on the other arch's. The compare operations
  // emit mips pseudo-instructions, which are checked and handled here.

  if (instr->arch_opcode() == kRISCV64Cmp) {
    cc = FlagsConditionToConditionCmp(condition);
    switch (cc) {
      case eq:
      case ne: {
        Register left = i.InputRegister(0);
        Operand right = i.InputOperand(1);
        if (instr->InputAt(1)->IsImmediate()) {
          if (is_int16(-right.immediate())) {
            if (right.immediate() == 0) {
              if (cc == eq) {
                __ RecordComment("[  Sltu(result, left, 1);");
                __ Sltu(result, left, 1);
                __ RecordComment("]");
              } else {
                __ RecordComment("[  Sltu(result, zero_reg, left);");
                __ Sltu(result, zero_reg, left);
                __ RecordComment("]");
              }
            } else {
              __ RecordComment(
                  "[  Dadd(result, left, Operand(-right.immediate()));");
              __ Dadd(result, left, Operand(-right.immediate()));
              __ RecordComment("]");
              if (cc == eq) {
                __ RecordComment("[  Sltu(result, result, 1);");
                __ Sltu(result, result, 1);
                __ RecordComment("]");
              } else {
                __ RecordComment("[  Sltu(result, zero_reg, result);");
                __ Sltu(result, zero_reg, result);
                __ RecordComment("]");
              }
            }
          } else {
            if (is_uint16(right.immediate())) {
              __ RecordComment("[  Xor(result, left, right);");
              __ Xor(result, left, right);
              __ RecordComment("]");
            } else {
              __ RecordComment("[  li(kScratchRegister, right);");
              __ li(kScratchRegister, right);
              __ RecordComment("]");
              __ RecordComment("[  Xor(result, left, kScratchRegister);");
              __ Xor(result, left, kScratchRegister);
              __ RecordComment("]");
            }
            if (cc == eq) {
              __ RecordComment("[  Sltu(result, result, 1);");
              __ Sltu(result, result, 1);
              __ RecordComment("]");
            } else {
              __ RecordComment("[  Sltu(result, zero_reg, result);");
              __ Sltu(result, zero_reg, result);
              __ RecordComment("]");
            }
          }
        } else {
          __ RecordComment("[  Xor(result, left, right);");
          __ Xor(result, left, right);
          __ RecordComment("]");
          if (cc == eq) {
            __ RecordComment("[  Sltu(result, result, 1);");
            __ Sltu(result, result, 1);
            __ RecordComment("]");
          } else {
            __ RecordComment("[  Sltu(result, zero_reg, result);");
            __ Sltu(result, zero_reg, result);
            __ RecordComment("]");
          }
        }
      } break;
      case lt:
      case ge: {
        Register left = i.InputRegister(0);
        Operand right = i.InputOperand(1);
        __ RecordComment("[  Slt(result, left, right);");
        __ Slt(result, left, right);
        __ RecordComment("]");
        if (cc == ge) {
          __ RecordComment("[  xori(result, result, 1);");
          __ xori(result, result, 1);
          __ RecordComment("]");
        }
      } break;
      case gt:
      case le: {
        Register left = i.InputRegister(1);
        Operand right = i.InputOperand(0);
        __ RecordComment("[  Slt(result, left, right);");
        __ Slt(result, left, right);
        __ RecordComment("]");
        if (cc == le) {
          __ RecordComment("[  xori(result, result, 1);");
          __ xori(result, result, 1);
          __ RecordComment("]");
        }
      } break;
      case lo:
      case hs: {
        Register left = i.InputRegister(0);
        Operand right = i.InputOperand(1);
        __ RecordComment("[  Sltu(result, left, right);");
        __ Sltu(result, left, right);
        __ RecordComment("]");
        if (cc == hs) {
          __ RecordComment("[  xori(result, result, 1);");
          __ xori(result, result, 1);
          __ RecordComment("]");
        }
      } break;
      case hi:
      case ls: {
        Register left = i.InputRegister(1);
        Operand right = i.InputOperand(0);
        __ RecordComment("[  Sltu(result, left, right);");
        __ Sltu(result, left, right);
        __ RecordComment("]");
        if (cc == ls) {
          __ RecordComment("[  xori(result, result, 1);");
          __ xori(result, result, 1);
          __ RecordComment("]");
        }
      } break;
      default:
        UNREACHABLE();
    }
    return;
  } else {
#if 0
    // tbd
    PrintF("AssembleArchBranch Unimplemented arch_opcode is : %d\n",
           instr->arch_opcode());
    UNIMPLEMENTED();
#endif
  }
}

void CodeGenerator::AssembleArchBinarySearchSwitch(Instruction* instr) {
  UNIMPLEMENTED();
}

void CodeGenerator::AssembleArchTableSwitch(Instruction* instr) {
  UNIMPLEMENTED();
}

void CodeGenerator::FinishFrame(Frame* frame) {
  auto call_descriptor = linkage()->GetIncomingDescriptor();

  const RegList saves_fpu = call_descriptor->CalleeSavedFPRegisters();
  if (saves_fpu != 0) {
    int count = base::bits::CountPopulation(saves_fpu);
    DCHECK_EQ(kNumCalleeSavedFPU, count);
    frame->AllocateSavedCalleeRegisterSlots(count *
                                            (kDoubleSize / kSystemPointerSize));
  }

  const RegList saves = call_descriptor->CalleeSavedRegisters();
  if (saves != 0) {
    int count = base::bits::CountPopulation(saves);
    DCHECK_EQ(kNumCalleeSaved, count + 1);
    frame->AllocateSavedCalleeRegisterSlots(count);
  }
}

void CodeGenerator::AssembleConstructFrame() {
  auto call_descriptor = linkage()->GetIncomingDescriptor();

  if (frame_access_state()->has_frame()) {
    if (call_descriptor->IsCFunctionCall()) {
      if (info()->GetOutputStackFrameType() == StackFrame::C_WASM_ENTRY) {
        __ RecordComment("[  StubPrologue(StackFrame::C_WASM_ENTRY);");
        __ StubPrologue(StackFrame::C_WASM_ENTRY);
        __ RecordComment("]");
        // Reserve stack space for saving the c_entry_fp later.
        __ RecordComment("[  Dsub(sp, sp, Operand(kSystemPointerSize));");
        __ Dsub(sp, sp, Operand(kSystemPointerSize));
        __ RecordComment("]");
      } else {
        // push ra and fp register. first addi sp, sp, 16
        // then store ra and fp on the stack
        __ RecordComment("[  Push(ra, fp);");
        __ Push(ra, fp);
        __ RecordComment("]");
        // copy sp to fp reg.
        __ RecordComment("[  mov(fp, sp);");
        __ mov(fp, sp);
        __ RecordComment("]");
      }
    } else if (call_descriptor->IsJSFunctionCall()) {
      __ RecordComment("[  Prologue();");
      __ Prologue();
      __ RecordComment("]");
      if (call_descriptor->PushArgumentCount()) {
        __ RecordComment("[  Push(kJavaScriptCallArgCountRegister);");
        __ Push(kJavaScriptCallArgCountRegister);
        __ RecordComment("]");
      }
    } else {
      __ RecordComment("[  StubPrologue(info()->GetOutputStackFrameType());");
      __ StubPrologue(info()->GetOutputStackFrameType());
      __ RecordComment("]");
      if (call_descriptor->IsWasmFunctionCall()) {
        __ RecordComment("[  Push(kWasmInstanceRegister);");
        __ Push(kWasmInstanceRegister);
        __ RecordComment("]");
      } else if (call_descriptor->IsWasmImportWrapper() ||
                 call_descriptor->IsWasmCapiFunction()) {
        // Wasm import wrappers are passed a tuple in the place of the instance.
        // Unpack the tuple into the instance and the target callable.
        // This must be done here in the codegen because it cannot be expressed
        // properly in the graph.
        __ ld(kJSFunctionRegister,
              FieldMemOperand(kWasmInstanceRegister, Tuple2::kValue2Offset));
        __ ld(kWasmInstanceRegister,
              FieldMemOperand(kWasmInstanceRegister, Tuple2::kValue1Offset));
        __ RecordComment("[  Push(kWasmInstanceRegister);");
        __ Push(kWasmInstanceRegister);
        __ RecordComment("]");
        if (call_descriptor->IsWasmCapiFunction()) {
          // Reserve space for saving the PC later.
          __ RecordComment("[  Dsub(sp, sp, Operand(kSystemPointerSize));");
          __ Dsub(sp, sp, Operand(kSystemPointerSize));
          __ RecordComment("]");
        }
      }
    }
  }

  int required_slots =
      frame()->GetTotalFrameSlotCount() - frame()->GetFixedSlotCount();

  if (info()->is_osr()) {
    // TurboFan OSR-compiled functions cannot be entered directly.
    __ RecordComment(
        "[  Abort(AbortReason::kShouldNotDirectlyEnterOsrFunction);");
    __ Abort(AbortReason::kShouldNotDirectlyEnterOsrFunction);
    __ RecordComment("]");
    // Unoptimized code jumps directly to this entrypoint while the unoptimized
    // frame is still on the stack. Optimized code uses OSR values directly from
    // the unoptimized frame. Thus, all that needs to be done is to allocate the
    // remaining stack slots.
    if (FLAG_code_comments) __ RecordComment("-- OSR entrypoint --");
    __ RecordComment("[  pc_offset();");
    osr_pc_offset_ = __ pc_offset();
    __ RecordComment("]");
    required_slots -= osr_helper()->UnoptimizedFrameSlots();
    ResetSpeculationPoison();
  }

  const RegList saves = call_descriptor->CalleeSavedRegisters();
  const RegList saves_fpu = call_descriptor->CalleeSavedFPRegisters();

  if (required_slots > 0) {
    DCHECK(frame_access_state()->has_frame());
    if (info()->IsWasm() && required_slots > 128) {
      // Not support wasm yet
#if 0
      // For WebAssembly functions with big frames we have to do the stack
      // overflow check before we construct the frame. Otherwise we may not
      // have enough space on the stack to call the runtime for the stack
      // overflow.
      Label done;

      // If the frame is bigger than the stack, we throw the stack overflow
      // exception unconditionally. Thereby we can avoid the integer overflow
      // check in the condition code.
      if ((required_slots * kSystemPointerSize) < (FLAG_stack_size * 1024)) {
        __ Ld(
             kScratchRegister,
             FieldMemOperand(kWasmInstanceRegister,
                             WasmInstanceObject::kRealStackLimitAddressOffset));
__ RecordComment("[  Ld(kScratchRegister, MemOperand(kScratchRegister));");
        __ Ld(kScratchRegister, MemOperand(kScratchRegister));
__ RecordComment("]");
        __ Dadd(kScratchRegister, kScratchRegister,
                 Operand(required_slots * kSystemPointerSize));
		// uge?
__ RecordComment("[  Branch(&done, ge, sp, Operand(kScratchRegister));");
        __ Branch(&done, ge, sp, Operand(kScratchRegister));
__ RecordComment("]");
      }

__ RecordComment("[  Call(wasm::WasmCode::kWasmStackOverflow, RelocInfo::WASM_STUB_CALL);");
      __ Call(wasm::WasmCode::kWasmStackOverflow, RelocInfo::WASM_STUB_CALL);
__ RecordComment("]");
      // We come from WebAssembly, there are no references for the GC.
      ReferenceMap* reference_map = new (zone()) ReferenceMap(zone());
      RecordSafepoint(reference_map, Safepoint::kNoLazyDeopt);
      if (FLAG_debug_code) {
__ RecordComment("[  ebreak();");
        __ ebreak();
__ RecordComment("]");
      }

__ RecordComment("[  bind(&done);");
      __ bind(&done);
__ RecordComment("]");
#endif
    }
  }

  const int returns = frame()->GetReturnSlotCount();

  // Skip callee-saved and return slots, which are pushed below.
  required_slots -= base::bits::CountPopulation(saves);
  required_slots -= base::bits::CountPopulation(saves_fpu);
  required_slots -= returns;
  if (required_slots > 0) {
    __ RecordComment(
        "[  Dadd(sp, sp, Operand(-required_slots * kSystemPointerSize));");
    __ Dadd(sp, sp, Operand(-required_slots * kSystemPointerSize));
    __ RecordComment("]");
  }

  if (saves_fpu != 0) {
    // Save callee-saved FPU registers.
    __ RecordComment("[  MultiPushFPU(saves_fpu);");
    __ MultiPushFPU(saves_fpu);
    __ RecordComment("]");
    DCHECK_EQ(kNumCalleeSavedFPU, base::bits::CountPopulation(saves_fpu));
  }

  if (saves != 0) {
    // Save callee-saved registers.
    __ RecordComment("[  MultiPush(saves);");
    __ MultiPush(saves);
    __ RecordComment("]");
    DCHECK_EQ(kNumCalleeSaved, base::bits::CountPopulation(saves) + 1);
  }

  if (returns != 0) {
    // Create space for returns.
    __ RecordComment(
        "[  Dadd(sp, sp, Operand(-returns * kSystemPointerSize));");
    __ Dadd(sp, sp, Operand(-returns * kSystemPointerSize));
    __ RecordComment("]");
  }
}

void CodeGenerator::AssembleReturn(InstructionOperand* pop) {
  auto call_descriptor = linkage()->GetIncomingDescriptor();

  const int returns = frame()->GetReturnSlotCount();
  if (returns != 0) {
    __ RecordComment("[  Dadd(sp, sp, Operand(returns * kSystemPointerSize));");
    __ Dadd(sp, sp, Operand(returns * kSystemPointerSize));
    __ RecordComment("]");
  }

  // Restore GP registers.
  const RegList saves = call_descriptor->CalleeSavedRegisters();
  if (saves != 0) {
    __ RecordComment("[  MultiPop(saves);");
    __ MultiPop(saves);
    __ RecordComment("]");
  }

  // Restore FPU registers.
  const RegList saves_fpu = call_descriptor->CalleeSavedFPRegisters();
  if (saves_fpu != 0) {
    __ RecordComment("[  MultiPopFPU(saves_fpu);");
    __ MultiPopFPU(saves_fpu);
    __ RecordComment("]");
  }

  RISCV64OperandConverter g(this, nullptr);
  if (call_descriptor->IsCFunctionCall()) {
    AssembleDeconstructFrame();
  } else if (frame_access_state()->has_frame()) {
    // Canonicalize JSFunction return sites for now unless they have an variable
    // number of stack slot pops.
    if (pop->IsImmediate() && g.ToConstant(pop).ToInt32() == 0) {
      if (return_label_.is_bound()) {
        __ RecordComment("[  Branch(&return_label_);");
        __ Branch(&return_label_);
        __ RecordComment("]");
        return;
      } else {
        __ RecordComment("[  bind(&return_label_);");
        __ bind(&return_label_);
        __ RecordComment("]");
        AssembleDeconstructFrame();
      }
    } else {
      AssembleDeconstructFrame();
    }
  }
  int pop_count = static_cast<int>(call_descriptor->StackParameterCount());
  if (pop->IsImmediate()) {
    pop_count += g.ToConstant(pop).ToInt32();
  } else {
    Register pop_reg = g.ToRegister(pop);
    // kSystemPointerSizeLog2 is 3
    __ RecordComment("[  slli(pop_reg, pop_reg, kSystemPointerSizeLog2);");
    __ slli(pop_reg, pop_reg, kSystemPointerSizeLog2);
    __ RecordComment("]");
    __ RecordComment("[  Dadd(sp, sp, pop_reg);");
    __ Dadd(sp, sp, pop_reg);
    __ RecordComment("]");
  }
  if (pop_count != 0) {
    __ RecordComment("[  Dadd(sp, sp, pop_count * kPointerSize);");
    __ Dadd(sp, sp, pop_count * kPointerSize);
    __ RecordComment("]");
  }
  __ RecordComment("[  Ret();");
  __ Ret();
  __ RecordComment("]");
}

void CodeGenerator::FinishCode() {
  // do nothing, as mips/mips64/ppc/s390
}

void CodeGenerator::PrepareForDeoptimizationExits(int deopt_count) {
  // do nothing, as mips/mips64/x64/ia32/ppc/s390
}

void CodeGenerator::AssembleMove(InstructionOperand* source,
                                 InstructionOperand* destination) {
  RISCV64OperandConverter g(this, nullptr);
  // Dispatch on the source and destination operand kinds.  Not all
  // combinations are possible.
  if (source->IsRegister()) {
    DCHECK(destination->IsRegister() || destination->IsStackSlot());
    Register src = g.ToRegister(source);
    if (destination->IsRegister()) {
      // move from reg to reg
      __ RecordComment("[  mov(g.ToRegister(destination), src);");
      __ mov(g.ToRegister(destination), src);
      __ RecordComment("]");
    } else {
      // move from reg to memory
      __ RecordComment("[  Sd(src, g.ToMemOperand(destination));");
      __ Sd(src, g.ToMemOperand(destination));
      __ RecordComment("]");
    }
  } else if (source->IsStackSlot()) {
    DCHECK(destination->IsRegister() || destination->IsStackSlot());
    MemOperand src = g.ToMemOperand(source);
    if (destination->IsRegister()) {
      // move from memory to reg
      __ RecordComment("[  Ld(g.ToRegister(destination), src);");
      __ Ld(g.ToRegister(destination), src);
      __ RecordComment("]");
    } else {
      // move from memory to memory
      Register temp = kScratchRegister;
      __ RecordComment("[  Ld(temp, src);");
      __ Ld(temp, src);
      __ RecordComment("]");
      __ RecordComment("[  Sd(temp, g.ToMemOperand(destination));");
      __ Sd(temp, g.ToMemOperand(destination));
      __ RecordComment("]");
    }
  } else if (source->IsConstant()) {
    Constant src = g.ToConstant(source);
    if (destination->IsRegister() || destination->IsStackSlot()) {
      Register dst = destination->IsRegister() ? g.ToRegister(destination)
                                               : kScratchRegister;
      switch (src.type()) {
        case Constant::kInt32:
          __ RecordComment("[  li(dst, Operand(src.ToInt32()));");
          __ li(dst, Operand(src.ToInt32()));
          __ RecordComment("]");
          break;
        case Constant::kFloat32:
          __ RecordComment(
              "[  li(dst, Operand::EmbeddedNumber(src.ToFloat32()));");
          __ li(dst, Operand::EmbeddedNumber(src.ToFloat32()));
          __ RecordComment("]");
          break;
        case Constant::kInt64:
          if (RelocInfo::IsWasmReference(src.rmode())) {
            __ RecordComment(
                "[  li(dst, Operand(src.ToInt64(), src.rmode()));");
            __ li(dst, Operand(src.ToInt64(), src.rmode()));
            __ RecordComment("]");
          } else {
            __ RecordComment("[  li(dst, Operand(src.ToInt64()));");
            __ li(dst, Operand(src.ToInt64()));
            __ RecordComment("]");
          }
          break;
        case Constant::kFloat64:
          __ RecordComment(
              "[  li(dst, Operand::EmbeddedNumber(src.ToFloat64().value()));");
          __ li(dst, Operand::EmbeddedNumber(src.ToFloat64().value()));
          __ RecordComment("]");
          break;
        case Constant::kExternalReference:
          __ RecordComment("[  li(dst, src.ToExternalReference());");
          __ li(dst, src.ToExternalReference());
          __ RecordComment("]");
          break;
        case Constant::kDelayedStringConstant:
          __ RecordComment("[  li(dst, src.ToDelayedStringConstant());");
          __ li(dst, src.ToDelayedStringConstant());
          __ RecordComment("]");
          break;
        case Constant::kHeapObject: {
          Handle<HeapObject> src_object = src.ToHeapObject();
          RootIndex index;
          if (IsMaterializableFromRoot(src_object, &index)) {
            __ RecordComment("[  LoadRoot(dst, index);");
            __ LoadRoot(dst, index);
            __ RecordComment("]");
          } else {
            __ RecordComment("[  li(dst, src_object);");
            __ li(dst, src_object);
            __ RecordComment("]");
          }
          break;
        }
        case Constant::kCompressedHeapObject:
          UNREACHABLE();
        case Constant::kRpoNumber:
          UNREACHABLE();  // TODO(titzer): loading RPO numbers on riscv64.
          break;
      }
      __ RecordComment("[  Sd(dst, g.ToMemOperand(destination));");
      if (destination->IsStackSlot()) __ Sd(dst, g.ToMemOperand(destination));
      __ RecordComment("]");
    } else if (src.type() == Constant::kFloat32) {
      if (destination->IsFPStackSlot()) {
        MemOperand dst = g.ToMemOperand(destination);
        if (bit_cast<int32_t>(src.ToFloat32()) == 0) {
          __ RecordComment("[  Sd(zero_reg, dst);");
          __ Sd(zero_reg, dst);
          __ RecordComment("]");
        } else {
          __ RecordComment(
              "[  li(kScratchRegister, "
              "Operand(bit_cast<int32_t>(src.ToFloat32())));");
          __ li(kScratchRegister, Operand(bit_cast<int32_t>(src.ToFloat32())));
          __ RecordComment("]");
          __ RecordComment("[  Sd(kScratchRegister, dst);");
          __ Sd(kScratchRegister, dst);
          __ RecordComment("]");
        }
      } else {
        DCHECK(destination->IsFPRegister());
        FloatRegister dst = g.ToSingleRegister(destination);
        __ RecordComment("[  Fmove(dst, src.ToFloat32());");
        __ Fmove(dst, src.ToFloat32());
        __ RecordComment("]");
      }
    } else {
      DCHECK_EQ(Constant::kFloat64, src.type());
      DoubleRegister dst = destination->IsFPRegister()
                               ? g.ToDoubleRegister(destination)
                               : kScratchDoubleReg;
      __ RecordComment("[  Fmove(dst, src.ToFloat64().value());");
      __ Fmove(dst, src.ToFloat64().value());
      __ RecordComment("]");
      if (destination->IsFPStackSlot()) {
        __ RecordComment("[  Fsd(dst, g.ToMemOperand(destination));");
        __ Fsd(dst, g.ToMemOperand(destination));
        __ RecordComment("]");
      }
    }
  } else if (source->IsFPRegister()) {
    MachineRepresentation rep = LocationOperand::cast(source)->representation();
    if (rep == MachineRepresentation::kSimd128) {
      UNREACHABLE();
    } else {
      FPURegister src = g.ToDoubleRegister(source);
      if (destination->IsFPRegister()) {
        FPURegister dst = g.ToDoubleRegister(destination);
        __ RecordComment("[  Fmovd(dst, src);");
        __ Fmovd(dst, src);
        __ RecordComment("]");
      } else {
        DCHECK(destination->IsFPStackSlot());
        __ RecordComment("[  Fsd(src, g.ToMemOperand(destination));");
        __ Fsd(src, g.ToMemOperand(destination));
        __ RecordComment("]");
      }
    }
  } else if (source->IsFPStackSlot()) {
    DCHECK(destination->IsFPRegister() || destination->IsFPStackSlot());
    MemOperand src = g.ToMemOperand(source);
    MachineRepresentation rep = LocationOperand::cast(source)->representation();
    if (rep == MachineRepresentation::kSimd128) {
      UNREACHABLE();
    } else {
      if (destination->IsFPRegister()) {
        __ RecordComment("[  Fld(g.ToDoubleRegister(destination), src);");
        __ Fld(g.ToDoubleRegister(destination), src);
        __ RecordComment("]");
      } else {
        DCHECK(destination->IsFPStackSlot());
        FPURegister temp = kScratchDoubleReg;
        __ RecordComment("[  Fld(temp, src);");
        __ Fld(temp, src);
        __ RecordComment("]");
        __ RecordComment("[  Fsd(temp, g.ToMemOperand(destination));");
        __ Fsd(temp, g.ToMemOperand(destination));
        __ RecordComment("]");
      }
    }
  } else {
    UNREACHABLE();
  }
}

void CodeGenerator::AssembleSwap(InstructionOperand* source,
                                 InstructionOperand* destination) {
  RISCV64OperandConverter g(this, nullptr);
  // Dispatch on the source and destination operand kinds.  Not all
  // combinations are possible.
  if (source->IsRegister()) {
    // Register-register.
    Register temp = kScratchRegister;
    Register src = g.ToRegister(source);
    if (destination->IsRegister()) {
      Register dst = g.ToRegister(destination);
      __ RecordComment("[  Move(temp, src);");
      __ Move(temp, src);
      __ RecordComment("]");
      __ RecordComment("[  Move(src, dst);");
      __ Move(src, dst);
      __ RecordComment("]");
      __ RecordComment("[  Move(dst, temp);");
      __ Move(dst, temp);
      __ RecordComment("]");
    } else {
      DCHECK(destination->IsStackSlot());
      MemOperand dst = g.ToMemOperand(destination);
      __ RecordComment("[  mov(temp, src);");
      __ mov(temp, src);
      __ RecordComment("]");
      __ RecordComment("[  Ld(src, dst);");
      __ Ld(src, dst);
      __ RecordComment("]");
      __ RecordComment("[  Sd(temp, dst);");
      __ Sd(temp, dst);
      __ RecordComment("]");
    }
  } else if (source->IsStackSlot()) {
    DCHECK(destination->IsStackSlot());
    Register temp_0 = kScratchRegister;
    Register temp_1 = kScratchRegister2;
    MemOperand src = g.ToMemOperand(source);
    MemOperand dst = g.ToMemOperand(destination);
    __ RecordComment("[  Ld(temp_0, src);");
    __ Ld(temp_0, src);
    __ RecordComment("]");
    __ RecordComment("[  Ld(temp_1, dst);");
    __ Ld(temp_1, dst);
    __ RecordComment("]");
    __ RecordComment("[  Sd(temp_0, dst);");
    __ Sd(temp_0, dst);
    __ RecordComment("]");
    __ RecordComment("[  Sd(temp_1, src);");
    __ Sd(temp_1, src);
    __ RecordComment("]");
  } else if (source->IsFPRegister()) {
    MachineRepresentation rep = LocationOperand::cast(source)->representation();
    if (rep == MachineRepresentation::kSimd128) {
      UNREACHABLE();
    } else {
      FPURegister temp = kScratchDoubleReg;
      FPURegister src = g.ToDoubleRegister(source);
      if (destination->IsFPRegister()) {
        FPURegister dst = g.ToDoubleRegister(destination);
        __ RecordComment("[  Fmovd(temp, src);");
        __ Fmovd(temp, src);
        __ RecordComment("]");
        __ RecordComment("[  Fmovd(src, dst);");
        __ Fmovd(src, dst);
        __ RecordComment("]");
        __ RecordComment("[  Fmovd(dst, temp);");
        __ Fmovd(dst, temp);
        __ RecordComment("]");
      } else {
        DCHECK(destination->IsFPStackSlot());
        MemOperand dst = g.ToMemOperand(destination);
        __ RecordComment("[  Fmovd(temp, src);");
        __ Fmovd(temp, src);
        __ RecordComment("]");
        __ RecordComment("[  Fld(src, dst);");
        __ Fld(src, dst);
        __ RecordComment("]");
        __ RecordComment("[  Fsd(temp, dst);");
        __ Fsd(temp, dst);
        __ RecordComment("]");
      }
    }
  } else if (source->IsFPStackSlot()) {
    DCHECK(destination->IsFPStackSlot());
    Register temp_0 = kScratchRegister;
    MemOperand src0 = g.ToMemOperand(source);
    MemOperand src1(src0.rm(), src0.offset() + kIntSize);
    MemOperand dst0 = g.ToMemOperand(destination);
    MemOperand dst1(dst0.rm(), dst0.offset() + kIntSize);
    MachineRepresentation rep = LocationOperand::cast(source)->representation();
    if (rep == MachineRepresentation::kSimd128) {
      UNREACHABLE();
    } else {
      FPURegister temp_1 = kScratchDoubleReg;
      __ RecordComment("[  Fld(temp_1, dst0);  // Save destination in temp_1.");
      __ Fld(temp_1, dst0);  // Save destination in temp_1.
      __ RecordComment("]");
      __ RecordComment(
          "[  Lw(temp_0, src0);   // Then use temp_0 to copy source to "
          "destination.");
      __ Lw(temp_0, src0);  // Then use temp_0 to copy source to destination.
      __ RecordComment("]");
      __ RecordComment("[  Sw(temp_0, dst0);");
      __ Sw(temp_0, dst0);
      __ RecordComment("]");
      __ RecordComment("[  Lw(temp_0, src1);");
      __ Lw(temp_0, src1);
      __ RecordComment("]");
      __ RecordComment("[  Sw(temp_0, dst1);");
      __ Sw(temp_0, dst1);
      __ RecordComment("]");
      __ RecordComment("[  Fsd(temp_1, src0);");
      __ Fsd(temp_1, src0);
      __ RecordComment("]");
    }
  } else {
    // No other combinations are possible.
    UNREACHABLE();
  }
}

void CodeGenerator::AssembleJumpTable(Label** targets, size_t target_count) {
  UNIMPLEMENTED();
}

#undef __

}  // namespace compiler
}  // namespace internal
}  // namespace v8
