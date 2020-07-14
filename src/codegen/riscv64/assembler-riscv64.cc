// Copyright 2013 the V8 project authors. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#if V8_TARGET_ARCH_RISCV64

#include "src/codegen/riscv64/assembler-riscv64.h"

#include "src/base/bits.h"
#include "src/base/cpu.h"
#include "src/codegen/register-configuration.h"
#include "src/codegen/riscv64/assembler-riscv64-inl.h"
#include "src/codegen/safepoint-table.h"
#include "src/codegen/string-constants.h"
#include "src/execution/frame-constants.h"

namespace v8 {
namespace internal {

// -----------------------------------------------------------------------------
// CpuFeatures implementation.

void CpuFeatures::ProbeImpl(bool cross_compile) {
  // riscv has no configuration options, no further probing is required.
  supported_ = 0;

  // Only use statically determined features for cross compile (snapshot).
  if (cross_compile) return;
}

void CpuFeatures::PrintTarget() {}
void CpuFeatures::PrintFeatures() {}

const int RelocInfo::kApplyMask =
    RelocInfo::ModeMask(RelocInfo::CODE_TARGET) |
    RelocInfo::ModeMask(RelocInfo::RUNTIME_ENTRY) |
    RelocInfo::ModeMask(RelocInfo::INTERNAL_REFERENCE) |
    RelocInfo::ModeMask(RelocInfo::WASM_CALL);

int ToNumber(Register reg) {
  DCHECK(reg.is_valid());
  const int kNumbers[] = {
      0,   // x0
      1,   // x1
      2,   // x2
      3,   // x3
      4,   // x4
      5,   // x5
      6,   // x6
      7,   // x7
      8,   // x8
      9,   // x9
      10,  // x10
      11,  // x11
      12,  // x12
      13,  // x13
      14,  // x14
      15,  // x15
      16,  // x16
      17,  // x17
      18,  // x18
      19,  // x19
      20,  // x20
      21,  // x21
      22,  // x22
      23,  // x23
      24,  // x24
      25,  // x25
      26,  // x26
      27,  // x27
      28,  // x28
      29,  // x29
      30,  // x30
      31,  // x31
  };
  return kNumbers[reg.code()];
}

int ToNumber(FPURegister reg) {
  DCHECK(reg.is_valid());
  const int kNumbers[] = {
      0,   // f0
      1,   // f1
      2,   // f2
      3,   // f3
      4,   // f4
      5,   // f5
      6,   // f6
      7,   // f7
      8,   // f8
      9,   // f9
      10,  // f10
      11,  // f11
      12,  // f12
      13,  // f13
      14,  // f14
      15,  // f15
      16,  // f16
      17,  // f17
      18,  // f18
      19,  // f19
      20,  // f20
      21,  // f21
      22,  // f22
      23,  // f23
      24,  // f24
      25,  // f25
      26,  // f26
      27,  // f27
      28,  // f28
      29,  // f29
      30,  // f30
      31,  // f31
  };
  return kNumbers[reg.code()];
}

Register ToRegister(int num) {
  DCHECK(num >= 0 && num < kNumRegisters);
  const Register kRegisters[] = {x0,  x1,  x2,  x3,  x4,  x5,  x6,  x7,
                                 x8,  x9,  x10, x11, x12, x13, x14, x15,
                                 x16, x17, x18, x19, x20, x21, x22, x23,
                                 x24, x25, x26, x27, x28, x29, x30, x31};
  return kRegisters[num];
}

FPURegister ToFPURegister(int num) {
  DCHECK(num >= 0 && num < kNumRegisters);
  const FPURegister kRegisters[] = {f0,  f1,  f2,  f3,  f4,  f5,  f6,  f7,
                                    f8,  f9,  f10, f11, f12, f13, f14, f15,
                                    f16, f17, f18, f19, f20, f21, f22, f23,
                                    f24, f25, f26, f27, f28, f29, f30, f31};
  return kRegisters[num];
}

// -----------------------------------------------------------------------------
// Implementation of Operand and MemOperand.
// See assembler-mips-inl.h for inlined constructors.

Operand::Operand(Handle<HeapObject> handle)
    : rm_(no_reg), rmode_(RelocInfo::FULL_EMBEDDED_OBJECT) {
  value_.immediate = static_cast<intptr_t>(handle.address());
}

Operand Operand::EmbeddedNumber(double value) {
  int32_t smi;
  if (DoubleToSmiInteger(value, &smi)) return Operand(Smi::FromInt(smi));
  Operand result(0, RelocInfo::FULL_EMBEDDED_OBJECT);
  result.is_heap_object_request_ = true;
  result.value_.heap_object_request = HeapObjectRequest(value);
  return result;
}

Operand Operand::EmbeddedStringConstant(const StringConstantBase* str) {
  Operand result(0, RelocInfo::FULL_EMBEDDED_OBJECT);
  result.is_heap_object_request_ = true;
  result.value_.heap_object_request = HeapObjectRequest(str);
  return result;
}

MemOperand::MemOperand(Register rm, int32_t offset) : Operand(rm) {
  offset_ = offset;
}

MemOperand::MemOperand(Register rm, int32_t unit, int32_t multiplier,
                       OffsetAddend offset_addend)
    : Operand(rm) {
  offset_ = unit * multiplier + offset_addend;
}

bool RelocInfo::IsCodedSpecially() {
  // The deserializer needs to know whether a pointer is specially coded.  Being
  // specially coded on x64 means that it is a relative 32 bit address, as used
  // by branch instructions.
  //  return (1 << rmode_) & kApplyMask;
  return false;
}
bool RelocInfo::IsInConstantPool() { return false; }
// -----------------------------------------------------------------------------
// Implementation of RelocInfo

uint32_t RelocInfo::wasm_call_tag() const {
  DCHECK(rmode_ == WASM_CALL || rmode_ == WASM_STUB_CALL);
  return ReadUnalignedValue<uint32_t>(pc_);
}

Address Assembler::target_address_at(Address pc) {
  Instr instr0 = instr_at(pc);
  Instr instr1 = instr_at(pc + 1 * kInstrSize);
  Instr instr3 = instr_at(pc + 3 * kInstrSize);
  Instr instr5 = instr_at(pc + 5 * kInstrSize);
  Instr instr7 = instr_at(pc + 7 * kInstrSize);
  // Interpret 8 instructions for address generated by li: See listing in
  // Assembler::set_target_address_at() just below.

  if (((instr0 & 0x7f) == LUI) &&
      (((instr1 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
      (((instr3 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
      (((instr5 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
      (((instr7 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI)) {
    // Assemble the 64 bit value.
    int64_t addr =
        static_cast<int64_t>(((uint64_t)((instr0 >> 12 & kImm19_0Mask)) << 44) |
                             ((uint64_t)((instr1 >> 20 & kImm11_0Mask)) << 32) |
                             ((uint64_t)(instr3 >> 20 & kImm11_0Mask) << 20) |
                             ((uint64_t)(instr5 >> 20 & kImm11_0Mask) << 8) |
                             ((uint64_t)(instr7 >> 20 & kImm8_0Mask)));
    // Sign extend to get canonical address.
    addr = (addr << 16) >> 16;
    return static_cast<Address>(addr);
  }
  // We should never get here, force a bad address if we do.
  UNREACHABLE();
}

// On RISCV64, a target address is stored in a 8-instruction sequence:
//    0: lui(rd, (j.imm64_ >> 44) & kImm20Mask);
//    1: ori(rd, rd, (j.imm64_ >> 32) & kImm12Mask);
//    2: dsll(rd, rd, 12);
//    3: ori(rd, rd, (j.imm64_ >> 20) & kImm12Mask);
//    4: dsll(rd, rd, 12);
//    5: ori(rd, rd, (j.imm64_ >> 8) & kImm12Mask);
//    6: dsll(rd, rd, 12);
//    7: ori(rd, rd, j.imm64_ & kImm8Mask);
// Patching the address must replace all the lui & ori instructions,
// and flush the i-cache.
//
// There is an optimization below, which emits a nop when the address
// fits in just 12 bits. This is unlikely to help, and should be benchmarked,
// and possibly removed.
void Assembler::set_target_value_at(Address pc, uint64_t target,
                                    ICacheFlushMode icache_flush_mode) {
  // There is an optimization where only 4 instructions are used to load address
  // in code on MIP64 because only 48-bits of address is effectively used.
  // It relies on fact the upper [63:48] bits are not used for virtual address
  // translation and they have to be set according to value of bit 47 in order
  // get canonical address.

  // const int kRdShift = 7;
  // const int kRs1Shift = 15;
  // const int kRs2Shift = 20;
  // const int kRs3Shift = 27;
  Instr instr1 = instr_at(pc + kInstrSize);
  // ori rd, immediate    //GetRd
  uint32_t rd_code = GetRd(instr1);
  uint32_t* p = reinterpret_cast<uint32_t*>(pc);
#ifdef DEBUG
  Instr instr0 = instr_at(pc);
  Instr instr3 = instr_at(pc + 3 * kInstrSize);
  Instr instr5 = instr_at(pc + 5 * kInstrSize);
  Instr instr7 = instr_at(pc + 7 * kInstrSize);

  // Check we have the result from a li macro-instruction.
  DCHECK(((instr0 & 0x7f) == LUI) &&
         (((instr1 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
         (((instr3 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
         (((instr5 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI) &&
         (((instr7 >> 12 & 0x7) == ORI_F3) && (instr1 & 0x7f) == ORI));
#endif
  // Must use 8 instructions to insure patchable code.
  // lui rd, imm(63_44).
  // ori rd, rd, imm(43_32).
  // slli rd, rd, 12.
  // ori rd rd, imm(31, 20).
  // slli rd, rd, 12.
  // ori rd rd, imm(19, 8).
  // slli rd, rd, 12.
  // ori rd rd, imm(7, 0).

  // lui
  *p = ((((target >> 44) & kImm19_0Mask) << kImmUShift) & kImm31_12Mask) |
       (rd_code << kRdShift) | LUI;
  // ori
  *(p + 1) =
      (((target >> 32) & kImm11_0Mask) << kImmIShift) | (rd_code << kRs1Shift) |
      ((ORI_F3 << kFunct3Shift) & kImm14_12Mask) | (rd_code << kRdShift) | ORI;
  // ori
  *(p + 1) =
      (((target >> 20) & kImm11_0Mask) << kImmIShift) | (rd_code << kRs1Shift) |
      ((ORI_F3 << kFunct3Shift) & kImm14_12Mask) | (rd_code << kRdShift) | ORI;
  // ori
  *(p + 1) =
      (((target >> 8) & kImm11_0Mask) << kImmIShift) | (rd_code << kRs1Shift) |
      ((ORI_F3 << kFunct3Shift) & kImm14_12Mask) | (rd_code << kRdShift) | ORI;
  // ori
  *(p + 1) = ((target & kImm8_0Mask) << kImmIShift) | (rd_code << kRs1Shift) |
             ((ORI_F3 << kFunct3Shift) & kImm14_12Mask) |
             (rd_code << kRdShift) | ORI;

  if (icache_flush_mode != SKIP_ICACHE_FLUSH) {
    FlushInstructionCache(pc, 8 * kInstrSize);
  }
}

void Assembler::AllocateAndInstallRequestedHeapObjects(Isolate* isolate) {
  DCHECK_IMPLIES(isolate == nullptr, heap_object_requests_.empty());
  for (auto& request : heap_object_requests_) {
    Handle<HeapObject> object;
    switch (request.kind()) {
      case HeapObjectRequest::kHeapNumber:
        object = isolate->factory()->NewHeapNumber<AllocationType::kOld>(
            request.heap_number());
        break;
      case HeapObjectRequest::kStringConstant:
        const StringConstantBase* str = request.string();
        CHECK_NOT_NULL(str);
        object = str->AllocateStringConstant(isolate);
        break;
    }
    Address pc = reinterpret_cast<Address>(buffer_start_) + request.offset();
    set_target_value_at(pc, reinterpret_cast<uint64_t>(object.location()));
  }
}

Assembler::Assembler(const AssemblerOptions& options,
                     std::unique_ptr<AssemblerBuffer> buffer)
    : AssemblerBase(options, std::move(buffer)),
      scratch_register_list_(0x30000000) {
  // Register x28/x29(t3/t4) for UseScratchRegisterScope::Acquire()
  trampoline_emitted_ = FLAG_force_long_branches;
  reloc_info_writer.Reposition(buffer_start_ + buffer_->size(), pc_);
  unbound_labels_count_ = 0;
}
void Assembler::GetCode(Isolate* isolate, CodeDesc* desc,
                        SafepointTableBuilder* safepoint_table_builder,
                        int handler_table_offset) {
  int code_comments_size = WriteCodeComments();

  DCHECK(pc_ <= reloc_info_writer.pos());  // No overlap.

  AllocateAndInstallRequestedHeapObjects(isolate);

  // Set up code descriptor.
  // TODO(jgruber): Reconsider how these offsets and sizes are maintained up to
  // this point to make CodeDesc initialization less fiddly.

  static constexpr int kConstantPoolSize = 0;
  const int instruction_size = pc_offset();
  const int code_comments_offset = instruction_size - code_comments_size;
  const int constant_pool_offset = code_comments_offset - kConstantPoolSize;
  const int handler_table_offset2 = (handler_table_offset == kNoHandlerTable)
                                        ? constant_pool_offset
                                        : handler_table_offset;
  const int safepoint_table_offset =
      (safepoint_table_builder == kNoSafepointTable)
          ? handler_table_offset2
          : safepoint_table_builder->GetCodeOffset();
  const int reloc_info_offset =
      static_cast<int>(reloc_info_writer.pos() - buffer_->start());
  CodeDesc::Initialize(desc, this, safepoint_table_offset,
                       handler_table_offset2, constant_pool_offset,
                       code_comments_offset, reloc_info_offset);
}

void Assembler::GrowBuffer() {
  // Compute new buffer size.
  int old_size = buffer_->size();
  int new_size = std::min(2 * old_size, old_size + 1 * MB);

  // Some internal data structures overflow for very large buffers,
  // they must ensure that kMaximalBufferSize is not too large.
  if (new_size > kMaximalBufferSize) {
    V8::FatalProcessOutOfMemory(nullptr, "Assembler::GrowBuffer");
  }

  // Set up new buffer.
  std::unique_ptr<AssemblerBuffer> new_buffer = buffer_->Grow(new_size);
  DCHECK_EQ(new_size, new_buffer->size());
  byte* new_start = new_buffer->start();

  // Copy the data.
  intptr_t pc_delta = new_start - buffer_start_;
  intptr_t rc_delta = (new_start + new_size) - (buffer_start_ + old_size);
  size_t reloc_size = (buffer_start_ + old_size) - reloc_info_writer.pos();
  memmove(new_start, buffer_start_, pc_offset());
  memmove(reloc_info_writer.pos() + rc_delta, reloc_info_writer.pos(),
          reloc_size);

  // Switch buffers.
  buffer_ = std::move(new_buffer);
  buffer_start_ = new_start;
  pc_ += pc_delta;
  reloc_info_writer.Reposition(reloc_info_writer.pos() + rc_delta,
                               reloc_info_writer.last_pc() + pc_delta);

  // None of our relocation types are pc relative pointing outside the code
  // buffer nor pc absolute pointing inside the code buffer, so there is no need
  // to relocate any emitted relocation entries.

  // Relocate internal references.
  for (auto pos : internal_reference_positions_) {
    Address address = reinterpret_cast<intptr_t>(buffer_start_) + pos;
    intptr_t internal_ref = ReadUnalignedValue<intptr_t>(address);
    internal_ref += pc_delta;
    WriteUnalignedValue<intptr_t>(address, internal_ref);
  }

  // Pending relocation entries are also relative, no need to relocate.
}

// need to recheck
const int kEndOfChain = -2;
// Determines the end of the Jump chain (a subset of the label link chain).
const int kEndOfJumpChain = 0;

bool Assembler::IsJal(Instr instr) { return GetOpcodeField(instr) == JAL; }

bool Assembler::IsJalr(Instr instr) { return GetOpcodeField(instr) == JALR; }

bool Assembler::IsLui(Instr instr) {
  uint32_t opcode = GetOpcodeField(instr);
  // Checks if the instruction is a load upper immediate.
  return opcode == LUI;
}

bool Assembler::IsOri(Instr instr) {
  uint32_t opcode = GetOpcodeField(instr);
  uint32_t funct3 = GetFunct3Filed(instr);
  // Checks if the instruction is a load upper immediate.
  return opcode == ORI && funct3 == ORI_F3;
}

bool Assembler::IsMv(Instr instr, Register rd, Register rs1) {
  uint32_t opcode = GetOpcodeField(instr);
  uint32_t rd_field = GetRd(instr);
  uint32_t rs1_field = GetRs1(instr);
  uint32_t imm = GetItype_imm(instr);
  uint32_t rd_reg = static_cast<uint32_t>(rd.code());
  uint32_t rs1_reg = static_cast<uint32_t>(rs1.code());
  // Checks if the instruction is a ADDI with 0 argument (aka mv).
  bool res =
      opcode == ADDI && rd_field == rd_reg && rs1_field == rs1_reg && imm == 0;
  return res;
}

bool Assembler::IsEmittedConstant(Instr instr) {
  uint32_t label_constant = GetLabelConst(instr);
  return label_constant == 0;  // Emitted label const in reg-exp engine.
}
void Assembler::CodeTargetAlign() { Align(4); }

static inline int32_t AddBranchOffset(int pos, Instr instr) {
  // int bits = OffsetSizeInBits(instr);
  int32_t imm = ((instr & kImm31Mask) >> 20) |
                ((instr & kBInstr30_25Mask) >> 21) |
                (instr & kBInstr11_8Mask) >> 8 | ((instr & kBInstr7Mask) << 3);
  int32_t offset = imm << 1;
  if (offset == kEndOfJumpChain) {
    // EndOfChain sentinel is returned directly, not relative to pc or pos.
    return kEndOfChain;
  } else {
    return pos + Assembler::kBranchPCOffset + offset;
  }
}


int Assembler::target_at(int pos, bool is_internal) {
  if (is_internal) {
    int64_t* p = reinterpret_cast<int64_t*>(buffer_start_ + pos);
    int64_t address = *p;
    if (address == kEndOfJumpChain) {
      return kEndOfChain;
    } else {
      int64_t instr_address = reinterpret_cast<int64_t>(p);
      DCHECK(instr_address - address < INT_MAX);
      int delta = static_cast<int>(instr_address - address);
      DCHECK(pos > delta);
      return pos - delta;
    }
  }
  Instr instr = instr_at(pos);
  // if ((instr & ~kImm16Mask) == 0) {
  //   // Emitted label constant, not part of a branch.
  //   if (instr == 0) {
  //     return kEndOfChain;
  //   } else {
  //     int32_t imm18 = ((instr & static_cast<int32_t>(kImm16Mask)) << 16) >>
  //     14; return (imm18 + pos);
  //   }
  // }
  // Check we have a branch or jump instruction.
  // DCHECK(IsBranch(instr) || IsJal(instr) || IsLui(instr));
  if (IsBranch(instr)) {
    return AddBranchOffset(pos, instr);
  } else if(IsAUIPC(instr)) {
    Instr instr_auipc = instr_at(pos);
    Instr instr_jalr = instr_at(pos + 4);
    int32_t imm_auipc = instr_auipc & (kImm19_0Mask << kImm12Shift);
    int32_t imm_jalr = instr_jalr & (kImm11_0Mask << kImm20Shift);
    imm_jalr = imm_jalr >> kImm20Shift;
    int32_t offset = imm_jalr + imm_auipc;
    if(offset  == kEndOfJumpChain) 
      return kEndOfChain;
    return offset + pos;
  } else {
    return kEndOfChain;
  }
  // else if (IsMv(instr, x31, x1)) {
  //   UNIMPLEMENTED();
  //   // int32_t imm32;
  //   // Instr instr_lui = instr_at(pos + 2 * kInstrSize);
  //   // Instr instr_ori = instr_at(pos + 3 * kInstrSize);
  //   // DCHECK(IsLui(instr_lui));
  //   // DCHECK(IsOri(instr_ori));
  //   // imm32 = (instr_lui & static_cast<int32_t>(kImm16Mask)) << kLuiShift;
  //   // imm32 |= (instr_ori & static_cast<int32_t>(kImm16Mask));
  //   // if (imm32 == kEndOfJumpChain) {
  //   //   // EndOfChain sentinel is returned directly, not relative to pc or
  //   pos.
  //   //   return kEndOfChain;
  //   // }
  //   // return pos + Assembler::kLongBranchPCOffset + imm32;
  // } else if (IsLui(instr)) {
  //   UNIMPLEMENTED();
  //   // if (IsNal(instr_at(pos + kInstrSize))) {
  //   //   int32_t imm32;
  //   //   Instr instr_lui = instr_at(pos + 0 * kInstrSize);
  //   //   Instr instr_ori = instr_at(pos + 2 * kInstrSize);
  //   //   DCHECK(IsLui(instr_lui));
  //   //   DCHECK(IsOri(instr_ori));
  //   //   imm32 = (instr_lui & static_cast<int32_t>(kImm16Mask)) << kLuiShift;
  //   //   imm32 |= (instr_ori & static_cast<int32_t>(kImm16Mask));
  //   //   if (imm32 == kEndOfJumpChain) {
  //   //     // EndOfChain sentinel is returned directly, not relative to pc or
  //   pos.
  //   //     return kEndOfChain;
  //   //   }
  //   //   return pos + Assembler::kLongBranchPCOffset + imm32;
  //   // } else {
  //   //   Instr instr_lui = instr_at(pos + 0 * kInstrSize);
  //   //   Instr instr_ori = instr_at(pos + 1 * kInstrSize);
  //   //   Instr instr_ori2 = instr_at(pos + 3 * kInstrSize);
  //   //   DCHECK(IsOri(instr_ori));
  //   //   DCHECK(IsOri(instr_ori2));

  //   //   // TODO(plind) create named constants for shift values.
  //   //   int64_t imm = static_cast<int64_t>(instr_lui & kImm16Mask) << 48;
  //   //   imm |= static_cast<int64_t>(instr_ori & kImm16Mask) << 32;
  //   //   imm |= static_cast<int64_t>(instr_ori2 & kImm16Mask) << 16;
  //   //   // Sign extend address;
  //   //   imm >>= 16;

  //   //   if (imm == kEndOfJumpChain) {
  //   //     // EndOfChain sentinel is returned directly, not relative to pc or
  //   pos.
  //   //     return kEndOfChain;
  //   //   } else {
  //   //     uint64_t instr_address = reinterpret_cast<int64_t>(buffer_start_ +
  //   pos);
  //   //     DCHECK(instr_address - imm < INT_MAX);
  //   //     int delta = static_cast<int>(instr_address - imm);
  //   //     DCHECK(pos > delta);
  //   //     return pos - delta;
  //   //   }
  //   // }
  // } else {
  //   DCHECK(IsJal(instr));
  //   UNIMPLEMENTED();
  //   // int32_t imm28 = (instr & static_cast<int32_t>(kImm26Mask)) << 2;
  //   // if (imm28 == kEndOfJumpChain) {
  //   //   // EndOfChain sentinel is returned directly, not relative to pc or
  //   pos.
  //   //   return kEndOfChain;
  //   // } else {
  //   //   // Sign extend 28-bit offset.
  //   //   int32_t delta = static_cast<int32_t>((imm28 << 4) >> 4);
  //   //   return pos + delta;
  //   // }
  // }
}

static Assembler::OffsetSize OffsetSizeInBits(Instr instr) {
  return Assembler::OffsetSize::kOffset12;
}

static inline Instr SetBranchOffset(int32_t pos, int32_t target_pos,
                                    Instr instr) {
  int32_t bits = OffsetSizeInBits(instr);
  int16_t offset = target_pos - pos;
  // DCHECK_EQ(offset & 1, 0);
  int16_t imm = offset >> 1;  // imm = offset >> 1
  // instr &= ~(kImm31Mask | kBInstr30_25Mask | kBInstr11_8Mask | kBInstr7Mask);
  instr &= 0x1FFF07F;
  // DCHECK(is_intn(imm, bits));
  int32_t temp = ((imm & kImm11Mask) >> 11) << kImmB1Shift |
                 ((imm & kImm9_4Mask) >> 4) << kImmB2Shift |
                 (imm & kImm3_0Mask) << kImmB3Shift |
                 ((imm & kImm10Mask) >> 10) << kImmB4Shift;

  return instr | temp;
}

void Assembler::target_at_put(int pos, int target_pos, bool is_internal) {
  if (is_internal) {
    uint64_t imm = reinterpret_cast<uint64_t>(buffer_start_) + target_pos;
    *reinterpret_cast<uint64_t*>(buffer_start_ + pos) = imm;
    return;
  }
  Instr instr = instr_at(pos);
  // if ((instr & ~kImm16Mask) == 0) {
  //   DCHECK(target_pos == kEndOfChain || target_pos >= 0);
  //   // Emitted label constant, not part of a branch.
  //   // Make label relative to Code pointer of generated Code object.
  //   instr_at_put(pos, target_pos + (Code::kHeaderSize - kHeapObjectTag));
  //   return;
  // }

  if (IsBranch(instr)) {
    instr = SetBranchOffset(pos, target_pos, instr);
    instr_at_put(pos, instr);
  } else if (IsAUIPC(instr)) {
    Instr instr_auipc = instr;
    Instr instr_jalr = instr_at(pos + 4);
    int64_t offset = target_pos - pos;
    DCHECK(is_int32(offset));
    int32_t auipc_arg, jalr_arg;
    GetBranchlongArg((int32_t)offset, auipc_arg, jalr_arg);
    instr_auipc = (instr_auipc & ~kImm31_12Mask) | ((auipc_arg & kImm19_0Mask) << kImm12Shift);
    instr_at_put(pos, instr_auipc);
    instr_jalr = (instr_jalr & ~kImm31_20Mask) | ((jalr_arg & kImm11_0Mask) << kImmIShift);
    instr_at_put(pos + 4, instr_jalr);
  } else {
    std::stringstream ss;
    ss << "target_at_put pos: " << pos << " target_pos: " << target_pos << std::endl;
    RecordComment(ss.str().c_str());
  }
  // else if (IsLui(instr)) {
  //   UNIMPLEMENTED();
  // //   if (IsNal(instr_at(pos + kInstrSize))) {
  // //     Instr instr_lui = instr_at(pos + 0 * kInstrSize);
  // //     Instr instr_ori = instr_at(pos + 2 * kInstrSize);
  // //     DCHECK(IsLui(instr_lui));
  // //     DCHECK(IsOri(instr_ori));
  // //     int32_t imm = target_pos - (pos + Assembler::kLongBranchPCOffset);
  // //     DCHECK_EQ(imm & 3, 0);
  // //     if (is_int16(imm + Assembler::kLongBranchPCOffset -
  // //                  Assembler::kBranchPCOffset)) {
  // //       // Optimize by converting to regular branch and link with 16-bit
  // //       // offset.
  // //       Instr instr_b = REGIMM | BGEZAL;  // Branch and link.
  // //       instr_b = SetBranchOffset(pos, target_pos, instr_b);
  // //       // Correct ra register to point to one instruction after jalr from
  // //       // TurboAssembler::BranchAndLinkLong.
  // //       Instr instr_a = DADDIU | ra.code() << kRsShift | ra.code() <<
  // //       kRtShift |
  // //                       kOptimizedBranchAndLinkLongReturnOffset;

  // //       instr_at_put(pos, instr_b);
  // //       instr_at_put(pos + 1 * kInstrSize, instr_a);
  // //     } else {
  // //       instr_lui &= ~kImm16Mask;
  // //       instr_ori &= ~kImm16Mask;

  // //       instr_at_put(pos + 0 * kInstrSize,
  // //                    instr_lui | ((imm >> kLuiShift) & kImm16Mask));
  // //       instr_at_put(pos + 2 * kInstrSize, instr_ori | (imm &
  // kImm16Mask));
  // //     }
  // //   } else {
  // //     Instr instr_lui = instr_at(pos + 0 * kInstrSize);
  // //     Instr instr_ori = instr_at(pos + 1 * kInstrSize);
  // //     Instr instr_ori2 = instr_at(pos + 3 * kInstrSize);
  // //     DCHECK(IsOri(instr_ori));
  // //     DCHECK(IsOri(instr_ori2));

  // //     uint64_t imm = reinterpret_cast<uint64_t>(buffer_start_) +
  // target_pos;
  // //     DCHECK_EQ(imm & 3, 0);

  // //     instr_lui &= ~kImm16Mask;
  // //     instr_ori &= ~kImm16Mask;
  // //     instr_ori2 &= ~kImm16Mask;

  // //     instr_at_put(pos + 0 * kInstrSize,
  // //                  instr_lui | ((imm >> 32) & kImm16Mask));
  // //     instr_at_put(pos + 1 * kInstrSize,
  // //                  instr_ori | ((imm >> 16) & kImm16Mask));
  // //     instr_at_put(pos + 3 * kInstrSize, instr_ori2 | (imm & kImm16Mask));
  // //   }
  // } else if (IsMv(instr, x31, ra)) {
  //   UNIMPLEMENTED();
  //   // Instr instr_lui = instr_at(pos + 2 * kInstrSize);
  //   // Instr instr_ori = instr_at(pos + 3 * kInstrSize);
  //   // DCHECK(IsLui(instr_lui));
  //   // DCHECK(IsOri(instr_ori));

  //   // int32_t imm_short = target_pos - (pos + Assembler::kBranchPCOffset);

  //   // if (is_int16(imm_short)) {
  //   //   // Optimize by converting to regular branch with 16-bit
  //   //   // offset
  //   //   Instr instr_b = BEQ;
  //   //   instr_b = SetBranchOffset(pos, target_pos, instr_b);

  //   //   Instr instr_j = instr_at(pos + 5 * kInstrSize);
  //   //   Instr instr_branch_delay;

  //   //   if (IsJump(instr_j)) {
  //   //     instr_branch_delay = instr_at(pos + 6 * kInstrSize);
  //   //   } else {
  //   //     instr_branch_delay = instr_at(pos + 7 * kInstrSize);
  //   //   }
  //   //   instr_at_put(pos, instr_b);
  //   //   instr_at_put(pos + 1 * kInstrSize, instr_branch_delay);
  //   // } else {
  //   //   int32_t imm = target_pos - (pos + Assembler::kLongBranchPCOffset);
  //   //   DCHECK_EQ(imm & 3, 0);

  //   //   instr_lui &= ~kImm16Mask;
  //   //   instr_ori &= ~kImm16Mask;

  //   //   instr_at_put(pos + 2 * kInstrSize,
  //   //                instr_lui | ((imm >> kLuiShift) & kImm16Mask));
  //   //   instr_at_put(pos + 3 * kInstrSize, instr_ori | (imm & kImm16Mask));
  //   // }
  // } else if (IsJal(instr)) {
  //   UNIMPLEMENTED();
  //   // int32_t imm28 = target_pos - pos;
  //   // DCHECK_EQ(imm28 & 3, 0);

  //   // uint32_t imm26 = static_cast<uint32_t>(imm28 >> 2);
  //   // DCHECK(is_uint26(imm26));
  //   // // Place 26-bit signed offset with markings.
  //   // // When code is committed it will be resolved to j/jal.
  //   // int32_t mark = IsJ(instr) ? kJRawMark : kJalRawMark;
  //   // instr_at_put(pos, mark | (imm26 & kImm26Mask));
  // } else {
  //   UNIMPLEMENTED();
  //   // int32_t imm28 = target_pos - pos;
  //   // DCHECK_EQ(imm28 & 3, 0);

  //   // uint32_t imm26 = static_cast<uint32_t>(imm28 >> 2);
  //   // DCHECK(is_uint26(imm26));
  //   // // Place raw 26-bit signed offset.
  //   // // When code is committed it will be resolved to j/jal.
  //   // instr &= ~kImm26Mask;
  //   // instr_at_put(pos, instr | (imm26 & kImm26Mask));
  // }
}

void Assembler::bind_to(Label* L, int pos) {
  DCHECK(0 <= pos && pos <= pc_offset());  // Must have valid binding position.
  int trampoline_pos = kInvalidSlotPos;
  bool is_internal = false;
  if (L->is_linked() && !trampoline_emitted_) {
    unbound_labels_count_--;
    if (!is_internal_reference(L)) {
      next_buffer_check_ += kTrampolineSlotsSize;
    }
  }

  while (L->is_linked()) {
    int fixup_pos = L->pos();
    int dist = pos - fixup_pos;  // pos = 264  fixup_pos = 0
    is_internal = is_internal_reference(L);
    next(L, is_internal);  // Call next before overwriting link with
                           // target at fixup_pos.
    Instr instr = instr_at(fixup_pos);
    if (is_internal) {
      target_at_put(fixup_pos, pos, is_internal);
    } else {
      if (IsBranch(instr)) {
        int branch_offset = BranchOffset(instr);
        if (dist > branch_offset) {
          if (trampoline_pos == kInvalidSlotPos) {
            trampoline_pos = get_trampoline_entry(fixup_pos);
            // CHECK_NE(trampoline_pos, kInvalidSlotPos);
          }
          CHECK((trampoline_pos - fixup_pos) <= branch_offset);
          target_at_put(fixup_pos, trampoline_pos, false);
          fixup_pos = trampoline_pos;
        }
        target_at_put(fixup_pos, pos, false);
      } else {
        // DCHECK(IsJal(instr) || IsLui(instr) ||
        //        IsEmittedConstant(instr) || IsMv(instr, x31, x1));
        target_at_put(fixup_pos, pos, false);
      }
    }
  }
  L->bind_to(pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pos > last_bound_pos_) last_bound_pos_ = pos;
}

void Assembler::bind(Label* L) {
  DCHECK(!L->is_bound());  // Label can only be bound once.
  bind_to(L, pc_offset());
}

void Assembler::nop() {
  // emit the binary of addi x0, x0, 0
  // std::cout<<"ASSEM: nop "<<std::endl;
  addi(x0, x0, 0);
}

void Assembler::Align(int m) {
  DCHECK(m >= 4 && base::bits::IsPowerOfTwo(m));
  // qj: when KInstrSize is 2 for C-ext, this CHECK takes effect
  DCHECK_EQ(pc_offset() & (kInstrSize - 1), 0);
  while ((pc_offset() & (m - 1)) != 0) {
    nop();
  }
}
void Assembler::db(uint8_t data) { dc8(data); }

void Assembler::dd(uint32_t data) { dc32(data); }

void Assembler::dq(uint64_t data) { dc64(data); }

void Assembler::dd(Label* label) {
  // uint64_t data;
  // CheckForEmitInForbiddenSlot();
  // if (label->is_bound()) {
  //   data = reinterpret_cast<uint64_t>(buffer_start_ + label->pos());
  // } else {
  //   data = jump_address(label);
  //   unbound_labels_count_++;
  //   internal_reference_positions_.insert(label->pos());
  // }
  // RecordRelocInfo(RelocInfo::INTERNAL_REFERENCE);
  // EmitHelper(data);
}

void Assembler::next(Label* L, bool is_internal) {
  DCHECK(L->is_linked());
  int link = target_at(L->pos(), is_internal);
  if (link == kEndOfChain) {
    L->Unuse();
  } else {
    DCHECK_GE(link, 0);
    L->link_to(link);
  }
}

bool Assembler::is_near(Label* L) {
  DCHECK(L->is_bound());
  return pc_offset() - L->pos() < kMaxBranchOffset - 4 * kInstrSize;
}

bool Assembler::is_near(Label* L, OffsetSize bits) {
  if (L == nullptr || !L->is_bound()) return true;
  return ((pc_offset() - L->pos()) < (1 << (bits - 1)) - 1 - 5 * kInstrSize);
}

bool Assembler::is_near_branch(Label* L) {
  DCHECK(L->is_bound());
  return is_near_b(L);
}

int Assembler::BranchOffset(Instr instr) {
  int bits = OffsetSize::kOffset12;
  return (1 << (bits + 2 - 1)) - 1;
}

void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  if (!ShouldRecordRelocInfo(rmode)) return;
  // We do not try to reuse pool constants.
  RelocInfo rinfo(reinterpret_cast<Address>(pc_), rmode, data, Code());
  DCHECK_GE(buffer_space(), kMaxRelocSize);  // Too late to grow buffer here.
  reloc_info_writer.Write(&rinfo);
}
// We have to use a temporary register for things that can be relocated even
// if they can be encoded in the MIPS's 16 bits of immediate-offset instruction
// space.  There is no guarantee that the relocated location can be similarly
// encoded.
bool Assembler::MustUseReg(RelocInfo::Mode rmode) {
  return !RelocInfo::IsNone(rmode);
}

uint32_t Assembler::GetOpcodeField(Instr instr) { return instr & kOpcodeMask; }
uint32_t Assembler::GetFunct3Filed(Instr instr) {
  return (instr & kImm14_12Mask) >> kImm12Shift;
}

uint32_t Assembler::GetRd(Instr instr) {
  return (instr & kRdFieldMask) >> kRdShift;
}
uint32_t Assembler::GetRs1(Instr instr) {
  return (instr & kRs1FieldMask) >> kRs1Shift;
}

uint32_t Assembler::GetItype_imm(Instr instr) {
  return (instr & kImm31_25Mask) >> kImm25Shift;
}

uint32_t Assembler::GetLabelConst(Instr instr) {
  return instr & ~kImm31_12Mask;
}

bool Assembler::IsBranch(Instr instr) {
  uint32_t opcode = GetOpcodeField(instr);
  // uint32_t rt_field = GetRtField(instr);
  // uint32_t rs_field = GetRsField(instr);
  // Checks if the instruction is a branch.
  bool isBranch = opcode == BEQ || opcode == BNE || opcode == BLT ||
                  opcode == BGE || opcode == BLTU || opcode == BGEU;
  // || opcode == BLEZL || opcode == BGTZL ||
  // (opcode == REGIMM && (rt_field == BLTZ || rt_field == BGEZ ||
  //                       rt_field == BLTZAL || rt_field == BGEZAL)) ||
  // (opcode == COP1 && rs_field == BC1) ||  // Coprocessor branch.
  // (opcode == COP1 && rs_field == BC1EQZ) ||
  // (opcode == COP1 && rs_field == BC1NEZ) || IsMsaBranch(instr);
  return isBranch;
}

bool Assembler::IsAUIPC(Instr instr) {
  uint32_t opcode = GetOpcodeField(instr);
  return opcode == AUIPC;
}

void Assembler::CheckTrampolinePool() {
  // Some small sequences of instructions must not be broken up by the
  // insertion of a trampoline pool; such sequences are protected by setting
  // either trampoline_pool_blocked_nesting_ or no_trampoline_pool_before_,
  // which are both checked here. Also, recursive calls to CheckTrampolinePool
  // are blocked by trampoline_pool_blocked_nesting_.
  if ((trampoline_pool_blocked_nesting_ > 0) ||
      (pc_offset() < no_trampoline_pool_before_)) {
    // Emission is currently blocked; make sure we try again as soon as
    // possible.
    if (trampoline_pool_blocked_nesting_ > 0) {
      next_buffer_check_ = pc_offset() + kInstrSize;
    } else {
      next_buffer_check_ = no_trampoline_pool_before_;
    }
    return;
  }

  DCHECK(!trampoline_emitted_);
  DCHECK_GE(unbound_labels_count_, 0);
  if (unbound_labels_count_ > 0) {
    // First we emit jump (2 instructions), then we emit trampoline pool.
    {
      BlockTrampolinePoolScope block_trampoline_pool(this);
      Label after_pool;
      b(&after_pool);
      nop();

      int pool_start = pc_offset();
      for (int i = 0; i < unbound_labels_count_; i++) {
        or_(t5, ra, zero_reg);
        auipc(ra, 0);    // Read PC into ra register.
        addi(t6, ra, 0);
        or_(ra, t5, zero_reg);
        // Instruction jr will take or_ from the next trampoline.
        // in its branch delay slot. This is the expected behavior
        // in order to decrease size of trampoline pool.
        jr(t6);
      }
      nop();
      bind(&after_pool);
      trampoline_ = Trampoline(pool_start, unbound_labels_count_);

      trampoline_emitted_ = true;
      // As we are only going to emit trampoline once, we need to prevent any
      // further emission.
      next_buffer_check_ = kMaxInt;
    }
  } else {
    // Number of branches to unbound label at this point is zero, so we can
    // move next buffer check to maximum.
    next_buffer_check_ =
        pc_offset() + kMaxBranchOffset - kTrampolineSlotsSize * 16;
  }
  return;
}

int32_t Assembler::branch_offset_helper(Label* L, OffsetSize bits) {
  int32_t target_pos;

  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();
      L->link_to(pc_offset());
    } else {
      L->link_to(pc_offset());
      if (!trampoline_emitted_) {
        unbound_labels_count_++;
        next_buffer_check_ -= kTrampolineSlotsSize;
      }
      return kEndOfJumpChain;
    }
  }

  int32_t offset = target_pos - pc_offset();
  DCHECK(is_intn(offset, bits));

  return offset;
}

uint64_t Assembler::branch_long_offset(Label* L) {
  int64_t target_pos;

  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link.
      L->link_to(pc_offset());
    } else {
      L->link_to(pc_offset());
      return kEndOfJumpChain;
    }
  }
  int64_t offset = target_pos - pc_offset();
  return static_cast<uint64_t>(offset);
}

// Returns the next free trampoline entry.
int32_t Assembler::get_trampoline_entry(int32_t pos) {
  int32_t trampoline_entry = kInvalidSlotPos;
  if (!internal_trampoline_exception_) {
    if (trampoline_.start() > pos) {
      trampoline_entry = trampoline_.take_slot();
    }

    if (kInvalidSlotPos == trampoline_entry) {
      internal_trampoline_exception_ = true;
    }
  }
  return trampoline_entry;
}

// R-type rs2 field, i.e., bit[24:20] is a 5-bit register index
// ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR_/AND_
// ADDW/SUBW/SLLW/SRLW/SRAW
// MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU
// MULW/DIVW/DIVUW/REMW/REMUW
// GIR1(GenInstr R-type 1)
void Assembler::GenInstrRType(Funct7 func7, Register rs2, Register rs1,
                              Funct3 func3, Register rd, Opcode opcode) {
  DCHECK(rs2.is_valid() && rs1.is_valid() && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2.code() << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// I-type rs2 filed,
// i.e. bit[24:20] is a imm, can be a 5 bit shamt or fixed value
// SLLI/SRLI/SRAI
// SLLIW/SRLIW/SRAIW
// GII0(GenInstr I-type 0)
// imm is a 5 bit shamt or other
void Assembler::GenInstrIType(Funct7 func7, int32_t imm, Register rs1,
                              Funct3 func3, Register rd, Opcode opcode) {
  int32_t shamt4_0 = imm & kImm4_0Mask;
  DCHECK((is_uint5(shamt4_0) || is_int5(shamt4_0)) && rs1.is_valid() &&
         rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (shamt4_0 << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// RV64I SHIFT IMM has 6 bit shamt
// I-type. bit[25:20] is a 6 bit shamt
// SLLI64/SRLI64/SRAI64
// GISI64(GenInstr Shift I type 64)
void Assembler::GenInstrShiftI64(Funct6 func6, int32_t imm, Register rs1,
                                 Funct3 func3, Register rd, Opcode opcode) {
  int32_t shamt5_0 = imm & kImm5_0Mask;
  DCHECK((is_uint6(shamt5_0) || is_int6(shamt5_0)) && rs1.is_valid() &&
         rd.is_valid());
  Instr instr = (func6 << kFunct6Shift) | (shamt5_0 << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// R-type atomic
// LR.W/SC.W/AMOSWAP.W/AMOADD.W/AMOXOR.W/AMOAND.W/AMOOR.W/AMOMIN.W/AMOMAX.W/AMOMINU.W/AMOMAXU.W
// LR.D/SC.D/AMOSWAP.D/AMOADD.D/AMOXOR.D/AMOAND.D/AMOOR.D/AMOMIN.D/AMOMAX.D/AMOMINU.D/AMOMAXU.D
// GIRA(GenInstr R-type Atomic_Instructions)
void Assembler::GenInstrRA(Funct5 funct5, uint32_t aq, uint32_t rl,
                           Register rs2, Register rs1, Funct3 func3,
                           Register rd, Opcode opcode) {
  DCHECK(is_uint1(aq) && is_uint1(rl) && rs2.is_valid() && rs1.is_valid() &&
         rd.is_valid());
  Instr instr = (funct5 << kR_A27Shift) | (aq << kR_A26Shift) |
                (rl << kR_A25Shift) | (rs2.code() << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// I-type, bit[31:20] is a 12-bit imm
// JALR
// LB/LH/LW/LBU/LHU/ADDI/SLTI/SLTIU/XORI/ORI/ANDI/ECALL/EBREAK
// LWU/LD/ADDIW
// FENCEi
// GII1(GenInstr I-type 1)
void Assembler::GenInstrIType(int32_t imm, Register rs1, Funct3 func3,
                              Register rd, Opcode opcode) {
  DCHECK(rs1.is_valid() && (is_int12(imm) || is_uint12(imm))&& rd.is_valid());
  Instr instr = (imm << kImmIShift) | (rs1.code() << kRs1Shift) |
                (func3 << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// I-type fence
// FENCE
// GIIF(GenInstr I-type Fence_Instructions)
void Assembler::GenInstrIFence(uint32_t fm, uint32_t pred, uint32_t succ,
                               Register rs1, Funct3 func3, Register rd,
                               Opcode opcode) {
  DCHECK(is_uint4(fm) && is_uint4(pred) && is_uint4(succ) && rs1.is_valid() &&
         rd.is_valid());
  Instr instr = (fm << Fence28Shift) | (pred << Fence24Shift) |
                (succ << Fence20Shift) | (rs1.code() << kRs1Shift) |
                (func3 << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// I-type Zicsr
// csrrw/csrrs/csrrc
// GII3(GenInstr I-type Zicsr_Instructions)
void Assembler::GenInstrIType(CSRRegister csr, Register rs1, Funct3 func3,
                              Register rd, Opcode opcode) {
  DCHECK(rs1.is_valid() && rd.is_valid());
  Instr instr = (csr << kFunct7Shift) | (rs1.code() << kRs1Shift) |
                (func3 << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// I-type Zicsr bit[19:15] is a 5-bit zimm
// csrrwi/csrrsi/csrrci
// GII2(GenInstr I-type 2)
void Assembler::GenInstrIType(CSRRegister csr, uint32_t zimm, Funct3 func3,
                              Register rd, Opcode opcode) {
  DCHECK(rd.is_valid() && is_uint5(zimm));
  Instr instr = csr << kImmIShift | ((zimm & kImm4_0Mask) << 15) |
                (func3 << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// S-type, Store
// SB/SH/SW
// SD
// GIS1(GenInstr S-type 1)
void Assembler::GenInstrSType(int32_t offset, Register rs2, Register rs1,
                              Funct3 func3, Opcode opcode) {
  DCHECK(is_int32(offset) && rs2.is_valid() && rs1.is_valid());
  Instr instr = ((offset & kImm11_5Mask) >> 5) << kImmS1Shift |
                (rs2.code() << kRs2Shift) | (rs1.code() << kRs1Shift) |
                ((func3 << kFunct3Shift) & kImm14_12Mask) |
                ((offset & kImm4_0Mask) << kImmS2Shift) | opcode;
  Emit(instr);
}

// B-type, Branch
// BEQ/BNE/BLT/BGE/BLTU/BGEU
// GIB(GenInstr B-type)
void Assembler::GenInstrBType(int16_t imm, Register rs2, Register rs1,
                              Funct3 func3, Opcode opcode) {
  // DCHECK(is_int12(imm)
  DCHECK(rs2.is_valid() && rs1.is_valid());
  Instr instr = ((imm & kImm11Mask) >> 11) << kImmB1Shift |
                ((imm & kImm9_4Mask) >> 4) << kImmB2Shift |
                (rs2.code() << kRs2Shift) | (rs1.code() << kRs1Shift) |
                ((func3 << kFunct3Shift) & kImm14_12Mask) |
                (imm & kImm3_0Mask) << kImmB3Shift |
                ((imm & kImm10Mask) >> 10) << kImmB4Shift | opcode;
  Emit(instr);
}

// U-type, bit[31:12] is a 20-bit imm
// LUI/AUIPC
// GIU(GenInstr U-type)
void Assembler::GenInstrUType(int32_t offset, Register rd, Opcode opcode) {
  DCHECK((is_int20(offset) || is_uint20(offset)) && rd.is_valid());
  Instr instr =
      (offset & kImm19_0Mask) << kImm12Shift | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// J-type JAL Deprecated because PIC jump
// offset is a 20-bit address which has been scaled by 2,
// so the actually layout in jal is:
// [19][9:0][10][18:11]
// JAL
// GIJ(GenInstr J-type)
void Assembler::GenInstrJType(int32_t offset, Register rd, Opcode opcode) {
  DCHECK(is_int21(offset) && rd.is_valid());
  Instr instr = ((offset & kImm19Mask) >> 19) << kImmJ1Shift |
                (offset & kImm9_0Mask) << kImmJ2Shift |
                ((offset & kImm10Mask) >> 10) << kImmJ3Shift |
                ((offset & kImm18_11Mask) >> 11) << kImmJ4Shift |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// RV32I Base Instruction Set
// GIU
void Assembler::lui(Register rd, int32_t i) { GenInstrUType(i, rd, LUI); }

// GIU
void Assembler::auipc(Register rd, int32_t i) { GenInstrUType(i, rd, AUIPC); }

// GIJ
// Do not use jal, refer to 750ce1e7300f4fa3edeb5353b9c74bd9c44db5ad
void Assembler::jal(Register rd, int32_t i) {
  //  Deprecated. Use PC-relative jumps instead.
  UNREACHABLE();
  /*
  DCHECK(is_int20(offset));
  GenInstrJType(i, rd, JAL);
  */
}

// GII1
void Assembler::jalr(Register rd, int32_t offset, Register rs1) {
  GenInstrIType(offset, rs1, JALR_F3, rd, JALR);
}

// GIB
void Assembler::beq(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BEQ_F3, BEQ);
}

// GIB
void Assembler::bne(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BNE_F3, BNE);
}

// GIB
void Assembler::blt(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BLT_F3, BLT);
}

// GIB
void Assembler::bge(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BGE_F3, BGE);
}

// GIB
void Assembler::bltu(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BLTU_F3, BLTU);
}

// GIB
void Assembler::bgeu(Register rs1, Register rs2, int16_t offset) {
  GenInstrBType(offset, rs2, rs1, BGEU_F3, BGEU);
}

// GII1
void Assembler::lb(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LB_F3, rd, LB);
}

// GII1
void Assembler::lh(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LH_F3, rd, LH);
}

// GII1
void Assembler::lw(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LW_F3, rd, LW);
}

// GII1
void Assembler::lbu(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LBU_F3, rd, LBU);
}

// GII1
void Assembler::lhu(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LHU_F3, rd, LHU);
}

// GIS1
void Assembler::sb(Register rs2, const MemOperand& rs1) {
  GenInstrSType(rs1.offset_, rs2, rs1.rm(), SB_F3, SB);
}

// GIS1
void Assembler::sh(Register rs2, const MemOperand& rs1) {
  GenInstrSType(rs1.offset_, rs2, rs1.rm(), SH_F3, SH);
}

// GIS1
void Assembler::sw(Register rs2, const MemOperand& rs1) {
  GenInstrSType(rs1.offset_, rs2, rs1.rm(), SW_F3, SW);
}

// GII1
void Assembler::addi(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, ADDI_F3, rd, ADDI);
}

// GII1
void Assembler::slti(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, SLTI_F3, rd, SLTI);
}

// GII1
void Assembler::sltiu(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, SLTIU_F3, rd, SLTIU);
}

// GII1
void Assembler::xori(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, XORI_F3, rd, XORI);
}

// GII1
void Assembler::ori(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, ORI_F3, rd, ORI);
}

// GII1
void Assembler::andi(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, ANDI_F3, rd, ANDI);
}

// GII0
void Assembler::slli(Register rd, Register rs1, uint32_t shamt) {
  GenInstrIType(SLLI_F7, shamt, rs1, SLLI_F3, rd, SLLI);
}

// GII0
void Assembler::srli(Register rd, Register rs1, uint32_t shamt) {
  GenInstrIType(SRLI_F7, shamt, rs1, SRLI_F3, rd, SRLI);
}

// GII0
void Assembler::srai(Register rd, Register rs1, uint32_t shamt) {
  GenInstrIType(SRAI_F7, shamt, rs1, SRAI_F3, rd, SRAI);
}

// GIR1
void Assembler::add(Register rd, Register rs1, Register rs2) {
  GenInstrRType(ADD_F7, rs2, rs1, ADD_F3, rd, ADD);
}

// GIR1
void Assembler::sub(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SUB_F7, rs2, rs1, SUB_F3, rd, SUB);
}

// GIR1
void Assembler::sll(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SLL_F7, rs2, rs1, SLL_F3, rd, SLL);
}

// GIR1
void Assembler::slt(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SLT_F7, rs2, rs1, SLT_F3, rd, SLT);
}

// GIR1
void Assembler::sltu(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SLTU_F7, rs2, rs1, SLTU_F3, rd, SLTU);
}

// GIR1
void Assembler::xor_(Register rd, Register rs1, Register rs2) {
  GenInstrRType(XOR_F7, rs2, rs1, XOR_F3, rd, XOR);
}

// GIR1
void Assembler::srl(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SRL_F7, rs2, rs1, SRL_F3, rd, SRL);
}

// GIR1
void Assembler::sra(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SRA_F7, rs2, rs1, SRA_F3, rd, SRA);
}

// GIR1
void Assembler::or_(Register rd, Register rs1, Register rs2) {
  GenInstrRType(OR_F7, rs2, rs1, OR_F3, rd, OR);
}

// GIR1
void Assembler::and_(Register rd, Register rs1, Register rs2) {
  GenInstrRType(AND_F7, rs2, rs1, AND_F3, rd, AND);
}

// GIIF
void Assembler::fence(Register rd, Register rs1, uint32_t fm, uint32_t pred,
                      uint32_t succ) {
  GenInstrIFence(fm, pred, succ, rs1, FENCE_F3, rd, FENCE);
}

// GII1
void Assembler::ecall() {
  GenInstrIType(0, zero_reg, ECALL_F3, zero_reg, ECALL);
}

// GII1
void Assembler::ebreak() {
  GenInstrIType(1, zero_reg, EBREAK_F3, zero_reg, EBREAK);
}

// RV64I Base Instruction Set (in addition to RV32I)
// GII1
void Assembler::lwu(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LWU_F3, rd, LWU);
}

// GII1
void Assembler::ld(Register rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), LD_F3, rd, LD);
}

// GIS1
void Assembler::sd(Register rs2, const MemOperand& rs1) {
  GenInstrSType(rs1.offset_, rs2, rs1.rm(), SD_F3, SD);
}

// GISI64
void Assembler::slli64(Register rd, Register rs1, uint32_t shamt) {
  GenInstrShiftI64(SLLI64_F6, shamt, rs1, SLLI64_F3, rd, SLLI64);
}

// GISI64
void Assembler::srli64(Register rd, Register rs1, uint32_t shamt) {
  GenInstrShiftI64(SRLI64_F6, shamt, rs1, SRLI64_F3, rd, SRLI64);
}

// GISI64
void Assembler::srai64(Register rd, Register rs1, uint32_t shamt) {
  GenInstrShiftI64(SRAI64_F6, shamt, rs1, SRAI64_F3, rd, SRAI64);
}

// GII1
void Assembler::addiw(Register rd, Register rs1, int32_t i) {
  DCHECK(rs1 != sp);
  DCHECK(rs1 != fp);
  GenInstrIType(i, rs1, ADDIW_F3, rd, ADDIW);
}

// GII0
void Assembler::slliw(Register rd, Register rs1, int32_t shamt) {
  GenInstrIType(SLLIW_F7, shamt, rs1, SLLIW_F3, rd, SLLIW);
}

// GII0
void Assembler::srliw(Register rd, Register rs1, uint32_t shamt) {
  GenInstrIType(SRLIW_F7, shamt, rs1, SRLIW_F3, rd, SRLIW);
}

// GII0
void Assembler::sraiw(Register rd, Register rs1, uint32_t shamt) {
  GenInstrIType(SRAIW_F7, shamt, rs1, SRAIW_F3, rd, SRAIW);
}

// GIR1
void Assembler::addw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(ADDW_F7, rs2, rs1, ADDW_F3, rd, ADDW);
}

// GIR1
void Assembler::subw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SUBW_F7, rs2, rs1, SUBW_F3, rd, SUBW);
}

// GIR1
void Assembler::sllw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SLLW_F7, rs2, rs1, SLLW_F3, rd, SLLW);
}

// GIR1
void Assembler::srlw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SRLW_F7, rs2, rs1, SRLW_F3, rd, SRLW);
}

// GIR1
void Assembler::sraw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(SRAW_F7, rs2, rs1, SRAW_F3, rd, SRAW);
}

//  RV32/RV64 Zifencei Standard Extension
// GII1
void Assembler::fencei(Register rd, Register rs1, int32_t i) {
  GenInstrIType(i, rs1, FENCEi_F3, rd, FENCEi);
}

//  RV32/RV64 Zicsr Standard Extension
// GII3
void Assembler::csrrw(Register rd, CSRRegister csr, Register rs1) {
  GenInstrIType(csr, rs1, CSRRW_F3, rd, CSRRW);
}

// GII3
void Assembler::csrrs(Register rd, CSRRegister csr, Register rs1) {
  GenInstrIType(csr, rs1, CSRRS_F3, rd, CSRRS);
}

// GII3
void Assembler::csrrc(Register rd, CSRRegister csr, Register rs1) {
  GenInstrIType(csr, rs1, CSRRC_F3, rd, CSRRC);
}

// GII2
void Assembler::csrrwi(Register rd, CSRRegister csr, uint32_t uimm) {
  GenInstrIType(csr, uimm, CSRRWI_F3, rd, CSRRWI);
}

// GII2
void Assembler::csrrsi(Register rd, CSRRegister csr, uint32_t uimm) {
  GenInstrIType(csr, uimm, CSRRSI_F3, rd, CSRRSI);
}

// GII2
void Assembler::csrrci(Register rd, CSRRegister csr, uint32_t uimm) {
  GenInstrIType(csr, uimm, CSRRCI_F3, rd, CSRRCI);
}

// RV32M Standard Extension
// GIR1
void Assembler::mul(Register rd, Register rs1, Register rs2) {
  GenInstrRType(MUL_F7, rs2, rs1, MUL_F3, rd, MUL);
}

// GIR1
void Assembler::mulh(Register rd, Register rs1, Register rs2) {
  GenInstrRType(MULH_F7, rs2, rs1, MULH_F3, rd, MULH);
}

// GIR1
void Assembler::mulhsu(Register rd, Register rs1, Register rs2) {
  GenInstrRType(MULHSU_F7, rs2, rs1, MULHSU_F3, rd, MULHSU);
}

// GIR1
void Assembler::mulhu(Register rd, Register rs1, Register rs2) {
  GenInstrRType(MULHU_F7, rs2, rs1, MULHU_F3, rd, MULHU);
}

// GIR1
void Assembler::div(Register rd, Register rs1, Register rs2) {
  GenInstrRType(DIV_F7, rs2, rs1, DIV_F3, rd, DIV);
}

// GIR1
void Assembler::divu(Register rd, Register rs1, Register rs2) {
  GenInstrRType(DIVU_F7, rs2, rs1, DIVU_F3, rd, DIVU);
}

// GIR1
void Assembler::rem(Register rd, Register rs1, Register rs2) {
  GenInstrRType(REM_F7, rs2, rs1, REM_F3, rd, REM);
}

// GIR1
void Assembler::remu(Register rd, Register rs1, Register rs2) {
  GenInstrRType(REMU_F7, rs2, rs1, REMU_F3, rd, REMU);
}

// RV64M Standard Extension (in addition to RV32M)
// GIR1
void Assembler::mulw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(MULW_F7, rs2, rs1, MULW_F3, rd, MULW);
}

// GIR1
void Assembler::divw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(DIVW_F7, rs2, rs1, DIVW_F3, rd, DIVW);
}

// GIR1
void Assembler::divuw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(DIVUW_F7, rs2, rs1, DIVUW_F3, rd, DIVUW);
}

// GIR1
void Assembler::remw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(REMW_F7, rs2, rs1, REMW_F3, rd, REMW);
}

// GIR1
void Assembler::remuw(Register rd, Register rs1, Register rs2) {
  GenInstrRType(REMUW_F7, rs2, rs1, REMUW_F3, rd, REMUW);
}

// RV32A Standard Extension
// GIRA
void Assembler::lrw(Register rd, Register rs1, uint32_t aq, uint32_t rl) {
  GenInstrRA(LR_F5, aq, rl, zero_reg, rs1, LRW_F3, rd, LRW);
}

// GIRA
void Assembler::scw(Register rd, Register rs1, Register rs2, uint32_t aq,
                    uint32_t rl) {
  GenInstrRA(SC_F5, aq, rl, rs2, rs1, SCW_F3, rd, SCW);
}

// GIRA
void Assembler::amoswapw(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOSWAP_F5, aq, rl, zero_reg, rs1, AMOSWAPW_F3, rd, AMOSWAPW);
}

// GIRA
void Assembler::amoaddw(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOADD_F5, aq, rl, rs2, rs1, AMOADDW_F3, rd, AMOADDW);
}

// GIRA
void Assembler::amoxorw(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOXOR_F5, aq, rl, zero_reg, rs1, AMOXORW_F3, rd, AMOXORW);
}

// GIRA
void Assembler::amoandw(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOAND_F5, aq, rl, rs2, rs1, AMOANDW_F3, rd, AMOANDW);
}

// GIRA
void Assembler::amoorw(Register rd, Register rs1, Register rs2, uint32_t aq,
                       uint32_t rl) {
  GenInstrRA(AMOOR_F5, aq, rl, rs2, rs1, AMOORW_F3, rd, AMOORW);
}

// GIRA
void Assembler::amominw(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOMIN_F5, aq, rl, rs2, rs1, AMOMINW_F3, rd, AMOMINW);
}

// GIRA
void Assembler::amomaxw(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOMAX_F5, aq, rl, zero_reg, rs1, AMOMAXW_F3, rd, AMOMAXW);
}

// GIRA
void Assembler::amominuw(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOMINU_F5, aq, rl, rs2, rs1, AMOMINUW_F3, rd, AMOMINUW);
}

// GIRA
void Assembler::amomaxuw(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOMAXU_F5, aq, rl, zero_reg, rs1, AMOMAXUW_F3, rd, AMOMAXUW);
}
// RV64A Standard Extension (in addition to RV32A)
// GIRA
void Assembler::lrd(Register rd, Register rs1, uint32_t aq, uint32_t rl) {
  GenInstrRA(LR_F5, aq, rl, zero_reg, rs1, LRD_F3, rd, LRD);
}

// GIRA
void Assembler::scd(Register rd, Register rs1, Register rs2, uint32_t aq,
                    uint32_t rl) {
  GenInstrRA(SC_F5, aq, rl, rs2, rs1, SCD_F3, rd, SCD);
}

// GIRA
void Assembler::amoswapd(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOSWAP_F5, aq, rl, zero_reg, rs1, AMOSWAPD_F3, rd, AMOSWAPD);
}

// GIRA
void Assembler::amoaddd(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOADD_F5, aq, rl, rs2, rs1, AMOADDD_F3, rd, AMOADDD);
}

// GIRA
void Assembler::amoxord(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOXOR_F5, aq, rl, zero_reg, rs1, AMOXORD_F3, rd, AMOXORD);
}

// GIRA
void Assembler::amoandd(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOAND_F5, aq, rl, rs2, rs1, AMOANDD_F3, rd, AMOANDD);
}

// GIRA
void Assembler::amoord(Register rd, Register rs1, Register rs2, uint32_t aq,
                       uint32_t rl) {
  GenInstrRA(AMOOR_F5, aq, rl, rs2, rs1, AMOORD_F3, rd, AMOORD);
}

// GIRA
void Assembler::amomind(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOMIN_F5, aq, rl, rs2, rs1, AMOMIND_F3, rd, AMOMIND);
}

// GIRA
void Assembler::amomaxd(Register rd, Register rs1, Register rs2, uint32_t aq,
                        uint32_t rl) {
  GenInstrRA(AMOMAX_F5, aq, rl, zero_reg, rs1, AMOMAXD_F3, rd, AMOMAXD);
}

// GIRA
void Assembler::amominud(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOMINU_F5, aq, rl, rs2, rs1, AMOMINUD_F3, rd, AMOMINUD);
}

// GIRA
void Assembler::amomaxud(Register rd, Register rs1, Register rs2, uint32_t aq,
                         uint32_t rl) {
  GenInstrRA(AMOMAXU_F5, aq, rl, zero_reg, rs1, AMOMAXUD_F3, rd, AMOMAXUD);
}

//  floating-point computational instructions
// R-type bit[24:20] is rs2 & bit[14:12] is Funct3
// FSGNJS/FSGNJNS/FSGNJXS/FMINS/FMAXS
// FSGNJD/FSGNJND/FSGNJXD/FMIND/FMAXD
// GIFPR1(GenInstr FloatPoint R type 1)
void Assembler::GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                               Funct3 func3, FPURegister rd, Opcode opcode) {
  DCHECK(rs2.is_valid() && rs1.is_valid() && rd.is_valid());
  Instr instr = ((func7 << kFunct7Shift) & kImm31_25Mask) |
                (rs2.code() << kRs2Shift) | (rs1.code() << kRs1Shift) |
                ((func3 << kFunct3Shift) & kImm14_12Mask) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is rs2 & bit[14:12] is Funct3
// feqs/flts/fles/feqd/fltd/fled
// GIFPR1_1(GenInstr FloatPoint R type 1)
// rd is Register
void Assembler::GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                               Funct3 func3, Register rd, Opcode opcode) {
  DCHECK(rs2.is_valid() && rs1.is_valid() && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2.code() << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is Funct3
//
// GIFPR2(GenInstr FloatPoint R type 2)
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                               Funct3 func3, FPURegister rd, Opcode opcode) {
  DCHECK(rs1.is_valid() && is_uint5(rs2i) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is Funct3
// rd is Register
// GIFPR2_1(GenInstr FloatPoint R type 2)
// fmvxw/fclasss/fclassd/fmvxd
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                               Funct3 func3, Register rd, Opcode opcode) {
  DCHECK(rs1.is_valid() && is_uint5(rs2i) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is Funct3
// rs1 is Register
// GIFPR2_2(GenInstr FloatPoint R type 2)
// fmvwx/fmvdx
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, Register rs1,
                               Funct3 func3, FPURegister rd, Opcode opcode) {
  DCHECK(rs1.is_valid() && is_uint5(rs2i) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is rm
// fsqrts/fsqrtd/fcvtsd/fcvtds
// GIFPR3(GenInstr FloatPoint R type 3)
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                               uint32_t rm, FPURegister rd, Opcode opcode) {
  DCHECK(is_uint5(rs2i) && rs1.is_valid() && is_uint3(rm) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (rm << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is rm
// rs1 is Register
// fcvtsw/fcvtswu/fcvtsl/fcvtslu/fcvtdw/fcvtdwu/fcvtdl/fcvtdlu
// GIFPR3_1(GenInstr FloatPoint R type 3)
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, Register rs1,
                               uint32_t rm, FPURegister rd, Opcode opcode) {
  DCHECK(is_uint5(rs2i) && rs1.is_valid() && is_uint3(rm) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (rm << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is imm & bit[14:12] is rm
// rd is Register
// fcvtws/fcvtwus/fcvtls/fcvtlus/fcvtwd/fcvtwud/fcvtld/fcvtlud
// GIFPR3_2(GenInstr FloatPoint R type 3)
void Assembler::GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                               uint32_t rm, Register rd, Opcode opcode) {
  DCHECK(is_uint5(rs2i) && rs1.is_valid() && is_uint3(rm) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2i << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (rm << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R-type bit[24:20] is rs2 & bit[14:12] is rm
// FADDS/FSUBS/FMULS/FDIVS
// FADDD/FSUBD/FMULD/FDIVD
// GIFPR4(GenInstr FloatPoint R type 4)
void Assembler::GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                               uint32_t rm, FPURegister rd, Opcode opcode) {
  DCHECK(rs2.is_valid() && rs1.is_valid() && is_uint3(rm) && rd.is_valid());
  Instr instr = (func7 << kFunct7Shift) | (rs2.code() << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (rm << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// I-type
// FLW
// FLD
// GII4(GenInstr I-type 4)
void Assembler::GenInstrIType(int32_t imm, Register rs1, Funct3 func3,
                              FPURegister rd, Opcode opcode) {
  DCHECK(is_int12(imm) && rs1.is_valid() && rd.is_valid());
  Instr instr = ((imm & kImm11_0Mask) << kImmIShift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// S-type
// FSW
// FSD
// GIS2(GenInstr S-type 2)
void Assembler::GenInstrSType(FPURegister rs2, Register rs1, Funct3 func3,
                              int32_t imm, Opcode opcode) {
  uint32_t imm11_5 = (imm & kImm11_5Mask) >> 5;
  uint32_t imm4_0 = imm & kImm4_0Mask;

  DCHECK((is_int7(imm11_5) || is_uint7(imm11_5)) && rs2.is_valid() &&
         rs1.is_valid() && is_uint5(imm4_0));
  Instr instr = (imm11_5 << kImmS1Shift) | (rs2.code() << kRs2Shift) |
                (rs1.code() << kRs1Shift) | (func3 << kFunct3Shift) |
                (imm4_0 << kRdShift) | opcode;
  Emit(instr);
}

// R4-type bit[14:12] is funct3
// GIFPR41(GenInstr FloatPoint R4 type 1)
void Assembler::GenInstrFR4Type(FPURegister rs3, Funct2 func2, FPURegister rs2,
                                FPURegister rs1, Funct3 func3, FPURegister rd,
                                Opcode opcode) {
  DCHECK(rs3.is_valid() && rs2.is_valid() && rs1.is_valid() && rd.is_valid());
  Instr instr = (rs3.code() << kRs3Shift) | (func2 << kFunct2Shift) |
                (rs2.code() << kRs2Shift) | (rs1.code() << kRs1Shift) |
                (func3 << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}
// R4-type bit[14:12] is rm
// FMADDS/FMSUBS/FNMSUBS/FNMADDS
// FMADDD/FMSUBD/FNMSUBD/FNMADDD
// GIFPR42(GenInstr FloatPoint R4 type 2)
void Assembler::GenInstrFR4Type(FPURegister rs3, Funct2 func2, FPURegister rs2,
                                FPURegister rs1, uint32_t rm, FPURegister rd,
                                Opcode opcode) {
  DCHECK(rs3.is_valid() && rs2.is_valid() && rs1.is_valid() && is_uint3(rm) &&
         rd.is_valid());
  Instr instr = (rs3.code() << kRs3Shift) | (func2 << kFunct2Shift) |
                (rs2.code() << kRs2Shift) | (rs1.code() << kRs1Shift) |
                (rm << kFunct3Shift) | (rd.code() << kRdShift) | opcode;
  Emit(instr);
}

// RV32F Standard Extension
// GII4
void Assembler::flw(FPURegister rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), FLW_F3, rd, FLW);
}

// GIS2
void Assembler::fsw(FPURegister rs2, const MemOperand& rs1) {
  GenInstrSType(rs2, rs1.rm(), FSW_F3, rs1.offset_, FSW);
}

// GIFPR42
void Assembler::fmadds(FPURegister rd, FPURegister rs1, FPURegister rs2,
                       FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FMADDS_F2, rs2, rs1, rm, rd, FMADDS);
}

// GIFPR42
void Assembler::fmsubs(FPURegister rd, FPURegister rs1, FPURegister rs2,
                       FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FMSUBS_F2, rs2, rs1, rm, rd, FMSUBS);
}

// GIFPR42
void Assembler::fnmsubs(FPURegister rd, FPURegister rs1, FPURegister rs2,
                        FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FNMSUBS_F2, rs2, rs1, rm, rd, FNMSUBS);
}

// GIFPR42
void Assembler::fnmadds(FPURegister rd, FPURegister rs1, FPURegister rs2,
                        FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FNMADDS_F2, rs2, rs1, rm, rd, FNMADDS);
}

// GIFPR4
void Assembler::fadds(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FADDS_F7, rs2, rs1, rm, rd, FADDS);
}

// GIFPR4
void Assembler::fsubs(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FSUBS_F7, rs2, rs1, rm, rd, FSUBS);
}

// GIFPR4
void Assembler::fmuls(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FMULS_F7, rs2, rs1, rm, rd, FMULS);
}

// GIFPR4
void Assembler::fdivs(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FDIVS_F7, rs2, rs1, rm, rd, FDIVS);
}

// GIFPR3
void Assembler::fsqrts(FPURegister rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FSQRTS_F7, 0, rs1, rm, rd, FSQRTS);
}

// GIFPR1
void Assembler::fsgnjs(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJS_F7, rs2, rs1, FSGNJS_F3, rd, FSGNJS);
}

// GIFPR1
void Assembler::fsgnjns(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJNS_F7, rs2, rs1, FSGNJNS_F3, rd, FSGNJNS);
}

// GIFPR1
void Assembler::fsgnjxs(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJXS_F7, rs2, rs1, FSGNJXS_F3, rd, FSGNJXS);
}

// GIFPR1
void Assembler::fmins(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FMINS_F7, rs2, rs1, FMINS_F3, rd, FMINS);
}

// GIFPR1
void Assembler::fmaxs(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FMAXS_F7, rs2, rs1, FMAXS_F3, rd, FMAXS);
}

// GIFPR3_2
void Assembler::fcvtws(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTWS_F7, 0, rs1, rm, rd, FCVTWS);
}

// GIFPR3_2
void Assembler::fcvtwus(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTWUS_F7, 1, rs1, rm, rd, FCVTWUS);
}

// GIFPR2_1
void Assembler::fmvxw(Register rd, FPURegister rs1) {
  GenInstrFRType(FMVXW_F7, 0, rs1, FMVXW_F3, rd, FMVXW);
}

// GIFPR1_1
void Assembler::feqs(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FEQS_F7, rs2, rs1, FEQS_F3, rd, FEQS);
}

// GIFPR1_1
void Assembler::flts(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FLTS_F7, rs2, rs1, FLTS_F3, rd, FLTS);
}

// GIFPR1_1
void Assembler::fles(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FLES_F7, rs2, rs1, FLES_F3, rd, FLES);
}

// GIFPR2_1
void Assembler::fclasss(Register rd, FPURegister rs1) {
  GenInstrFRType(FCLASSS_F7, 0, rs1, FCLASSS_F3, rd, FCLASSS);
}

// GIFPR3_1
void Assembler::fcvtsw(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTSW_F7, 0, rs1, rm, rd, FCVTSW);
}

// GIFPR3_1
void Assembler::fcvtswu(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTSWU_F7, 1, rs1, rm, rd, FCVTSWU);
}

// GIFPR2_2
void Assembler::fmvwx(FPURegister rd, Register rs1) {
  GenInstrFRType(FMVWX_F7, 0, rs1, FMVWX_F3, rd, FMVWX);
}

// RV64F Standard Extension (in addition to RV32F)
// GIFPR3_2
void Assembler::fcvtls(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTLS_F7, 2, rs1, rm, rd, FCVTLS);
}

// GIFPR3_2
void Assembler::fcvtlus(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTLUS_F7, 3, rs1, rm, rd, FCVTLUS);
}

// GIFPR3_1
void Assembler::fcvtsl(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTSL_F7, 2, rs1, rm, rd, FCVTSL);
}

// GIFPR3_1
void Assembler::fcvtslu(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTSLU_F7, 3, rs1, rm, rd, FCVTSLU);
}

// RV32D Standard Extension
// GII4
void Assembler::fld(FPURegister rd, const MemOperand& rs1) {
  GenInstrIType(rs1.offset_, rs1.rm(), FLD_F3, rd, FLD);
}

// GIS2
void Assembler::fsd(FPURegister rs2, const MemOperand& rs1) {
  GenInstrSType(rs2, rs1.rm(), FSD_F3, rs1.offset_, FSD);
}

// GIFPR42
void Assembler::fmaddd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                       FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FMADDD_F2, rs2, rs1, rm, rd, FMADDD);
}

// GIFPR42
void Assembler::fmsubd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                       FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FMSUBD_F2, rs2, rs1, rm, rd, FMSUBD);
}

// GIFPR42
void Assembler::fnmsubd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                        FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FNMSUBD_F2, rs2, rs1, rm, rd, FNMSUBD);
}

// GIFPR42
void Assembler::fnmaddd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                        FPURegister rs3, uint32_t rm) {
  GenInstrFR4Type(rs3, FNMADDD_F2, rs2, rs1, rm, rd, FNMADDD);
}

// GIFPR4
void Assembler::faddd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FADDD_F7, rs2, rs1, rm, rd, FADDD);
}

// GIFPR4
void Assembler::fsubd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FSUBD_F7, rs2, rs1, rm, rd, FSUBD);
}

// GIFPR4
void Assembler::fmuld(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FMULD_F7, rs2, rs1, rm, rd, FMULD);
}

// GIFPR4
void Assembler::fdivd(FPURegister rd, FPURegister rs1, FPURegister rs2,
                      uint32_t rm) {
  GenInstrFRType(FDIVD_F7, rs2, rs1, rm, rd, FDIVD);
}

// GIFPR3
void Assembler::fsqrtd(FPURegister rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FSQRTD_F7, 0, rs1, rm, rd, FSQRTD);
}

// GIFPR1
void Assembler::fsgnjd(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJD_F7, rs2, rs1, FSGNJD_F3, rd, FSGNJD);
}

// GIFPR1
void Assembler::fsgnjnd(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJND_F7, rs2, rs1, FSGNJND_F3, rd, FSGNJND);
}

// GIFPR1
void Assembler::fsgnjxd(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FSGNJXD_F7, rs2, rs1, FSGNJXD_F3, rd, FSGNJXD);
}

// GIFPR1
void Assembler::fmind(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FMIND_F7, rs2, rs1, FMIND_F3, rd, FMIND);
}

// GIFPR1
void Assembler::fmaxD(FPURegister rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FMAXD_F7, rs2, rs1, FMAXD_F3, rd, FMAXD);
}

// GIFPR3
void Assembler::fcvtsd(FPURegister rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTSD_F7, 1, rs1, rm, rd, FCVTSD);
}

// GIFPR3
void Assembler::fcvtds(FPURegister rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTDS_F7, 0, rs1, rm, rd, FCVTDS);
}

// GIFPR1_1
void Assembler::feqd(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FEQD_F7, rs2, rs1, FEQD_F3, rd, FEQD);
}

// GIFPR1_1
void Assembler::fltd(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FLTD_F7, rs2, rs1, FLTD_F3, rd, FLTD);
}

// GIFPR1_1
void Assembler::fled(Register rd, FPURegister rs1, FPURegister rs2) {
  GenInstrFRType(FLED_F7, rs2, rs1, FLED_F3, rd, FLED);
}

// GIFPR2_1
void Assembler::fclassd(Register rd, FPURegister rs1) {
  GenInstrFRType(FCLASSD_F7, 0, rs1, FCLASSD_F3, rd, FCLASSD);
}

// GIFPR3_2
void Assembler::fcvtwd(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTWD_F7, 0, rs1, rm, rd, FCVTWD);
}

// GIFPR3_2
void Assembler::fcvtwud(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTWUD_F7, 1, rs1, rm, rd, FCVTWUD);
}

// GIFPR3_1
void Assembler::fcvtdw(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTDW_F7, 0, rs1, rm, rd, FCVTDW);
}

// GIFPR3_1
void Assembler::fcvtdwu(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTDWU_F7, 1, rs1, rm, rd, FCVTDWU);
}

// RV64D Standard Extension (in addition to RV32D)
// GIFPR3_2
void Assembler::fcvtld(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTLD_F7, 2, rs1, rm, rd, FCVTLD);
}

// GIFPR3_2
void Assembler::fcvtlud(Register rd, FPURegister rs1, uint32_t rm) {
  GenInstrFRType(FCVTLUD_F7, 3, rs1, rm, rd, FCVTLUD);
}

// GIFPR2_1
void Assembler::fmvxd(Register rd, FPURegister rs1) {
  GenInstrFRType(FMVXD_F7, 0, rs1, FMVXD_F3, rd, FMVXD);
}

// GIFPR3_1
void Assembler::fcvtdl(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTDL_F7, 2, rs1, rm, rd, FCVTDL);
}

// GIFPR3_1
void Assembler::fcvtdlu(FPURegister rd, Register rs1, uint32_t rm) {
  GenInstrFRType(FCVTDLU_F7, 3, rs1, rm, rd, FCVTDLU);
}

// GIFPR2_2
void Assembler::fmvdx(FPURegister rd, Register rs1) {
  GenInstrFRType(FMVDX_F7, 0, rs1, FMVDX_F3, rd, FMVDX);
}

// RV32Q Standard Extension
#if 0
void Assembler::flq(FPURegister rs1, FPURegister rd, uint32_t imm) {
  GenInstrImmediate(rs1, FLQ_F3, rd, imm, FLQ);
}
void Assembler::fsq(FPURegister rs2, FPURegister rs1, uint32_t imm) {
  GenInstrImmediate(rs2, rs1, FSQ_F3, imm, FSQ);
}
void Assembler::fmaddq(FPURegister rs3, FPURegister rs2,
                        FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(rs3, 3, rs2, rs1, rm, rd, FMADDQ);
}
void Assembler::fmsubq(FPURegister rs3, FPURegister rs2,
                        FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(rs3, 3, rs2, rs1, rm, rd, FMSUBQ);
}
void Assembler::fnmsubq(FPURegister rs3, FPURegister rs2,
                        FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(rs3, 3, rs2, rs1, rm, rd, FNMSUBQ);
}
void Assembler::fnmaddq(FPURegister rs3, FPURegister rs2,
                        FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(rs3, 3, rs2, rs1, rm, rd, FNMADDQ);
}
void Assembler::faddq(FPURegister rs2, FPURegister rs1,
                      FPURegister rd, uint32_t rm) {
  GenInstrFRType(FADDQ_F7, rs2, rs1, rm, rd, FADDQ);
}
void Assembler::fsubq(FPURegister rs2, FPURegister rs1,
                      FPURegister rd, uint32_t rm) {
  GenInstrFRType(FSUBQ_F7, rs2, rs1, rm, rd, FSUBQ);
}
void Assembler::fmulq(FPURegister rs2, FPURegister rs1,
                      FPURegister rd, uint32_t rm) {
  GenInstrFRType(FMULQ_F7, rs2, rs1, rm, rd, FMULQ);
}
void Assembler::fdivq(FPURegister rs2, FPURegister rs1,
                      FPURegister rd, uint32_t rm) {
  GenInstrFRType(FDIVQ_F7, rs2, rs1, rm, rd, FDIVQ);
}
void Assembler::fsqrtq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FSQRTQ_F7, 0, rs1, rm, rd, FSQRTQ);
}
void Assembler::fsgnjq(FPURegister rs2, FPURegister rs1) {
  GenInstrFRType(FSGNJQ_F7, rs2, rs1, FSGNJQ_F3, rd, FSGNJQ);
}
void Assembler::fsgnjnq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FSGNJNQ_F7, rs2, rs1, FSGNJNQ_F3, rd, FSGNJNQ);
}
void Assembler::fsgnjxq(FPURegister rs2, FPURegister rs1,
                        FPURegister rd, uint32_t rm) {
  GenInstrFRType(FSGNJXQ_F7, rs2, rs1, FSGNJXQ_F3, rd, FSGNJXQ);
}
void Assembler::fminq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FMINQ_F7, rs2, rs1, FMINQ_F3, rd, FMINQ);
}
void Assembler::fmaxq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FMAXQ_F7, rs2, rs1, FMAXQ_F3, rd, FMAXQ);
}
void Assembler::fcvtsq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTSQ_F7, 3, rs1, rm, rd, FCVTSQ);
}
void Assembler::fcvtqs(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTQS_F7, 0, rs1, rm, rd, FCVTQS);
}
void Assembler::fcvtdq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTDQ_F7, 3, rs1, rm, rd, FCVTDQ);
}
void Assembler::fcvtqd(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTQD_F7, 1, rs1, rm, rd, FCVTQD);
}
void Assembler::feqq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FEQQ_F7, rs2, rs1, FEQQ_F3, rd, FEQQ);
}
void Assembler::fltq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FLTQ_F7, rs2, rs1, FLTQ_F3, rd, FLTQ);
}
void Assembler::fleq(FPURegister rs2, FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FLEQ_F7, rs2, rs1, FLEQ_F3, rd, FLEQ);
}
void Assembler::fclassq(FPURegister rs1, FPURegister rd) {
  GenInstrFRType(FCLASSQ_F7, 0, rs1, FCLASSQ_F3, rd, FCLASSQ);
}
void Assembler::fcvtwq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTWQ_F7, 0, rs1, rm, rd, FCVTWQ);
}
void Assembler::fcvtwuq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTWUQ_F7, 1, rs1, rm, rd, FCVTWUQ);
}
void Assembler::fcvtqw(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTQW_F7, 0, rs1, rm, rd, FCVTQW);
}
void Assembler::fcvtqwu(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTQWU_F7, 1, rs1, rm, rd, FCVTQWU);
}


// RV64Q Standard Extension (in addition to RV32Q)
void Assembler::fcvtlq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTLQ_F7, 2, rs1, rm, rd, FCVTLQ);
}
void Assembler::fcvtluq(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTLUQ_F7, 3, rs1, rm, rd, FCVTLUQ);
}
void Assembler::fcvtql(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTQL_F7, 2, rs1, rm, rd, FCVTQL);
}
void Assembler::fcvtdlu(FPURegister rs1, FPURegister rd, uint32_t rm) {
  GenInstrFRType(FCVTDLU_F7, 3, rs1, rm, rd, FCVTDLU);
}
#endif

void Assembler::b(int16_t offset) { beq(zero_reg, zero_reg, offset); }

// Pesudo instruction

void Assembler::mv(Register rd, Register rs) { addi(rd, rs, 0); }

void Assembler::not_(Register rd, Register rs) { xori(rd, rs, -1); }

void Assembler::neg(Register rd, Register rs) { sub(rd, zero_reg, rs); }

void Assembler::negw(Register rd, Register rs) { subw(rd, zero_reg, rs); }

void Assembler::sextw(Register rd, Register rs) { addiw(rd, rs, 0); }

void Assembler::seqz(Register rd, Register rs) { sltiu(rd, rs, 1); }

void Assembler::snez(Register rd, Register rs) { sltu(rd, zero_reg, rs); }

void Assembler::sltz(Register rd, Register rs) { slt(rd, rs, zero_reg); }

void Assembler::sgtz(Register rd, Register rs) { slt(rd, zero_reg, rs); }

void Assembler::fmvs(FPURegister rd, FPURegister rs) { fsgnjs(rd, rs, rs); }

void Assembler::fabss(FPURegister rd, FPURegister rs) { fsgnjxs(rd, rs, rs); }

void Assembler::fnegs(FPURegister rd, FPURegister rs) { fsgnjns(rd, rs, rs); }

void Assembler::fmvd(FPURegister rd, FPURegister rs) { fsgnjd(rd, rs, rs); }

void Assembler::fabsd(FPURegister rd, FPURegister rs) { fsgnjxd(rd, rs, rs); }

void Assembler::fnegd(FPURegister rd, FPURegister rs) { fsgnjnd(rd, rs, rs); }

void Assembler::beqz(Register rs, int16_t offset) { beq(rs, zero_reg, offset); }

void Assembler::bnez(Register rs, int16_t offset) { bne(rs, zero_reg, offset); }

void Assembler::blez(Register rs, int16_t offset) { bge(zero_reg, rs, offset); }

void Assembler::bgez(Register rs, int16_t offset) { bge(rs, zero_reg, offset); }

void Assembler::bltz(Register rs, int16_t offset) { blt(rs, zero_reg, offset); }

void Assembler::bgtz(Register rs, int16_t offset) { blt(zero_reg, rs, offset); }

void Assembler::bgt(Register rs, Register rt, int16_t offset) {
  blt(rt, rs, offset);
}

void Assembler::ble(Register rs, Register rt, int16_t offset) {
  bge(rt, rs, offset);
}

void Assembler::bgtu(Register rs, Register rt, int16_t offset) {
  bltu(rt, rs, offset);
}

void Assembler::bleu(Register rs, Register rt, int16_t offset) {
  bgeu(rt, rs, offset);
}

void Assembler::jr(Register rs) { jalr(x0, 0, rs); }

void Assembler::jalr(Register rs) { jalr(x1, 0, rs); }

void Assembler::ret(Register rs) { jalr(x0, 0, x1); }

void Assembler::rdinstreth(Register rd) { csrrs(rd, instreth, zero_reg); }

void Assembler::rdcycleh(Register rd) { csrrs(rd, cycleh, zero_reg); }

void Assembler::rdtimeh(Register rd) { csrrs(rd, timeh, zero_reg); }

void Assembler::csrr(Register rd, CSRRegister csr) { csrrs(rd, csr, zero_reg); }

void Assembler::csrw(CSRRegister csr, Register rs) { csrrs(zero_reg, csr, rs); }

void Assembler::csrs(CSRRegister csr, Register rs) { csrrs(zero_reg, csr, rs); }

void Assembler::csrc(CSRRegister csr, Register rs) { csrrs(zero_reg, csr, rs); }

void Assembler::csrwi(CSRRegister csr, uint32_t uimm) {
  csrrwi(zero_reg, csr, uimm);
}

void Assembler::csrsi(CSRRegister csr, uint32_t uimm) {
  csrrwi(zero_reg, csr, uimm);
}

void Assembler::csrci(CSRRegister csr, uint32_t uimm) {
  csrrwi(zero_reg, csr, uimm);
}

void Assembler::frcsr(Register rd) { csrrs(rd, fcsr, zero_reg); }

void Assembler::frcsr(Register rd, Register rs) { csrrs(rd, fcsr, rs); }

void Assembler::fscsr(Register rs) { csrrw(zero_reg, fcsr, rs); }

void Assembler::frrm(Register rd) { csrrs(rd, frm, zero_reg); }

void Assembler::fsrm(Register rd, Register rs) { csrrs(rd, frm, rs); }

void Assembler::fsrm(Register rs) { csrrw(zero_reg, frm, rs); }

void Assembler::frflags(Register rd) { csrrs(rd, fflags, zero_reg); }

void Assembler::fsflags(Register rd, Register rs) { csrrs(rd, fflags, rs); }

void Assembler::fsflags(Register rs) { csrrw(zero_reg, fflags, rs); }

void Assembler::lla(Register rd, Label* l) { lla(rd, branch_offset(l)); }

void Assembler::lla(Register rd, int offset) {
  int32_t offset11 = offset & kImm11Mask;
  auipc(rd, (offset >> 12) + offset11);
  addi(rd, rd, offset & kImm11_0Mask);
}

void Assembler::call(int offset) {
  int32_t offset11 = offset & kImm11Mask;
  auipc(x1, (offset >> 12) + offset11);
  jalr(x1, offset & kImm11_0Mask, x1);
}

// ------------Memory-instructions-------------

void Assembler::AdjustBaseAndOffset(MemOperand* src,
                                    OffsetAccessType access_type,
                                    int second_access_add_to_offset) {
  // This method is used to adjust the base register and offset pair
  // for a load/store when the offset doesn't fit into int12_t.
  // It is assumed that 'base + offset' is sufficiently aligned for memory
  // operands that are machine word in size or smaller. For doubleword-sized
  // operands it's assumed that 'base' is a multiple of 8, while 'offset'
  // may be a multiple of 4 (e.g. 4-byte-aligned long and double arguments
  // and spilled variables on the stack accessed relative to the stack
  // pointer register).
  // We preserve the "alignment" of 'offset' by adjusting it by a multiple of 8.

  bool doubleword_aligned = (src->offset() & (kDoubleSize - 1)) == 0;
  bool two_accesses = static_cast<bool>(access_type) || !doubleword_aligned;
  DCHECK_LE(second_access_add_to_offset, 7);  // Must be <= 7.

  // is_int12 must be passed a signed value, hence the static cast below.
  if (is_int12(src->offset()) &&
      (!two_accesses || is_int16(static_cast<int32_t>(
                            src->offset() + second_access_add_to_offset)))) {
    // Nothing to do: 'offset' (and, if needed, 'offset + 4', or other specified
    // value) fits into int16_t.
    return;
  }

  // DCHECK(src->rm() !=
  // at);  // Must not overwrite the register 'base' while loading 'offset'.

#ifdef DEBUG
  // Remember the "(mis)alignment" of 'offset', it will be checked at the end.
  uint32_t misalignment = src->offset() & (kDoubleSize - 1);
#endif

  // Do not load the whole 32-bit 'offset' if it can be represented as
  // a sum of two 12-bit signed offsets. This can save an instruction or two.
  // To simplify matters, only do this for a symmetric range of offsets from
  // about -4KB to about +4KB, allowing further addition of 4 when accessing
  // 64-bit variables with two 32-bit accesses.
  constexpr int32_t kMinOffsetForSimpleAdjustment =
      0x7F8;  // Max int12_t that's a multiple of 8.
  constexpr int32_t kMaxOffsetForSimpleAdjustment =
      2 * kMinOffsetForSimpleAdjustment;

  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  if (0 <= src->offset() && src->offset() <= kMaxOffsetForSimpleAdjustment) {
    addi(scratch, src->rm(), kMinOffsetForSimpleAdjustment);
    src->offset_ -= kMinOffsetForSimpleAdjustment;
  } else if (-kMaxOffsetForSimpleAdjustment <= src->offset() &&
             src->offset() < 0) {
    addi(scratch, src->rm(), -kMinOffsetForSimpleAdjustment);
    src->offset_ += kMinOffsetForSimpleAdjustment;
  } else {
    // Now that all shorter options have been exhausted, load the full 32-bit
    // offset.
    int32_t loaded_offset = RoundDown(src->offset(), kDoubleSize);
    //to modify to use getHiLowImm32
    int32_t Hi20;
    int32_t Lo12;
    GetBranchlongArg(loaded_offset, Hi20, Lo12);
    lui(scratch, Hi20);
    addi(scratch, scratch, Lo12);  // Load 32-bit offset.
    add(scratch, scratch, src->rm());
    src->offset_ -= loaded_offset;
  }
  src->rm_ = scratch;

  DCHECK(is_int12(src->offset()));
  if (two_accesses) {
    DCHECK(is_int12(
        static_cast<int32_t>(src->offset() + second_access_add_to_offset)));
  }
  DCHECK(misalignment == (src->offset() & (kDoubleSize - 1)));
}

UseScratchRegisterScope::UseScratchRegisterScope(Assembler* assembler)
    : available_(assembler->GetScratchRegisterList()),
      old_available_(*available_) {}

UseScratchRegisterScope::~UseScratchRegisterScope() {
  *available_ = old_available_;
}
Register UseScratchRegisterScope::Acquire() {
  DCHECK_NOT_NULL(available_);
  DCHECK_NE(*available_, 0);
  int index = static_cast<int>(base::bits::CountTrailingZeros32(*available_));
  *available_ &= ~(1UL << index);

  return Register::from_code(index);
}

bool UseScratchRegisterScope::hasAvailable() const { return *available_ != 0; }

}  // namespace internal
}  // namespace v8

#endif  // V8_TARGET_ARCH_RISCV
