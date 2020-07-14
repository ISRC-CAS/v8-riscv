// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if V8_TARGET_ARCH_RISCV64

#include "src/regexp/riscv64/regexp-macro-assembler-riscv64.h"

#include "src/codegen/macro-assembler.h"
#include "src/logging/log.h"
#include "src/objects/objects-inl.h"
#include "src/regexp/regexp-macro-assembler.h"
#include "src/regexp/regexp-stack.h"
#include "src/snapshot/embedded/embedded-data.h"
#include "src/strings/unicode.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm_)

const int RegExpMacroAssemblerRISCV64::kRegExpCodeSize;

RegExpMacroAssemblerRISCV64::RegExpMacroAssemblerRISCV64(Isolate* isolate,
                                                         Zone* zone, Mode mode,
                                                         int registers_to_save)
    : NativeRegExpMacroAssembler(isolate, zone),
      masm_(new MacroAssembler(isolate, CodeObjectRequired::kYes,
                               NewAssemblerBuffer(kRegExpCodeSize))),
      mode_(mode),
      num_registers_(registers_to_save),
      num_saved_registers_(registers_to_save),
      entry_label_(),
      start_label_(),
      success_label_(),
      backtrack_label_(),
      exit_label_() {
  DCHECK_EQ(0, registers_to_save % 2);
  // ...
}

RegExpMacroAssemblerRISCV64::~RegExpMacroAssemblerRISCV64() {
  delete masm_;
  // Unuse labels in case we throw away the assembler without calling GetCode.
  entry_label_.Unuse();
  start_label_.Unuse();
  success_label_.Unuse();
  backtrack_label_.Unuse();
  exit_label_.Unuse();
  check_preempt_label_.Unuse();
  stack_overflow_label_.Unuse();
}

int RegExpMacroAssemblerRISCV64::stack_limit_slack() {
  return RegExpStack::kStackLimitSlack;
}

// Helper function for reading a value out of a stack frame.
template <typename T>
static T& frame_entry(Address re_frame, int frame_offset) {
  return reinterpret_cast<T&>(Memory<int32_t>(re_frame + frame_offset));
}

template <typename T>
static T* frame_entry_address(Address re_frame, int frame_offset) {
  return reinterpret_cast<T*>(re_frame + frame_offset);
}

int RegExpMacroAssemblerRISCV64::CheckStackGuardState(Address* return_address,
                                                      Address raw_code,
                                                      Address re_frame) {
  Code re_code = Code::cast(Object(raw_code));
  return NativeRegExpMacroAssembler::CheckStackGuardState(
      frame_entry<Isolate*>(re_frame, kIsolate),
      frame_entry<int>(re_frame, kStartIndex),
      static_cast<RegExp::CallOrigin>(frame_entry<int>(re_frame, kDirectCall)),
      return_address, re_code,
      frame_entry_address<Address>(re_frame, kInputString),
      frame_entry_address<const byte*>(re_frame, kInputStart),
      frame_entry_address<const byte*>(re_frame, kInputEnd));
}

void RegExpMacroAssemblerRISCV64::CallCheckStackGuardState(Register scratch) {}
void RegExpMacroAssemblerRISCV64::AdvanceCurrentPosition(int by) {}

void RegExpMacroAssemblerRISCV64::AdvanceRegister(int reg, int by) {}
RegExpMacroAssembler::IrregexpImplementation
RegExpMacroAssemblerRISCV64::Implementation() {
  return kRISCV64Implementation;
}

Handle<HeapObject> RegExpMacroAssemblerRISCV64::GetCode(Handle<String> source) {
  CodeDesc code_desc;
  Isolate* isolate = this->isolate();
  // masm_.GetCode(isolate, &code_desc);
  Handle<Code> code = Factory::CodeBuilder(isolate, code_desc, Code::REGEXP)
                          .set_self_reference(masm_->CodeObject())
                          .Build();
  // PROFILE(isolate,
  //        RegExpCodeCreateEvent(Handle<AbstractCode>::cast(code), source));
  return Handle<HeapObject>::cast(code);
}

}  // namespace internal
}  // namespace v8

#undef __

#endif  // V8_TARGET_ARCH_RISCV64
