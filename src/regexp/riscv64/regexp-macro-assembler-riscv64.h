// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_REGEXP_RISCV64_REGEXP_MACRO_ASSEMBLER_RISCV64_H_
#define V8_REGEXP_RISCV64_REGEXP_MACRO_ASSEMBLER_RISCV64_H_

#include "src/codegen/macro-assembler.h"
#include "src/codegen/riscv64/assembler-riscv64.h"
#include "src/regexp/regexp-macro-assembler.h"

namespace v8 {
namespace internal {

class V8_EXPORT_PRIVATE RegExpMacroAssemblerRISCV64
    : public NativeRegExpMacroAssembler {
 public:
  RegExpMacroAssemblerRISCV64(Isolate* isolate, Zone* zone, Mode mode,
                              int registers_to_save);
  virtual ~RegExpMacroAssemblerRISCV64();
  virtual void AbortedCodeGeneration() { masm_->AbortedCodeGeneration(); }
  virtual void AdvanceCurrentPosition(int by);
  virtual void AdvanceRegister(int reg, int by);
  virtual void Backtrack() {}
  virtual void Bind(Label* label) {}
  virtual int stack_limit_slack();
  virtual void CheckAtStart(int cp_offset, Label* on_at_start) {}
  virtual void CheckCharacter(unsigned c, Label* on_equal) {}
  virtual void CheckCharacterAfterAnd(unsigned c, unsigned mask,
                                      Label* on_equal) {}
  virtual void CheckCharacterGT(uc16 limit, Label* on_greater) {}
  virtual void CheckCharacterLT(uc16 limit, Label* on_less) {}
  virtual void CheckCharacters(Vector<const uc16> str, int cp_offset,
                               Label* on_failure, bool check_end_of_string) {}
  // A "greedy loop" is a loop that is both greedy and with a simple
  // body. It has a particularly simple implementation.
  virtual void CheckGreedyLoop(Label* on_tos_equals_current_position) {}
  virtual void CheckNotAtStart(int cp_offset, Label* on_not_at_start) {}
  virtual void CheckNotBackReference(int start_reg, bool read_backward,
                                     Label* on_no_match) {}
  virtual void CheckNotBackReferenceIgnoreCase(int start_reg,
                                               bool read_backward,
                                               Label* on_no_match) {}
  virtual void CheckNotCharacter(unsigned c, Label* on_not_equal) {}
  virtual void CheckNotCharacterAfterAnd(unsigned c, unsigned mask,
                                         Label* on_not_equal) {}
  virtual void CheckNotCharacterAfterMinusAnd(uc16 c, uc16 minus, uc16 mask,
                                              Label* on_not_equal) {}
  virtual void CheckCharacterInRange(uc16 from, uc16 to, Label* on_in_range) {}
  virtual void CheckCharacterNotInRange(uc16 from, uc16 to,
                                        Label* on_not_in_range) {}
  virtual void CheckBitInTable(Handle<ByteArray> table, Label* on_bit_set) {}

  // Checks whether the given offset from the current position is before
  // the end of the string.
  virtual void CheckPosition(int cp_offset, Label* on_outside_input) {}
  virtual bool CheckSpecialCharacterClass(uc16 type, Label* on_no_match) {
    return false;
  }
  virtual void Fail() {}
  virtual Handle<HeapObject> GetCode(Handle<String> source);
  virtual void GoTo(Label* label) {}
  virtual void IfRegisterGE(int reg, int comparand, Label* if_ge) {}
  virtual void IfRegisterLT(int reg, int comparand, Label* if_lt) {}
  virtual void IfRegisterEqPos(int reg, Label* if_eq) {}
  virtual IrregexpImplementation Implementation();
  virtual void LoadCurrentCharacterImpl(int cp_offset, Label* on_end_of_input,
                                        bool check_bounds, int characters,
                                        int eats_at_least) {}
  virtual void PopCurrentPosition() {}
  virtual void PopRegister(int register_index) {}
  virtual void PushBacktrack(Label* label) {}
  virtual void PushCurrentPosition() {}
  virtual void PushRegister(int register_index,
                            StackCheckFlag check_stack_limit) {}
  virtual void ReadCurrentPositionFromRegister(int reg) {}
  virtual void ReadStackPointerFromRegister(int reg) {}
  virtual void SetCurrentPositionFromEnd(int by) {}
  virtual void SetRegister(int register_index, int to) {}
  virtual bool Succeed() { return true; }
  virtual void WriteCurrentPositionToRegister(int reg, int cp_offset) {}
  virtual void ClearRegisters(int reg_from, int reg_to) {}
  virtual void WriteStackPointerToRegister(int reg) {}

  // Called from RegExp if the stack-guard is triggered.
  // If the code object is relocated, the return address is fixed before
  // returning.
  // {raw_code} is an Address because this is called via ExternalReference.
  static int CheckStackGuardState(Address* return_address, Address raw_code,
                                  Address re_frame);
  // Generate a call to CheckStackGuardState.
  void CallCheckStackGuardState(Register scratch);

 private:
  // Offsets from rbp of function parameters and stored registers.
  static const int kFramePointer = 0;
  // Above the frame pointer - function parameters and return address.
  static const int kReturn_eip = kFramePointer + kSystemPointerSize;
  static const int kFrameAlign = kReturn_eip + kSystemPointerSize;
  // Above the frame pointer - Stored registers and stack passed parameters.
  // Callee-saved registers x19-x29, where x29 is the old frame pointer.
  static const int kCalleeSavedRegisters = 0;
  // Return address.
  // It is placed above the 11 callee-saved registers.
  static const int kReturnAddress =
      kCalleeSavedRegisters + 11 * kSystemPointerSize;

  static const int kInputString = kFramePointer - kSystemPointerSize;
  static const int kStartIndex = kInputString - kSystemPointerSize;
  static const int kInputStart = kStartIndex - kSystemPointerSize;
  static const int kInputEnd = kInputStart - kSystemPointerSize;
  static const int kRegisterOutput = kInputEnd - kSystemPointerSize;

  // For the case of global regular expression, we have room to store at least
  // one set of capture results.  For the case of non-global regexp, we ignore
  // this value.
  static const int kNumOutputRegisters = kRegisterOutput - kSystemPointerSize;
  static const int kStackHighEnd = kFrameAlign;
  static const int kDirectCall = kStackHighEnd + kSystemPointerSize;
  static const int kIsolate = kDirectCall + kSystemPointerSize;
  static const int kBackup_rbx = kNumOutputRegisters - kSystemPointerSize;
  static const int kLastCalleeSaveRegister = kBackup_rbx;

  // When adding local variables remember to push space for them in
  // the frame in GetCode.
  static const int kSuccessfulCaptures =
      kLastCalleeSaveRegister - kSystemPointerSize;
  static const int kStringStartMinusOne =
      kSuccessfulCaptures - kSystemPointerSize;
  static const int kBacktrackCount = kStringStartMinusOne - kSystemPointerSize;

  // First register address. Following registers are below it on the stack.
  static const int kRegisterZero = kBacktrackCount - kSystemPointerSize;

  // Initial size of code buffer.
  static const int kRegExpCodeSize = 1024;

  // Load a number of characters at the given offset from the
  // current position, into the current-character register.
  void LoadCurrentCharacterUnchecked(int cp_offset, int character_count);

  // Check whether preemption has been requested.
  void CheckPreemption();

  // Check whether we are exceeding the stack limit on the backtrack stack.
  void CheckStackLimit();

  Isolate* isolate() const { return masm_->isolate(); }

  MacroAssembler* masm_;

  // Which mode to generate code for (LATIN1 or UC16).
  Mode mode_;

  // One greater than maximal register index actually used.
  int num_registers_;

  // Number of registers to output at the end (the saved registers
  // are always 0..num_saved_registers_-1)
  int num_saved_registers_;

  // Labels used internally.
  Label entry_label_;
  Label start_label_;
  Label success_label_;
  Label backtrack_label_;
  Label exit_label_;
  Label check_preempt_label_;
  Label stack_overflow_label_;
};

}  // namespace internal
}  // namespace v8

#endif  // V8_REGEXP_RISCV64_REGEXP_MACRO_ASSEMBLER_RISCV64_H_
