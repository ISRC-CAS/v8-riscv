// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef INCLUDED_FROM_MACRO_ASSEMBLER_H
#error This header must be included via macro-assembler.h
#endif

#ifndef V8_CODEGEN_RISCV64_MACRO_ASSEMBLER_RISCV64_H_
#define V8_CODEGEN_RISCV64_MACRO_ASSEMBLER_RISCV64_H_

#include <vector>

#include "src/base/bits.h"
#include "src/codegen/bailout-reason.h"
#include "src/codegen/riscv64/assembler-riscv64.h"
#include "src/common/globals.h"

// Simulator specific helpers.
#if USE_SIMULATOR
// TODO(all): If possible automatically prepend an indicator like
// UNIMPLEMENTED or LOCATION.
#define ASM_UNIMPLEMENTED(message) __ Debug(message, __LINE__, NO_PARAM)
#define ASM_UNIMPLEMENTED_BREAK(message) \
  __ Debug(message, __LINE__,            \
           FLAG_ignore_asm_unimplemented_break ? NO_PARAM : BREAK)
#if DEBUG
#define ASM_LOCATION(message) __ Debug("LOCATION: " message, __LINE__, NO_PARAM)
#define ASM_LOCATION_IN_ASSEMBLER(message) \
  Debug("LOCATION: " message, __LINE__, NO_PARAM)
#else
#define ASM_LOCATION(message)
#define ASM_LOCATION_IN_ASSEMBLER(message)
#endif
#else
#define ASM_UNIMPLEMENTED(message)
#define ASM_UNIMPLEMENTED_BREAK(message)
#define ASM_LOCATION(message)
#define ASM_LOCATION_IN_ASSEMBLER(message)
#endif

namespace v8 {
namespace internal {

// Allow programmer to use Branch Delay Slot of Branches, Jumps, Calls.
enum BranchDelaySlot { USE_DELAY_SLOT, PROTECT };

enum RememberedSetAction { EMIT_REMEMBERED_SET, OMIT_REMEMBERED_SET };
// Flags used for the li macro-assembler function.
enum LiFlags {
  // If the constant value can be represented in just 16 bits, then
  // optimize the li to use a single instruction, rather than lui/ori/dsll
  // sequence. A number of other optimizations that emits less than
  // maximum number of instructions exists.
  OPTIMIZE_SIZE = 0,
  // Always use 6 instructions (lui/ori/dsll sequence) for release 2 or 4
  // instructions for release 6 (lui/ori/dahi/dati), even if the constant
  // could be loaded with just one, so that this value is patchable later.
  CONSTANT_SIZE = 1,
  // For address loads only 4 instruction are required. Used to mark
  // constant load that will be used as address without relocation
  // information. It ensures predictable code size, so specific sites
  // in code are patchable.
  ADDRESS_LOAD = 2
};
#define LS_MACRO_LIST(V)                                     \
  V(Ldrb, Register&, rt, LDRB_w)                             \
  V(Strb, Register&, rt, STRB_w)                             \
  V(Ldrsb, Register&, rt, rt.Is64Bits() ? LDRSB_x : LDRSB_w) \
  V(Ldrh, Register&, rt, LDRH_w)                             \
  V(Strh, Register&, rt, STRH_w)                             \
  V(Ldrsh, Register&, rt, rt.Is64Bits() ? LDRSH_x : LDRSH_w) \
  V(Ldr, CPURegister&, rt, LoadOpFor(rt))                    \
  V(Str, CPURegister&, rt, StoreOpFor(rt))                   \
  V(Ldrsw, Register&, rt, LDRSW_x)

#define LSPAIR_MACRO_LIST(V)                             \
  V(Ldp, CPURegister&, rt, rt2, LoadPairOpFor(rt, rt2))  \
  V(Stp, CPURegister&, rt, rt2, StorePairOpFor(rt, rt2)) \
  V(Ldpsw, CPURegister&, rt, rt2, LDPSW_x)

#define LDA_STL_MACRO_LIST(V) \
  V(Ldarb, ldarb)             \
  V(Ldarh, ldarh)             \
  V(Ldar, ldar)               \
  V(Ldaxrb, ldaxrb)           \
  V(Ldaxrh, ldaxrh)           \
  V(Ldaxr, ldaxr)             \
  V(Stlrb, stlrb)             \
  V(Stlrh, stlrh)             \
  V(Stlr, stlr)

#define STLX_MACRO_LIST(V) \
  V(Stlxrb, stlxrb)        \
  V(Stlxrh, stlxrh)        \
  V(Stlxr, stlxr)

// ----------------------------------------------------------------------------
// Static helper functions

// Generate a MemOperand for loading a field from an object.
inline MemOperand FieldMemOperand(Register object, int offset) {
  return MemOperand(object, offset - kHeapObjectTag);
}

// ----------------------------------------------------------------------------
// MacroAssembler

enum BranchType {
  // Copies of architectural conditions.
  // The associated conditions can be used in place of those, the code will
  // take care of reinterpreting them with the correct type.
  integer_eq = eq,
  integer_ne = ne,
  integer_hs = hs,
  integer_lo = lo,
  integer_mi = mi,
  integer_pl = pl,
  integer_vs = vs,
  integer_vc = vc,
  integer_hi = hi,
  integer_ls = ls,
  integer_ge = ge,
  integer_lt = lt,
  integer_gt = gt,
  integer_le = le,
  integer_al = al,
  integer_nv = nv,

  // These two are *different* from the architectural codes al and nv.
  // 'always' is used to generate unconditional branches.
  // 'never' is used to not generate a branch (generally as the inverse
  // branch type of 'always).
  always,
  never,
  // cbz and cbnz
  reg_zero,
  reg_not_zero,
  // tbz and tbnz
  reg_bit_clear,
  reg_bit_set,

  // Aliases.
  kBranchTypeFirstCondition = eq,
  kBranchTypeLastCondition = nv,
  kBranchTypeFirstUsingReg = reg_zero,
  kBranchTypeFirstUsingBit = reg_bit_clear
};

inline BranchType InvertBranchType(BranchType type) {
  if (kBranchTypeFirstCondition <= type && type <= kBranchTypeLastCondition) {
    return static_cast<BranchType>(
        NegateCondition(static_cast<Condition>(type)));
  } else {
    return static_cast<BranchType>(type ^ 1);
  }
}

enum SmiCheck { INLINE_SMI_CHECK, OMIT_SMI_CHECK };
enum RAStatus { kRAHasNotBeenSaved, kRAHasBeenSaved };
enum TargetAddressStorageMode {
  CAN_INLINE_TARGET_ADDRESS,
  NEVER_INLINE_TARGET_ADDRESS
};
enum DiscardMoveMode { kDontDiscardForSameWReg, kDiscardForSameWReg };

// The macro assembler supports moving automatically pre-shifted immediates for
// arithmetic and logical instructions, and then applying a post shift in the
// instruction to undo the modification, in order to reduce the code emitted for
// an operation. For example:
//
//  Add(x0, x0, 0x1f7de) => movz x16, 0xfbef; add x0, x0, x16, lsl #1.
//
// This optimisation can be only partially applied when the stack pointer is an
// operand or destination, so this enumeration is used to control the shift.
enum PreShiftImmMode {
  kNoShift,          // Don't pre-shift.
  kLimitShiftForSP,  // Limit pre-shift for add/sub extend use.
  kAnyShift          // Allow any pre-shift.
};

class V8_EXPORT_PRIVATE TurboAssembler : public TurboAssemblerBase {
 public:
  using TurboAssemblerBase::TurboAssemblerBase;

  // We should not use near calls or jumps for calls to external references,
  // since the code spaces are not guaranteed to be close to each other.
  bool CanUseNearCallOrJump(RelocInfo::Mode rmode) {
    return rmode != RelocInfo::EXTERNAL_REFERENCE;
  }

  static bool IsNearCallOffset(int64_t offset);

  // Arguments macros.
#define COND_TYPED_ARGS Condition cond, Register r1, const Operand &r2
#define COND_ARGS cond, r1, r2

  // Cases when relocation is not needed.
#define DECLARE_NORELOC_PROTOTYPE(Name, target_type)                      \
  void Name(target_type target);                                          \
  void Name(target_type target, COND_TYPED_ARGS);

#define DECLARE_BRANCH_PROTOTYPES(Name)   \
  DECLARE_NORELOC_PROTOTYPE(Name, Label*) \
  DECLARE_NORELOC_PROTOTYPE(Name, int32_t)

  DECLARE_BRANCH_PROTOTYPES(Branch)
  DECLARE_BRANCH_PROTOTYPES(BranchAndLink)
  DECLARE_BRANCH_PROTOTYPES(BranchShort)

#undef DECLARE_BRANCH_PROTOTYPES
#undef COND_TYPED_ARGS
#undef COND_ARGS
  // Activation support.
  void EnterFrame(StackFrame::Type type);
  void EnterFrame(StackFrame::Type type, bool load_constant_pool_pointer_reg) {
    // Out-of-line constant pool not implemented on riscv.
    UNREACHABLE();
  }
  void LeaveFrame(StackFrame::Type type);
  // Generates function and stub prologue code.
  void StubPrologue(StackFrame::Type type);
  void Prologue();

  // -------------------------------------------------------------------------
  // Smi utilities.
  void SmiTag(Register dst, Register src);

  void SmiTag(Register reg) { SmiTag(reg, reg); }

  void SmiUntag(Register dst, const MemOperand& src);

  void SmiUntag(Register dst, Register src);

  void SmiUntag(Register reg) { SmiUntag(reg, reg); }

  // Removes current frame and its arguments from the stack preserving
  // the arguments and a return address pushed to the stack for the next call.
  // Both |callee_args_count| and |caller_args_count| do not include
  // receiver. |callee_args_count| is not modified. |caller_args_count|
  // is trashed.
  void PrepareForTailCall(Register callee_args_count,
                          Register caller_args_count, Register scratch0,
                          Register scratch1);

// jump, Call, and Ret pseudo instructions implementing inter-working.
#define COND_ARGS \
  Condition cond = al, Register rs = x0, const Operand &rt = Operand(x0)
  void Branch(Label* L, Condition cond, Register rs, RootIndex index);
  void BranchLong(Label* L);
  void Jump(Register target, COND_ARGS);
  void Jump(Label* target, COND_ARGS);
  void Jump(const ExternalReference& reference) override;

  void Jump(Handle<Code> code, RelocInfo::Mode rmode, COND_ARGS);
  void Jump(intptr_t target, RelocInfo::Mode rmode, COND_ARGS);
  void Jump(Address target, RelocInfo::Mode rmode, COND_ARGS);

  template <typename T>
  inline void Jump(Register target, Condition cond, Register rs, const T& rt) {
    Jump(target, cond, rs, Operand(rt));
  }

  template <typename T>
  inline void Jump(Label* target, Condition cond, Register rs, const T& rt) {
    Jump(target, cond, rs, Operand(rt));
  }

  template <typename T>
  inline void JumpIfEqual(Register a, T b, Label* dest) {
    Jump(dest, eq, a, Operand(b));
  }

  template <typename T>
  inline void JumpIfLessThan(Register a, T b, Label* dest) {
    Jump(dest, lt, a, Operand(b));
  }

  void Ret(COND_ARGS);
  // Calls Abort(msg) if the condition cc is not satisfied.
  // Use --debug_code to enable.
  void Assert(Condition cc, AbortReason reason, Register rs, Operand rt);

  void Abort(AbortReason msg);

  // Like Assert(), but always enabled.
  void Check(Condition cc, AbortReason reason, Register rs, Operand rt);

  void LoadFromConstantsTable(Register destination,
                              int constant_index) override;
  void LoadRootRegisterOffset(Register destination, intptr_t offset) override;
  void LoadRootRelative(Register destination, int32_t offset) override;

  void LoadCodeObjectEntry(Register destination, Register code_object) override;
  void CallBuiltinByIndex(Register builtin_index) override;
  void CallCodeObject(Register code_object) override;
  void JumpCodeObject(Register code_object) override;
  void LoadRoot(Register destination, RootIndex index) override;
  void LoadRoot(Register destination, RootIndex index, Condition cond,
                Register src1, const Operand& src2);
  void Trap() override;
  void DebugBreak() override;
  void CallForDeoptimization(Address target, int deopt_id, Label* exit,
                             DeoptimizeKind kind);

  void jmp(Label* L);
  void jmp(Register x) {}
  void InitializeRootRegister() {}

  void ResetSpeculationPoisonRegister();
  // Compute the start of the generated instruction stream from the current PC.
  // This is an alternative to embedding the {CodeObject} handle as a reference.
  void ComputeCodeStartAddress(Register dst);
  bool IsDoubleZeroRegSet() { return has_double_zero_reg_set_; }
  void Move(Register dst, Smi source);
  void mov(Register rd, Register rt);
  void Move(Register dst, Register src);
  void Move(Register dst, int64_t src);
  void Move(Register dst, ExternalReference src);

  void Fmove(FPURegister fd, Register rn);
  void Fmove(Register rd, FPURegister fn);
  // floating point reg to floating point reg
  void Fmovs(FPURegister fd, FPURegister fs);
  void Fmovd(FPURegister fd, FPURegister fs);
  // Provide explicit double and float interfaces for FP immediate moves, rather
  // than relying on implicit C++ casts. This allows signalling NaNs to be
  // preserved when the immediate matches the format of fd. Most systems convert
  // signalling NaNs to quiet NaNs when converting between float and double.
  void Fmove(FPURegister fd, double imm);
  void Fmove(FPURegister fd, float imm);
  // Provide a template to allow other types to

  void Call(Register target, COND_ARGS);
  void Call(Address target, RelocInfo::Mode rmode, COND_ARGS);
  void Call(Handle<Code> code, RelocInfo::Mode rmode = RelocInfo::CODE_TARGET,
            COND_ARGS);
  void Call(Label* target);

  void Lb(Register rd, const MemOperand& rs);
  void Lh(Register rd, const MemOperand& rs);
  void Lw(Register rd, const MemOperand& rs);
  void Lbu(Register rd, const MemOperand& rs);
  void Lhu(Register rd, const MemOperand& rs);
  void Sb(Register rd, const MemOperand& rs);
  void Sh(Register rd, const MemOperand& rs);
  void Sw(Register rd, const MemOperand& rs);
  void Lwu(Register rd, const MemOperand& rs);
  void Ld(Register rd, const MemOperand& rs);
  void Sd(Register rd, const MemOperand& rs);

  void Flw(FPURegister fd, const MemOperand& src);
  void Fsw(FPURegister fs, const MemOperand& dst);

  void Fld(FPURegister fd, const MemOperand& src);
  void Fsd(FPURegister fs, const MemOperand& dst);

  void Float32Max(FPURegister dst, FPURegister src1, FPURegister src2,
                  Label* out_of_line);
  int CalculateStackPassedDWords(int num_reg_arguments,
                                 int num_double_arguments);
  void PrepareCallCFunction(int num_reg_arguments, int num_double_registers,
                            Register scratch);
  void PrepareCallCFunction(int num_reg_arguments, Register scratch);

  // Control-flow integrity:

  // Define a function entrypoint. This doesn't emit any code for this
  // architecture, as control-flow integrity is not supported for it.
  void CodeEntry() {}
  // Define an exception handler.
  void ExceptionHandler() {}
  // Define an exception handler and bind a label.
  void BindExceptionHandler(Label* label) { bind(label); }
#define DEFINE_INSTRUCTION(instr)                          \
  void instr(Register rd, Register rs, const Operand& rt); \
  void instr(Register rd, Register rs, Register rt) {      \
    instr(rd, rs, Operand(rt));                            \
  }                                                        \
  void instr(Register rs, Register rt, int32_t j) { instr(rs, rt, Operand(j)); }

#define DEFINE_INSTRUCTION2(instr)                                 \
  void instr(Register rs, const Operand& rt);                      \
  void instr(Register rs, Register rt) { instr(rs, Operand(rt)); } \
  void instr(Register rs, int32_t j) { instr(rs, Operand(j)); }

  DEFINE_INSTRUCTION(Add)
  DEFINE_INSTRUCTION(Dadd)
  DEFINE_INSTRUCTION(Sub)
  DEFINE_INSTRUCTION(Dsub)
  DEFINE_INSTRUCTION(Mul)
  DEFINE_INSTRUCTION(Dmul)
  DEFINE_INSTRUCTION(Dmulh)
  DEFINE_INSTRUCTION(Dmulhu)
  DEFINE_INSTRUCTION2(Mult)
  DEFINE_INSTRUCTION2(Dmult)
  DEFINE_INSTRUCTION2(Multu)
  DEFINE_INSTRUCTION2(Dmultu)
  DEFINE_INSTRUCTION(Div)
  DEFINE_INSTRUCTION(Divu)
  DEFINE_INSTRUCTION(Ddiv)
  DEFINE_INSTRUCTION(Ddivu)
  DEFINE_INSTRUCTION(Mod)
  DEFINE_INSTRUCTION(Modu)
  DEFINE_INSTRUCTION(Dmod)
  DEFINE_INSTRUCTION(Dmodu)

  DEFINE_INSTRUCTION(And)
  DEFINE_INSTRUCTION(Or)
  DEFINE_INSTRUCTION(Xor)
  DEFINE_INSTRUCTION(Nor)
  DEFINE_INSTRUCTION2(Neg)
  DEFINE_INSTRUCTION(Slt)
  DEFINE_INSTRUCTION(Sltu)
  DEFINE_INSTRUCTION(Sle)
  DEFINE_INSTRUCTION(Sleu)
  DEFINE_INSTRUCTION(Sgt)
  DEFINE_INSTRUCTION(Sgtu)
  DEFINE_INSTRUCTION(Sge)
  DEFINE_INSTRUCTION(Sgeu)

  DEFINE_INSTRUCTION(Ror)
  DEFINE_INSTRUCTION(Dror)
#undef DEFINE_INSTRUCTION
#undef DEFINE_INSTRUCTION2
  // Load int32 in the rd register.
  void li(Register rd, Operand j, LiFlags mode = OPTIMIZE_SIZE);
  inline void li(Register rd, int64_t j, LiFlags mode = OPTIMIZE_SIZE) {
    li(rd, Operand(j), mode);
  }
  // inline void li(Register rd, int32_t j, LiFlags mode = OPTIMIZE_SIZE) {
  //   li(rd, Operand(static_cast<int64_t>(j)), mode);
  // }
  void li(Register dst, Handle<HeapObject> value, LiFlags mode = OPTIMIZE_SIZE);
  void li(Register dst, ExternalReference value, LiFlags mode = OPTIMIZE_SIZE);
  void li(Register dst, const StringConstantBase* string,
          LiFlags mode = OPTIMIZE_SIZE);
  void Movz(Register rd, Register rs, Register rt);
  inline void Move_d(FPURegister dst, FPURegister src) {
    if (dst != src) {
      fsgnjd(dst, src, src);
    }
  }

  inline void Move_s(FPURegister dst, FPURegister src) {
    if (dst != src) {
      fsgnjs(dst, src, src);
    }
  }

  void push(Register src) {
    Dadd(sp, sp, Operand(-kPointerSize));
    Sd(src, MemOperand(sp, 0));
  }
  void Push(Register src) { push(src); }
  void Push(Handle<HeapObject> handle);
  void Push(Smi smi);

  // Push two registers. Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2) {
    Dsub(sp, sp, Operand(2 * kPointerSize));
    Sd(src1, MemOperand(sp, 1 * kPointerSize));
    Sd(src2, MemOperand(sp, 0 * kPointerSize));
  }

  // Push three registers. Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2, Register src3) {
    Dsub(sp, sp, Operand(3 * kPointerSize));
    Sd(src1, MemOperand(sp, 2 * kPointerSize));
    Sd(src2, MemOperand(sp, 1 * kPointerSize));
    Sd(src3, MemOperand(sp, 0 * kPointerSize));
  }

  // Push four registers. Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2, Register src3, Register src4) {
    Dsub(sp, sp, Operand(4 * kPointerSize));
    Sd(src1, MemOperand(sp, 3 * kPointerSize));
    Sd(src2, MemOperand(sp, 2 * kPointerSize));
    Sd(src3, MemOperand(sp, 1 * kPointerSize));
    Sd(src4, MemOperand(sp, 0 * kPointerSize));
  }

  // Push five registers. Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2, Register src3, Register src4,
            Register src5) {
    Dsub(sp, sp, Operand(5 * kPointerSize));
    Sd(src1, MemOperand(sp, 4 * kPointerSize));
    Sd(src2, MemOperand(sp, 3 * kPointerSize));
    Sd(src3, MemOperand(sp, 2 * kPointerSize));
    Sd(src4, MemOperand(sp, 1 * kPointerSize));
    Sd(src5, MemOperand(sp, 0 * kPointerSize));
  }

  void Push(Register src, Condition cond, Register tst1, Register tst2) {
    // Since we don't have conditional execution we use a Branch.
    // Branch(3, cond, tst1, Operand(tst2)); //TODO
    Dsub(sp, sp, Operand(kPointerSize));
    Sd(src, MemOperand(sp, 0));
  }

  void pop(Register dst) {
    Ld(dst, MemOperand(sp, 0));
    Dadd(sp, sp, Operand(kPointerSize));
  }
  void Pop(Register dst) { pop(dst); }

  // Pop two registers. Pops rightmost register first (from lower address).
  void Pop(Register src1, Register src2) {
    DCHECK(src1 != src2);
    Ld(src2, MemOperand(sp, 0 * kPointerSize));
    Ld(src1, MemOperand(sp, 1 * kPointerSize));
    Dadd(sp, sp, 2 * kPointerSize);
  }

  // Pop three registers. Pops rightmost register first (from lower address).
  void Pop(Register src1, Register src2, Register src3) {
    Ld(src3, MemOperand(sp, 0 * kPointerSize));
    Ld(src2, MemOperand(sp, 1 * kPointerSize));
    Ld(src1, MemOperand(sp, 2 * kPointerSize));
    Dadd(sp, sp, 3 * kPointerSize);
  }

  void Pop(Register src1, Register src2, Register src3, Register src4) {
    Ld(src4, MemOperand(sp, 0 * kPointerSize));
    Ld(src3, MemOperand(sp, 1 * kPointerSize));
    Ld(src2, MemOperand(sp, 2 * kPointerSize));
    Ld(src1, MemOperand(sp, 3 * kPointerSize));
    Dadd(sp, sp, 4 * kPointerSize);
  }

  void Pop(uint32_t count = 1) { Dadd(sp, sp, Operand(count * kPointerSize)); }
  void PushStandardFrame(Register function_reg);
  // Push a fixed frame, consisting of ra, fp.
  void PushCommonFrame(Register marker_reg = no_reg);
  void CheckPageFlag(Register object, Register scratch, int mask, Condition cc,
                     Label* condition_met);

  void CallRecordWriteStub(Register object, Operand address,
                           RememberedSetAction remembered_set_action,
                           SaveFPRegsMode fp_mode);
  void CallRecordWriteStub(Register object, Operand address,
                           RememberedSetAction remembered_set_action,
                           SaveFPRegsMode fp_mode, Address wasm_target);
  void CallRecordWriteStub(Register object, Operand address,
                           RememberedSetAction remembered_set_action,
                           SaveFPRegsMode fp_mode, Handle<Code> code_target,
                           Address wasm_target);
  void SaveRegisters(RegList registers);

  void RestoreRegisters(RegList registers);

  void CallEphemeronKeyBarrier(Register object, Register address,
                               SaveFPRegsMode fp_mode);

  // Calls a C function and cleans up the space for arguments allocated
  // by PrepareCallCFunction. The called function is not allowed to trigger a
  // garbage collection, since that might move the code and invalidate the
  // return address (unless this is somehow accounted for by the called
  // function).
  void CallCFunctionHelper(Register function, int num_reg_arguments,
                           int num_double_arguments);
  void CallCFunction(ExternalReference function, int num_arguments);
  void CallCFunction(ExternalReference function, int num_reg_arguments,
                     int num_double_arguments);
  void CallCFunction(Register function, int num_arguments);
  void CallCFunction(Register function, int num_reg_arguments,
                     int num_double_arguments);

  void StoreReturnAddressAndCall(Register target);

  // See comments at the beginning of Builtins::Generate_CEntry.
  inline void PrepareCEntryArgs(int num_args) { li(a0, num_args); }
  inline void PrepareCEntryFunction(const ExternalReference& ref) {
    li(a1, ref);
  }

  // from the stack, clobbering only the sp register.
  void Drop(int count, uint64_t unit_size = kSystemPointerSize);

  // Push caller saved registers on the stack, and return the number of bytes
  // stack pointer is adjusted.
  int PushCallerSaved(SaveFPRegsMode fp_mode, Register exclusion1 = no_reg,
                      Register exclusion2 = no_reg,
                      Register exclusion3 = no_reg);
  // Calculate how much stack space (in bytes) are required to store caller
  // registers excluding those specified in the arguments.
  int RequiredStackSizeForCallerSaved(SaveFPRegsMode fp_mode,
                                      Register exclusion1 = no_reg,
                                      Register exclusion2 = no_reg,
                                      Register exclusion3 = no_reg) const;

  void MultiPush(RegList regs);
  void MultiPushFPU(RegList regs);
  void MultiPop(RegList regs);
  void MultiPopFPU(RegList regs);

  void JumpIfSmi(Register value, Label* smi_label,
                 Label* not_smi_label = nullptr);
  void LoadEntryFromBuiltinIndex(Register builtin_index);

  void GenScaledAddress(Register rd, Register base, Register index,
                        int64_t left_shift_amount) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    slli(scratch, index, left_shift_amount);
    Dadd(rd, base, scratch);
  }

  void TryConvertDoubleToInt64(Register result, DoubleRegister double_input,
                               Label* done);

  void DecompressTaggedPointer(const Register& destination,
                               const MemOperand& field_operand);

  void DecompressTaggedPointer(const Register& destination,
                                const Register& source);

  void LoadTaggedPointerField(const Register& destination,
                              const MemOperand& field_operand);

 protected:
  inline Register GetRtAsRegisterHelper(const Operand& rt, Register scratch);
  inline int32_t GetOffset(int32_t offset, Label* L, OffsetSize bits);

 private:
  static const int kSmiShift = kSmiTagSize + kSmiShiftSize;
  bool has_double_zero_reg_set_ = false;
  void BranchShortHelper(int16_t offset, Label* L);
  bool BranchShortHelper(int16_t offset, Label* L, Condition cond, Register rs,
                         const Operand& rt);
  bool BranchShortCheck(int32_t offset, Label* L, Condition cond, Register rs,
                        const Operand& rt);
};

class V8_EXPORT_PRIVATE MacroAssembler : public TurboAssembler {
 public:
  using TurboAssembler::TurboAssembler;

#define DECLARE_FUNCTION(FN, OP) \
  inline void FN(const Register& rs, const Register& rt, const Register& rn);
  STLX_MACRO_LIST(DECLARE_FUNCTION)
#undef DECLARE_FUNCTION
  void LoadMap(Register destination, Register object);
  void Swap(Register reg1, Register reg2, Register scratch = no_reg);
  void TailCallRuntime(Runtime::FunctionId fid);
  void JumpToInstructionStream(Address entry);
  // Load the global proxy from the current context.
  void LoadGlobalProxy(Register dst) {
    LoadNativeContextSlot(Context::GLOBAL_PROXY_INDEX, dst);
  }
  void LoadNativeContextSlot(int index, Register dst);

  // Unlink the stack handler on top of the stack from the stack handler chain.
  // Must preserve the result register.
  void PopStackHandler();

  void PushCalleeSavedRegisters();
  void PopCalleeSavedRegisters();
  // ---------------------------------------------------------------------------
  // Runtime calls
  // Call a runtime routine.
  void CallRuntime(const Runtime::Function* f, int num_arguments,
                   SaveFPRegsMode save_doubles = kDontSaveFPRegs);

  // Convenience function: Same as above, but takes the fid instead.
  void CallRuntime(Runtime::FunctionId fid,
                   SaveFPRegsMode save_doubles = kDontSaveFPRegs) {
    const Runtime::Function* function = Runtime::FunctionForId(fid);
    CallRuntime(function, function->nargs, save_doubles);
  }

  // Convenience function: Same as above, but takes the fid instead.
  void CallRuntime(Runtime::FunctionId fid, int num_arguments,
                   SaveFPRegsMode save_doubles = kDontSaveFPRegs) {
    CallRuntime(Runtime::FunctionForId(fid), num_arguments, save_doubles);
  }

  // Jump to the builtin routine.
  void JumpToExternalReference(const ExternalReference& builtin,
                               bool builtin_exit_frame = false);

  void GetInstanceType(Register object, Register map, Register type_reg);

  void AssertFunction(Register object);
  void AssertConstructor(Register object);
  void AssertBoundFunction(Register object);

  void JumpIfRoot(const Register& obj, RootIndex index, Label* if_equal);
  void JumpIfNotRoot(const Register& obj, RootIndex index, Label* if_not_equal);

  void PushRoot(RootIndex index) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    LoadRoot(scratch, index);
    Push(scratch);
  }

  void InvokeFunctionCode(Register function, Register new_target,
                          Register expected_parameter_count,
                          Register actual_parameter_count, InvokeFlag flag);

  void LoadWeakValue(Register out, Register in, Label* target_if_cleared);

  void JumpIfNotSmi(Register value, Label* not_smi_label);

  template <typename Field>
  void DecodeField(Register dst, Register src) {
    DCHECK_LT(Field::kSize, 64);
    static const int shift = Field::kShift;
    static const uint64_t mask = (1UL << (64 - Field::kSize)) - 1;
    srli(dst, src, shift);
    And(dst, dst, Operand(mask));
  }

  template <typename Field>
  void DecodeField(Register reg) {
    DecodeField<Field>(reg, reg);
  }

  // ---------------------------------------------------------------------------
  // Garbage collector support (GC).

  // Notify the garbage collector that we wrote a pointer into an object.
  // |object| is the object being stored into, |value| is the object being
  // stored.
  // The offset is the offset from the start of the object, not the offset from
  // the tagged HeapObject pointer.  For use with FieldMemOperand(reg, off).
  void RecordWriteField(
      Register object, int offset, Register value, RAStatus ra_status,
      SaveFPRegsMode save_fp,
      RememberedSetAction remembered_set_action = EMIT_REMEMBERED_SET,
      SmiCheck smi_check = INLINE_SMI_CHECK);

  // For a given |object| notify the garbage collector that the slot at |offset|
  // has been written. |value| is the object being stored.
  void RecordWrite(
      Register object, Operand offset, Register value, RAStatus ra_status,
      SaveFPRegsMode save_fp,
      RememberedSetAction remembered_set_action = EMIT_REMEMBERED_SET,
      SmiCheck smi_check = INLINE_SMI_CHECK);

  void AssertUndefinedOrAllocationSite(Register object, Register scratch);

  void AssertNotSmi(Register object);

  void EnterExitFrame(bool save_doubles, const Register& scratch,
                      int extra_space,
                      StackFrame::Type frame_type = StackFrame::EXIT);

  void LeaveExitFrame(bool restore_doubles, const Register& scratch,
                      const Register& scratch2, int extra_space);

  void JumpIfIsInRange(Register value, unsigned lower_limit,
                       unsigned higher_limit, Label* on_in_range);

  void IncrementCounter(StatsCounter* counter, int value, Register scratch1,
                        Register scratch2);

  void InvokeFunctionWithNewTarget(Register function, Register new_target,
                                   Register actual_parameter_count,
                                   InvokeFlag flag);

  void InvokePrologue(Register expected_parameter_count,
                      Register actual_parameter_count, Label* done,
                      InvokeFlag flag);

  void CheckDebugHook(Register fun, Register new_target,
                      Register expected_parameter_count,
                      Register actual_parameter_count);

  void AssertGeneratorObject(Register object);

  void MaybeDropFrames();

  void InvokeFunction(Register function, Register expected_parameter_count,
                      Register actual_parameter_count, InvokeFlag flag);
};
}  // namespace internal
}  // namespace v8

#define ACCESS_MASM(masm) masm->

#endif  // V8_CODEGEN_RISCV64_MACRO_ASSEMBLER_RISCV64_H_
