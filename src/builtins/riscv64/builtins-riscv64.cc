// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if V8_TARGET_ARCH_RISCV64

#include "src/api/api-arguments.h"
#include "src/codegen/code-factory.h"
#include "src/debug/debug.h"
#include "src/deoptimizer/deoptimizer.h"
#include "src/execution/frame-constants.h"
#include "src/execution/frames.h"
#include "src/logging/counters.h"
// For interpreter_entry_return_pc_offset. TODO(jkummerow): Drop.
#include "src/codegen/macro-assembler-inl.h"
#include "src/codegen/register-configuration.h"
#include "src/heap/heap-inl.h"
#include "src/objects/cell.h"
#include "src/objects/foreign.h"
#include "src/objects/heap-number.h"
#include "src/objects/js-generator.h"
#include "src/objects/objects-inl.h"
#include "src/objects/smi.h"
#include "src/runtime/runtime.h"
#include "src/wasm/wasm-objects.h"

#if defined(V8_OS_WIN)
#include "src/diagnostics/unwinding-info-win64.h"
#endif  // V8_OS_WIN

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)

void Generate_StackOverflowCheck(MacroAssembler* masm, Register num_args,
                                 Label* stack_overflow);

static void GenerateTailCallToReturnedCode(MacroAssembler* masm,
                                           Runtime::FunctionId function_id) {
  // ----------- S t a t e -------------
  //  -- a1 : target function (preserved for callee)
  //  -- a3 : new target (preserved for callee)
  // -----------------------------------
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    // Push a copy of the function onto the stack.
    // Push a copy of the target function and the new target.
    __ Push(a1, a3, a1);

    __ CallRuntime(function_id, 1);
    __ Move(a2, a0);
    // Restore target function and new target.
    __ Pop(a1, a3);
  }
  static_assert(kJavaScriptCallCodeStartRegister == a2, "ABI mismatch");
  __ JumpCodeObject(a2);
}

static void ReplaceClosureCodeWithOptimizedCode(MacroAssembler* masm,
                                                Register optimized_code,
                                                Register closure,
                                                Register scratch1) {
  // Store code entry in the closure.
  __ Sd(optimized_code, FieldMemOperand(closure, JSFunction::kCodeOffset));
  __ mov(scratch1, optimized_code);  // Write barrier clobbers scratch1 below.
  __ RecordWriteField(closure, JSFunction::kCodeOffset, scratch1,
                      kRAHasNotBeenSaved, kDontSaveFPRegs, OMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
}

static void TailCallOptimizedCodeSlot(MacroAssembler* masm,
                                      Register optimized_code_entry,
                                      Register scratch1, Register scratch2) {
  // ----------- S t a t e -------------
  //  -- a3 : new target (preserved for callee if needed, and caller)
  //  -- a1 : target function (preserved for callee if needed, and caller)
  // -----------------------------------
  DCHECK(!AreAliased(optimized_code_entry, a1, a3, scratch1, scratch2));

  Register closure = a1;

  // Check if the optimized code is marked for deopt. If it is, call the
  // runtime to clear it.
  Label found_deoptimized_code;
  __ Ld(a5,
        FieldMemOperand(optimized_code_entry, Code::kCodeDataContainerOffset));
  __ Lw(a5, FieldMemOperand(a5, CodeDataContainer::kKindSpecificFlagsOffset));
  __ And(a5, a5, Operand(1 << Code::kMarkedForDeoptimizationBit));
  __ Jump(&found_deoptimized_code, ne, a5, Operand(zero_reg));

  // Optimized code is good, get it into the closure and link the closure into
  // the optimized functions list, then tail call the optimized code.
  // The feedback vector is no longer used, so re-use it as a scratch
  // register.
  ReplaceClosureCodeWithOptimizedCode(masm, optimized_code_entry, closure,
                                      scratch1);

  static_assert(kJavaScriptCallCodeStartRegister == a2, "ABI mismatch");
  __ Dadd(a2, optimized_code_entry,
          Operand(Code::kHeaderSize - kHeapObjectTag));
  __ Jump(a2);

  // Optimized code slot contains deoptimized code, evict it and re-enter the
  // closure's code.
  __ bind(&found_deoptimized_code);
  GenerateTailCallToReturnedCode(masm, Runtime::kEvictOptimizedCodeSlot);
}

void Builtins::Generate_Adaptor(MacroAssembler* masm, Address address) {
  __ CodeEntry();

  __ li(kJavaScriptCallExtraArg1Register, ExternalReference::Create(address));
  __ Jump(BUILTIN_CODE(masm->isolate(), AdaptorWithBuiltinExitFrame),
          RelocInfo::CODE_TARGET);
}

// The construct stub for ES5 constructor functions and ES6 class constructors.
void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0     : number of arguments
  //  -- a1     : constructor function
  //  -- a3     : new target
  //  -- lr     : return address
  //  -- cp     : context pointer
  //  -- sp[...]: constructor arguments
  // -----------------------------------

  ASM_LOCATION("Builtins::Generate_JSConstructStubGeneric");

  // Enter a construct frame.
  {
    FrameScope scope(masm, StackFrame::CONSTRUCT);
    Label post_instantiation_deopt_entry, not_create_implicit_receiver;

    if (__ emit_debug_code()) {
      // Check that FrameScope pushed the context on to the stack already.
      __ Ld(t0, MemOperand(sp, 0));
      Label not_equal;
      __ Jump(&not_equal, ne, t0, cp);
      __ Abort(AbortReason::kUnexpectedValue);
      __ bind(&not_equal);
    }

    // Preserve the incoming parameters on the stack.
    __ SmiTag(a0);
    __ Push(a0, a1, zero_reg, a3);

    // ----------- S t a t e -------------
    //  --        sp[0*kSystemPointerSize]: new target
    //  --        sp[1*kSystemPointerSize]: padding
    //  -- a1 and sp[2*kSystemPointerSize]: constructor function
    //  --        sp[3*kSystemPointerSize]: number of arguments (tagged)
    //  --        sp[4*kSystemPointerSize]: context (pushed by FrameScope)
    // -----------------------------------

    __ Ld(a4, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
    __ Lwu(a4, FieldMemOperand(a4, SharedFunctionInfo::kFlagsOffset));
    __ DecodeField<SharedFunctionInfo::FunctionKindBits>(a4);
    __ JumpIfIsInRange(a4, kDefaultDerivedConstructor, kDerivedConstructor,
                       &not_create_implicit_receiver);

    // If not derived class constructor: Allocate the new receiver object.
    __ IncrementCounter(masm->isolate()->counters()->constructed_objects(), 1,
                        a4, a5);
    __ Call(BUILTIN_CODE(masm->isolate(), FastNewObject),
            RelocInfo::CODE_TARGET);
    __ Jump(&post_instantiation_deopt_entry);

    // Else: use TheHoleValue as receiver for constructor call
    __ bind(&not_create_implicit_receiver);
    __ LoadRoot(a0, RootIndex::kTheHoleValue);

    // ----------- S t a t e -------------
    //  -- a0: receiver
    //  -- Slot 4 / sp[0*kSystemPointerSize]: new target
    //  -- Slot 3 / sp[1*kSystemPointerSize]: padding
    //  -- Slot 2 / sp[2*kSystemPointerSize]: constructor function
    //  -- Slot 1 / sp[3*kSystemPointerSize]: number of arguments (tagged)
    //  -- Slot 0 / sp[4*kSystemPointerSize]: context
    // -----------------------------------
    // Deoptimizer enters here.
    masm->isolate()->heap()->SetConstructStubCreateDeoptPCOffset(
        masm->pc_offset());

    __ bind(&post_instantiation_deopt_entry);

    // Restore new target from the top of the stack.
    __ Pop(a3);

    // Restore constructor function and argument count.
    __ Ld(a1, MemOperand(fp, ConstructFrameConstants::kConstructorOffset));
    __ SmiUntag(s2, MemOperand(fp, ConstructFrameConstants::kLengthOffset));

    // Copy arguments to the expression stack. The called function pops the
    // receiver along with its arguments, so we need an extra receiver on the
    // stack, in case we have to return it later.

    // Overwrite the new target with a receiver.
    __ Push(a0);

    // Push two further copies of the receiver. One will be popped by the called
    // function. The second acts as padding if the number of arguments plus
    // receiver is odd - pushing receiver twice avoids Jumping. It also means
    // that we don't have to handle the even and odd cases specially on
    // InvokeFunction's return, as top of stack will be the receiver in either
    // case.
    __ Push(a0, a0);

    // ----------- S t a t e -------------
    //  --                              a0: receiver
    //  --                              a3: new target
    //  --                              s2: number of arguments (untagged)
    //  --        sp[0*kSystemPointerSize]: implicit receiver (overwrite if argc
    //  odd)
    //  --        sp[1*kSystemPointerSize]: implicit receiver
    //  --        sp[2*kSystemPointerSize]: implicit receiver
    //  --        sp[3*kSystemPointerSize]: padding
    //  -- a1 and sp[4*kSystemPointerSize]: constructor function
    //  --        sp[5*kSystemPointerSize]: number of arguments (tagged)
    //  --        sp[6*kSystemPointerSize]: context
    // -----------------------------------

    // Round the number of arguments down to the next even number, and claim
    // slots for the arguments. If the number of arguments was odd, the last
    // argument will overwrite one of the receivers pushed above.
    __ And(t0, s2, ~((int64_t)(1)));

    // Check if we have enough stack space to push all arguments.
    Label enough_stack_space, stack_overflow;
    Generate_StackOverflowCheck(masm, t0, &stack_overflow);
    __ Jump(&enough_stack_space);

    __ bind(&stack_overflow);
    // Restore the context from the frame.
    __ Ld(cp, MemOperand(fp, ConstructFrameConstants::kContextOffset));
    __ CallRuntime(Runtime::kThrowStackOverflow);
    __ DebugBreak();

    __ bind(&enough_stack_space);
    __ slli(t0, t0, kPointerSizeLog2);
    __ Dsub(sp, sp, t0);

    // Set up pointer to last argument.
    __ Dadd(t2, fp, Operand(StandardFrameConstants::kCallerSPOffset));
    Label loop, entry;
    // Copy the arguments.
    {
      __ Move(t0, s2);
      __ Jump(&entry);
      __ bind(&loop);
      __ GenScaledAddress(t2, t2, t0, kPointerSizeLog2);
      __ Ld(t1, MemOperand(t2));
      __ slli(t4, t0, kPointerSizeLog2);
      __ Dadd(t4, sp, t4);
      __ Sd(t1, MemOperand(t4));
      __ bind(&entry);
      __ Dsub(t0, t0, Operand(1));
      __ Jump(&loop, ge, t0, zero_reg);
    }

    // Call the function.
    __ Move(a0, s2);
    __ InvokeFunctionWithNewTarget(a1, a3, a0, CALL_FUNCTION);

    // ----------- S t a t e -------------
    //  -- sp[0*kSystemPointerSize]: implicit receiver
    //  -- sp[1*kSystemPointerSize]: padding
    //  -- sp[2*kSystemPointerSize]: constructor function
    //  -- sp[3*kSystemPointerSize]: number of arguments
    //  -- sp[4*kSystemPointerSize]: context
    // -----------------------------------

    // Store offset of return address for deoptimizer.
    masm->isolate()->heap()->SetConstructStubInvokeDeoptPCOffset(
        masm->pc_offset());

    // Restore the context from the frame.
    __ Ld(cp, MemOperand(fp, ConstructFrameConstants::kContextOffset));

    // If the result is an object (in the ECMA sense), we should get rid
    // of the receiver and use the result; see ECMA-262 section 13.2.2-7
    // on page 74.
    Label use_receiver, do_throw, leave_frame;

    // If the result is undefined, we jump out to using the implicit receiver.
    __ Jump(&use_receiver, eq, a0, (int64_t)RootIndex::kUndefinedValue);

    // Otherwise we do a smi check and fall through to check if the return value
    // is a valid receiver.

    // If the result is a smi, it is *not* an object in the ECMA sense.
    __ JumpIfSmi(a0, &use_receiver);

    // If the type of the result (stored in its map) is less than
    // FIRST_JS_RECEIVER_TYPE, it is not an object in the ECMA sense.
    STATIC_ASSERT(LAST_JS_RECEIVER_TYPE == LAST_TYPE);
    __ GetInstanceType(a0, t2, t2);
    __ Jump(&leave_frame, ge, t2, Operand(FIRST_JS_RECEIVER_TYPE));
    __ Jump(&use_receiver);

    __ bind(&do_throw);
    __ CallRuntime(Runtime::kThrowConstructorReturnedNonObject);

    // Throw away the result of the constructor invocation and use the
    // on-stack receiver as the result.
    __ bind(&use_receiver);
    __ Ld(x0, MemOperand(sp));
    // CompareRoot
    __ LoadRoot(t0, RootIndex::kTheHoleValue);
    __ Jump(&do_throw, eq, x0, t0);

    __ bind(&leave_frame);
    // Restore smi-tagged arguments count from the frame.
    __ SmiUntag(a1, MemOperand(fp, ConstructFrameConstants::kLengthOffset));
    // Leave construct frame.
  }
  // Remove caller arguments from the stack and return.
  __ And(a1, a1, ~((int64_t)(1)));  // If args number is odd, the last arg
                                    // overwriting the last receiver.
  __ slli(a1, a1, kXRegSizeLog2);
  __ Dadd(sp, sp, a1);
  __ Dadd(sp, sp, 2 * kXRegSize);  // Retain a recvier on stack.

  // ----------- S t a t e -------------
  //  -- sp[0*kSystemPointerSize]: implicit receiver
  //  -- sp[1*kSystemPointerSize]: padding
  //  -- sp[2*kSystemPointerSize]: constructor function
  //  -- sp[3*kSystemPointerSize]: number of arguments
  //  -- sp[4*kSystemPointerSize]: context
  // -----------------------------------
  __ Ret();
}

void Generate_JSBuiltinsConstructStubHelper(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0     : number of arguments
  //  -- a1     : constructor function
  //  -- a3     : new target
  //  -- cp     : context
  //  -- ra     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------

  // Enter a construct frame.
  {
    FrameScope scope(masm, StackFrame::CONSTRUCT);

    // Preserve the incoming parameters on the stack.
    __ SmiTag(a0);
    __ Push(cp, a0);
    __ SmiUntag(a0);

    // The receiver for the builtin/api call.
    __ PushRoot(RootIndex::kTheHoleValue);

    // Set up pointer to last argument.
    __ Dadd(t2, fp, Operand(StandardFrameConstants::kCallerSPOffset));

    // Copy arguments and receiver to the expression stack.
    Label loop, entry;
    __ mov(t3, a0);
    // ----------- S t a t e -------------
    //  --                        a0: number of arguments (untagged)
    //  --                        a3: new target
    //  --                        t2: pointer to last argument
    //  --                        t3: counter
    //  --        sp[0*kPointerSize]: the hole (receiver)
    //  --        sp[1*kPointerSize]: number of arguments (tagged)
    //  --        sp[2*kPointerSize]: context
    // -----------------------------------
    __ jmp(&entry);
    __ bind(&loop);
    __ GenScaledAddress(t0, t2, t3, kPointerSizeLog2);
    __ Ld(t1, MemOperand(t0));
    __ push(t1);
    __ bind(&entry);
    __ Dadd(t3, t3, Operand(-1));
    __ Jump(&loop, ge, t3, Operand(zero_reg));

    // Call the function.
    // a0: number of arguments (untagged)
    // a1: constructor function
    // a3: new target
    __ InvokeFunctionWithNewTarget(a1, a3, a0, CALL_FUNCTION);

    // Restore context from the frame.
    __ Ld(cp, MemOperand(fp, ConstructFrameConstants::kContextOffset));
    // Restore smi-tagged arguments count from the frame.
    __ Ld(a1, MemOperand(fp, ConstructFrameConstants::kLengthOffset));
    // Leave construct frame.
  }

  // Remove caller arguments from the stack and return.
  __ SmiUntag(a4, a1);
  __ slli(a4, a4, kPointerSizeLog2);
  __ Dadd(sp, sp, a4);
  __ Dadd(sp, sp, kPointerSize);
  __ Ret();
}

void Builtins::Generate_JSBuiltinsConstructStub(MacroAssembler* masm) {
  Generate_JSBuiltinsConstructStubHelper(masm);
}
void Builtins::Generate_ConstructedNonConstructable(MacroAssembler* masm) {
  FrameScope scope(masm, StackFrame::INTERNAL);
}

namespace {

enum StackLimitKind { kInterruptStackLimit, kRealStackLimit };

void LoadStackLimit(MacroAssembler* masm, Register destination,
                    StackLimitKind kind) {
  DCHECK(masm->root_array_available());
  Isolate* isolate = masm->isolate();
  ExternalReference limit =
      kind == StackLimitKind::kRealStackLimit
          ? ExternalReference::address_of_real_jslimit(isolate)
          : ExternalReference::address_of_jslimit(isolate);
  DCHECK(TurboAssembler::IsAddressableThroughRootRegister(isolate, limit));

  intptr_t offset =
      TurboAssembler::RootRegisterOffsetForExternalReference(isolate, limit);
  __ Ld(destination, MemOperand(kRootRegister, offset));
}

// Called with the native C calling convention. The corresponding function
// signature is either:
//
//   using JSEntryFunction = GeneratedCode<Address(
//       Address root_register_value, Address new_target, Address target,
//       Address receiver, intptr_t argc, Address** argv)>;
// or
//   using JSEntryFunction = GeneratedCode<Address(
//       Address root_register_value, MicrotaskQueue* microtask_queue)>;
//
// Input is either:
//   a0: root_register_value.
//   a1: new_target.
//   a2: target.
//   a3: receiver.
//   a4: argc.
//   a5: argv.
// or
//   a0: root_register_value.
//   a1: microtask_queue.
// Output:
//   a0: result.
void Generate_JSEntryVariant(MacroAssembler* masm, StackFrame::Type type,
                             Builtins::Name entry_trampoline) {
  {
    NoRootArrayScope no_root_array(masm);
    // save ra fp callee save register  x27~x18 x9 fs0~fs11
    __ PushCalleeSavedRegisters();
    // Set up the reserved register for 0.0.
    __ Fmove(kDoubleRegZero, 0.0);
    // Initialize the root register.
    __ Move(kRootRegister, a0);
  }
  // Build an entry frame (see layout below).
  int64_t bad_frame_pointer = -1L;  // Bad frame pointer to fail if it is used.
  __ Move(t0, bad_frame_pointer);
  __ Move(t1, StackFrame::TypeToMarker(type));
  __ Move(t2, ExternalReference::Create(IsolateAddressId::kCEntryFPAddress,
                                        masm->isolate()));
  __ Ld(t3, MemOperand(t2));
  __ Push(t0, t1, x0, t3);

  __ Dsub(fp, sp, EntryFrameConstants::kCallerFPOffset);

  ExternalReference js_entry_sp = ExternalReference::Create(
      IsolateAddressId::kJSEntrySPAddress, masm->isolate());
  __ Move(t0, js_entry_sp);
  __ Ld(t1, MemOperand(t0));

  DCHECK_EQ(StackFrame::INNER_JSENTRY_FRAME, 0);
  Label done, invoke, exit, handler_entry;
  __ Move(t2, x0);
  __ Jump(&done, ne, t1, 0);  // If t1 is zero, this is the outermost frame.
  __ Move(t2, StackFrame::OUTERMOST_JSENTRY_FRAME);
  __ Sd(fp, MemOperand(t0));

  __ bind(&done);
  __ Push(t2, x0);
  // Finish an entry frame
  // sp[0] : 0
  // sp[1] : JS entry frame marker.
  // sp[2] : C entry fp
  // sp[3] ? stack frame marker.
  // sp[4] : stack frame marker
  // sp[5] : bad frame pointer 0xFFF...FF   <- fp points here

  __ jmp(&invoke);
  // Jump to a faked try block that does the invoke, with a faked catch
  // block that sets the pending exception.

  // Prevent the constant pool from being emitted between the record of the
  // handler_entry position and the first instruction of the sequence here.
  // There is no risk because Assembler::Emit() emits the instruction before
  // checking for constant pool emission, but we do not want to depend on
  // that.
  {
    __ bind(&handler_entry);

    // Store the current pc as the handler offset. It's used later to create the
    // handler table.
    masm->isolate()->builtins()->SetJSEntryHandlerOffset(handler_entry.pos());

    // Caught exception: Store result (exception) in the pending exception
    // field in the JSEnv and return a failure sentinel.  Coming in here the
    // fp will be invalid because the PushStackHandler below sets it to 0 to
    // signal the existence of the JSEntry frame.
    __ li(s1, ExternalReference::Create(
                  IsolateAddressId::kPendingExceptionAddress, masm->isolate()));
    __ Sd(t0, MemOperand(s1));  // We come back from 'invoke'. result is in v0.
    __ LoadRoot(t0, RootIndex::kException);
    __ jmp(&exit);  // b exposes Jump delay slot.
    __ nop();       // Jump delay slot nop.
  }
  __ bind(&invoke);
  // Push new stack handler.
  static_assert(StackHandlerConstants::kSize == 2 * kSystemPointerSize,
                "Unexpected offset for StackHandlerConstants::kSize");
  static_assert(StackHandlerConstants::kNextOffset == 0 * kSystemPointerSize,
                "Unexpected offset for StackHandlerConstants::kNextOffset");
  __ Move(t1, ExternalReference::Create(IsolateAddressId::kHandlerAddress,
                                        masm->isolate()));
  __ Ld(t0, MemOperand(t1));
  __ Push(x0, t0);

  __ Sd(sp, MemOperand(t1));

  Handle<Code> trampoline_code =
      masm->isolate()->builtins()->builtin_handle(entry_trampoline);
  __ Call(trampoline_code, RelocInfo::CODE_TARGET);

  __ Pop(t0, x0);
  __ Move(t1, ExternalReference::Create(IsolateAddressId::kHandlerAddress,
                                        masm->isolate()));

  __ Drop(StackHandlerConstants::kSlotCount - 2);
  __ Sd(t0, MemOperand(t1));
  __ bind(&exit);  // a0 holds result

  // a0 holds the result.
  // The stack pointer points to the top of the entry frame pushed on entry from
  // C++ (at the beginning of this stub):
  // sp[0] : padding.
  // sp[1] : JS entry frame marker.
  // sp[2] : C entry FP.
  // sp[3] : stack frame marker.
  // sp[4] : stack frame marker.
  // sp[5] : bad frame pointer 0xFFF...FF   <- fp points here.

  // Check if the current stack frame is marked as the outermost JS frame.
  Label non_outermost_js_2;
  {
    Register c_entry_fp = t1;

    __ Ld(t0, MemOperand(sp, 1 * kXRegSizeLog2));
    __ Ld(c_entry_fp, MemOperand(sp, 2 * kXRegSizeLog2));
    __ Jump(&non_outermost_js_2, ne, t0, StackFrame::OUTERMOST_JSENTRY_FRAME);
    __ Move(t2, js_entry_sp);
    __ Sd(x0, MemOperand(t2));
    __ bind(&non_outermost_js_2);

    // Restore the top frame descriptors from the stack.
    __ Move(t2, ExternalReference::Create(IsolateAddressId::kCEntryFPAddress,
                                          masm->isolate()));
    __ Sd(c_entry_fp, MemOperand(t2));
  }

  static_assert(
      EntryFrameConstants::kFixedFrameSize % (2 * kSystemPointerSize) == 0,
      "Size of entry frame is not a multiple of 16 bytes");
  __ Drop(EntryFrameConstants::kFixedFrameSize / kSystemPointerSize);
  // Restore callee save registers
  __ PopCalleeSavedRegisters();
  __ Ret();
}
}  // namespace

void Builtins::Generate_JSEntry(MacroAssembler* masm) {
  Generate_JSEntryVariant(masm, StackFrame::ENTRY,
                          Builtins::kJSEntryTrampoline);
}

void Builtins::Generate_JSConstructEntry(MacroAssembler* masm) {
  Generate_JSEntryVariant(masm, StackFrame::CONSTRUCT_ENTRY,
                          Builtins::kJSConstructEntryTrampoline);
}

void Builtins::Generate_JSRunMicrotasksEntry(MacroAssembler* masm) {
  Generate_JSEntryVariant(masm, StackFrame::ENTRY,
                          Builtins::kRunMicrotasksTrampoline);
}

void Generate_StackOverflowCheck(MacroAssembler* masm, Register num_args,
                                 Label* stack_overflow) {
  UseScratchRegisterScope temps(masm);
  Register scratch = temps.Acquire();
  LoadStackLimit(masm, scratch, StackLimitKind::kRealStackLimit);
  __ Dsub(scratch, sp, scratch);
  __ slli(scratch, scratch, kPointerSizeLog2);
  __ Jump(stack_overflow, lt, scratch, num_args);
}

static void GetSharedFunctionInfoBytecode(MacroAssembler* masm,
                                          Register sfi_data,
                                          Register scratch1) {
  Label done;
  __ GetInstanceType(sfi_data, scratch1, scratch1);
  __ Jump(&done, ne, scratch1, Operand(INTERPRETER_DATA_TYPE));
  __ Ld(sfi_data,
        FieldMemOperand(sfi_data, InterpreterData::kBytecodeArrayOffset));

  __ bind(&done);
}

// Input:
//   a1: new.target.
//   a2: function.
//   a3: receiver.
//   a4: argc.
//   a5: argv.
// Output:
//   a0: result.
static void Generate_JSEntryTrampolineHelper(MacroAssembler* masm,
                                             bool is_construct) {
  Register new_target = a1;
  Register function = a2;
  Register receiver = a3;
  Register argc = a4;
  Register argv = a5;
  Register scratch = t0;
  Register slots_to_claim = t1;
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ Move(scratch, ExternalReference::Create(
                         IsolateAddressId::kContextAddress, masm->isolate()));
    __ Ld(cp, MemOperand(scratch));
    __ Add(slots_to_claim, argc, 3);
    __ And(slots_to_claim, slots_to_claim, ~0x01);

    Label enough_stack_space, stack_overflow;
    Generate_StackOverflowCheck(masm, slots_to_claim, &stack_overflow);
    __ jmp(&enough_stack_space);

    __ bind(&stack_overflow);
    __ CallRuntime(Runtime::kThrowStackOverflow);
    // Unreachable code.
    __ DebugBreak();

    __ bind(&enough_stack_space);

    __ Push(x0);
    __ Push(receiver, function);

    Label done, loop;
    // Copy arguments to the stack in a loop, in reverse order.
    // a4: argc.
    // a5: argv.

    // Skip the argument set up if we have no arguments.
    __ Jump(&done, eq, argc, x0);

    __ Add(t3, argv, argc);  // t3 = argv + argc
    __ Move(t4, argv);       // t4 = argv

    __ bind(&loop);
    __ Ld(t5, MemOperand(t4));   // t5 = load(t4)
    __ Push(t5);                 // push(t5)
    __ Add(t4, t4, 1);           // t4 = t4 + 1
    __ Jump(&loop, lt, t4, t3);  // t4 < t3 jump loop

    __ bind(&done);

    __ Move(a0, argc);
    __ Move(a3, new_target);
    __ Move(a1, function);
    //  a0: argc
    //  a1: function
    //  a3: new.target.

    // Initialize all JavaScript callee-saved registers, since they will be seen
    // by the garbage collector as part of handlers.

    __ LoadRoot(x9, RootIndex::kUndefinedValue);
    __ Move(x17, x9);
    __ Move(x18, x9);
    __ Move(x19, x9);
    __ Move(x20, x9);
    __ Move(x21, x9);
    __ Move(x23, x9);
    __ Move(x24, x9);
    __ Move(x25, x9);
    __ Move(x27, x9);
    __ Move(x28, x9);
    // x22 root register
    // x26 cp

    Handle<Code> builtin = is_construct
                               ? BUILTIN_CODE(masm->isolate(), Construct)
                               : masm->isolate()->builtins()->Call();
    __ Call(builtin, RelocInfo::CODE_TARGET);
  }
  __ Ret();
}

void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, false);
}

void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, true);
}

void Builtins::Generate_RunMicrotasksTrampoline(MacroAssembler* masm) {
  // This expects two C++ function parameters passed by Invoke() in
  // execution.cc.
  //   a0: root_register_value
  //   a1: microtask_queue
  __ Move(RunMicrotasksDescriptor::MicrotaskQueueRegister(), a1);
  __ Jump(BUILTIN_CODE(masm->isolate(), RunMicrotasks), RelocInfo::CODE_TARGET);
}

static void LeaveInterpreterFrame(MacroAssembler* masm, Register scratch) {
  Register args_count = scratch;

  // Get the arguments + receiver count.
  __ Ld(args_count,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeArrayFromFp));
  __ Lw(t0, FieldMemOperand(args_count, BytecodeArray::kParameterSizeOffset));

  // Leave the frame (also dropping the register file).
  __ LeaveFrame(StackFrame::INTERPRETED);

  // Drop receiver + arguments.
  __ Dadd(sp, sp, args_count);
}

// Advance the current bytecode offset. This simulates what all bytecode
// handlers do upon completion of the underlying operation. Will bail out to a
// label if the bytecode (without prefix) is a return bytecode. Will not advance
// the bytecode offset if the current bytecode is a JumpLoop, instead just
// re-executing the JumpLoop to jump to the correct bytecode.
static void AdvanceBytecodeOffsetOrReturn(MacroAssembler* masm,
                                          Register bytecode_array,
                                          Register bytecode_offset,
                                          Register bytecode, Register scratch1,
                                          Register scratch2, Register scratch3,
                                          Label* if_return) {
  Register bytecode_size_table = scratch1;
  Register original_bytecode_offset = scratch2;
  // The bytecode offset value will be increased by one in wide and extra wide
  // cases. In the case of having a wide or extra wide JumpLoop bytecode, we
  // will restore the original bytecode. In order to simplify the code, we have
  // a backup of it.
  DCHECK(!AreAliased(bytecode_array, bytecode_offset, bytecode_size_table,
                     bytecode, original_bytecode_offset));

  __ Move(bytecode_size_table,
          ExternalReference::bytecode_size_table_address());
  __ Move(original_bytecode_offset, bytecode_offset);

  // Check if the bytecode is a Wide or ExtraWide prefix bytecode.
  Label process_bytecode, extra_wide;
  STATIC_ASSERT(0 == static_cast<int>(interpreter::Bytecode::kWide));
  STATIC_ASSERT(1 == static_cast<int>(interpreter::Bytecode::kExtraWide));
  STATIC_ASSERT(2 == static_cast<int>(interpreter::Bytecode::kDebugBreakWide));
  STATIC_ASSERT(3 ==
                static_cast<int>(interpreter::Bytecode::kDebugBreakExtraWide));

  __ Jump(&process_bytecode, hi, bytecode, Operand(0x3));
  __ And(scratch2, bytecode, Operand(0x1));
  __ Jump(&extra_wide, ne, scratch2, Operand(x0));
  // The code to load the next bytecode is common to both wide and extra wide.
  // We can hoist them up here since they do not modify the flags after Tst.
  __ Dadd(bytecode_offset, bytecode_offset, Operand(1));
  __ Dadd(scratch2, bytecode_array, bytecode_offset);
  __ Lbu(bytecode, MemOperand(scratch2));
  __ Dadd(bytecode_size_table, bytecode_size_table,
          Operand(kIntSize * interpreter::Bytecodes::kBytecodeCount));
  __ jmp(&process_bytecode);

  __ bind(&extra_wide);
  // Load the next bytecode and update table to the extra wide scaled table.
  __ Dadd(bytecode_offset, bytecode_offset, Operand(1));
  __ Dadd(scratch2, bytecode_array, bytecode_offset);
  __ Lbu(bytecode, MemOperand(scratch2));
  __ Dadd(bytecode_size_table, bytecode_size_table,
          Operand(2 * kIntSize * interpreter::Bytecodes::kBytecodeCount));

  __ bind(&process_bytecode);

// Bailout to the return label if this is a return bytecode.
#define JUMP_IF_EQUAL(NAME)        \
  __ Jump(if_return, eq, bytecode, \
          Operand(static_cast<int>(interpreter::Bytecode::k##NAME)));
  RETURN_BYTECODE_LIST(JUMP_IF_EQUAL)
#undef JUMP_IF_EQUAL

  // If this is a JumpLoop, re-execute it to perform the jump to the beginning
  // of the loop.
  Label end, not_jump_loop;
  __ Jump(&not_jump_loop, ne, bytecode,
          Operand(static_cast<int>(interpreter::Bytecode::kJumpLoop)));
  // We need to restore the original bytecode_offset since we might have
  // increased it to skip the wide / extra-wide prefix bytecode.
  __ Move(bytecode_offset, original_bytecode_offset);
  __ jmp(&end);

  __ bind(&not_jump_loop);
  // Otherwise, load the size of the current bytecode and advance the offset.
  __ GenScaledAddress(scratch2, bytecode_size_table, bytecode, 2);
  __ Lw(scratch2, MemOperand(scratch2));
  __ Dadd(bytecode_offset, bytecode_offset, scratch2);

  __ bind(&end);
}

// Tail-call |function_id| if |smi_entry| == |marker|
static void TailCallRuntimeIfMarkerEquals(MacroAssembler* masm,
                                          Register smi_entry,
                                          OptimizationMarker marker,
                                          Runtime::FunctionId function_id) {
  Label no_match;
  __ Jump(&no_match, ne, smi_entry, Operand(Smi::FromEnum(marker)));
  GenerateTailCallToReturnedCode(masm, function_id);
  __ bind(&no_match);
}

static void MaybeOptimizeCode(MacroAssembler* masm, Register feedback_vector,
                              Register optimization_marker) {
  // ----------- S t a t e -------------
  //  -- a3 : new target (preserved for callee if needed, and caller)
  //  -- a1 : target function (preserved for callee if needed, and caller)
  //  -- feedback vector (preserved for caller if needed)
  //  -- optimization_marker : a Smi containing a non-zero optimization marker.
  // -----------------------------------
  TailCallRuntimeIfMarkerEquals(masm, optimization_marker,
                                OptimizationMarker::kLogFirstExecution,
                                Runtime::kFunctionFirstExecution);
  TailCallRuntimeIfMarkerEquals(masm, optimization_marker,
                                OptimizationMarker::kCompileOptimized,
                                Runtime::kCompileOptimized_NotConcurrent);
  TailCallRuntimeIfMarkerEquals(masm, optimization_marker,
                                OptimizationMarker::kCompileOptimizedConcurrent,
                                Runtime::kCompileOptimized_Concurrent);

  // Otherwise, the marker is InOptimizationQueue, so fall through hoping
  // that an interrupt will eventually update the slot with optimized code.
  if (FLAG_debug_code) {
    __ Assert(eq, AbortReason::kExpectedOptimizationSentinel,
              optimization_marker,
              Operand(Smi::FromEnum(OptimizationMarker::kInOptimizationQueue)));
  }
}

// Generate code for entering a JS function with the interpreter.
// On entry to the function the receiver and arguments have been pushed on the
// stack left to right.  The actual argument count matches the formal parameter
// count expected by the function.
//
// The live registers are:
//   - a1: the JS function object being called.
//   - a3: the incoming new target or generator object
//   - cp: our context.
//   - fp: our caller's frame pointer.
//   - ra: return address.
//
// The function builds an interpreter frame.  See InterpreterFrameConstants in
// frames.h for its layout.
void Builtins::Generate_InterpreterEntryTrampoline(MacroAssembler* masm) {
  Register closure = a1;
  Register feedback_vector = a2;

  // Get the bytecode array from the function object and load it into
  // kInterpreterBytecodeArrayRegister.
  __ Ld(a0, FieldMemOperand(closure, JSFunction::kSharedFunctionInfoOffset));
  __ Ld(kInterpreterBytecodeArrayRegister,
        FieldMemOperand(a0, SharedFunctionInfo::kFunctionDataOffset));
  GetSharedFunctionInfoBytecode(masm, kInterpreterBytecodeArrayRegister, a4);

  // The bytecode array could have been flushed from the shared function info,
  // if so, call into CompileLazy.
  Label compile_lazy;
  __ GetInstanceType(kInterpreterBytecodeArrayRegister, a0, a0);
  __ Jump(&compile_lazy, ne, a0, Operand(BYTECODE_ARRAY_TYPE));

  // Load the feedback vector from the closure.
  __ Ld(feedback_vector,
        FieldMemOperand(closure, JSFunction::kFeedbackCellOffset));
  __ Ld(feedback_vector, FieldMemOperand(feedback_vector, Cell::kValueOffset));

  Label push_stack_frame;
  // Check if feedback vector is valid. If valid, check for optimized code
  // and update invocation count. Otherwise, setup the stack frame.
  __ Ld(a4, FieldMemOperand(feedback_vector, HeapObject::kMapOffset));
  __ Lh(a4, FieldMemOperand(a4, Map::kInstanceTypeOffset));
  __ Jump(&push_stack_frame, ne, a4, Operand(FEEDBACK_VECTOR_TYPE));

  // Read off the optimized code slot in the feedback vector, and if there
  // is optimized code or an optimization marker, call that instead.
  Register optimized_code_entry = a4;
  __ Ld(optimized_code_entry,
        FieldMemOperand(feedback_vector,
                        FeedbackVector::kOptimizedCodeWeakOrSmiOffset));
  Label optimized_code_slot_not_empty;

  __ Jump(&optimized_code_slot_not_empty, ne, optimized_code_entry,
          Operand(Smi::FromEnum(OptimizationMarker::kNone)));

  Label not_optimized;
  __ bind(&not_optimized);

  // Increment invocation count for the function.
  __ Lw(a4, FieldMemOperand(feedback_vector,
                            FeedbackVector::kInvocationCountOffset));
  __ Add(a4, a4, Operand(1));
  __ Sw(a4, FieldMemOperand(feedback_vector,
                            FeedbackVector::kInvocationCountOffset));

  // Open a frame scope to indicate that there is a frame on the stack.  The
  // MANUAL indicates that the scope shouldn't actually generate code to set up
  // the frame (that is done below).
  __ bind(&push_stack_frame);
  FrameScope frame_scope(masm, StackFrame::MANUAL);
  __ PushStandardFrame(closure);

  // Reset code age and the OSR arming. The OSR field and BytecodeAgeOffset are
  // 8-bit fields next to each other, so we could just optimize by writing a
  // 16-bit. These static asserts guard our assumption is valid.
  STATIC_ASSERT(BytecodeArray::kBytecodeAgeOffset ==
                BytecodeArray::kOsrNestingLevelOffset + kCharSize);
  STATIC_ASSERT(BytecodeArray::kNoAgeBytecodeAge == 0);

  __ Sh(x0, FieldMemOperand(kInterpreterBytecodeArrayRegister,
                            BytecodeArray::kOsrNestingLevelOffset));

  // Load initial bytecode offset.
  __ li(kInterpreterBytecodeOffsetRegister,
        Operand(BytecodeArray::kHeaderSize - kHeapObjectTag));

  // Push bytecode array and Smi tagged bytecode array offset.
  __ SmiTag(a4, kInterpreterBytecodeOffsetRegister);
  __ Push(kInterpreterBytecodeArrayRegister, a4);

  // Allocate the local and temporary register file on the stack.
  Label stack_overflow;
  {
    // Load frame size (word) from the BytecodeArray object.
    __ Lw(a4, FieldMemOperand(kInterpreterBytecodeArrayRegister,
                              BytecodeArray::kFrameSizeOffset));

    // Do a stack check to ensure we don't go over the limit.
    __ Dsub(a5, sp, Operand(a4));
    LoadStackLimit(masm, a2, StackLimitKind::kRealStackLimit);
    __ Jump(&stack_overflow, lo, a5, Operand(a2));

    // If ok, push undefined as the initial value for all register file entries.
    Label loop_header;
    Label loop_check;
    __ LoadRoot(a5, RootIndex::kUndefinedValue);
    __ Jump(&loop_check);
    __ bind(&loop_header);
    // TODO(rmcilroy): Consider doing more than one push per loop iteration.
    __ push(a5);
    // Continue loop if not done.
    __ bind(&loop_check);
    __ Dsub(a4, a4, Operand(kPointerSize));
    __ Jump(&loop_header, ge, a4, Operand(zero_reg));
  }
  // If the bytecode array has a valid incoming new target or generator object
  // register, initialize it with incoming value which was passed in a3.
  Label no_incoming_new_target_or_generator_register;
  __ Lw(a5, FieldMemOperand(
                kInterpreterBytecodeArrayRegister,
                BytecodeArray::kIncomingNewTargetOrGeneratorRegisterOffset));
  __ Jump(&no_incoming_new_target_or_generator_register, eq, a5, Operand(x0));

  __ GenScaledAddress(a5, fp, a5, kPointerSizeLog2);
  __ Sd(a3, MemOperand(a5));
  __ bind(&no_incoming_new_target_or_generator_register);

  // Perform interrupt stack check.
  // TODO(solanes): Merge with the real stack limit check above.
  Label stack_check_interrupt, after_stack_check_interrupt;
  LoadStackLimit(masm, a5, StackLimitKind::kInterruptStackLimit);
  __ Jump(&stack_check_interrupt, lo, sp, Operand(a5));
  __ bind(&after_stack_check_interrupt);

  // Load accumulator as undefined.
  __ LoadRoot(kInterpreterAccumulatorRegister, RootIndex::kUndefinedValue);

  // Load the dispatch table into a register and dispatch to the bytecode
  // handler at the current bytecode offset.
  Label do_dispatch;
  __ bind(&do_dispatch);
  __ li(kInterpreterDispatchTableRegister,
        ExternalReference::interpreter_dispatch_table_address(masm->isolate()));
  __ Dadd(a0, kInterpreterBytecodeArrayRegister,
          kInterpreterBytecodeOffsetRegister);
  __ Lb(a7, MemOperand(a0));
  __ GenScaledAddress(kScratchRegister, kInterpreterDispatchTableRegister, a7,
                      kPointerSizeLog2);
  __ Ld(kJavaScriptCallCodeStartRegister, MemOperand(kScratchRegister));
  __ Call(kJavaScriptCallCodeStartRegister);
  masm->isolate()->heap()->SetInterpreterEntryReturnPCOffset(masm->pc_offset());

  // Any returns to the entry trampoline are either due to the return bytecode
  // or the interpreter tail calling a builtin and then a dispatch.

  // Get bytecode array and bytecode offset from the stack frame.
  __ Ld(kInterpreterBytecodeArrayRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeArrayFromFp));
  __ Ld(kInterpreterBytecodeOffsetRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));
  __ SmiUntag(kInterpreterBytecodeOffsetRegister);

  // Either return, or advance to the next bytecode and dispatch.
  Label do_return;

  __ Dadd(a1, kInterpreterBytecodeArrayRegister,
          kInterpreterBytecodeOffsetRegister);
  __ Lbu(a1, MemOperand(a1));

  AdvanceBytecodeOffsetOrReturn(masm, kInterpreterBytecodeArrayRegister,
                                kInterpreterBytecodeOffsetRegister, a1, a2, a3,
                                a4, &do_return);
  __ jmp(&do_dispatch);
  __ bind(&do_return);

  // The return value is in v0.
  LeaveInterpreterFrame(masm, t0);
  __ Jump(ra);

  __ bind(&stack_check_interrupt);
  // Modify the bytecode offset in the stack to be kFunctionEntryBytecodeOffset
  // for the call to the StackGuard.
  __ li(kInterpreterBytecodeOffsetRegister,
        Operand(Smi::FromInt(BytecodeArray::kHeaderSize - kHeapObjectTag +
                             kFunctionEntryBytecodeOffset)));
  __ Sd(kInterpreterBytecodeOffsetRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));
  __ CallRuntime(Runtime::kStackGuard);

  // After the call, restore the bytecode array, bytecode offset and accumulator
  // registers again. Also, restore the bytecode offset in the stack to its
  // previous value.
  __ Ld(kInterpreterBytecodeArrayRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeArrayFromFp));
  __ li(kInterpreterBytecodeOffsetRegister,
        Operand(BytecodeArray::kHeaderSize - kHeapObjectTag));
  __ LoadRoot(kInterpreterAccumulatorRegister, RootIndex::kUndefinedValue);

  __ SmiTag(a5, kInterpreterBytecodeOffsetRegister);
  __ Sd(a5, MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));

  __ jmp(&after_stack_check_interrupt);

  __ bind(&optimized_code_slot_not_empty);
  Label maybe_has_optimized_code;

  // Check if optimized code marker is actually a weak reference to the
  // optimized code as opposed to an optimization marker.
  __ JumpIfNotSmi(optimized_code_entry, &maybe_has_optimized_code);
  MaybeOptimizeCode(masm, feedback_vector, optimized_code_entry);
  // Fall through if there's no runnable optimized code.
  __ jmp(&not_optimized);

  __ bind(&maybe_has_optimized_code);
  // Load code entry from the weak reference, if it was cleared, resume
  // execution of unoptimized code.
  __ LoadWeakValue(optimized_code_entry, optimized_code_entry, &not_optimized);
  TailCallOptimizedCodeSlot(masm, optimized_code_entry, t2, a5);

  __ bind(&compile_lazy);
  GenerateTailCallToReturnedCode(masm, Runtime::kCompileLazy);
  // Unreachable code.
  __ DebugBreak();

  __ bind(&stack_overflow);
  __ CallRuntime(Runtime::kThrowStackOverflow);
  // Unreachable code.
  __ DebugBreak();
}

static void Generate_InterpreterPushArgs(MacroAssembler* masm,
                                         Register num_args, Register index,
                                         Register scratch, Register scratch2) {
  // Find the address of the last argument.
  __ mov(scratch2, num_args);
  __ slli(scratch2, scratch2, kPointerSizeLog2);
  __ Dsub(scratch2, index, Operand(scratch2));

  // Push the arguments.
  Label loop_header, loop_check;
  __ Jump(&loop_check);
  __ bind(&loop_header);
  __ Ld(scratch, MemOperand(index));
  __ Dadd(index, index, Operand(-kPointerSize));
  __ push(scratch);
  __ bind(&loop_check);
  __ Jump(&loop_header, hi, index, Operand(scratch2));
}

// static
void Builtins::Generate_ResumeGeneratorTrampoline(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0 : the value to pass to the generator
  //  -- a1 : the JSGeneratorObject to resume
  //  -- ra : return address
  // -----------------------------------
  __ AssertGeneratorObject(a1);

  // Store input value into generator object.
  __ Sd(a0, FieldMemOperand(a1, JSGeneratorObject::kInputOrDebugPosOffset));
  __ RecordWriteField(a1, JSGeneratorObject::kInputOrDebugPosOffset, a0,
                      kRAHasNotBeenSaved, kDontSaveFPRegs);

  // Load suspended function and context.
  __ Ld(a4, FieldMemOperand(a1, JSGeneratorObject::kFunctionOffset));
  __ Ld(cp, FieldMemOperand(a4, JSFunction::kContextOffset));

  // Flood function if we are stepping.
  Label prepare_step_in_if_stepping, prepare_step_in_suspended_generator;
  Label stepping_prepared;
  ExternalReference debug_hook =
      ExternalReference::debug_hook_on_function_call_address(masm->isolate());
  __ li(a5, debug_hook);
  __ Lb(a5, MemOperand(a5));
  __ Jump(&prepare_step_in_if_stepping, ne, a5, Operand(zero_reg));

  // Flood function if we need to continue stepping in the suspended generator.
  ExternalReference debug_suspended_generator =
      ExternalReference::debug_suspended_generator_address(masm->isolate());
  __ li(a5, debug_suspended_generator);
  __ Ld(a5, MemOperand(a5));
  __ Jump(&prepare_step_in_suspended_generator, eq, a1, Operand(a5));
  __ bind(&stepping_prepared);

  // Check the stack for overflow. We are not trying to catch interruptions
  // (i.e. debug break and preemption) here, so check the "real stack limit".
  Label stack_overflow;
  LoadStackLimit(masm, kScratchRegister, StackLimitKind::kRealStackLimit);
  __ Jump(&stack_overflow, lo, sp, Operand(kScratchRegister));

  // Push receiver.
  __ Ld(a5, FieldMemOperand(a1, JSGeneratorObject::kReceiverOffset));
  __ Push(a5);

  // ----------- S t a t e -------------
  //  -- a1    : the JSGeneratorObject to resume
  //  -- a4    : generator function
  //  -- cp    : generator context
  //  -- ra    : return address
  //  -- sp[0] : generator receiver
  // -----------------------------------

  // Push holes for arguments to generator function. Since the parser forced
  // context allocation for any variables in generators, the actual argument
  // values have already been copied into the context and these dummy values
  // will never be used.
  __ Ld(a3, FieldMemOperand(a4, JSFunction::kSharedFunctionInfoOffset));
  __ Lhu(a3,
         FieldMemOperand(a3, SharedFunctionInfo::kFormalParameterCountOffset));
  __ Ld(t1,
        FieldMemOperand(a1, JSGeneratorObject::kParametersAndRegistersOffset));
  {
    Label done_loop, loop;
    __ Move(t2, zero_reg);
    __ bind(&loop);
    __ Dsub(a3, a3, Operand(1));
    __ Jump(&done_loop, lt, a3, Operand(zero_reg));
    __ GenScaledAddress(kScratchRegister, t1, t2, kPointerSizeLog2);
    __ Ld(kScratchRegister,
          FieldMemOperand(kScratchRegister, FixedArray::kHeaderSize));
    __ Push(kScratchRegister);
    __ Dadd(t2, t2, Operand(1));
    __ Jump(&loop);
    __ bind(&done_loop);
  }

  // Underlying function needs to have bytecode available.
  if (FLAG_debug_code) {
    __ Ld(a3, FieldMemOperand(a4, JSFunction::kSharedFunctionInfoOffset));
    __ Ld(a3, FieldMemOperand(a3, SharedFunctionInfo::kFunctionDataOffset));
    GetSharedFunctionInfoBytecode(masm, a3, a0);
    __ GetInstanceType(a3, a3, a3);
    __ Assert(eq, AbortReason::kMissingBytecodeArray, a3,
              Operand(BYTECODE_ARRAY_TYPE));
  }

  // Resume (Ignition/TurboFan) generator object.
  {
    __ Ld(a0, FieldMemOperand(a4, JSFunction::kSharedFunctionInfoOffset));
    __ Lhu(a0, FieldMemOperand(
                   a0, SharedFunctionInfo::kFormalParameterCountOffset));
    // We abuse new.target both to indicate that this is a resume call and to
    // pass in the generator object.  In ordinary calls, new.target is always
    // undefined because generator functions are non-constructable.
    __ Move(a3, a1);
    __ Move(a1, a4);
    static_assert(kJavaScriptCallCodeStartRegister == a2, "ABI mismatch");
    __ Ld(a2, FieldMemOperand(a1, JSFunction::kCodeOffset));
    __ Dadd(a2, a2, Operand(Code::kHeaderSize - kHeapObjectTag));
    __ Jump(a2);
  }

  __ bind(&prepare_step_in_if_stepping);
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ Push(a1, a4);
    // Push hole as receiver since we do not use it for stepping.
    __ PushRoot(RootIndex::kTheHoleValue);
    __ CallRuntime(Runtime::kDebugOnFunctionCall);
    __ Pop(a1);
    __ Ld(a4, FieldMemOperand(a1, JSGeneratorObject::kFunctionOffset));
  }
  __ Jump(&stepping_prepared);

  __ bind(&prepare_step_in_suspended_generator);
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ Push(a1);
    __ CallRuntime(Runtime::kDebugPrepareStepInSuspendedGenerator);
    __ Pop(a1);
    __ Ld(a4, FieldMemOperand(a1, JSGeneratorObject::kFunctionOffset));
  }
  __ Jump(&stepping_prepared);

  __ bind(&stack_overflow);
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ CallRuntime(Runtime::kThrowStackOverflow);
    __ DebugBreak();  // This should be unreachable.
  }
}

// static
void Builtins::Generate_InterpreterPushArgsThenCallImpl(
    MacroAssembler* masm, ConvertReceiverMode receiver_mode,
    InterpreterPushArgsMode mode) {
  DCHECK(mode != InterpreterPushArgsMode::kArrayFunction);
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a2 : the address of the first argument to be pushed. Subsequent
  //          arguments should be consecutive above this, in the same order as
  //          they are to be pushed onto the stack.
  //  -- a1 : the target to call (can be any Object).
  // -----------------------------------
  Label stack_overflow;

  __ Dadd(a3, a0, Operand(1));  // Add one for reciver.

  // Push "undefined" as the receiver arg if we need to.
  if (receiver_mode == ConvertReceiverMode::kNullOrUndefined) {
    __ PushRoot(RootIndex::kUndefinedValue);
    __ Dsub(a3, a3, Operand(1));  // Subtract one for receiver.
  }

  Generate_StackOverflowCheck(masm, a3, &stack_overflow);

  // This function modifies a2, t0 and a4.
  Generate_InterpreterPushArgs(masm, a3, a2, a4, t0);

  if (mode == InterpreterPushArgsMode::kWithFinalSpread) {
    __ Pop(a2);                   // Pass the spread in a register
    __ Dsub(a0, a0, Operand(1));  // Subtract one for spread
  }

  // Call the target.
  if (mode == InterpreterPushArgsMode::kWithFinalSpread) {
    __ Jump(BUILTIN_CODE(masm->isolate(), CallWithSpread),
            RelocInfo::CODE_TARGET);
  } else {
    __ Jump(masm->isolate()->builtins()->Call(ConvertReceiverMode::kAny),
            RelocInfo::CODE_TARGET);
  }

  __ bind(&stack_overflow);
  {
    __ TailCallRuntime(Runtime::kThrowStackOverflow);
    // Unreachable code.
    __ DebugBreak();
  }
}

// static
void Builtins::Generate_InterpreterPushArgsThenConstructImpl(
    MacroAssembler* masm, InterpreterPushArgsMode mode) {
  // ----------- S t a t e -------------
  // -- a0 : argument count (not including receiver)
  // -- a3 : new target
  // -- a1 : constructor to call
  // -- a2 : allocation site feedback if available, undefined otherwise.
  // -- a4 : address of the first argument
  // -----------------------------------
  Label stack_overflow;
  // Push a slot for the receiver.
  __ push(zero_reg);

  Generate_StackOverflowCheck(masm, a0, &stack_overflow);

  // This function modifies t0, a4 and a5.
  Generate_InterpreterPushArgs(masm, a0, a4, a5, t0);

  if (mode == InterpreterPushArgsMode::kWithFinalSpread) {
    __ Pop(a2);                   // Pass the spread in a register
    __ Dsub(a0, a0, Operand(1));  // Subtract one for spread
  } else {
    __ AssertUndefinedOrAllocationSite(a2, t0);
  }

  if (mode == InterpreterPushArgsMode::kArrayFunction) {
    __ AssertFunction(a1);

    // Tail call to the function-specific construct stub (still in the caller
    // context at this point).
    __ Jump(BUILTIN_CODE(masm->isolate(), ArrayConstructorImpl),
            RelocInfo::CODE_TARGET);
  } else if (mode == InterpreterPushArgsMode::kWithFinalSpread) {
    // Call the constructor with a0, a1, and a3 unmodified.
    __ Jump(BUILTIN_CODE(masm->isolate(), ConstructWithSpread),
            RelocInfo::CODE_TARGET);
  } else {
    DCHECK_EQ(InterpreterPushArgsMode::kOther, mode);
    // Call the constructor with a0, a1, and a3 unmodified.
    __ Jump(BUILTIN_CODE(masm->isolate(), Construct), RelocInfo::CODE_TARGET);
  }

  __ bind(&stack_overflow);
  {
    __ TailCallRuntime(Runtime::kThrowStackOverflow);
    // Unreachable code.
    __ DebugBreak();
  }
}

static void Generate_InterpreterEnterBytecode(MacroAssembler* masm) {
  // Set the return address to the correct point in the interpreter entry
  // trampoline.
  Label builtin_trampoline, trampoline_loaded;
  Smi interpreter_entry_return_pc_offset(
      masm->isolate()->heap()->interpreter_entry_return_pc_offset());
  DCHECK_NE(interpreter_entry_return_pc_offset, Smi::zero());

  // If the SFI function_data is an InterpreterData, the function will have a
  // custom copy of the interpreter entry trampoline for profiling. If so,
  // get the custom trampoline, otherwise grab the entry address of the global
  // trampoline.
  __ Ld(t0, MemOperand(fp, StandardFrameConstants::kFunctionOffset));
  __ Ld(t0, FieldMemOperand(t0, JSFunction::kSharedFunctionInfoOffset));
  __ Ld(t0, FieldMemOperand(t0, SharedFunctionInfo::kFunctionDataOffset));
  __ GetInstanceType(t0, kInterpreterDispatchTableRegister,
                     kInterpreterDispatchTableRegister);
  __ Jump(&builtin_trampoline, ne, kInterpreterDispatchTableRegister,
          Operand(INTERPRETER_DATA_TYPE));

  __ Ld(t0, FieldMemOperand(t0, InterpreterData::kInterpreterTrampolineOffset));
  __ Dadd(t0, t0, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ Jump(&trampoline_loaded);

  __ bind(&builtin_trampoline);
  __ li(t0, ExternalReference::
                address_of_interpreter_entry_trampoline_instruction_start(
                    masm->isolate()));
  __ Ld(t0, MemOperand(t0));

  __ bind(&trampoline_loaded);
  __ Dadd(ra, t0, Operand(interpreter_entry_return_pc_offset.value()));

  // Initialize the dispatch table register.
  __ li(kInterpreterDispatchTableRegister,
        ExternalReference::interpreter_dispatch_table_address(masm->isolate()));

  // Get the bytecode array pointer from the frame.
  __ Ld(kInterpreterBytecodeArrayRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeArrayFromFp));

  if (FLAG_debug_code) {
    // Check function data field is actually a BytecodeArray object.
    __ And(kScratchRegister, kInterpreterBytecodeArrayRegister,
           Operand(kSmiTagMask));
    __ Assert(ne,
              AbortReason::kFunctionDataShouldBeBytecodeArrayOnInterpreterEntry,
              kScratchRegister, Operand(zero_reg));
    __ GetInstanceType(kInterpreterBytecodeArrayRegister, a1, a1);
    __ Assert(eq,
              AbortReason::kFunctionDataShouldBeBytecodeArrayOnInterpreterEntry,
              a1, Operand(BYTECODE_ARRAY_TYPE));
  }

  // Get the target bytecode offset from the frame.
  __ SmiUntag(kInterpreterBytecodeOffsetRegister,
              MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));

  if (FLAG_debug_code) {
    Label okay;
    __ Jump(&okay, ge, kInterpreterBytecodeOffsetRegister,
            Operand(BytecodeArray::kHeaderSize - kHeapObjectTag));
    // Unreachable code.
    __ DebugBreak();
    __ bind(&okay);
  }

  // Dispatch to the target bytecode.
  __ Dadd(a1, kInterpreterBytecodeArrayRegister,
          kInterpreterBytecodeOffsetRegister);
  __ Lbu(a7, MemOperand(a1));
  __ GenScaledAddress(a1, kInterpreterDispatchTableRegister, a7,
                      kPointerSizeLog2);
  __ Ld(kJavaScriptCallCodeStartRegister, MemOperand(a1));
  __ Jump(kJavaScriptCallCodeStartRegister);
}

void Builtins::Generate_InterpreterEnterBytecodeAdvance(MacroAssembler* masm) {
  // Advance the current bytecode offset stored within the given interpreter
  // stack frame. This simulates what all bytecode handlers do upon completion
  // of the underlying operation.
  __ Ld(kInterpreterBytecodeArrayRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeArrayFromFp));
  __ Ld(kInterpreterBytecodeOffsetRegister,
        MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));
  __ SmiUntag(kInterpreterBytecodeOffsetRegister);

  Label enter_bytecode, function_entry_bytecode;
  __ Jump(&function_entry_bytecode, eq, kInterpreterBytecodeOffsetRegister,
          Operand(BytecodeArray::kHeaderSize - kHeapObjectTag +
                  kFunctionEntryBytecodeOffset));

  // Load the current bytecode.
  __ Dadd(a1, kInterpreterBytecodeArrayRegister,
          kInterpreterBytecodeOffsetRegister);
  __ Lbu(a1, MemOperand(a1));

  // Advance to the next bytecode.
  Label if_return;
  AdvanceBytecodeOffsetOrReturn(masm, kInterpreterBytecodeArrayRegister,
                                kInterpreterBytecodeOffsetRegister, a1, a2, a3,
                                a4, &if_return);

  __ bind(&enter_bytecode);
  // Convert new bytecode offset to a Smi and save in the stackframe.
  __ SmiTag(a2, kInterpreterBytecodeOffsetRegister);
  __ Sd(a2, MemOperand(fp, InterpreterFrameConstants::kBytecodeOffsetFromFp));

  Generate_InterpreterEnterBytecode(masm);

  __ bind(&function_entry_bytecode);
  // If the code deoptimizes during the implicit function entry stack interrupt
  // check, it will have a bailout ID of kFunctionEntryBytecodeOffset, which is
  // not a valid bytecode offset. Detect this case and advance to the first
  // actual bytecode.
  __ li(kInterpreterBytecodeOffsetRegister,
        Operand(BytecodeArray::kHeaderSize - kHeapObjectTag));
  __ Jump(&enter_bytecode);

  // We should never take the if_return path.
  __ bind(&if_return);
  __ Abort(AbortReason::kInvalidBytecodeAdvance);
}

void Builtins::Generate_InterpreterEnterBytecodeDispatch(MacroAssembler* masm) {
  Generate_InterpreterEnterBytecode(masm);
}

namespace {
void Generate_ContinueToBuiltinHelper(MacroAssembler* masm,
                                      bool java_script_builtin,
                                      bool with_result) {
  const RegisterConfiguration* config(RegisterConfiguration::Default());
  int allocatable_register_count = config->num_allocatable_general_registers();
  if (with_result) {
    // Overwrite the hole inserted by the deoptimizer with the return value from
    // the LAZY deopt point.
    __ Sd(a0,
          MemOperand(
              sp, config->num_allocatable_general_registers() * kPointerSize +
                      BuiltinContinuationFrameConstants::kFixedFrameSize));
  }
  for (int i = allocatable_register_count - 1; i >= 0; --i) {
    int code = config->GetAllocatableGeneralCode(i);
    __ Pop(Register::from_code(code));
    if (java_script_builtin && code == kJavaScriptCallArgCountRegister.code()) {
      __ SmiUntag(Register::from_code(code));
    }
  }
  __ Ld(fp, MemOperand(
                sp, BuiltinContinuationFrameConstants::kFixedFrameSizeFromFp));
  // Load builtin index (stored as a Smi) and use it to get the builtin start
  // address from the builtins table.
  __ Pop(t0);
  __ Dadd(sp, sp,
          Operand(BuiltinContinuationFrameConstants::kFixedFrameSizeFromFp));
  __ Pop(ra);
  __ LoadEntryFromBuiltinIndex(t0);
  __ Jump(t0);
}
}  // namespace

void Builtins::Generate_ContinueToCodeStubBuiltin(MacroAssembler* masm) {
  Generate_ContinueToBuiltinHelper(masm, false, false);
}

void Builtins::Generate_ContinueToCodeStubBuiltinWithResult(
    MacroAssembler* masm) {
  Generate_ContinueToBuiltinHelper(masm, false, true);
}

void Builtins::Generate_ContinueToJavaScriptBuiltin(MacroAssembler* masm) {
  Generate_ContinueToBuiltinHelper(masm, true, false);
}

void Builtins::Generate_ContinueToJavaScriptBuiltinWithResult(
    MacroAssembler* masm) {
  Generate_ContinueToBuiltinHelper(masm, true, true);
}

void Builtins::Generate_NotifyDeoptimized(MacroAssembler* masm) {
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ CallRuntime(Runtime::kNotifyDeoptimized);
  }

  // Pop TOS register and padding.
  DCHECK_EQ(kInterpreterAccumulatorRegister.code(), a0.code());
  __ Pop(a0, zero_reg);
  __ Ret();
}

void Builtins::Generate_InterpreterOnStackReplacement(MacroAssembler* masm) {
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ CallRuntime(Runtime::kCompileForOnStackReplacement);
  }

  // If the code object is null, just return to the caller.
  __ Ret(eq, a0, Operand(Smi::zero()));

  // Drop the handler frame that is be sitting on top of the actual
  // JavaScript frame. This is the case then OSR is triggered from bytecode.
  __ LeaveFrame(StackFrame::STUB);

  // Load deoptimization data from the code object.
  // <deopt_data> = <code>[#deoptimization_data_offset]
  __ Ld(a1, MemOperand(a0, Code::kDeoptimizationDataOffset - kHeapObjectTag));

  // Load the OSR entrypoint offset from the deoptimization data.
  // <osr_offset> = <deopt_data>[#header_size + #osr_pc_offset]
  __ SmiUntag(a1, MemOperand(a1, FixedArray::OffsetOfElementAt(
                                     DeoptimizationData::kOsrPcOffsetIndex) -
                                     kHeapObjectTag));

  // Compute the target address = code_obj + header_size + osr_offset
  // <entry_addr> = <code_obj> + #header_size + <osr_offset>
  __ Dadd(a0, a0, a1);
  __ addi(ra, a0, Code::kHeaderSize - kHeapObjectTag);

  // And "return" to the OSR entry point of the function.
  __ Ret();
}

// static
void Builtins::Generate_FunctionPrototypeApply(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0    : argc
  //  -- sp[0] : argArray
  //  -- sp[4] : thisArg
  //  -- sp[8] : receiver
  // -----------------------------------

  Register argc = a0;
  Register arg_array = a2;
  Register receiver = a1;
  Register this_arg = a5;
  Register undefined_value = a3;
  Register scratch = a4;

  __ LoadRoot(undefined_value, RootIndex::kUndefinedValue);

  // 1. Load receiver into a1, argArray into a2 (if present), remove all
  // arguments from the stack (including the receiver), and push thisArg (if
  // present) instead.
  {
    // Claim (2 - argc) dummy arguments form the stack, to put the stack in a
    // consistent state for a simple pop operation.

    __ Dsub(sp, sp, Operand(2 * kPointerSize));
    __ GenScaledAddress(sp, sp, argc, kPointerSizeLog2);
    __ mov(scratch, argc);
    __ Pop(this_arg, arg_array);  // Overwrite argc

    Label not_zero;
    __ Jump(&not_zero, ne, scratch, x0);
    __ Move(arg_array, undefined_value);  // if argc == 0
    __ Move(this_arg, undefined_value);   // if argc == 0
    __ bind(&not_zero);

    Label not_one;
    __ Jump(&not_one, ne, scratch, 1);
    __ Move(arg_array, undefined_value);  // if argc == 1
    __ bind(&not_one);

    __ Ld(receiver, MemOperand(sp));
    __ Sd(this_arg, MemOperand(sp));
  }

  // ----------- S t a t e -------------
  //  -- a2    : argArray
  //  -- a1    : receiver
  //  -- a3    : undefined root value
  //  -- sp[0] : thisArg
  // -----------------------------------

  // 2. We don't need to check explicitly for callable receiver here,
  // since that's the first thing the Call/CallWithArrayLike builtins
  // will do.

  // 3. Tail call with no arguments if argArray is null or undefined.
  Label no_arguments;
  __ JumpIfRoot(arg_array, RootIndex::kNullValue, &no_arguments);
  __ Jump(&no_arguments, eq, arg_array, Operand(undefined_value));

  // 4a. Apply the receiver to the given argArray.
  __ Jump(BUILTIN_CODE(masm->isolate(), CallWithArrayLike),
          RelocInfo::CODE_TARGET);

  // 4b. The argArray is either null or undefined, so we tail call without any
  // arguments to the receiver.
  __ bind(&no_arguments);
  {
    __ mov(a0, zero_reg);
    DCHECK(receiver == a1);
    __ Jump(masm->isolate()->builtins()->Call(), RelocInfo::CODE_TARGET);
  }
}

// static
void Builtins::Generate_FunctionPrototypeCall(MacroAssembler* masm) {
  // 1. Make sure we have at least one argument.
  // a0: actual number of arguments
  {
    Label done;
    __ Jump(&done, ne, a0, Operand(zero_reg));
    __ PushRoot(RootIndex::kUndefinedValue);
    __ Dadd(a0, a0, Operand(1));
    __ bind(&done);
  }

  // 2. Get the function to call (passed as receiver) from the stack.
  // a0: actual number of arguments
  __ GenScaledAddress(kScratchRegister, sp, a0, kPointerSizeLog2);
  __ Ld(a1, MemOperand(kScratchRegister));

  // 3. Shift arguments and return address one slot down on the stack
  //    (overwriting the original receiver).  Adjust argument count to make
  //    the original first argument the new receiver.
  // a0: actual number of arguments
  // a1: function
  {
    Label loop;
    // Calculate the copy start address (destination). Copy end address is sp.
    __ GenScaledAddress(a2, sp, a0, kPointerSizeLog2);
    __ bind(&loop);
    __ Ld(kScratchRegister, MemOperand(a2, -kPointerSize));
    __ Sd(kScratchRegister, MemOperand(a2));
    __ Dsub(a2, a2, Operand(kPointerSize));
    __ Jump(&loop, ne, a2, Operand(sp));
    // Adjust the actual number of arguments and remove the top element
    // (which is a copy of the last argument).
    __ Dsub(a0, a0, Operand(1));
    __ Pop();
  }

  // 4. Call the callable.
  __ Jump(masm->isolate()->builtins()->Call(), RelocInfo::CODE_TARGET);
}

void Builtins::Generate_ReflectApply(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0     : argc
  //  -- sp[0]  : argumentsList  (if argc ==3)
  //  -- sp[4]  : thisArgument   (if argc >=2)
  //  -- sp[8]  : target         (if argc >=1)
  //  -- sp[12] : receiver
  // -----------------------------------

  Register argc = a0;
  Register arguments_list = a2;
  Register target = a1;
  Register this_argument = a5;
  Register undefined_value = a3;
  Register scratch = a4;

  __ LoadRoot(undefined_value, RootIndex::kUndefinedValue);

  // 1. Load target into a1 (if present), argumentsList into a2 (if present),
  // remove all arguments from the stack (including the receiver), and push
  // thisArgument (if present) instead.
  {
    // Claim (3 - argc) dummy arguments form the stack, to put the stack in a
    // consistent state for a simple pop operation.

    __ Dsub(sp, sp, Operand(3 * kPointerSize));
    __ GenScaledAddress(sp, sp, argc, kPointerSizeLog2);
    __ mov(scratch, argc);
    __ Pop(target, this_argument, arguments_list);
    __ Movz(arguments_list, undefined_value, scratch);  // if argc == 0
    __ Movz(this_argument, undefined_value, scratch);   // if argc == 0
    __ Movz(target, undefined_value, scratch);          // if argc == 0
    __ Dsub(scratch, scratch, Operand(1));
    __ Movz(arguments_list, undefined_value, scratch);  // if argc == 1
    __ Movz(this_argument, undefined_value, scratch);   // if argc == 1
    __ Dsub(scratch, scratch, Operand(1));
    __ Movz(arguments_list, undefined_value, scratch);  // if argc == 2

    __ Sd(this_argument, MemOperand(sp, 0));  // Overwrite receiver
  }

  // ----------- S t a t e -------------
  //  -- a2    : argumentsList
  //  -- a1    : target
  //  -- a3    : undefined root value
  //  -- sp[0] : thisArgument
  // -----------------------------------

  // 2. We don't need to check explicitly for callable target here,
  // since that's the first thing the Call/CallWithArrayLike builtins
  // will do.
  // 3. Apply the target to the given argumentsList.
  __ Jump(BUILTIN_CODE(masm->isolate(), CallWithArrayLike),
          RelocInfo::CODE_TARGET);
}

void Builtins::Generate_ReflectConstruct(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0     : argc
  //  -- sp[0]  : new.target (optional) (dummy value if argc <= 2)
  //  -- sp[4]  : argumentsList         (dummy value if argc <= 1)
  //  -- sp[8]  : target                (dummy value if argc == 0)
  //  -- sp[12] : receiver
  // -----------------------------------
  Register argc = a0;
  Register arguments_list = a2;
  Register target = a1;
  Register new_target = a3;
  Register undefined_value = a4;
  Register scratch = a5;

  __ LoadRoot(undefined_value, RootIndex::kUndefinedValue);

  // 1. Load target into a1 (if present), argumentsList into a2 (if present),
  // new.target into a3 (if present, otherwise use target), remove all
  // arguments from the stack (including the receiver), and push thisArgument
  // (if present) instead.
  {
    // Claim (3 - argc) dummy arguments form the stack, to put the stack in a
    // consistent state for a simple pop operation.

    __ Dsub(sp, sp, Operand(3 * kPointerSize));
    __ GenScaledAddress(sp, sp, argc, kPointerSizeLog2);
    __ mov(scratch, argc);
    __ Pop(target, arguments_list, new_target);
    __ Movz(arguments_list, undefined_value, scratch);  // if argc == 0
    __ Movz(new_target, undefined_value, scratch);      // if argc == 0
    __ Movz(target, undefined_value, scratch);          // if argc == 0
    __ Dsub(scratch, scratch, Operand(1));
    __ Movz(arguments_list, undefined_value, scratch);  // if argc == 1
    __ Movz(new_target, target, scratch);               // if argc == 1
    __ Dsub(scratch, scratch, Operand(1));
    __ Movz(new_target, target, scratch);  // if argc == 2

    __ Sd(undefined_value, MemOperand(sp, 0));  // Overwrite receiver
  }

  // ----------- S t a t e -------------
  //  -- a2    : argumentsList
  //  -- a1    : target
  //  -- a3    : new.target
  //  -- sp[0] : receiver (undefined)
  // -----------------------------------

  // 2. We don't need to check explicitly for constructor target here,
  // since that's the first thing the Construct/ConstructWithArrayLike
  // builtins will do.

  // 3. We don't need to check explicitly for constructor new.target here,
  // since that's the second thing the Construct/ConstructWithArrayLike
  // builtins will do.

  // 4. Construct the target with the given new.target and argumentsList.
  __ Jump(BUILTIN_CODE(masm->isolate(), ConstructWithArrayLike),
          RelocInfo::CODE_TARGET);
}

namespace {

void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
  __ Push(ra, fp);
  __ Move(a4, StackFrame::TypeToMarker(StackFrame::ARGUMENTS_ADAPTOR));
  __ Push(a4, a1);
  __ Push(a0, x0);
  __ Dadd(fp, sp,
          Operand(ArgumentsAdaptorFrameConstants::kFixedFrameSizeFromFp));
}

void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- v0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
  __ Ld(a1, MemOperand(fp, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ mov(sp, fp);
  __ Pop(fp, ra);
  __ slli(a1, a1, kPointerSizeLog2);
  __ Dadd(sp, sp, a4);
  // Adjust for the receiver
  __ Dadd(sp, sp, Operand(kPointerSize));
}

}  // namespace

// static
void Builtins::Generate_CallOrConstructVarargs(MacroAssembler* masm,
                                               Handle<Code> code) {
  // ----------- S t a t e -------------
  //  -- a1 : target
  //  -- a0 : number of parameters on the stack (not including the receiver)
  //  -- a2 : arguments list (a FixedArray)
  //  -- a4 : len (number of elements to push from args)
  //  -- a3 : new.target (for [[Construct]])
  // -----------------------------------
  if (masm->emit_debug_code()) {
    // Allow a2 to be a FixedArray, or a FixedDoubleArray if a4 == 0.
    Label ok, fail;
    __ AssertNotSmi(a2);
    __ GetInstanceType(a2, t6, t6);
    __ Jump(&ok, eq, t6, Operand(FIXED_ARRAY_TYPE));
    __ Jump(&fail, ne, t6, Operand(FIXED_DOUBLE_ARRAY_TYPE));
    __ Jump(&ok, eq, a4, Operand(zero_reg));
    // Fall through.
    __ bind(&fail);
    __ Abort(AbortReason::kOperandIsNotAFixedArray);

    __ bind(&ok);
  }

  Register args = a2;
  Register len = a4;

  // Check for stack overflow.
  Label stack_overflow;
  Generate_StackOverflowCheck(masm, len, &stack_overflow);

  // Push arguments onto the stack (thisArgument is already on the stack).
  {
    Label done, push, loop;
    Register src = a6;
    Register scratch = len;

    __ addi(src, args, FixedArray::kHeaderSize - kHeapObjectTag);
    __ Jump(&done, eq, len, Operand(zero_reg));
    __ Dadd(a0, a0, len);  // The 'len' argument for Call() or Construct().
    __ slli(scratch, len, kPointerSizeLog2);
    __ Dsub(scratch, sp, Operand(scratch));
    __ LoadRoot(t1, RootIndex::kTheHoleValue);
    __ bind(&loop);
    __ Ld(a5, MemOperand(src));
    __ Jump(&push, ne, a5, Operand(t1));
    __ LoadRoot(a5, RootIndex::kUndefinedValue);
    __ bind(&push);
    __ addi(src, src, kPointerSize);
    __ Push(a5);
    __ Jump(&loop, ne, scratch, Operand(sp));
    __ bind(&done);
  }

  // Tail-call to the actual Call or Construct builtin.
  __ Jump(code, RelocInfo::CODE_TARGET);

  __ bind(&stack_overflow);
  __ TailCallRuntime(Runtime::kThrowStackOverflow);
}
// static
void Builtins::Generate_CallOrConstructForwardVarargs(MacroAssembler* masm,
                                                      CallOrConstructMode mode,
                                                      Handle<Code> code) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a3 : the new.target (for [[Construct]] calls)
  //  -- a1 : the target to call (can be any Object)
  //  -- a2 : start index (to support rest parameters)
  // -----------------------------------

  // Check if new.target has a [[Construct]] internal method.
  if (mode == CallOrConstructMode::kConstruct) {
    Label new_target_constructor, new_target_not_constructor;
    __ JumpIfSmi(a3, &new_target_not_constructor);
    __ ld(t1, FieldMemOperand(a3, HeapObject::kMapOffset));
    __ lbu(t1, FieldMemOperand(t1, Map::kBitFieldOffset));
    __ And(t1, t1, Operand(Map::Bits1::IsConstructorBit::kMask));
    __ Jump(&new_target_constructor, ne, t1, Operand(zero_reg));
    __ bind(&new_target_not_constructor);
    {
      FrameScope scope(masm, StackFrame::MANUAL);
      __ EnterFrame(StackFrame::INTERNAL);
      __ Push(a3);
      __ CallRuntime(Runtime::kThrowNotConstructor);
    }
    __ bind(&new_target_constructor);
  }

  // Check if we have an arguments adaptor frame below the function frame.
  Label arguments_adaptor, arguments_done;
  __ Ld(a6, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ Ld(a7, MemOperand(a6, CommonFrameConstants::kContextOrFrameTypeOffset));
  __ Jump(&arguments_adaptor, eq, a7,
          Operand(StackFrame::TypeToMarker(StackFrame::ARGUMENTS_ADAPTOR)));
  {
    __ Ld(a7, MemOperand(fp, StandardFrameConstants::kFunctionOffset));
    __ Ld(a7, FieldMemOperand(a7, JSFunction::kSharedFunctionInfoOffset));
    __ Lhu(a7, FieldMemOperand(
                   a7, SharedFunctionInfo::kFormalParameterCountOffset));
    __ mov(a6, fp);
  }
  __ Jump(&arguments_done);
  __ bind(&arguments_adaptor);
  {
    // Just get the length from the ArgumentsAdaptorFrame.
    __ SmiUntag(a7,
                MemOperand(a6, ArgumentsAdaptorFrameConstants::kLengthOffset));
  }
  __ bind(&arguments_done);

  Label stack_done, stack_overflow;
  __ Sub(a7, a7, a2);
  __ Jump(&stack_done, le, a7, Operand(zero_reg));
  {
    // Check for stack overflow.
    Generate_StackOverflowCheck(masm, a7, &stack_overflow);

    // Forward the arguments from the caller frame.
    {
      Label loop;
      __ Dadd(a0, a0, a7);
      __ bind(&loop);
      {
        __ GenScaledAddress(kScratchRegister, a6, a7, kPointerSizeLog2);
        __ Ld(kScratchRegister, MemOperand(kScratchRegister, 1 * kPointerSize));
        __ push(kScratchRegister);
        __ Sub(a7, a7, Operand(1));
        __ Jump(&loop, ne, a7, Operand(zero_reg));
      }
    }
  }
  __ Jump(&stack_done);
  __ bind(&stack_overflow);
  __ TailCallRuntime(Runtime::kThrowStackOverflow);
  __ bind(&stack_done);

  // Tail-call to the {code} handler.
  __ Jump(code, RelocInfo::CODE_TARGET);
}

// static
void Builtins::Generate_CallFunction(MacroAssembler* masm,
                                     ConvertReceiverMode mode) {
  ASM_LOCATION("Builtins::Generate_CallFunction");
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSFunction)
  // -----------------------------------
  __ AssertFunction(a1);

  // See ES6 section 9.2.1 [[Call]] ( thisArgument, argumentsList)
  // Check that function is not a "classConstructor".
  Label class_constructor;
  __ Ld(a2, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  __ Lwu(a3, FieldMemOperand(a2, SharedFunctionInfo::kFlagsOffset));
  __ And(t0, a3, SharedFunctionInfo::IsClassConstructorBit::kMask);
  __ Jump(&class_constructor, ne, t0, x0);
  // Enter the context of the function; ToObject has to run in the function
  // context, and we also need to take the global proxy from the function
  // context in case of conversion.
  __ Ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));
  // We need to convert the receiver for non-native sloppy mode functions.
  Label done_convert;
  __ And(t0, a3,
         SharedFunctionInfo::IsNativeBit::kMask |
             SharedFunctionInfo::IsStrictBit::kMask);
  __ Jump(&done_convert, eq, t0, x0);
  {
    // ----------- S t a t e ------------
    //  -- a0 : the number of arguments (not including the receiver)
    //  -- a1 : the function to call (checked to be a JSFunction)
    //  -- a2 : the shared function info.
    //  -- cp : the function context.
    // -----------------------------------
    if (mode == ConvertReceiverMode::kNullOrUndefined) {
      // Patch receiver to global proxy.
      __ LoadGlobalProxy(a3);
    } else {
      Label convert_to_object, convert_receiver;
      __ slli(t0, a0, kPointerSizeLog2);
      __ Dadd(t0, sp, t0);
      __ Ld(a3, MemOperand(t0));
      __ JumpIfSmi(a3, &convert_to_object);
      STATIC_ASSERT(LAST_JS_RECEIVER_TYPE == LAST_TYPE);
      __ GetInstanceType(a3, a4, a4);
      __ Jump(&done_convert, hs, a4, Operand(FIRST_JS_RECEIVER_TYPE));
      if (mode != ConvertReceiverMode::kNotNullOrUndefined) {
        Label convert_global_proxy;
        __ JumpIfRoot(a3, RootIndex::kUndefinedValue, &convert_global_proxy);
        __ JumpIfNotRoot(a3, RootIndex::kNullValue, &convert_to_object);
        __ bind(&convert_global_proxy);
        {
          // Patch receiver to global proxy.
          __ LoadGlobalProxy(a3);
        }
        __ jmp(&convert_receiver);
      }
      __ bind(&convert_to_object);
      {
        // Convert receiver using ToObject.
        // TODO(bmeurer): Inline the allocation here to avoid building the frame
        // in the fast case? (fall back to AllocateInNewSpace?)
        FrameScope scope(masm, StackFrame::INTERNAL);
        __ SmiTag(a0);
        __ Push(a0, a1, cp);
        __ Move(a0, a3);
        __ Call(BUILTIN_CODE(masm->isolate(), ToObject),
                RelocInfo::CODE_TARGET);
        __ Move(a3, a0);
        __ Pop(cp, a1, a0);
        __ SmiUntag(x0);
      }
      __ Ld(a2, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
      __ bind(&convert_receiver);
    }
    __ slli(a0, a0, kPointerSizeLog2);
    __ Dadd(t0, sp, a0);
    __ Sd(a3, MemOperand(t0));
  }
  __ bind(&done_convert);
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSFunction)
  //  -- a2 : the shared function info.
  //  -- cp : the function context.
  // -----------------------------------
  __ Lhu(a2,
        FieldMemOperand(a2, SharedFunctionInfo::kFormalParameterCountOffset));
  __ InvokeFunctionCode(a1, no_reg, a2, a0, JUMP_FUNCTION);
  // The function is a "classConstructor", need to raise an exception.
  __ bind(&class_constructor);
  {
    FrameScope frame(masm, StackFrame::INTERNAL);
    __ Push(a1);
    __ CallRuntime(Runtime::kThrowConstructorNonCallableError);
  }
}
#if 0
namespace {

void Generate_PushBoundArguments(MacroAssembler* masm) {
}

}  // namespace
#endif
// static
void Builtins::Generate_CallBoundFunctionImpl(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSBoundFunction)
  // -----------------------------------
  __ AssertBoundFunction(a1);

  // Patch the receiver to [[BoundThis]].
  {
    __ Ld(kScratchRegister,
          FieldMemOperand(a1, JSBoundFunction::kBoundThisOffset));
    __ GenScaledAddress(a4, sp, a0, kPointerSizeLog2);
    __ Sd(kScratchRegister, MemOperand(a4));
  }

  // Load [[BoundArguments]] into a2 and length of that into a4.
  __ Ld(a2, FieldMemOperand(a1, JSBoundFunction::kBoundArgumentsOffset));
  __ SmiUntag(a4, FieldMemOperand(a2, FixedArray::kLengthOffset));

  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSBoundFunction)
  //  -- a2 : the [[BoundArguments]] (implemented as FixedArray)
  //  -- a4 : the number of [[BoundArguments]]
  // -----------------------------------

  // Reserve stack space for the [[BoundArguments]].
  {
    Label done;
    __ slli(a5, a4, kPointerSizeLog2);
    __ Dsub(sp, sp, Operand(a5));
    // Check the stack for overflow. We are not trying to catch interruptions
    // (i.e. debug break and preemption) here, so check the "real stack limit".
    LoadStackLimit(masm, kScratchRegister, StackLimitKind::kRealStackLimit);
    __ Jump(&done, hs, sp, Operand(kScratchRegister));
    // Restore the stack pointer.
    __ Dadd(sp, sp, Operand(a5));
    {
      FrameScope scope(masm, StackFrame::MANUAL);
      __ EnterFrame(StackFrame::INTERNAL);
      __ CallRuntime(Runtime::kThrowStackOverflow);
    }
    __ bind(&done);
  }

  // Relocate arguments down the stack.
  {
    Label loop, done_loop;
    __ mov(a5, zero_reg);
    __ bind(&loop);
    __ Jump(&done_loop, gt, a5, Operand(a0));
    __ GenScaledAddress(a6, sp, a4, kPointerSizeLog2);
    __ Ld(kScratchRegister, MemOperand(a6));
    __ GenScaledAddress(a6, sp, a5, kPointerSizeLog2);
    __ Sd(kScratchRegister, MemOperand(a6));
    __ Dadd(a4, a4, Operand(1));
    __ Dadd(a5, a5, Operand(1));
    __ Jump(&loop);
    __ bind(&done_loop);
  }

  // Copy [[BoundArguments]] to the stack (below the arguments).
  {
    Label loop, done_loop;
    __ SmiUntag(a4, FieldMemOperand(a2, FixedArray::kLengthOffset));
    __ Dadd(a2, a2, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
    __ bind(&loop);
    __ Dsub(a4, a4, Operand(1));
    __ Jump(&done_loop, lt, a4, Operand(zero_reg));
    __ GenScaledAddress(a5, a2, a4, kPointerSizeLog2);
    __ Ld(kScratchRegister, MemOperand(a5));
    __ GenScaledAddress(a5, sp, a0, kPointerSizeLog2);
    __ Sd(kScratchRegister, MemOperand(a5));
    __ Dadd(a0, a0, Operand(1));
    __ Jump(&loop);
    __ bind(&done_loop);
  }

  // Call the [[BoundTargetFunction]] via the Call builtin.
  __ Ld(a1, FieldMemOperand(a1, JSBoundFunction::kBoundTargetFunctionOffset));
  __ Jump(BUILTIN_CODE(masm->isolate(), Call_ReceiverIsAny),
          RelocInfo::CODE_TARGET);
}

// static
void Builtins::Generate_Call(MacroAssembler* masm, ConvertReceiverMode mode) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the target to call (can be any Object).
  // -----------------------------------
  Label non_callable, non_smi;
  __ JumpIfSmi(a1, &non_callable);
  __ bind(&non_smi);
  __ GetInstanceType(a1, t1, t0);
  Label not_js_function, not_js_bound_function_type, not_js_proxy_type;
  __ RecordComment("[Jump(masm->isolate()->builtins()->CallFunction(mode),");
  __ Jump(masm->isolate()->builtins()->CallFunction(mode),
          RelocInfo::CODE_TARGET, eq, t0, Operand(JS_FUNCTION_TYPE));
  __ RecordComment("]");

  __ Jump(BUILTIN_CODE(masm->isolate(), CallBoundFunction),
          RelocInfo::CODE_TARGET, eq, t0, Operand(JS_BOUND_FUNCTION_TYPE));

  // Check if target has a [[Call]] internal method.
  __ Ld(t1, FieldMemOperand(t1, Map::kBitFieldOffset));
  __ And(t1, t1, Map::Bits1::IsCallableBit::kMask);
  __ Jump(&non_callable, eq, t1, Operand(x0));

  __ Jump(BUILTIN_CODE(masm->isolate(), CallProxy), RelocInfo::CODE_TARGET, eq,
          t0, Operand(JS_PROXY_TYPE));

  // 2. Call to something else, which might have a [[Call]] internal method (if
  // not we raise an exception).
  // Overwrite the original receiver with the (original) target.
  __ slli(t0, a0, kPointerSizeLog2);
  __ Dadd(t0, sp, t0);
  __ Sd(a1, MemOperand(t0));
  // Let the "call_as_function_delegate" take care of the rest.
  __ LoadNativeContextSlot(Context::CALL_AS_FUNCTION_DELEGATE_INDEX, a1);
  __ Jump(masm->isolate()->builtins()->CallFunction(
              ConvertReceiverMode::kNotNullOrUndefined),
          RelocInfo::CODE_TARGET);

  // 3. Call to something that is not callable.
  __ bind(&non_callable);
  {
    FrameScope scope(masm, StackFrame::INTERNAL);
    __ Push(a1);
    __ CallRuntime(Runtime::kThrowCalledNonCallable);
  }
}

// static
void Builtins::Generate_ConstructFunction(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the constructor to call (checked to be a JSFunction)
  //  -- a3 : the new target (checked to be a constructor)
  // -----------------------------------
  __ AssertConstructor(a1);
  __ AssertFunction(a1);

  // Calling convention for function specific ConstructStubs require
  // a2 to contain either an AllocationSite or undefined.
  __ LoadRoot(a2, RootIndex::kUndefinedValue);

  Label call_generic_stub;

  // Jump to JSBuiltinsConstructStub or JSConstructStubGeneric.
  __ Ld(a4, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  __ Lwu(a4, FieldMemOperand(a4, SharedFunctionInfo::kFlagsOffset));
  __ And(a4, a4, Operand(SharedFunctionInfo::ConstructAsBuiltinBit::kMask));
  __ Jump(&call_generic_stub, eq, a4, Operand(zero_reg));

  __ Jump(BUILTIN_CODE(masm->isolate(), JSBuiltinsConstructStub),
          RelocInfo::CODE_TARGET);

  __ bind(&call_generic_stub);
  __ Jump(BUILTIN_CODE(masm->isolate(), JSConstructStubGeneric),
          RelocInfo::CODE_TARGET);
}

// static
void Builtins::Generate_ConstructBoundFunction(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSBoundFunction)
  //  -- a3 : the new target (checked to be a constructor)
  // -----------------------------------
  __ AssertConstructor(a1);
  __ AssertBoundFunction(a1);

  // Load [[BoundArguments]] into a2 and length of that into a4.
  __ Ld(a2, FieldMemOperand(a1, JSBoundFunction::kBoundArgumentsOffset));
  __ SmiUntag(a4, FieldMemOperand(a2, FixedArray::kLengthOffset));

  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the function to call (checked to be a JSBoundFunction)
  //  -- a3 : the new target (checked to be a constructor)
  // -----------------------------------
  // Reserve stack space for the [[BoundArguments]].
  {
    Label done;
    __ slli(a5, a4, kPointerSizeLog2);
    __ Dsub(sp, sp, Operand(a5));
    // Check the stack for overflow. We are not trying to catch interruptions
    // (i.e. debug break and preemption) here, so check the "real stack limit".
    LoadStackLimit(masm, kScratchRegister, StackLimitKind::kRealStackLimit);
    __ Jump(&done, hs, sp, Operand(kScratchRegister));
    // Restore the stack pointer.
    __ Dadd(sp, sp, Operand(a5));
    {
      FrameScope scope(masm, StackFrame::MANUAL);
      __ EnterFrame(StackFrame::INTERNAL);
      __ CallRuntime(Runtime::kThrowStackOverflow);
    }
    __ bind(&done);
  }

  // Relocate arguments down the stack.
  {
    Label loop, done_loop;
    __ mov(a5, zero_reg);
    __ bind(&loop);
    __ Jump(&done_loop, ge, a5, Operand(a0));
    __ GenScaledAddress(a6, sp, a4, kPointerSizeLog2);
    __ Ld(kScratchRegister, MemOperand(a6));
    __ GenScaledAddress(a6, sp, a5, kPointerSizeLog2);
    __ Sd(kScratchRegister, MemOperand(a6));
    __ Dadd(a4, a4, Operand(1));
    __ Dadd(a5, a5, Operand(1));
    __ Jump(&loop);
    __ bind(&done_loop);
  }

  // Copy [[BoundArguments]] to the stack (below the arguments).
  {
    Label loop, done_loop;
    __ SmiUntag(a4, FieldMemOperand(a2, FixedArray::kLengthOffset));
    __ Dadd(a2, a2, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
    __ bind(&loop);
    __ Dsub(a4, a4, Operand(1));
    __ Jump(&done_loop, lt, a4, Operand(zero_reg));
    __ GenScaledAddress(a5, a2, a4, kPointerSizeLog2);
    __ Ld(kScratchRegister, MemOperand(a5));
    __ GenScaledAddress(a5, sp, a0, kPointerSizeLog2);
    __ Sd(kScratchRegister, MemOperand(a5));
    __ Dadd(a0, a0, Operand(1));
    __ Jump(&loop);
    __ bind(&done_loop);
  }

  // Patch new.target to [[BoundTargetFunction]] if new.target equals target.
  {
    Label skip_load;
    __ Jump(&skip_load, ne, a1, Operand(a3));
    __ Ld(a3, FieldMemOperand(a1, JSBoundFunction::kBoundTargetFunctionOffset));
    __ bind(&skip_load);
  }

  // Construct the [[BoundTargetFunction]] via the Construct builtin.
  __ Ld(a1, FieldMemOperand(a1, JSBoundFunction::kBoundTargetFunctionOffset));
  __ Jump(BUILTIN_CODE(masm->isolate(), Construct), RelocInfo::CODE_TARGET);
}

// static
void Builtins::Generate_Construct(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0 : the number of arguments (not including the receiver)
  //  -- a1 : the constructor to call (can be any Object)
  //  -- a3 : the new target (either the same as the constructor or
  //          the JSFunction on which new was invoked initially)
  // -----------------------------------
  // Check if target is a Smi.
  Label non_constructor, non_proxy;
  __ JumpIfSmi(a1, &non_constructor);

  // Check if target has a [[Construct]] internal method.
  __ Ld(t1, FieldMemOperand(a1, HeapObject::kMapOffset));
  __ Lbu(t3, FieldMemOperand(t1, Map::kBitFieldOffset));
  __ And(t3, t3, Operand(Map::Bits1::IsConstructorBit::kMask));
  __ Jump(&non_constructor, eq, t3, Operand(zero_reg));

  // Dispatch based on instance type.
  __ Lhu(t2, FieldMemOperand(t1, Map::kInstanceTypeOffset));
  __ Jump(BUILTIN_CODE(masm->isolate(), ConstructFunction),
          RelocInfo::CODE_TARGET, eq, t2, Operand(JS_FUNCTION_TYPE));

  // Only dispatch to bound functions after checking whether they are
  // constructors.
  __ Jump(BUILTIN_CODE(masm->isolate(), ConstructBoundFunction),
          RelocInfo::CODE_TARGET, eq, t2, Operand(JS_BOUND_FUNCTION_TYPE));

  // Only dispatch to proxies after checking whether they are constructors.
  __ Jump(&non_proxy, ne, t2, Operand(JS_PROXY_TYPE));
  __ Jump(BUILTIN_CODE(masm->isolate(), ConstructProxy),
          RelocInfo::CODE_TARGET);

  // Called Construct on an exotic Object with a [[Construct]] internal method.
  __ bind(&non_proxy);
  {
    // Overwrite the original receiver with the (original) target.
    __ GenScaledAddress(kScratchRegister, sp, a0, kPointerSizeLog2);
    __ Sd(a1, MemOperand(kScratchRegister));
    // Let the "call_as_constructor_delegate" take care of the rest.
    __ LoadNativeContextSlot(Context::CALL_AS_CONSTRUCTOR_DELEGATE_INDEX, a1);
    __ Jump(masm->isolate()->builtins()->CallFunction(),
            RelocInfo::CODE_TARGET);
  }

  // Called Construct on an Object that doesn't have a [[Construct]] internal
  // method.
  __ bind(&non_constructor);
  __ Jump(BUILTIN_CODE(masm->isolate(), ConstructedNonConstructable),
          RelocInfo::CODE_TARGET);
}

void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // ASM_LOCATION("Builtins::Generate_ArgumentsAdaptorTrampoline");
  // ----------- S t a t e -------------
  //  -- a0 : actual number of arguments
  //  -- a1 : function (passed through to callee)
  //  -- a2 : expected number of arguments
  //  -- a3 : new target (passed through to callee)
  // -----------------------------------

  // The frame we are about to construct will look like:
  //
  //  slot      Adaptor frame
  //       +-----------------+--------------------------------
  //  -n-1 |    receiver     |                            ^
  //       |  (parameter 0)  |                            |
  //       |- - - - - - - - -|                            |
  //  -n   |                 |                          Caller
  //  ...  |       ...       |                       frame slots --> actual args
  //  -2   |  parameter n-1  |                            |
  //       |- - - - - - - - -|                            |
  //  -1   |   parameter n   |                            v
  //  -----+-----------------+--------------------------------
  //   0   |   return addr   |                            ^
  //       |- - - - - - - - -|                            |
  //   1   | saved frame ptr | <-- frame ptr              |
  //       |- - - - - - - - -|                            |
  //   2   |Frame Type Marker|                            |
  //       |- - - - - - - - -|                            |
  //   3   |    function     |                          Callee
  //       |- - - - - - - - -|                        frame slots
  //   4   |     num of      |                            |
  //       |   actual args   |                            |
  //       |- - - - - - - - -|                            |
  //   5   |     padding     |                            |
  //       |-----------------+----                        |
  //  [6]  |    [padding]    |   ^                        |
  //       |- - - - - - - - -|   |                        |
  // 6+pad |    receiver     |   |                        |
  //       |  (parameter 0)  |   |                        |
  //       |- - - - - - - - -|   |                        |
  // 7+pad |   parameter 1   |   |                        |
  //       |- - - - - - - - -| Frame slots ----> expected args
  // 8+pad |   parameter 2   |   |                        |
  //       |- - - - - - - - -|   |                        |
  //       |                 |   |                        |
  //  ...  |       ...       |   |                        |
  //       |   parameter m   |   |                        |
  //       |- - - - - - - - -|   |                        |
  //       |   [undefined]   |   |                        |
  //       |- - - - - - - - -|   |                        |
  //       |                 |   |                        |
  //       |       ...       |   |                        |
  //       |   [undefined]   |   v   <-- stack ptr        v
  //  -----+-----------------+---------------------------------
  //
  // There is an optional slot of padding above the receiver to ensure stack
  // alignment of the arguments.
  // If the number of expected arguments is larger than the number of actual
  // arguments, the remaining expected slots will be filled with undefined.
  Label invoke, dont_adapt_arguments, stack_overflow;

  Label enough, too_few;
  __ Jump(&dont_adapt_arguments, eq, a2, Operand(kDontAdaptArgumentsSentinel));
  // We use Uless as the number of argument should always be greater than 0.
  __ Jump(&too_few, lo, a0, Operand(a2));

  {  // Enough parameters: actual >= expected.
    // a0: actual number of arguments
    // a1: function
    // a2: expected number of arguments
    // a3: new target (passed through to callee)
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);
    Generate_StackOverflowCheck(masm, a2, &stack_overflow);

    // Calculate copy start address into a0 and copy end address into a4.
    __ slli(a0, a0, kPointerSizeLog2);
    __ Dadd(a0, fp, a0);
    // Adjust for return address and receiver.
    __ Dadd(a0, a0, Operand(2 * kPointerSize));
    // Compute copy end address.
    __ slli(a4, a2, kPointerSizeLog2);
    __ Dsub(a4, a0, Operand(a4));

    // Copy the arguments (including the receiver) to the new stack frame.
    // a0: copy start address
    // a1: function
    // a2: expected number of arguments
    // a3: new target (passed through to callee)
    // a4: copy end address

    Label copy;
    __ bind(&copy);
    __ Ld(a5, MemOperand(a0));
    __ push(a5);
    __ addi(a0, a0, -kPointerSize);
    __ Jump(&copy, ne, a0, Operand(a4));

    __ jmp(&invoke);
  }

  {  // Too few parameters: Actual < expected.
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);
    Generate_StackOverflowCheck(masm, a2, &stack_overflow);

    // Calculate copy start address into a0 and copy end address into a7.
    // a0: actual number of arguments
    // a1: function
    // a2: expected number of arguments
    // a3: new target (passed through to callee)
    __ slli(a0, a0, kPointerSizeLog2);
    __ Dadd(a0, fp, a0);
    // Adjust for return address and receiver.
    __ Dadd(a0, a0, Operand(2 * kPointerSize));
    // Compute copy end address. Also adjust for return address.
    __ Dadd(a7, fp, kPointerSize);

    // Copy the arguments (including the receiver) to the new stack frame.
    // a0: copy start address
    // a1: function
    // a2: expected number of arguments
    // a3: new target (passed through to callee)
    // a7: copy end address
    Label copy;
    __ bind(&copy);
    __ Ld(a4, MemOperand(a0));  // Adjusted above for return addr and receiver.
    __ Dsub(sp, sp, kPointerSize);
    __ Sd(a4, MemOperand(sp));  // In the delay slot.
    __ Dsub(a0, a0, kPointerSize);
    __ Jump(&copy, ne, a0, Operand(a7));

    // Fill the remaining expected arguments with undefined.
    // a1: function
    // a2: expected number of arguments
    // a3: new target (passed through to callee)
    __ LoadRoot(a5, RootIndex::kUndefinedValue);
    __ slli(a6, a2, kPointerSizeLog2);
    __ Dsub(a4, fp, Operand(a6));
    // Adjust for frame.
    __ Dsub(a4, a4,
            Operand(ArgumentsAdaptorFrameConstants::kFixedFrameSizeFromFp +
                    kPointerSize));

    Label fill;
    __ bind(&fill);
    __ Dsub(sp, sp, kPointerSize);
    __ Sd(a5, MemOperand(sp));
    __ Jump(&fill, ne, sp, Operand(a4));
  }

  // Call the entry point.
  __ bind(&invoke);
  __ mov(a0, a2);
  // a0 : expected number of arguments
  // a1 : function (passed through to callee)
  // a3: new target (passed through to callee)
  static_assert(kJavaScriptCallCodeStartRegister == a2, "ABI mismatch");
  __ LoadTaggedPointerField(
      a2, FieldMemOperand(a1, JSFunction::kCodeOffset));
  __ CallCodeObject(a2);

  // Store offset of return address for deoptimizer.
  masm->isolate()->heap()->SetArgumentsAdaptorDeoptPCOffset(masm->pc_offset());

  // Exit frame and return.
  LeaveArgumentsAdaptorFrame(masm);
  __ Ret();

  // -------------------------------------------
  // Don't adapt arguments.
  // -------------------------------------------
  __ bind(&dont_adapt_arguments);
  static_assert(kJavaScriptCallCodeStartRegister == a2, "ABI mismatch");
  __ LoadTaggedPointerField(
      a2, FieldMemOperand(a1, JSFunction::kCodeOffset));
  __ CallCodeObject(a2);

  __ bind(&stack_overflow);
  {
    FrameScope frame(masm, StackFrame::MANUAL);
    __ CallRuntime(Runtime::kThrowStackOverflow);
    __ DebugBreak();
  }
}

void Builtins::Generate_WasmCompileLazy(MacroAssembler* masm) {
  // The function index was put in t0 by the jump table trampoline.
  // Convert to Smi for the runtime call
  __ SmiTag(kWasmCompileLazyFuncIndexRegister);
  {
    HardAbortScope hard_abort(masm);  // Avoid calls to Abort.
    FrameScope scope(masm, StackFrame::WASM_COMPILE_LAZY);

    // Save all parameter registers (see wasm-linkage.cc). They might be
    // overwritten in the runtime call below. We don't have any callee-saved
    // registers in wasm, so no need to store anything else.
    // constexpr RegList gp_regs = Register::ListOf(a0, a2, a3, a4, a5, a6, a7);
    // constexpr RegList fp_regs =
    //     DoubleRegister::ListOf(f2, f4, f6, f8, f10, f12, f14);
    __ Push(a7, a6, a5, a4);
    __ Push(a3, a2, a1, a0);

    // __ Push(fa7, fa6, fa5, fa4);
    // __ Push(fa3, fa2, fa1, fa0);

    // Pass instance and function index as an explicit arguments to the runtime
    // function.
    __ Push(kWasmInstanceRegister, kWasmCompileLazyFuncIndexRegister);
    // Initialize the JavaScript context with 0. CEntry will use it to
    // set the current context on the isolate.
    __ Move(kContextRegister, Smi::zero());
    __ CallRuntime(Runtime::kWasmCompileLazy, 2);
    __ mov(t0, a0);

    // Restore registers.
    // __ Pop(fa0, fa1, fa2, fa3);
    // __ Pop(fa4, fa5, fa6, fa7);

    __ Pop(a0, a1, a2, a3);
    __ Pop(a4, a5, a6, a7);
  }
  // Finally, jump to the entrypoint.
  __ Jump(t0);
}

void Builtins::Generate_CEntry(MacroAssembler* masm, int result_size,
                               SaveFPRegsMode save_doubles, ArgvMode argv_mode,
                               bool builtin_exit_frame) {
  // Called from JavaScript; parameters are on stack as if calling JS function
  // a0: number of arguments including receiver
  // a1: pointer to builtin function
  // fp: frame pointer    (restored after C call)
  // sp: stack pointer    (restored as callee's sp after C call)
  // cp: current context  (C callee-saved)
  //
  // If argv_mode == kArgvInRegister:
  // a2: pointer to the first argument

  // The stack on entry holds the arguments and the receiver, with the receiver
  // at the highest address:
  //
  //    sp[argc-1]: receiver
  //    sp[argc-2]: arg[argc-2]
  //    ...           ...
  //    sp[1]:      arg[1]
  //    sp[0]:      arg[0]
  //
  // The arguments are in reverse order, so that arg[argc-2] is actually the
  // first argument to the target function and arg[0] is the last.

  if (argv_mode == kArgvInRegister) {
    // Move argv into the correct register.
    __ mov(s3, a2);
  } else {
    // Compute the argv pointer in a callee-saved register.
    __ GenScaledAddress(s3, sp, a0, kPointerSizeLog2);
    __ Dsub(s3, s3, kPointerSize);
  }

  // Enter the exit frame that transitions from JavaScript to C++.
  FrameScope scope(masm, StackFrame::MANUAL);
  __ EnterExitFrame(
      save_doubles == kSaveFPRegs, t0, 4,
      builtin_exit_frame ? StackFrame::BUILTIN_EXIT : StackFrame::EXIT);

  __ Sd(s2, MemOperand(sp, 1 * kSystemPointerSize));
  __ Sd(s3, MemOperand(sp, 2 * kSystemPointerSize));
  __ Sd(s4, MemOperand(sp, 3 * kSystemPointerSize));
  // s2: number of arguments  including receiver (C callee-saved)
  // s3: pointer to first argument (C callee-saved)
  // s4: pointer to builtin function (C callee-saved)

  // Prepare arguments for C routine.
  // a0 = argc
  __ mov(s2, a0);
  __ mov(s4, a1);

  // We are calling compiled C/C++ code. a0 and a1 hold our two arguments. We
  // also need to reserve the 4 argument slots on the stack.

  //__ AssertStackIsAligned();

  // a0 = argc, a1 = argv, a2 = isolate
  __ li(a2, ExternalReference::isolate_address(masm->isolate()));
  __ mov(a1, s3);

  __ StoreReturnAddressAndCall(s4);

  // Result returned in a0 or a1:a0 - do not destroy these registers!

  //  a0    result0      The return code from the call.
  //  a1    result1      For calls which return ObjectPair.
  //  s2   argc
  //  s3   argv
  //  s4   target

  // Check result for exception sentinel.
  Label exception_returned;
  __ LoadRoot(a4, RootIndex::kException);
  __ Jump(&exception_returned, eq, a4, Operand(a0));
  // The call succeeded, so unwind the stack and return.
  __ Move(t0, s2);
  // Restore callee-saved registers x21-x23.
  __ Ld(s2, MemOperand(sp, 1 * kSystemPointerSize));
  __ Ld(s3, MemOperand(sp, 2 * kSystemPointerSize));
  __ Ld(s4, MemOperand(sp, 3 * kSystemPointerSize));

  // Check that there is no pending exception, otherwise we
  // should have returned the exception sentinel.
  if (FLAG_debug_code) {
    Label okay;
    ExternalReference pending_exception_address = ExternalReference::Create(
        IsolateAddressId::kPendingExceptionAddress, masm->isolate());
    __ li(a2, pending_exception_address);
    __ Ld(a2, MemOperand(a2));
    __ LoadRoot(a4, RootIndex::kTheHoleValue);
    // Cannot use check here as it attempts to generate call into runtime.
    __ Jump(&okay, eq, a4, Operand(a2));
    __ DebugBreak();
    __ bind(&okay);
  }

  // Exit C frame and return.
  // a0,a1: result
  // t0: argc
  // sp: stack pointer
  // fp: frame pointer
  __ LeaveExitFrame(save_doubles == kSaveFPRegs, t1, t2, 4);
  if (argv_mode == kArgvOnStack) {
    // Drop the remaining stack slots and return from the stub.
    __ Dsub(sp, sp, Operand(t0));
  }
  __ Ret();

  // Handling of exception.
  __ bind(&exception_returned);

  ExternalReference pending_handler_context_address = ExternalReference::Create(
      IsolateAddressId::kPendingHandlerContextAddress, masm->isolate());
  ExternalReference pending_handler_entrypoint_address =
      ExternalReference::Create(
          IsolateAddressId::kPendingHandlerEntrypointAddress, masm->isolate());
  ExternalReference pending_handler_fp_address = ExternalReference::Create(
      IsolateAddressId::kPendingHandlerFPAddress, masm->isolate());
  ExternalReference pending_handler_sp_address = ExternalReference::Create(
      IsolateAddressId::kPendingHandlerSPAddress, masm->isolate());

  // Ask the runtime for help to determine the handler. This will set a0 to
  // contain the current pending exception, don't clobber it.
  ExternalReference find_handler =
      ExternalReference::Create(Runtime::kUnwindAndFindExceptionHandler);
  {
    FrameScope scope(masm, StackFrame::MANUAL);
    __ PrepareCallCFunction(3, 0, a0);
    __ Move(a0, x0);
    __ Move(a1, x0);
    __ li(a2, ExternalReference::isolate_address(masm->isolate()));
    __ CallCFunction(find_handler, 3);
  }

  // Retrieve the handler context, SP and FP.
  __ li(cp, pending_handler_context_address);
  __ Ld(cp, MemOperand(cp));
  __ li(sp, pending_handler_sp_address);
  __ Ld(sp, MemOperand(sp));
  __ li(fp, pending_handler_fp_address);
  __ Ld(fp, MemOperand(fp));

  // If the handler is a JS frame, restore the context to the frame. Note that
  // the context will be set to (cp == 0) for non-JS frames.
  Label zero;
  __ Jump(&zero, eq, cp, Operand(zero_reg));
  __ Sd(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ bind(&zero);

  // Reset the masking register. This is done independent of the underlying
  // feature flag {FLAG_untrusted_code_mitigations} to make the snapshot work
  // with both configurations. It is safe to always do this, because the
  // underlying register is caller-saved and can be arbitrarily clobbered.
  __ ResetSpeculationPoisonRegister();

  // Compute the handler entry address and jump to it.
  __ li(t0, pending_handler_entrypoint_address);
  __ Ld(t0, MemOperand(t0));
  __ Jump(t0);
}

void Builtins::Generate_DoubleToI(MacroAssembler* masm) {
  Label done;
  Register result = t0;

  HardAbortScope hard_abort(masm);  // Avoid calls to Abort.
  UseScratchRegisterScope temps(masm);
  Register scratch1 = temps.Acquire();
  Register scratch2 = temps.Acquire();
  DoubleRegister double_scratch = ft11;

  // Account for saved regs.
  const int kArgumentOffset = 2 * kSystemPointerSize;

  __ Push(result, scratch1);  // scratch1 is also pushed to preserve alignment.
  __ Fld(double_scratch, MemOperand(sp, kArgumentOffset));

  // Try to convert with a FPU convert instruction.  This handles all
  // non-saturating cases.
  __ TryConvertDoubleToInt64(result, double_scratch, &done);
  __ Fmove(result, double_scratch);

  // If we reach here we need to manually convert the input to an int32.

  // Extract the exponent.
  Register exponent = scratch1;
  {
    static const int shift = HeapNumber::kMantissaBits;
    DCHECK_LT(HeapNumber::kExponentBits, 64);
    static const uint64_t mask =
        ((1UL) << (64 - HeapNumber::kExponentBits)) - 1;
    __ srli(exponent, result, shift);
    __ And(exponent, exponent, Operand(mask));
  }

  // It the exponent is >= 84 (kMantissaBits + 32), the result is always 0 since
  // the mantissa gets shifted completely out of the int32_t result.
  Label not_zero;
  __ Jump(&not_zero, lt, exponent,
          HeapNumber::kExponentBias + HeapNumber::kMantissaBits + 32);
  __ Move(result, zero_reg);
  __ Jump(&done);
  __ bind(&not_zero);

  // The Fcvtzs sequence handles all cases except where the conversion causes
  // signed overflow in the int64_t target. Since we've already handled
  // exponents >= 84, we can guarantee that 63 <= exponent < 84.

  if (masm->emit_debug_code()) {
    // Exponents less than this should have been handled by the Fcvt case.
    Label L;
    __ Jump(&L, ge, exponent, HeapNumber::kExponentBias + 63);
    __ Abort(AbortReason::kUnexpectedValue);
    __ bind(&L);
  }

  // Isolate the mantissa bits, and set the implicit '1'.
  Register mantissa = scratch2;
  {
    DCHECK_LT(HeapNumber::kMantissaBits, 64);
    static const uint64_t mask =
        ((1UL) << (64 - HeapNumber::kMantissaBits)) - 1;
    __ And(mantissa, mantissa, Operand(mask));
  }

  __ Or(mantissa, mantissa, Operand(1ULL << HeapNumber::kMantissaBits));

  // Negate the mantissa if necessary.
  __ And(result, result, Operand(kXSignMask));
  Label zero;
  __ Jump(&zero, eq, result, x0);
  __ Sub(mantissa, zero_reg, mantissa);
  __ bind(&zero);

  // Shift the mantissa bits in the correct place. We know that we have to shift
  // it left here, because exponent >= 63 >= kMantissaBits.
  __ Sub(exponent, exponent,
         HeapNumber::kExponentBias + HeapNumber::kMantissaBits);
  __ sll(result, mantissa, exponent);

  __ bind(&done);
  __ Sd(result, MemOperand(sp, kArgumentOffset));
  __ Pop(scratch1, result);
  __ Ret();
}
namespace {

// The number of register that CallApiFunctionAndReturn will need to save on
// the stack. The space for these registers need to be allocated in the
// ExitFrame before calling CallApiFunctionAndReturn.
constexpr int kCallApiFunctionSpillSpace = 4;

int AddressOffset(ExternalReference ref0, ExternalReference ref1) {
  return static_cast<int>(ref0.address() - ref1.address());
}

// Calls an API function. Allocates HandleScope, extracts returned value
// from handle and propagates exceptions.
// 'stack_space' is the space to be unwound on exit (includes the call JS
// arguments space and the additional space allocated for the fast call).
// 'spill_offset' is the offset from the stack pointer where
// CallApiFunctionAndReturn can spill registers.
void CallApiFunctionAndReturn(MacroAssembler* masm, Register function_address,
                              ExternalReference thunk_ref, int stack_space,
                              MemOperand* stack_space_operand,
                              MemOperand return_value_operand) {
  Isolate* isolate = masm->isolate();
  ExternalReference next_address =
      ExternalReference::handle_scope_next_address(isolate);
  const int kNextOffset = 0;
  const int kLimitOffset = AddressOffset(
      ExternalReference::handle_scope_limit_address(isolate), next_address);
  const int kLevelOffset = AddressOffset(
      ExternalReference::handle_scope_level_address(isolate), next_address);

  DCHECK(function_address == a1 || function_address == a2);

  Label profiler_enabled, end_profiler_check;
  __ li(t6, ExternalReference::is_profiling_address(isolate));
  __ Lb(t6, MemOperand(t6, 0));
  __ Jump(&profiler_enabled, ne, t6, Operand(zero_reg));
  __ li(t6, ExternalReference::address_of_runtime_stats_flag());
  __ Lw(t6, MemOperand(t6, 0));
  __ Jump(&profiler_enabled, ne, t6, Operand(zero_reg));
  {
    // Call the api function directly.
    __ mov(t6, function_address);
    __ Jump(&end_profiler_check);
  }
  __ bind(&profiler_enabled);
  {
    // Additional parameter is the address of the actual callback.
    __ li(t6, thunk_ref);
  }
  __ bind(&end_profiler_check);
  // Allocate HandleScope in callee-save registers.
  __ li(s5, next_address);
  __ Ld(s0, MemOperand(s5, kNextOffset));
  __ Ld(s1, MemOperand(s5, kLimitOffset));
  __ Lw(s2, MemOperand(s5, kLevelOffset));
  __ Add(s2, s2, Operand(1));
  __ Sw(s2, MemOperand(s5, kLevelOffset));

  __ StoreReturnAddressAndCall(t6);

  Label promote_scheduled_exception;
  Label delete_allocated_handles;
  Label leave_exit_frame;
  Label return_value_loaded;

  // Load value from ReturnValue.
  __ Ld(t0, return_value_operand);
  __ bind(&return_value_loaded);

  // No more valid handles (the result handle was the last one). Restore
  // previous handle scope.
  __ Sd(s0, MemOperand(s5, kNextOffset));
  if (__ emit_debug_code()) {
    __ Lw(a1, MemOperand(s5, kLevelOffset));
    Label L;
    __ Jump(&L, eq, a1, Operand(s2));
    __ Abort(AbortReason::kUnexpectedLevelAfterReturnFromApiCall);
    __ bind(&L);
  }
  __ Sub(s2, s2, Operand(1));
  __ Sw(s2, MemOperand(s5, kLevelOffset));
  __ Ld(kScratchRegister, MemOperand(s5, kLimitOffset));
  __ Jump(&delete_allocated_handles, ne, s1, Operand(kScratchRegister));

  // Leave the API exit frame.
  __ bind(&leave_exit_frame);

  if (stack_space_operand == nullptr) {
    DCHECK_NE(stack_space, 0);
    __ li(s0, Operand(stack_space));
  } else {
    DCHECK_EQ(stack_space, 0);
    STATIC_ASSERT(kCArgSlotCount == 0);
    __ Ld(s0, *stack_space_operand);
  }

  static constexpr bool kDontSaveDoubles = false;
  __ LeaveExitFrame(kDontSaveDoubles, t4, t5, kCallApiFunctionSpillSpace);

  // Check if the function scheduled an exception.
  __ LoadRoot(a4, RootIndex::kTheHoleValue);
  __ li(kScratchRegister,
        ExternalReference::scheduled_exception_address(isolate));
  __ Ld(a5, MemOperand(kScratchRegister));
  __ Jump(&promote_scheduled_exception, ne, a4, Operand(a5));

  __ Ret();

  // Re-throw by promoting a scheduled exception.
  __ bind(&promote_scheduled_exception);
  __ TailCallRuntime(Runtime::kPromoteScheduledException);

  // HandleScope limit has changed. Delete allocated extensions.
  __ bind(&delete_allocated_handles);
  __ Sd(s1, MemOperand(s5, kLimitOffset));
  __ mov(s0, t0);
  __ mov(a0, t0);
  __ PrepareCallCFunction(1, s1);
  __ li(a0, ExternalReference::isolate_address(isolate));
  __ CallCFunction(ExternalReference::delete_handle_scope_extensions(), 1);
  __ mov(t0, s0);
  __ jmp(&leave_exit_frame);
}
}  // namespace

void Builtins::Generate_CallApiCallback(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- cp                  : context
  //  -- a1                  : api function address
  //  -- a2                  : arguments count (not including the receiver)
  //  -- a3                  : call data
  //  -- a0                  : holder
  //  --
  //  -- sp[0]               : last argument
  //  -- ...
  //  -- sp[(argc - 1) * 8]  : first argument
  //  -- sp[(argc + 0) * 8]  : receiver
  // -----------------------------------
  Register api_function_address = a1;
  Register argc = a2;
  Register call_data = a3;
  Register holder = a0;
  Register scratch = t0;
  Register base = t1;  // For addressing MemOperands on the stack.
  DCHECK(!AreAliased(api_function_address, argc, call_data, holder, scratch,
                     base));
  using FCA = FunctionCallbackArguments;
  STATIC_ASSERT(FCA::kArgsLength == 6);
  STATIC_ASSERT(FCA::kNewTargetIndex == 5);
  STATIC_ASSERT(FCA::kDataIndex == 4);
  STATIC_ASSERT(FCA::kReturnValueOffset == 3);
  STATIC_ASSERT(FCA::kReturnValueDefaultValueIndex == 2);
  STATIC_ASSERT(FCA::kIsolateIndex == 1);
  STATIC_ASSERT(FCA::kHolderIndex == 0);
  // Set up FunctionCallbackInfo's implicit_args on the stack as follows:
  //
  // Target state:
  //   sp[0 * kPointerSize]: kHolder
  //   sp[1 * kPointerSize]: kIsolate
  //   sp[2 * kPointerSize]: undefined (kReturnValueDefaultValue)
  //   sp[3 * kPointerSize]: undefined (kReturnValue)
  //   sp[4 * kPointerSize]: kData
  //   sp[5 * kPointerSize]: undefined (kNewTarget)

  // Set up the base register for addressing through MemOperands. It will point
  // at the receiver (located at sp + argc * kPointerSize).
  __ GenScaledAddress(base, sp, argc, kPointerSizeLog2);

  // Reserve space on the stack.
  __ Dsub(sp, sp, Operand(FCA::kArgsLength * kPointerSize));

  // kHolder.
  __ Sd(holder, MemOperand(sp, 0 * kPointerSize));

  // kIsolate.
  __ li(scratch, ExternalReference::isolate_address(masm->isolate()));
  __ Sd(scratch, MemOperand(sp, 1 * kPointerSize));

  // kReturnValueDefaultValue and kReturnValue.
  __ LoadRoot(scratch, RootIndex::kUndefinedValue);
  __ Sd(scratch, MemOperand(sp, 2 * kPointerSize));
  __ Sd(scratch, MemOperand(sp, 3 * kPointerSize));

  // kData.
  __ Sd(call_data, MemOperand(sp, 4 * kPointerSize));

  // kNewTarget.
  __ Sd(scratch, MemOperand(sp, 5 * kPointerSize));

  // Keep a pointer to kHolder (= implicit_args) in a scratch register.
  // We use it below to set up the FunctionCallbackInfo object.
  __ Move(scratch, sp);

  // Allocate the v8::Arguments structure in the arguments' space since
  // it's not controlled by GC.
  static constexpr bool kDontSaveDoubles = false;
  FrameScope frame_scope(masm, StackFrame::MANUAL);
  __ EnterExitFrame(kDontSaveDoubles, t2, kCallApiFunctionSpillSpace);

  // EnterExitFrame may align the sp.

  // FunctionCallbackInfo::implicit_args_ (points at kHolder as set up above).
  // Arguments are after the return address (pushed by EnterExitFrame()).
  __ Sd(scratch, MemOperand(sp, 1 * kPointerSize));

  // FunctionCallbackInfo::values_ (points at the first varargs argument passed
  // on the stack).
  __ Dsub(scratch, base, Operand(1 * kPointerSize));
  __ Sd(scratch, MemOperand(sp, 2 * kPointerSize));

  // FunctionCallbackInfo::length_.
  // Stored as int field, 32-bit integers within struct on stack always left
  // justified by n64 ABI.
  __ Sw(argc, MemOperand(sp, 3 * kPointerSize));

  // We also store the number of bytes to drop from the stack after returning
  // from the API function here.
  // Note: Unlike on other architectures, this stores the number of slots to
  // drop, not the number of bytes.
  __ Dadd(scratch, argc, Operand(FCA::kArgsLength + 1 /* receiver */));
  __ Sd(scratch, MemOperand(sp, 4 * kPointerSize));

  // v8::InvocationCallback's argument.
  DCHECK(!AreAliased(api_function_address, scratch, a0));
  __ Dadd(a0, sp, Operand(1 * kPointerSize));

  ExternalReference thunk_ref = ExternalReference::invoke_function_callback();

  // There are two stack slots above the arguments we constructed on the stack.
  // TODO(jgruber): Document what these arguments are.
  static constexpr int kStackSlotsAboveFCA = 2;
  MemOperand return_value_operand(
      fp, (kStackSlotsAboveFCA + FCA::kReturnValueOffset) * kPointerSize);

  static constexpr int kUseStackSpaceOperand = 0;
  MemOperand stack_space_operand(sp, 4 * kPointerSize);

  AllowExternalCallThatCantCauseGC scope(masm);
  CallApiFunctionAndReturn(masm, api_function_address, thunk_ref,
                           kUseStackSpaceOperand, &stack_space_operand,
                           return_value_operand);
}

void Builtins::Generate_CallApiGetter(MacroAssembler* masm) {
  // Build v8::PropertyCallbackInfo::args_ array on the stack and push property
  // name below the exit frame to make GC aware of them.
  STATIC_ASSERT(PropertyCallbackArguments::kShouldThrowOnErrorIndex == 0);
  STATIC_ASSERT(PropertyCallbackArguments::kHolderIndex == 1);
  STATIC_ASSERT(PropertyCallbackArguments::kIsolateIndex == 2);
  STATIC_ASSERT(PropertyCallbackArguments::kReturnValueDefaultValueIndex == 3);
  STATIC_ASSERT(PropertyCallbackArguments::kReturnValueOffset == 4);
  STATIC_ASSERT(PropertyCallbackArguments::kDataIndex == 5);
  STATIC_ASSERT(PropertyCallbackArguments::kThisIndex == 6);
  STATIC_ASSERT(PropertyCallbackArguments::kArgsLength == 7);

  Register receiver = ApiGetterDescriptor::ReceiverRegister();
  Register holder = ApiGetterDescriptor::HolderRegister();
  Register callback = ApiGetterDescriptor::CallbackRegister();
  Register scratch = a4;
  DCHECK(!AreAliased(receiver, holder, callback, scratch));

  Register api_function_address = a2;

  // Here and below +1 is for name() pushed after the args_ array.
  using PCA = PropertyCallbackArguments;
  __ Dsub(sp, sp, (PCA::kArgsLength + 1) * kPointerSize);
  __ Sd(receiver, MemOperand(sp, (PCA::kThisIndex + 1) * kPointerSize));
  __ Ld(scratch, FieldMemOperand(callback, AccessorInfo::kDataOffset));
  __ Sd(scratch, MemOperand(sp, (PCA::kDataIndex + 1) * kPointerSize));
  __ LoadRoot(scratch, RootIndex::kUndefinedValue);
  __ Sd(scratch, MemOperand(sp, (PCA::kReturnValueOffset + 1) * kPointerSize));
  __ Sd(scratch, MemOperand(sp, (PCA::kReturnValueDefaultValueIndex + 1) *
                                    kPointerSize));
  __ li(scratch, ExternalReference::isolate_address(masm->isolate()));
  __ Sd(scratch, MemOperand(sp, (PCA::kIsolateIndex + 1) * kPointerSize));
  __ Sd(holder, MemOperand(sp, (PCA::kHolderIndex + 1) * kPointerSize));
  // should_throw_on_error -> false
  DCHECK_EQ(0, Smi::zero().ptr());
  __ Sd(zero_reg,
        MemOperand(sp, (PCA::kShouldThrowOnErrorIndex + 1) * kPointerSize));
  __ Ld(scratch, FieldMemOperand(callback, AccessorInfo::kNameOffset));
  __ Sd(scratch, MemOperand(sp, 0 * kPointerSize));

  // v8::PropertyCallbackInfo::args_ array and name handle.
  const int kStackUnwindSpace = PropertyCallbackArguments::kArgsLength + 1;

  // Load address of v8::PropertyAccessorInfo::args_ array and name handle.
  __ mov(a0, sp);                              // a0 = Handle<Name>
  __ Dadd(a1, a0, Operand(1 * kPointerSize));  // a1 = v8::PCI::args_

  const int kApiStackSpace = 1;
  FrameScope frame_scope(masm, StackFrame::MANUAL);
  __ EnterExitFrame(false, t0, kApiStackSpace);

  // Create v8::PropertyCallbackInfo object on the stack and initialize
  // it's args_ field.
  __ Sd(a1, MemOperand(sp, 1 * kPointerSize));
  __ Dadd(a1, sp, Operand(1 * kPointerSize));
  // a1 = v8::PropertyCallbackInfo&

  ExternalReference thunk_ref =
      ExternalReference::invoke_accessor_getter_callback();

  __ Ld(scratch, FieldMemOperand(callback, AccessorInfo::kJsGetterOffset));
  __ Ld(api_function_address,
        FieldMemOperand(scratch, Foreign::kForeignAddressOffset));

  // +3 is to skip prolog, return address and name handle.
  MemOperand return_value_operand(
      fp, (PropertyCallbackArguments::kReturnValueOffset + 3) * kPointerSize);
  MemOperand* const kUseStackSpaceConstant = nullptr;
  CallApiFunctionAndReturn(masm, api_function_address, thunk_ref,
                           kStackUnwindSpace, kUseStackSpaceConstant,
                           return_value_operand);
}

void Builtins::Generate_DirectCEntry(MacroAssembler* masm) {
  // The sole purpose of DirectCEntry is for movable callers (e.g. any general
  // purpose Code object) to be able to call into C functions that may trigger
  // GC and thus move the caller.
  //
  // DirectCEntry places the return address on the stack (updated by the GC),
  // making the call GC safe. The irregexp backend relies on this.
  // state:
  //       t6: the C++ function.
  __ Sd(ra, MemOperand(sp, 0));  // Store the return address.
  __ Call(t6);                   // Call the C++ function.
  __ Ld(ra, MemOperand(sp, 0));  // Return to calling code.

  if (FLAG_debug_code && FLAG_enable_slow_asserts) {
    // In case of an error the return address may point to a memory area
    // filled with kZapValue by the GC. Dereference the address and check for
    // this.
    Label L;
    __ Ld(a4, MemOperand(t6));
    __ Jump(&L, ne, a4, Operand(reinterpret_cast<uint64_t>(kZapValue)));
    if (__ emit_debug_code()) {
      __ Abort(AbortReason::kReceivedInvalidReturnAddress);
    }
    __ bind(&L);
  }
  __ Ret();
}

void Builtins::Generate_WasmDebugBreak(MacroAssembler* masm) {
  //UNIMPLEMENTED();
  __ nop();
}

#undef __

}  // namespace internal
}  // namespace v8

#endif  // V8_TARGET_ARCH_RISCV64
