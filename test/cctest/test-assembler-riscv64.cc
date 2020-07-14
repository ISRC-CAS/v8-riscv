#include <iostream>  // NOLINT(readability/streams)

#include "src/init/v8.h"

#include "src/base/utils/random-number-generator.h"
#include "src/codegen/assembler-inl.h"
#include "src/codegen/macro-assembler.h"
#include "src/diagnostics/disassembler.h"
#include "src/execution/simulator.h"
#include "src/heap/factory.h"

#include "test/cctest/cctest.h"

namespace v8 {
namespace internal {

// Define these function prototypes to match JSEntryFunction in execution.cc.
// TODO(riscv64): Refine these signatures per test case.
using F1 = void*(int x, int p1, int p2, int p3, int p4);
using F2 = void*(int x, int y, int p2, int p3, int p4);
using F3 = void*(void* p, int p1, int p2, int p3, int p4);
using F4 = void*(int64_t x, int64_t y, int64_t p2, int64_t p3, int64_t p4);
using F5 = void*(void* p0, void* p1, int p2, int p3, int p4);

#define __ assm.

TEST(RISCV1) {
  CcTest::InitializeVM();
  Isolate* isolate = CcTest::i_isolate();
  HandleScope scope(isolate);

  MacroAssembler assm(isolate, v8::internal::CodeObjectRequired::kYes);

  Label exit, error;

  // ----- Test all instructions.

  // Test lui, ori, and addiu, used in the li pseudo-instruction.
  // This way we can then safely load registers with chosen values.

  __ ori(a4, zero_reg, 0);
  __ lui(a4, 0x12345);
  __ ori(a4, a4, 0);
  __ ori(a4, a4, 0xF0F);
  __ ori(a4, a4, 0x0F0);
  __ addi(a5, a4, 1);
  __ addi(a6, a5, -0x10);

  // Load values in temporary registers.
  __ li(a4, 0x00000004);
  __ li(a5, 0x00001234);
  __ li(a6, 0x12345678);
  __ li(a7, 0x7FFFFFFF);
  __ li(t0, 0xFFFFFFFC);
  __ li(t1, 0xFFFFEDCC);
  __ li(t2, 0xEDCBA988);
  __ li(t3, 0x80000000);

  __ srliw(a0, a6, 8);    // 0x00123456
  __ slliw(a0, a0, 11);   // 0x91A2B000
  __ sraiw(a0, a0, 3);    // 0xF2345600
  __ sraw(a0, a0, a4);  // 0xFF234560
  __ sllw(a0, a0, a4);  // 0xF2345600
  __ srlw(a0, a0, a4);  // 0x0F234560
  __ Branch(&error, ne, a0, Operand(0x0F234560));
  __ nop();

  __ addw(a0, a4, a5);  // 0x00001238
  __ subw(a0, a0, a4);  // 0x00001234
  __ Branch(&error, ne, a0, Operand(0x00001234));
  __ nop();
  __ addw(a1, a7, a4);  // 32bit addw result is sign-extended into 64bit reg.
  __ Branch(&error, ne, a1, Operand(0xFFFFFFFF80000003));
  __ nop();
  __ sub(a1, t3, a4);  // 0x7FFFFFFC
  __ Branch(&error, ne, a1, Operand(0x7FFFFFFC));
  __ nop();

  __ and_(a0, a5, a6);  // 0x0000000000001230
  __ or_(a0, a0, a5);   // 0x0000000000001234
  __ xor_(a0, a0, a6);  // 0x000000001234444C
  __ nor(a0, a0, a6);   // 0xFFFFFFFFEDCBA987
  __ Branch(&error, ne, a0, Operand(0xFFFFFFFFEDCBA983));
  __ nop();

  // Shift both 32bit number to left, to preserve meaning of next comparison.
  __ slli(a7, a7, 32);
  __ slli(t3, t3, 32);

  __ slt(a0, t3, a7);
  __ Branch(&error, ne, a0, Operand(0x1));
  __ nop();
  __ sltu(a0, t3, a7);
  __ Branch(&error, ne, a0, Operand(zero_reg));
  __ nop();

  // Restore original values in registers.
  __ srli(a7, a7, 32);
  __ srli(t3, t3, 32);
  // End of SPECIAL class.

  __ addiw(a0, zero_reg, 0x7421);  // 0x00007421
  __ addiw(a0, a0, -0x1);          // 0x00007420
  __ addiw(a0, a0, -0x20);         // 0x00007400
  __ Branch(&error, ne, a0, Operand(0x00007400));
  __ nop();
  __ addiw(a1, a7, 0x1);  // 0x80000000 - result is sign-extended.
  __ Branch(&error, ne, a1, Operand(0xFFFFFFFF80000000));
  __ nop();

  __ slt(a0, a4, a5);  // 0x1
  __ slt(a0, a6, a4);  // 0x0
  __ Branch(&error, ne, a0, Operand(zero_reg));
  __ nop();
  __ slti(a0, a5, 0x00002000);  // 0x1
  __ slti(a0, a0, 0x00008000);  // 0x1
  __ Branch(&error, ne, a0, Operand(0x1));
  __ nop();

  __ andi(a0, a5, 0xF0F0);  // 0x00001030
  __ ori(a0, a0, 0x8A00);   // 0x00009A30
  __ xori(a0, a0, 0x83CC);  // 0x000019FC
  __ Branch(&error, ne, a0, Operand(0x000019FC));
  __ nop();
  __ lui(a1, 0x8123);  // Result is sign-extended into 64bit register.
  __ Branch(&error, ne, a1, Operand(0xFFFFFFFF81230000));
  __ nop();

  // Everything was correctly executed. Load the expected result.
  __ li(a0, 0x31415926);
  __ b(&exit);
  __ nop();

  __ bind(&error);
  // Got an error. Return a wrong result.
  __ li(a0, 666);

  __ bind(&exit);
  __ jr(ra);
  __ nop();

  CodeDesc desc;
  assm.GetCode(isolate, &desc);
  Handle<Code> code = Factory::CodeBuilder(isolate, desc, Code::STUB).Build();
  auto f = GeneratedCode<F2>::FromCode(*code);
  int64_t res = reinterpret_cast<int64_t>(f.Call(0xAB0, 0xC, 0, 0, 0));

  CHECK_EQ(0x31415926L, res);
}

TEST(RISCV2) {
  // Test BRANCH improvements.
  CcTest::InitializeVM();
  Isolate* isolate = CcTest::i_isolate();
  HandleScope scope(isolate);

  MacroAssembler assm(isolate, v8::internal::CodeObjectRequired::kYes);
  Label exit, exit2, exit3;

  __ Branch(&exit, ge, a0, Operand(zero_reg));
  __ Branch(&exit2, ge, a0, Operand(0x00001FFF));
  __ Branch(&exit3, ge, a0, Operand(0x0001FFFF));

  __ bind(&exit);
  __ bind(&exit2);
  __ bind(&exit3);
  __ jr(ra);
  __ nop();

  CodeDesc desc;
  assm.GetCode(isolate, &desc);
  Handle<Code> code = Factory::CodeBuilder(isolate, desc, Code::STUB).Build();
  USE(code);
}


#undef __

}  // namespace internal
}  // namespace v8