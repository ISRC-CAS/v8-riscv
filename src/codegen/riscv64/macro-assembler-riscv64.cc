// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if V8_TARGET_ARCH_RISCV64

#include "src/base/bits.h"
#include "src/base/division-by-constant.h"
#include "src/codegen/assembler.h"
#include "src/codegen/callable.h"
#include "src/codegen/code-factory.h"
#include "src/codegen/external-reference-table.h"
#include "src/codegen/macro-assembler-inl.h"
#include "src/codegen/register-configuration.h"
#include "src/debug/debug.h"
#include "src/deoptimizer/deoptimizer.h"
#include "src/execution/frame-constants.h"
#include "src/execution/frames-inl.h"
#include "src/heap/heap-inl.h"  // For MemoryChunk.
#include "src/init/bootstrapper.h"
#include "src/logging/counters.h"
#include "src/runtime/runtime.h"
#include "src/snapshot/embedded/embedded-data.h"
#include "src/snapshot/snapshot.h"
#include "src/wasm/wasm-code-manager.h"

namespace v8 {
namespace internal {

static inline bool IsZero(const Operand& rt) {
  if (rt.is_reg()) {
    return rt.rm() == zero_reg;
  } else {
    return rt.immediate() == 0;
  }
}
// ---------------------------------------------------------------------------
// Instruction macros.

void MacroAssembler::Swap(Register reg1, Register reg2, Register scratch) {
  if (scratch == no_reg) {
    Xor(reg1, reg1, Operand(reg2));
    Xor(reg2, reg2, Operand(reg1));
    Xor(reg1, reg1, Operand(reg2));
  } else {
    addi(scratch, reg1, 0);
    addi(reg1, reg2, 0);
    addi(reg2, scratch, 0);
  }
}
int TurboAssembler::RequiredStackSizeForCallerSaved(SaveFPRegsMode fp_mode,
                                                    Register exclusion1,
                                                    Register exclusion2,
                                                    Register exclusion3) const {
  int bytes = 0;
  RegList exclusions = 0;
  if (exclusion1 != no_reg) {
    exclusions |= exclusion1.bit();
    if (exclusion2 != no_reg) {
      exclusions |= exclusion2.bit();
      if (exclusion3 != no_reg) {
        exclusions |= exclusion3.bit();
      }
    }
  }

  RegList list = kJSCallerSaved & ~exclusions;
  bytes += NumRegs(list) * kPointerSize;

  if (fp_mode == kSaveFPRegs) {
    bytes += NumRegs(kCallerSavedDoubles) * kDoubleSize;
  }

  return bytes;
}

void TurboAssembler::Add(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    addw(rd, rs, rt.rm());
  } else {
    if (is_int12(rt.immediate()) && !MustUseReg(rt.rmode())) {
      addiw(rd, rs, static_cast<int32_t>(rt.immediate()));
    }
#if 0
    else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      DCHECK(rs != scratch);
      li(scratch, rt);
      addw(rd, rs, scratch);
    }
#endif
  }
}

void TurboAssembler::Dadd(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    add(rd, rs, rt.rm());
  } else {
    if (is_int12(rt.immediate()) && !MustUseReg(rt.rmode())) {
      addi(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      DCHECK(rs != scratch);
      li(scratch, rt);
      add(rd, rs, scratch);
    }
  }
}

void TurboAssembler::Dsub(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    sub(rd, rs, rt.rm());
  } else if (is_int12(-rt.immediate()) && !MustUseReg(rt.rmode())) {
    addi(rd, rs,
         static_cast<int32_t>(
             -rt.immediate()));  // No dsubiu instr, use daddiu(x, y, -imm).
  }
#if 0
  else {
    DCHECK(rs != at);
    int li_count = InstrCountForLi64Bit(rt.immediate());
    int li_neg_count = InstrCountForLi64Bit(-rt.immediate());
    if (li_neg_count < li_count && !MustUseReg(rt.rmode())) {
      // Use load -imm and daddu when loading -imm generates one instruction.
      DCHECK(rt.immediate() != std::numeric_limits<int32_t>::min());
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      li(scratch, Operand(-rt.immediate()));
      Dadd(rd, rs, scratch);
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      li(scratch, rt);
      sub(rd, rs, scratch);
    }
  }
#endif
}

void TurboAssembler::Mul(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    mulw(rd, rs, rt.rm());
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    mulw(rd, rs, scratch);
  }
#endif
}

void TurboAssembler::Dmul(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    mul(rd, rs, rt.rm());
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    mul(rd, rs, scratch);
  }
#endif
}

void TurboAssembler::Dmulh(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    mulh(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    mulh(rd, rs, scratch);
  }
}

void TurboAssembler::Dmulhu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    mulhu(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    mulhu(rd, rs, scratch);
  }
}

void TurboAssembler::Push(Smi smi) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  li(scratch, Operand(smi));
  push(scratch);
}

void TurboAssembler::Push(Handle<HeapObject> handle) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  li(scratch, Operand(handle));
  push(scratch);
}

void TurboAssembler::Mult(Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Dmult(Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Multu(Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Dmultu(Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Div(Register res, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    divw(res, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    divw(res, rs, scratch);
  }
}

void TurboAssembler::Divu(Register res, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    divuw(res, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    divuw(res, rs, scratch);
  }
}

void TurboAssembler::Ddiv(Register res, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    div(res, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    div(res, rs, scratch);
  }
}

void TurboAssembler::Ddivu(Register res, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    divu(res, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    divu(res, rs, scratch);
  }
}

void TurboAssembler::Movz(Register rd, Register rs, Register rt) {
  Label not_zero;
  Jump(&not_zero, ne, rt, Operand(zero_reg));
  mov(rd, rs);
  bind(&not_zero);
}

void TurboAssembler::Mod(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    remw(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    remw(rd, rs, scratch);
  }
}

void TurboAssembler::Modu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    remuw(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    remuw(rd, rs, scratch);
  }
}

void TurboAssembler::Dmod(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    rem(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    rem(rd, rs, scratch);
  }
}

void TurboAssembler::Dmodu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    remu(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    remu(rd, rs, scratch);
  }
}

void TurboAssembler::Or(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    or_(rd, rs, rt.rm());
  } else {
    if (is_uint16(rt.immediate()) && !MustUseReg(rt.rmode())) {
      ori(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      DCHECK(rs != scratch);
      li(scratch, rt);
      or_(rd, rs, scratch);
    }
  }
}

void TurboAssembler::Xor(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    xor_(rd, rs, rt.rm());
  } else {
    if (is_uint16(rt.immediate()) && !MustUseReg(rt.rmode())) {
      xori(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      DCHECK(rs != scratch);
      li(scratch, rt);
      xor_(rd, rs, scratch);
    }
  }
}

void TurboAssembler::Nor(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    or_(rd, rs, rt.rm());
  } else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, rt);
    or_(rd, rs, scratch);
  }
  not_(rd, rs);
}

void TurboAssembler::Neg(Register rs, const Operand& rt) {
  sub(rs, zero_reg, rt.rm());
}

void TurboAssembler::Slt(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    slt(rd, rs, rt.rm());
  } else {
    if (is_int12(rt.immediate()) && !MustUseReg(rt.rmode())) {
      slti(rd, rs, static_cast<int32_t>(rt.immediate()));
    }
#if 0
    else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      // BlockTrampolinePoolScope block_trampoline_pool(this);
      Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
      DCHECK(rs != scratch);
      li(scratch, rt);
      slt(rd, rs, scratch);
    }
#endif
  }
}

void TurboAssembler::Sltu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    sltu(rd, rs, rt.rm());
  } else {
    const uint64_t int16_min = std::numeric_limits<int16_t>::min();
    if (is_uint15(rt.immediate()) && !MustUseReg(rt.rmode())) {
      // Imm range is: [0, 32767].
      sltiu(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else if (is_uint15(rt.immediate() - int16_min) &&
               !MustUseReg(rt.rmode())) {
      // Imm range is: [max_unsigned-32767,max_unsigned].
      sltiu(rd, rs, static_cast<uint16_t>(rt.immediate()));
    }
#if 0
    else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      // BlockTrampolinePoolScope block_trampoline_pool(this);
      Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
      DCHECK(rs != scratch);
      li(scratch, rt);
      sltu(rd, rs, scratch);
    }
#endif
  }
}

void TurboAssembler::Sle(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    slt(rd, rt.rm(), rs);
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
    // BlockTrampolinePoolScope block_trampoline_pool(this);
    DCHECK(rs != scratch);
    li(scratch, rt);
    slt(rd, scratch, rs);
  }
#endif
  xori(rd, rd, 1);
}

void TurboAssembler::Sleu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    sltu(rd, rt.rm(), rs);
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
    // BlockTrampolinePoolScope block_trampoline_pool(this);
    DCHECK(rs != scratch);
    li(scratch, rt);
    sltu(rd, scratch, rs);
  }
#endif
  xori(rd, rd, 1);
}

void TurboAssembler::Sge(Register rd, Register rs, const Operand& rt) {
  Slt(rd, rs, rt);
  xori(rd, rd, 1);
}

void TurboAssembler::Sgeu(Register rd, Register rs, const Operand& rt) {
  Sltu(rd, rs, rt);
  xori(rd, rd, 1);
}

void TurboAssembler::Sgt(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    slt(rd, rt.rm(), rs);
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
    // BlockTrampolinePoolScope block_trampoline_pool(this);
    DCHECK(rs != scratch);
    li(scratch, rt);
    slt(rd, scratch, rs);
  }
#endif
}

void TurboAssembler::Sgtu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    sltu(rd, rt.rm(), rs);
  }
#if 0
  else {
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.hasAvailable() ? temps.Acquire() : t8;
    // BlockTrampolinePoolScope block_trampoline_pool(this);
    DCHECK(rs != scratch);
    li(scratch, rt);
    sltu(rd, scratch, rs);
  }
#endif
}

void TurboAssembler::Ror(Register rd, Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Dror(Register rd, Register rs, const Operand& rt) {
  // UNIMPLEMENTED();
  nop();
}

void TurboAssembler::Lb(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lb(rd, source);
}

void TurboAssembler::Lh(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lh(rd, source);
}

void TurboAssembler::Lw(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lw(rd, source);
}

void TurboAssembler::Lbu(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lbu(rd, source);
}

void TurboAssembler::Lhu(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lhu(rd, source);
}
void TurboAssembler::Sb(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  sb(rd, source);
}

void TurboAssembler::Sh(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  sh(rd, source);
}

void TurboAssembler::Sw(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  sw(rd, source);
}

void TurboAssembler::Lwu(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  AdjustBaseAndOffset(&source);
  lwu(rd, source);
}

void TurboAssembler::Sd(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  // TODO: make the source have proper base-register and int12 offset with
  // 8-byte alignment AdjustBaseAndOffset(&source);
  sd(rd, source);
}

void TurboAssembler::Flw(FPURegister fd, const MemOperand& src) {
  MemOperand tmp = src;
  AdjustBaseAndOffset(&tmp);
  flw(fd, tmp);
}

void TurboAssembler::Fsw(FPURegister fs, const MemOperand& src) {
  MemOperand tmp = src;
  AdjustBaseAndOffset(&tmp);
  fsw(fs, tmp);
}

void TurboAssembler::Fld(FPURegister fd, const MemOperand& src) {
  MemOperand tmp = src;
  AdjustBaseAndOffset(&tmp);
  fld(fd, tmp);
}

void TurboAssembler::Fsd(FPURegister fs, const MemOperand& src) {
  MemOperand tmp = src;
  AdjustBaseAndOffset(&tmp);
  fsd(fs, tmp);
}
// DSub for 64bit, Sub for 32 bits
void TurboAssembler::Sub(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    subw(rd, rs, rt.rm());
  }
#if 0
  else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    li(scratch, rt);
    dsubu(rd, rs, scratch);
  }
#endif
}

void TurboAssembler::And(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    and_(rd, rs, rt.rm());
  } else {
    if (is_int12(rt.immediate()) && !MustUseReg(rt.rmode())) {
      andi(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      DCHECK(rs != scratch);
      li(scratch, rt);
      and_(rd, rs, scratch);
    }
  }
}

// ------------Memory-instructions-------------

// TODO: RISCV's MRI address mode doesn't assume the address be aligned
// naturally to data-sized boundaries. But the offset is a signed int12. We just
// need to make sure the offset is suitable.
#if 0
void Assembler::AdjustBaseAndOffset(MemOperand* src,
                                    OffsetAccessType access_type,
                                    int second_access_add_to_offset) {
  // This method is used to adjust the base register and offset pair
  // for a load/store when the offset doesn't fit into int16_t.
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

  // is_int16 must be passed a signed value, hence the static cast below.
  if (is_int16(src->offset()) &&
      (!two_accesses || is_int16(static_cast<int32_t>(
                            src->offset() + second_access_add_to_offset)))) {
    // Nothing to do: 'offset' (and, if needed, 'offset + 4', or other specified
    // value) fits into int16_t.
    return;
  }

  DCHECK(src->rm() !=
         at);  // Must not overwrite the register 'base' while loading 'offset'.

#ifdef DEBUG
  // Remember the "(mis)alignment" of 'offset', it will be checked at the end.
  uint32_t misalignment = src->offset() & (kDoubleSize - 1);
#endif

  // Do not load the whole 32-bit 'offset' if it can be represented as
  // a sum of two 16-bit signed offsets. This can save an instruction or two.
  // To simplify matters, only do this for a symmetric range of offsets from
  // about -64KB to about +64KB, allowing further addition of 4 when accessing
  // 64-bit variables with two 32-bit accesses.
  constexpr int32_t kMinOffsetForSimpleAdjustment =
      0x7FF8;  // Max int16_t that's a multiple of 8.
  constexpr int32_t kMaxOffsetForSimpleAdjustment =
      2 * kMinOffsetForSimpleAdjustment;

  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  if (0 <= src->offset() && src->offset() <= kMaxOffsetForSimpleAdjustment) {
    daddiu(scratch, src->rm(), kMinOffsetForSimpleAdjustment);
    src->offset_ -= kMinOffsetForSimpleAdjustment;
  } else if (-kMaxOffsetForSimpleAdjustment <= src->offset() &&
             src->offset() < 0) {
    daddiu(scratch, src->rm(), -kMinOffsetForSimpleAdjustment);
    src->offset_ += kMinOffsetForSimpleAdjustment;
  } else if (kArchVariant == kMips64r6) {
    // On r6 take advantage of the daui instruction, e.g.:
    //    daui   at, base, offset_high
    //   [dahi   at, 1]                       // When `offset` is close to +2GB.
    //    lw     reg_lo, offset_low(at)
    //   [lw     reg_hi, (offset_low+4)(at)]  // If misaligned 64-bit load.
    // or when offset_low+4 overflows int16_t:
    //    daui   at, base, offset_high
    //    daddiu at, at, 8
    //    lw     reg_lo, (offset_low-8)(at)
    //    lw     reg_hi, (offset_low-4)(at)
    int16_t offset_low = static_cast<uint16_t>(src->offset());
    int32_t offset_low32 = offset_low;
    int16_t offset_high = static_cast<uint16_t>(src->offset() >> 16);
    bool increment_hi16 = offset_low < 0;
    bool overflow_hi16 = false;

    if (increment_hi16) {
      offset_high++;
      overflow_hi16 = (offset_high == -32768);
    }
    daui(scratch, src->rm(), static_cast<uint16_t>(offset_high));

    if (overflow_hi16) {
      dahi(scratch, 1);
    }

    if (two_accesses && !is_int16(static_cast<int32_t>(
                            offset_low32 + second_access_add_to_offset))) {
      // Avoid overflow in the 16-bit offset of the load/store instruction when
      // adding 4.
      daddiu(scratch, scratch, kDoubleSize);
      offset_low32 -= kDoubleSize;
    }

    src->offset_ = offset_low32;
  } else {
    // Do not load the whole 32-bit 'offset' if it can be represented as
    // a sum of three 16-bit signed offsets. This can save an instruction.
    // To simplify matters, only do this for a symmetric range of offsets from
    // about -96KB to about +96KB, allowing further addition of 4 when accessing
    // 64-bit variables with two 32-bit accesses.
    constexpr int32_t kMinOffsetForMediumAdjustment =
        2 * kMinOffsetForSimpleAdjustment;
    constexpr int32_t kMaxOffsetForMediumAdjustment =
        3 * kMinOffsetForSimpleAdjustment;
    if (0 <= src->offset() && src->offset() <= kMaxOffsetForMediumAdjustment) {
      daddiu(scratch, src->rm(), kMinOffsetForMediumAdjustment / 2);
      daddiu(scratch, scratch, kMinOffsetForMediumAdjustment / 2);
      src->offset_ -= kMinOffsetForMediumAdjustment;
    } else if (-kMaxOffsetForMediumAdjustment <= src->offset() &&
               src->offset() < 0) {
      daddiu(scratch, src->rm(), -kMinOffsetForMediumAdjustment / 2);
      daddiu(scratch, scratch, -kMinOffsetForMediumAdjustment / 2);
      src->offset_ += kMinOffsetForMediumAdjustment;
    } else {
      // Now that all shorter options have been exhausted, load the full 32-bit
      // offset.
      int32_t loaded_offset = RoundDown(src->offset(), kDoubleSize);
      lui(scratch, (loaded_offset >> kLuiShift) & kImm16Mask);
      ori(scratch, scratch, loaded_offset & kImm16Mask);  // Load 32-bit offset.
      daddu(scratch, scratch, src->rm());
      src->offset_ -= loaded_offset;
    }
  }
  src->rm_ = scratch;

  DCHECK(is_int16(src->offset()));
  if (two_accesses) {
    DCHECK(is_int16(
        static_cast<int32_t>(src->offset() + second_access_add_to_offset)));
  }
  DCHECK(misalignment == (src->offset() & (kDoubleSize - 1)));
}
#endif

// BRANCH_ARGS_CHECK checks that conditional jump arguments are correct.
#define BRANCH_ARGS_CHECK(cond, rs, rt)                           \
  DCHECK((cond == al && rs == zero_reg && rt.rm() == zero_reg) || \
         (cond != al && (rs != zero_reg || rt.rm() != zero_reg)))

void TurboAssembler::BranchShort(int32_t offset) {
  DCHECK(is_int13(offset));
  BranchShortHelper(offset, nullptr);
}

void TurboAssembler::BranchShort(Label* L) { BranchShortHelper(0, L); }

void TurboAssembler::BranchShort(int32_t offset, Condition cond, Register rs,
                                 const Operand& rt) {
  BranchShortCheck(offset, nullptr, cond, rs, rt);
}

void TurboAssembler::BranchShort(Label* L, Condition cond, Register rs,
                                 const Operand& rt) {
  BranchShortCheck(0, L, cond, rs, rt);
}

bool TurboAssembler::BranchShortCheck(int32_t offset, Label* L, Condition cond,
                                      Register rs, const Operand& rt) {
  BRANCH_ARGS_CHECK(cond, rs, rt);
  if (!L) {
    DCHECK(is_int13(offset));
    return BranchShortHelper(offset, nullptr, cond, rs, rt);
  } else {
    DCHECK_EQ(offset, 0);
    return BranchShortHelper(0, L, cond, rs, rt);
  }
  return false;
}

void TurboAssembler::BranchShortHelper(int16_t offset, Label* L) {
  DCHECK(L == nullptr || offset == 0);
  offset = GetOffset(offset, L, OffsetSize::kOffset13);
  offset >>= 1;
  b(offset);
}

bool TurboAssembler::BranchShortHelper(int16_t offset, Label* L, Condition cond,
                                       Register rs, const Operand& rt) {
  DCHECK(L == nullptr || offset == 0);
  if (!is_near(L, OffsetSize::kOffset13)) return false;

  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch = temps.hasAvailable() ? temps.Acquire() : t6;
  int32_t offset32;

  // Be careful to always use shifted_branch_offset only just before the
  // branch instruction, as the location will be remember for patching the
  // target.
  {
    BlockTrampolinePoolScope block_trampoline_pool(this);
    switch (cond) {
      case al:
        offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
        offset32 >>= 1;
        b(offset32);
        break;
      case eq:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(rs, zero_reg, offset32);
        } else {
          // We don't want any other register but scratch clobbered.
          scratch = GetRtAsRegisterHelper(rt, scratch);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(rs, scratch, offset32);
        }
        break;
      case ne:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(rs, zero_reg, offset32);
        } else {
          // We don't want any other register but scratch clobbered.
          scratch = GetRtAsRegisterHelper(rt, scratch);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(rs, scratch, offset32);
        }
        break;

      // Signed comparison.
      case gt:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bgtz(rs, offset32);
        } else {
          Slt(scratch, GetRtAsRegisterHelper(rt, scratch), rs);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(scratch, zero_reg, offset32);
        }
        break;
      case ge:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bgez(rs, offset32);
        } else {
          Slt(scratch, rs, rt);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(scratch, zero_reg, offset32);
        }
        break;
      case lt:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bltz(rs, offset32);
        } else {
          Slt(scratch, rs, rt);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(scratch, zero_reg, offset32);
        }
        break;
      case le:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          blez(rs, offset32);
        } else {
          Slt(scratch, GetRtAsRegisterHelper(rt, scratch), rs);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(scratch, zero_reg, offset32);
        }
        break;

      // Unsigned comparison.
      case hi:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(rs, zero_reg, offset32);
        } else {
          Sltu(scratch, GetRtAsRegisterHelper(rt, scratch), rs);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(scratch, zero_reg, offset32);
        }
        break;
      case hs:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          b(offset32);
        } else {
          Sltu(scratch, rs, rt);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(scratch, zero_reg, offset32);
        }
        break;
      case lo:
        if (IsZero(rt)) {
          return true;  // No code needs to be emitted.
        } else {
          Sltu(scratch, rs, rt);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          bne(scratch, zero_reg, offset32);
        }
        break;
      case ls:
        if (IsZero(rt)) {
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(rs, zero_reg, offset32);
        } else {
          Sltu(scratch, GetRtAsRegisterHelper(rt, scratch), rs);
          offset32 = GetOffset(offset, L, OffsetSize::kOffset13);
          offset32 >>= 1;
          beq(scratch, zero_reg, offset32);
        }
        break;
      default:
        UNREACHABLE();
    }
  }

  return true;
}

int32_t TurboAssembler::GetOffset(int32_t offset, Label* L, OffsetSize bits) {
  if (L) {
    offset = branch_offset_helper(L, bits);
  } else {
    DCHECK(is_intn(offset, bits));
  }
  return offset;
}

Register TurboAssembler::GetRtAsRegisterHelper(const Operand& rt,
                                               Register scratch) {
  Register r2 = no_reg;
  if (rt.is_reg()) {
    r2 = rt.rm();
  } else {
    r2 = scratch;
    li(r2, rt);
  }

  return r2;
}
void TurboAssembler::Ld(Register rd, const MemOperand& rs) {
  MemOperand source = rs;
  // TODO: make the source have proper base-register
  // and int12 offset with 8-byte alignment
  AdjustBaseAndOffset(&source);
  ld(rd, source);
}

void TurboAssembler::Ret(Condition cond, Register rs, const Operand& rt) {
  Jump(ra, cond, rs, rt);
}
void TurboAssembler::Jump(intptr_t target, RelocInfo::Mode rmode,
                          Condition cond, Register rs, const Operand& rt) {
  Label skip;
  if (cond != al) {
    // Branch is to be implement
    Branch(&skip, NegateCondition(cond), rs, rt);
  }
  // The first instruction of 'li' may be placed in the delay slot.
  // This is not an issue, t6 is expected to be clobbered anyway.
  { 
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(rs != scratch);
    li(scratch, Operand(target, rmode));
    Jump(scratch, al, zero_reg, Operand(zero_reg));
    bind(&skip);
  }
}

void TurboAssembler::Jump(Address target, RelocInfo::Mode rmode, Condition cond,
                          Register rs, const Operand& rt) {
  DCHECK(!RelocInfo::IsCodeTarget(rmode));
  Jump(static_cast<intptr_t>(target), rmode, cond, rs, rt);
}

void TurboAssembler::Jump(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register rs, const Operand& rt) {
  DCHECK(RelocInfo::IsCodeTarget(rmode));
  UseScratchRegisterScope temps(this);
  Register temp =  temps.Acquire();
  DCHECK(temp != rs);
  if (root_array_available_ && options().isolate_independent_code) {
    IndirectLoadConstant(temp, code);
    //Dadd(temp, temp, Operand(Code::kHeaderSize - kHeapObjectTag));
    Jump(temp, cond, rs, rt);
    return;
  } else if (options().inline_offheap_trampolines) {
    int builtin_index = Builtins::kNoBuiltinId;
    if (isolate()->builtins()->IsBuiltinHandle(code, &builtin_index) &&
        Builtins::IsIsolateIndependent(builtin_index)) {
      // Inline the trampoline.
      RecordCommentForOffHeapTrampoline(builtin_index);
      CHECK_NE(builtin_index, Builtins::kNoBuiltinId);
      EmbeddedData d = EmbeddedData::FromBlob();
      Address entry = d.InstructionStartOfBuiltin(builtin_index);
      li(temp, Operand(entry, RelocInfo::OFF_HEAP_TARGET));
      Jump(temp, cond, rs, rt);
      return;
    }
  }

  Jump(static_cast<intptr_t>(code.address()), rmode, cond, rs, rt);
}

void TurboAssembler::Jump(const ExternalReference& reference) {
  li(x31, reference);
  Jump(x31);
}

void TurboAssembler::Jump(Register target, Condition cond, Register rs,
                          const Operand& rt) {
  if (cond == al) {
    // jump register
    // pseudoinstruction: jr rs
    // base instr: jalr x0, rs, 0
    jalr(x0, 0, target);
  } else {
    // tbd
    Branch(8, NegateCondition(cond), rs, rt);
    jalr(x0, 0, target);
  }
}

void TurboAssembler::Jump(Label* target, Condition cond, Register rs,
                          const Operand& rt) {
  Branch(target, cond, rs, rt);
}

void TurboAssembler::li(Register rd, Operand j, LiFlags mode) {
  DCHECK(!j.is_reg());
  if (MustUseReg(j.rmode())) {
    int64_t imm;
    if (j.IsHeapObjectRequest()) {
      RequestHeapObject(j.heap_object_request());
      imm = 0;
    } else {
      imm = j.immediate();
    }

    RecordRelocInfo(j.rmode(), imm);

    // In worst case, use a sequence of 8 instructions
    // LUI + ADDIW + SLLI + ADDI + SLLI + ADDI + SLLI + ADDI
    // to be optimized in future

    // lui + addiw load upper 32 bits imm64 to rd
    lui(rd, (imm >> 44) & kImm19_0Mask);
    ori(rd, rd, (imm >> 32) & kImm11_0Mask);

    // process the lower 32bits of imm64
    // high 12 bits of lower 32bit of immm64
    slli(rd, rd, 12);
    ori(rd, rd, (imm >> 20) & kImm11_0Mask);
    // middle 12 bits of lower 32bits of imm64
    slli(rd, rd, 12);
    ori(rd, rd, (imm >> 8) & kImm11_0Mask);
    // lower 8 bits of lower 32bits imm64
    slli(rd, rd, 8);
    ori(rd, rd, imm & kImm8_0Mask);
  } else {
    int64_t imm = j.immediate();
    if (is_int32(imm)) {
      int32_t Hi20;
      int32_t Lo12;
      GetBranchlongArg(imm, Hi20, Lo12);
      if (Hi20 == 0) {
        addi(rd, x0, Lo12);
      } else if (Lo12 == 0) {
        lui(rd, Hi20);
      } else {
        lui(rd, Hi20);
        addi(rd, rd, Lo12);
      }
      return;
    }
    // In worst case, use a sequence of 8 instructions
    // LUI + ADDIW + SLLI + ADDI + SLLI + ADDI + SLLI + ADDI
    // to be optimized in future

    // LUI + ADDIW load upper 32 bits imm64 to rd
    lui(rd, (imm >> 44) & kImm19_0Mask);
    ori(rd, rd, (imm >> 32) & kImm11_0Mask);

    // process the lower 32bits of imm64
    // high 12 bits of lower 32bit of immm64
    slli(rd, rd, 12);
    ori(rd, rd, (imm >> 20) & kImm11_0Mask);
    // middle 12 bits of lower 32bits of imm64
    slli(rd, rd, 12);
    ori(rd, rd, (imm >> 8) & kImm11_0Mask);
    // lower 8 bits of lower 32bits imm64
    slli(rd, rd, 8);
    ori(rd, rd, imm & kImm8_0Mask);
  }
}

void TurboAssembler::LoadFromConstantsTable(Register destination,
                                            int constant_index) {
  DCHECK(RootsTable::IsImmortalImmovable(RootIndex::kBuiltinsConstantsTable));
  LoadRoot(destination, RootIndex::kBuiltinsConstantsTable);
  Ld(destination,
     FieldMemOperand(destination,
                     FixedArray::kHeaderSize + constant_index * kPointerSize));
}
void TurboAssembler::LoadRootRegisterOffset(Register destination,
                                            intptr_t offset) {
  if (offset == 0) {
    Move(destination, kRootRegister);
  } else {
    Dadd(destination, kRootRegister, Operand(offset));
  }
}

void TurboAssembler::LoadRootRelative(Register destination, int32_t offset) {
  Ld(destination, MemOperand(kRootRegister, offset));
}

void TurboAssembler::LoadCodeObjectEntry(Register destination,
                                         Register code_object) {
  // Code objects are called differently depending on whether we are generating
  // builtin code (which will later be embedded into the binary) or compiling
  // user JS code at runtime.
  // * Builtin code runs in --jitless mode and thus must not call into on-heap
  //   Code targets. Instead, we dispatch through the builtins entry table.
  // * Codegen at runtime does not have this restriction and we can use the
  //   shorter, branchless instruction sequence. The assumption here is that
  //   targets are usually generated code and not builtin Code objects.

  if (options().isolate_independent_code) {
    DCHECK(root_array_available());
    Label if_code_is_off_heap, out;
    UseScratchRegisterScope temps(this);
    Register scratch = t1; //TODO:UseScratchRegisterScope

    DCHECK(!AreAliased(destination, scratch));
    DCHECK(!AreAliased(code_object, scratch));

    // Check whether the Code object is an off-heap trampoline. If so, call its
    // (off-heap) entry point directly without going through the (on-heap)
    // trampoline.  Otherwise, just call the Code object as always.
    Lw(scratch, FieldMemOperand(code_object, Code::kFlagsOffset));
    Jump(&if_code_is_off_heap, ne, scratch,
         Operand(Code::IsOffHeapTrampoline::kMask));

    // Not an off-heap trampoline object, the entry point is at
    // Code::raw_instruction_start().
    Dadd(destination, code_object, Code::kHeaderSize - kHeapObjectTag);
    Jump(&out);

    // An off-heap trampoline, the entry point is loaded from the builtin entry
    // table.
    bind(&if_code_is_off_heap);
    Lw(scratch, FieldMemOperand(code_object, Code::kBuiltinIndexOffset));
    slli(destination, scratch, kSystemPointerSizeLog2);
    Dadd(destination, destination, kRootRegister);
    Ld(destination,
        MemOperand(destination, IsolateData::builtin_entry_table_offset()));
    bind(&out);
  } else {
    Dadd(destination, code_object, Code::kHeaderSize - kHeapObjectTag);
  }
}

void TurboAssembler::CallBuiltinByIndex(Register builtin_index) {
  LoadEntryFromBuiltinIndex(builtin_index);
  Call(builtin_index);
}

void TurboAssembler::CallCodeObject(Register code_object) {
  LoadCodeObjectEntry(code_object, code_object);
  Call(code_object);
}
void TurboAssembler::JumpCodeObject(Register code_object) {
  LoadCodeObjectEntry(code_object, code_object);
  Jump(code_object);
}
// -----------------------------------------------------------------------------
// JavaScript invokes.

void TurboAssembler::PrepareForTailCall(Register callee_args_count,
                                        Register caller_args_count,
                                        Register scratch0, Register scratch1) {
  // 
  UNIMPLEMENTED();
}

void TurboAssembler::Trap() { ebreak(); }

void TurboAssembler::DebugBreak() { ebreak(); }

void TurboAssembler::Assert(Condition cc, AbortReason reason, Register rs,
                            Operand rt) {
  if (emit_debug_code()) Check(cc, reason, rs, rt);
}

void TurboAssembler::Branch(int32_t offset) {
  DCHECK(is_int13(offset));
  BranchShort(offset);
}

void TurboAssembler::Branch(int32_t target, Condition cond, Register rs,
                            const Operand& rt) {
  bool is_near = BranchShortCheck(target, nullptr, cond, rs, rt);
  DCHECK(is_near);
  USE(is_near);
}

void TurboAssembler::Branch(Label* L) {
  if (L->is_bound()) {
    if (is_near_branch(L)) {
      BranchShort(L);
    } else {
      BranchLong(L);
    }
  } else {
    if (is_trampoline_emitted()) {
      BranchLong(L);
    } else {
      BranchLong(L);
    }
  }
}

void TurboAssembler::Branch(Label* L, Condition cond, Register rs,
                            const Operand& rt) {
  if (L->is_bound()) {
    if (!BranchShortCheck(0, L, cond, rs, rt)) {
      if (cond != al) {
        Label skip;
        Condition neg_cond = NegateCondition(cond);
        BranchShort(&skip, neg_cond, rs, rt);
        BranchLong(L);
        bind(&skip);
      } else {
        BranchLong(L);
      }
    }
  } else {
    if (is_trampoline_emitted()) {
      if (cond != al) {
        Label skip;
        Condition neg_cond = NegateCondition(cond);
        BranchShort(&skip, neg_cond, rs, rt);
        BranchLong(L);
        bind(&skip);
      } else {
        BranchLong(L);
      }
    } else {
      if (cond != al) {
        Label skip;
        Condition neg_cond = NegateCondition(cond);
        BranchShort(&skip, neg_cond, rs, rt);
        BranchLong(L);
        bind(&skip);
      } else {
        BranchLong(L);
      }
    }
  }
}

void TurboAssembler::Branch(Label* L, Condition cond, Register rs,
                            RootIndex index) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  LoadRoot(scratch, index);
  Branch(L, cond, rs, Operand(scratch));
}

void TurboAssembler::BranchLong(Label* L) {
  // Generate position independent long branch.
  BlockTrampolinePoolScope block_trampoline_pool(this);
  int64_t imm64;
  imm64 = branch_long_offset(L);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(is_int32(imm64));
  int32_t auipc_arg, jalr_arg;
  GetBranchlongArg(imm64, auipc_arg, jalr_arg);
  RecordComment("[auipc(scratch, auipc_arg);");
  auipc(scratch, auipc_arg);
  RecordComment("]");
  RecordComment("[jalr(x0, jalr_arg, scratch);");
  jalr(x0, jalr_arg, scratch);
  RecordComment("]");
}

void TurboAssembler::Abort(AbortReason reason) {
  Label abort_start;
  bind(&abort_start);
#ifdef DEBUG
  const char* msg = GetAbortReason(reason);
  RecordComment("Abort message: ");
  RecordComment(msg);
#endif

  // Avoid emitting call to builtin if requested.
  if (trap_on_abort()) {
    ebreak();
    return;
  }

  if (should_abort_hard()) {
    // We don't care if we constructed a frame. Just pretend we did.
    FrameScope assume_frame(this, StackFrame::NONE);
    PrepareCallCFunction(0, a2);
    li(a2, Operand(static_cast<int>(reason)));
    CallCFunction(ExternalReference::abort_with_reason(), 1);
    return;
  }

  Move(a2, Smi::FromInt(static_cast<int>(reason)));

  // Disable stub call restrictions to always allow calls to abort.
  if (!has_frame()) {
    // We don't actually want to generate a pile of code for this, so just
    // claim there is a stack frame, without generating one.
    FrameScope scope(this, StackFrame::NONE);
    Call(BUILTIN_CODE(isolate(), Abort), RelocInfo::CODE_TARGET);
  } else {
    Call(BUILTIN_CODE(isolate(), Abort), RelocInfo::CODE_TARGET);
  }
  // Will not return here.
  if (is_trampoline_pool_blocked()) {
    // If the calling code cares about the exact number of
    // instructions generated, we insert padding here to keep the size
    // of the Abort macro constant.
    // Currently in debug mode with debug_code enabled the number of
    // generated instructions is 14, so we use this as a maximum value.
    static const int kExpectedAbortInstructions = 14;
    int abort_instructions = InstructionsGeneratedSince(&abort_start);
    DCHECK_LE(abort_instructions, kExpectedAbortInstructions);
    while (abort_instructions++ < kExpectedAbortInstructions) {
      nop();
    }
  }
}

void TurboAssembler::CallForDeoptimization(Address target, int deopt_id,
                                           Label* exit, DeoptimizeKind kind) {}

void TurboAssembler::jmp(Label* L) { Branch(L); }

void TurboAssembler::ResetSpeculationPoisonRegister() {
  li(kSpeculationPoisonRegister, -1);
}

void TurboAssembler::ComputeCodeStartAddress(Register dst) {
  lla(dst, -pc_offset());
}

void TurboAssembler::Fmovs(FPURegister fd, FPURegister fs) {
  if (fd != fs) {
    fmvs(fd, fs);
  }
}

void TurboAssembler::Fmovd(FPURegister fd, FPURegister fs) {
  if (fd != fs) {
    fmvd(fd, fs);
  }
}

void TurboAssembler::Fmove(FPURegister fd, Register rn) { fmvdx(fd, rn); }

void TurboAssembler::Fmove(FPURegister fd, double imm) {
  uint64_t bits = bit_cast<uint64_t>(imm);
  //  if imm is +0 or -0
  if (bits == 0 || bits == 0x8000000000000000) {
    fcvtdl(fd, zero_reg, kRTZ);
    return;
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  lui(scratch, (bits >> 44) & kImm19_0Mask);
  ori(scratch, scratch, (bits >> 32) & kImm11_0Mask);
  slli(scratch, scratch, 12);
  ori(scratch, scratch, (bits >> 20) & kImm11_0Mask);
  slli(scratch, scratch, 12);
  ori(scratch, scratch, (bits >> 8) & kImm11_0Mask);
  slli(scratch, scratch, 8);
  ori(scratch, scratch, bits & kImm8_0Mask);
  fmvdx(fd, scratch);
}

void TurboAssembler::Fmove(FPURegister fd, float imm) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  int32_t bits = bit_cast<int32_t>(imm);
  lui(scratch, (bits >> 12) & kImm19_0Mask);
  ori(scratch, scratch, (bits & kImm11_0Mask));
  fmvwx(fd, scratch);
}

void TurboAssembler::Fmove(Register rd, FPURegister fn) { fmvxd(rd, fn); }
void TurboAssembler::Move(Register dst, Smi src) { li(dst, Operand(src)); }
void TurboAssembler::mov(Register rd, Register rt) { addi(rd, rt, 0); }
void TurboAssembler::Move(Register dst, Register src) {
  if (dst != src) {
    mov(dst, src);
  }
}

void TurboAssembler::Move(Register dst, int64_t src) {
  if (src == 0) {
    Move(dst, zero_reg);
  } else {
    li(dst, Operand(src));
  }
}

void TurboAssembler::Move(Register dst, ExternalReference src) { li(dst, src); }

void TurboAssembler::Call(Register target, Condition cond, Register rs,
                          const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  if (cond == al) {
    jalr(target);
  } else {
    BRANCH_ARGS_CHECK(cond, rs, rt);
    Branch(8, NegateCondition(cond), rs, rt);
    jalr(target);
  }
}

void TurboAssembler::Call(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register rs, const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);

  if (root_array_available_ && options().isolate_independent_code) {
    // use (kRootRegister, builtinentryoffset)
    IndirectLoadConstant(x31, code);
    //  Dadd(x31, x31, Operand(Code::kHeaderSize - kHeapObjectTag));
    Call(x31, cond, rs, rt);
    return;
  } else if (options().inline_offheap_trampolines) {
    int builtin_index = Builtins::kNoBuiltinId;
    if (isolate()->builtins()->IsBuiltinHandle(code, &builtin_index) &&
        Builtins::IsIsolateIndependent(builtin_index)) {
      // Inline the trampoline.
      RecordCommentForOffHeapTrampoline(builtin_index);
      CHECK_NE(builtin_index, Builtins::kNoBuiltinId);
      EmbeddedData d = EmbeddedData::FromBlob();
      Address entry = d.InstructionStartOfBuiltin(builtin_index);
      li(x31, Operand(entry, RelocInfo::OFF_HEAP_TARGET));
      Call(x31, cond, rs, rt);
      return;
    }
  }

  DCHECK(RelocInfo::IsCodeTarget(rmode));
  DCHECK(code->IsExecutable());
  Call(code.address(), rmode, cond, rs, rt);
}

void TurboAssembler::Call(Address target, RelocInfo::Mode rmode, Condition cond,
                          Register rs, const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  li(x31, Operand(static_cast<int64_t>(target), rmode), ADDRESS_LOAD);
  Call(x31, cond, rs, rt);
}

void TurboAssembler::Prologue() {
  Push(ra, fp, cp, a1);
  Dadd(fp, sp, StandardFrameConstants::kFixedFrameSizeFromFp);
}

void TurboAssembler::StubPrologue(StackFrame::Type type) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  li(scratch, Operand(StackFrame::TypeToMarker(type)));
  PushCommonFrame(scratch);
}

void TurboAssembler::EnterFrame(StackFrame::Type type) {
  UseScratchRegisterScope temps(this);
  Register type_reg = temps.Acquire();
  Move(type_reg, StackFrame::TypeToMarker(type));
  Push(ra, fp, type_reg);
  const int kFrameSize = TypedFrameConstants::kFixedFrameSizeFromFp;
  Dadd(fp, sp, kFrameSize);
  // sp[2] : ra
  // sp[1] : old fp    <-- fp
  // sp[0] : type
}

void TurboAssembler::LeaveFrame(StackFrame::Type type) {
  // Drop the execution stack down to the frame pointer and restore
  // the caller frame pointer and return address.
  Move(sp, fp);
  Pop(fp, ra);
}

void TurboAssembler::li(Register dst, Handle<HeapObject> value, LiFlags mode) {
  if (root_array_available_ && options().isolate_independent_code) {
    IndirectLoadConstant(dst, value);
    return;
  }
  li(dst, Operand(value), mode);
}

void TurboAssembler::li(Register dst, ExternalReference value, LiFlags mode) {
  if (root_array_available_ && options().isolate_independent_code) {
    IndirectLoadExternalReference(dst, value);
    return;
  }
  li(dst, Operand(value), mode);
}

void TurboAssembler::li(Register dst, const StringConstantBase* string,
                        LiFlags mode) {
  li(dst, Operand::EmbeddedStringConstant(string), mode);
}

// Ref: https://github.com/riscv/riscv-elf-psabi-doc/
// For RISCV64, XLEN is 64 bit.
// The base integer calling convention provides eight argument registers, a0-a7,
// the first two of which are also used to return values.
// 1. Scalars that are at most XLEN bits wide are passed in a single argument
// register, or on the stack by value if none is available. When passed in
// registers or on the stack, integer scalars narrower than XLEN bits are
// widened according to the sign of their type up to 32 bits, then sign-extended
// to XLEN bits.
// When passed in registers or on the stack, floating-point types narrower than
// XLEN bits are widened to XLEN bits, with the upper bits undefined.
// 2. Scalars that are 2xXLEN bits wide are passed in a pair of argument
// registers, with the low-order XLEN bits in the lower-numbered register and
// the high-order XLEN bits in the higher-numbered register. If no argument
// registers are available, the scalar is passed on the stack by value. If
// exactly one register is available, the low-order XLEN bits are passed in the
// register and the high-order XLEN bits are passed on the stack.
//
// Scalars wider than 2 x XLEN are passed by reference and are replaced in the
// argument list with the address.

static const int kRegisterPassedArguments = 8;

int TurboAssembler::CalculateStackPassedDWords(int num_reg_arguments,
                                               int num_double_arguments) {
  int stack_passed_dwords = 0;
  num_reg_arguments += num_double_arguments;

  // Up to eight simple arguments are passed in registers a0..a7.
  if (num_reg_arguments > kRegisterPassedArguments) {
    stack_passed_dwords += num_reg_arguments - kRegisterPassedArguments;
  }
  // no slot alloc for the a0~a7 arguments
  // stack_passed_words += kCArgSlotCount;
  return stack_passed_dwords;
}

void TurboAssembler::CheckPageFlag(Register object, Register scratch, int mask,
                                   Condition cc, Label* condition_met) {
  And(scratch, object, Operand(~kPageAlignmentMask));
  Ld(scratch, MemOperand(scratch, MemoryChunk::kFlagsOffset));
  And(scratch, scratch, Operand(mask));
  Branch(condition_met, cc, scratch, Operand(zero_reg));
}

void TurboAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          int num_double_arguments,
                                          Register scratch) {
  int frame_alignment = base::OS::ActivationFrameAlignment();

  // Up to eight simple arguments in a0..a3, a4..a7, No argument slots.
  // Remaining arguments are pushed on the stack, above (higher
  // address than) the (O32) argument slots. (arg slot calculation handled by
  // CalculateStackPassedWords()).
  // Function arguments are usually passed in the a0 to a7 registers, not on the
  // stack. An argument is only passed via the stack if there is no room in the
  // a* registers left.

  int stack_passed_arguments =
      CalculateStackPassedDWords(num_reg_arguments, num_double_arguments);
  // TODO: Would RISCV64 ISA would run on a system with 32 bit pointer size?
  if (frame_alignment > kPointerSize) {
    // Make stack end at alignment and make room for num_arguments - 4 words
    // and the original value of sp.
    addi(scratch, sp, 0);
    Dsub(sp, sp, Operand((stack_passed_arguments + 1) * kPointerSize));
    DCHECK(base::bits::IsPowerOfTwo(frame_alignment));
    And(sp, sp, Operand(-frame_alignment));
    Sd(scratch, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    Dsub(sp, sp, Operand(stack_passed_arguments * kPointerSize));
  }
}

void TurboAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          Register scratch) {
  PrepareCallCFunction(num_reg_arguments, 0, scratch);
}

void TurboAssembler::PushCommonFrame(Register marker_reg) {
  if (marker_reg.is_valid()) {
    Push(ra, fp, marker_reg);
    Dadd(fp, sp, Operand(kPointerSize));
  } else {
    Push(ra, fp);
    mov(fp, sp);
  }
}

void TurboAssembler::PushStandardFrame(Register function_reg) {
  int offset = -StandardFrameConstants::kContextOffset;
  if (function_reg.is_valid()) {
    Push(ra, fp, cp, function_reg);
    offset += kPointerSize;
  } else {
    Push(ra, fp, cp);
  }
  Dadd(fp, sp, Operand(offset));
}

void TurboAssembler::SaveRegisters(RegList registers) {
  DCHECK_GT(NumRegs(registers), 0);
  RegList regs = 0;
  for (int i = 0; i < Register::kNumRegisters; ++i) {
    if ((registers >> i) & 1u) {
      regs |= Register::from_code(i).bit();
    }
  }
  MultiPush(regs);
}

void TurboAssembler::RestoreRegisters(RegList registers) {
  DCHECK_GT(NumRegs(registers), 0);
  RegList regs = 0;
  for (int i = 0; i < Register::kNumRegisters; ++i) {
    if ((registers >> i) & 1u) {
      regs |= Register::from_code(i).bit();
    }
  }
  MultiPop(regs);
}

void TurboAssembler::CallEphemeronKeyBarrier(Register object, Register address,
                                             SaveFPRegsMode fp_mode) {
  EphemeronKeyBarrierDescriptor descriptor;
  RegList registers = descriptor.allocatable_registers();

  SaveRegisters(registers);

  Register object_parameter(
      descriptor.GetRegisterParameter(EphemeronKeyBarrierDescriptor::kObject));
  Register slot_parameter(descriptor.GetRegisterParameter(
      EphemeronKeyBarrierDescriptor::kSlotAddress));
  Register fp_mode_parameter(
      descriptor.GetRegisterParameter(EphemeronKeyBarrierDescriptor::kFPMode));

  Push(object);
  Push(address);

  Pop(slot_parameter);
  Pop(object_parameter);

  Move(fp_mode_parameter, Smi::FromEnum(fp_mode));
  Call(isolate()->builtins()->builtin_handle(Builtins::kEphemeronKeyBarrier),
       RelocInfo::CODE_TARGET);
  RestoreRegisters(registers);
}

void TurboAssembler::CallRecordWriteStub(
    Register object, Operand address, RememberedSetAction remembered_set_action,
    SaveFPRegsMode fp_mode) {
  CallRecordWriteStub(
      object, address, remembered_set_action, fp_mode,
      isolate()->builtins()->builtin_handle(Builtins::kRecordWrite),
      kNullAddress);
}

void TurboAssembler::CallRecordWriteStub(
    Register object, Operand address, RememberedSetAction remembered_set_action,
    SaveFPRegsMode fp_mode, Address wasm_target) {
  CallRecordWriteStub(object, address, remembered_set_action, fp_mode,
                      Handle<Code>::null(), wasm_target);
}

void TurboAssembler::CallRecordWriteStub(
    Register object, Operand address, RememberedSetAction remembered_set_action,
    SaveFPRegsMode fp_mode, Handle<Code> code_target, Address wasm_target) {
  // TODO(albertnetymk): For now we ignore remembered_set_action and fp_mode,
  // i.e. always emit remember set and save FP registers in RecordWriteStub. If
  // large performance regression is observed, we should use these values to
  // avoid unnecessary work.

  RecordWriteDescriptor descriptor;
  RegList registers = descriptor.allocatable_registers();

  SaveRegisters(registers);
  Register object_parameter(
      descriptor.GetRegisterParameter(RecordWriteDescriptor::kObject));
  Register slot_parameter(
      descriptor.GetRegisterParameter(RecordWriteDescriptor::kSlot));
  Register remembered_set_parameter(
      descriptor.GetRegisterParameter(RecordWriteDescriptor::kRememberedSet));
  Register fp_mode_parameter(
      descriptor.GetRegisterParameter(RecordWriteDescriptor::kFPMode));

  Push(object);
  // Push(address);

  Pop(slot_parameter);
  Pop(object_parameter);

  Move(remembered_set_parameter, Smi::FromEnum(remembered_set_action));
  Move(fp_mode_parameter, Smi::FromEnum(fp_mode));
  if (code_target.is_null()) {
    Call(wasm_target, RelocInfo::WASM_STUB_CALL);
  } else {
    Call(code_target, RelocInfo::CODE_TARGET);
  }

  RestoreRegisters(registers);
}

void TurboAssembler::StoreReturnAddressAndCall(Register target) {
  // This generates the final instruction sequence for calls to C functions
  // once an exit frame has been constructed.
  //
  // Note that this assumes the caller code (i.e. the Code object currently
  // being generated) is immovable or that the callee function cannot trigger
  // GC, since the callee function will return to it.

  // Compute the return address in ra to return to after the jump below. The pc
  // is already at '+ 8' from the current instruction; but return is after three
  // instructions, so add another 4 to pc to get the return address.
  auipc(ra, 4);
  Sd(ra, MemOperand(sp));
  Call(target);
}

void TurboAssembler::CallCFunction(ExternalReference function,
                                   int num_reg_arguments,
                                   int num_double_arguments) {
  UseScratchRegisterScope temps(this);
  Register scratch =temps.Acquire();
  li(scratch, function);
  CallCFunctionHelper(scratch, num_reg_arguments, num_double_arguments);
}

void TurboAssembler::CallCFunction(Register function, int num_reg_arguments,
                                   int num_double_arguments) {
  CallCFunctionHelper(function, num_reg_arguments, num_double_arguments);
}

void TurboAssembler::CallCFunction(ExternalReference function,
                                   int num_arguments) {
  CallCFunction(function, num_arguments, 0);
}

void TurboAssembler::CallCFunction(Register function, int num_arguments) {
  CallCFunction(function, num_arguments, 0);
}

void TurboAssembler::CallCFunctionHelper(Register function,
                                         int num_reg_arguments,
                                         int num_double_arguments) {
  DCHECK_LE(num_reg_arguments + num_double_arguments, kMaxCParameters);
  DCHECK(has_frame());
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
  // The argument stots are presumed to have been set up by

  // Just call directly. The function called cannot cause a GC, or
  // allow preemption, so the return address in the link register
  // stays correct.
  {
    // Save the frame pointer and PC so that the stack layout remains iterable,
    // even without an ExitFrame which normally exists between JS and C frames.
    // 't' registers are caller-saved so this is safe as a scratch register.
    Register pc_scratch = t1;
    Register scratch = t2;
    DCHECK(!AreAliased(pc_scratch, scratch, function));

    auipc(pc_scratch, 0);

    // See x64 code for reasoning about how to address the isolate data fields.
    if (root_array_available()) {
      Sd(pc_scratch, MemOperand(kRootRegister,
                                IsolateData::fast_c_call_caller_pc_offset()));
      Sd(fp, MemOperand(kRootRegister,
                        IsolateData::fast_c_call_caller_fp_offset()));
    } else {
      DCHECK_NOT_NULL(isolate());
      li(scratch, ExternalReference::fast_c_call_caller_pc_address(isolate()));
      Sd(pc_scratch, MemOperand(scratch));
      li(scratch, ExternalReference::fast_c_call_caller_fp_address(isolate()));
      Sd(fp, MemOperand(scratch));
    }

    Call(function);

    // We don't unset the PC; the FP is the source of truth.
    if (root_array_available()) {
      Sd(zero_reg, MemOperand(kRootRegister,
                              IsolateData::fast_c_call_caller_fp_offset()));
    } else {
      DCHECK_NOT_NULL(isolate());
      li(scratch, ExternalReference::fast_c_call_caller_fp_address(isolate()));
      Sd(zero_reg, MemOperand(scratch));
    }
  }

  int stack_passed_arguments =
      CalculateStackPassedDWords(num_reg_arguments, num_double_arguments);

  if (base::OS::ActivationFrameAlignment() > kPointerSize) {
    Ld(sp, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    Dadd(sp, sp, Operand(stack_passed_arguments * kPointerSize));
  }
}

void TurboAssembler::Drop(int count, uint64_t unit_size) {
  DCHECK_GE(count, 0);
  uint64_t size = count * unit_size;
  DCHECK_EQ(size % 16, 0);
  if (size == 0) {
    return;
  }
  Dadd(sp, sp, size);
}

int TurboAssembler::PushCallerSaved(SaveFPRegsMode fp_mode, Register exclusion1,
                                    Register exclusion2, Register exclusion3) {
  int bytes = 0;
  RegList exclusions = 0;
  if (exclusion1 != no_reg) {
    exclusions |= exclusion1.bit();
    if (exclusion2 != no_reg) {
      exclusions |= exclusion2.bit();
      if (exclusion3 != no_reg) {
        exclusions |= exclusion3.bit();
      }
    }
  }

  RegList list = kJSCallerSaved & ~exclusions;
  MultiPush(list);
  bytes += NumRegs(list) * kPointerSize;

  if (fp_mode == kSaveFPRegs) {
    MultiPushFPU(kCallerSavedDoubles);
    bytes += NumRegs(kCallerSavedDoubles) * kDoubleSize;
  }

  return bytes;
}

void TurboAssembler::MultiPush(RegList regs) {
  int16_t num_to_push = base::bits::CountPopulation(regs);
  int16_t stack_offset = num_to_push * kPointerSize;

  Dsub(sp, sp, Operand(stack_offset));
  for (int16_t i = kNumRegisters - 1; i >= 0; i--) {
    if ((regs & (1 << i)) != 0) {
      stack_offset -= kPointerSize;
      Sd(ToRegister(i), MemOperand(sp, stack_offset));
    }
  }
}

void TurboAssembler::MultiPop(RegList regs) {
  int16_t stack_offset = 0;

  for (int16_t i = 0; i < kNumRegisters; i++) {
    if ((regs & (1 << i)) != 0) {
      Ld(ToRegister(i), MemOperand(sp, stack_offset));
      stack_offset += kPointerSize;
    }
  }
  addi(sp, sp, stack_offset);
}

void TurboAssembler::MultiPushFPU(RegList regs) {
  int16_t num_to_push = base::bits::CountPopulation(regs);
  int16_t stack_offset = num_to_push * kDoubleSize;

  Dsub(sp, sp, Operand(stack_offset));
  for (int16_t i = kNumRegisters - 1; i >= 0; i--) {
    if ((regs & (1 << i)) != 0) {
      stack_offset -= kDoubleSize;
      Fsd(FPURegister::from_code(i), MemOperand(sp, stack_offset));
    }
  }
}

void TurboAssembler::MultiPopFPU(RegList regs) {
  int16_t stack_offset = 0;

  for (int16_t i = 0; i < kNumRegisters; i++) {
    if ((regs & (1 << i)) != 0) {
      Fld(FPURegister::from_code(i), MemOperand(sp, stack_offset));
      stack_offset += kDoubleSize;
    }
  }
  addi(sp, sp, stack_offset);
}

void TurboAssembler::TryConvertDoubleToInt64(Register result,
                                             DoubleRegister double_input,
                                             Label* done) {
  // Try to convert with an FPU convert instruction. It's trivial to compute
  // the modulo operation on an integer register so we convert to a 64-bit
  // integer.
  //
  // Fcvtzs will saturate to INT64_MIN (0x800...00) or INT64_MAX (0x7FF...FF)
  // when the double is out of range. NaNs and infinities will be converted to 0
  // (as ECMA-262 requires).
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(scratch != result);
  mov(scratch, zero_reg);
  fscsr(scratch);
  fcvtld(result, double_input, 0x1);
  // The values INT64_MIN (0x800...00) or INT64_MAX (0x7FF...FF) are not
  // representable using a double, so if the result is one of those then we know
  // that saturation occurred, and we need to manually handle the conversion.
  //
  fscsr(scratch);
  And(scratch, scratch,
      Operand(kFCSRInvalidOpFlagMask | kFCSRUnderflowFlagMask |
              kFCSRUnderflowFlagMask));
  Jump(done, eq, scratch, zero_reg);
}

void TurboAssembler::JumpIfSmi(Register value, Label* smi_label,
                               Label* not_smi_label) {
  STATIC_ASSERT((kSmiTagSize == 1) && (kSmiTag == 0));
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(scratch != value);
  if (smi_label) {
    And(scratch, value, 0x01);
    Jump(smi_label, eq, scratch, Operand(x0));
    if (not_smi_label) {
      jmp(not_smi_label);
    }
  } else {
    DCHECK(not_smi_label);
    And(scratch, value, 0x01);
    Jump(not_smi_label, ne, scratch, Operand(x0));
  }
}

void TurboAssembler::SmiTag(Register dst, Register src) {
  DCHECK(SmiValuesAre32Bits() || SmiValuesAre31Bits());
  slli(dst, src, kSmiShift);
}

void TurboAssembler::SmiUntag(Register dst, Register src) {
  DCHECK(SmiValuesAre32Bits() || SmiValuesAre31Bits());
  srai(dst, src, kSmiShift);
}

void TurboAssembler::SmiUntag(Register dst, const MemOperand& src) {
  Ld(dst, src);
  SmiUntag(dst);
}

// MacroAssembler

void MacroAssembler::LoadMap(Register destination, Register object) {
  Ld(destination, FieldMemOperand(object, HeapObject::kMapOffset));
}

void MacroAssembler::LoadNativeContextSlot(int index, Register dst) {
  LoadMap(dst, cp);
  Ld(dst,
     FieldMemOperand(dst, Map::kConstructorOrBackPointerOrNativeContextOffset));
  Ld(dst, MemOperand(dst, Context::SlotOffset(index)));
}

void MacroAssembler::JumpToInstructionStream(Address entry) {
  li(kOffHeapTrampolineRegister, Operand(entry, RelocInfo::OFF_HEAP_TARGET));
  Jump(kOffHeapTrampolineRegister);
}

void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid) {
  const Runtime::Function* function = Runtime::FunctionForId(fid);
  DCHECK_EQ(1, function->result_size);
  if (function->nargs >= 0) {
    PrepareCEntryArgs(function->nargs);
  }
  JumpToExternalReference(ExternalReference::Create(fid));
}

void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin,
                                             bool builtin_exit_frame) {
  PrepareCEntryFunction(builtin);
  Handle<Code> code = CodeFactory::CEntry(isolate(), 1, kDontSaveFPRegs,
                                          kArgvOnStack, builtin_exit_frame);
  Jump(code, RelocInfo::CODE_TARGET, al, zero_reg, Operand(zero_reg));
}

void MacroAssembler::CallRuntime(const Runtime::Function* f, int num_arguments,
                                 SaveFPRegsMode save_doubles) {
  // All arguments must be on the stack before this function is called.
  // a0 holds the return value after the call.

  // Check that the number of arguments matches what the function expects.
  // If f->nargs is -1, the function can accept a variable number of arguments.
  CHECK(f->nargs < 0 || f->nargs == num_arguments);

  // Place the necessary arguments.
  Move(a0, num_arguments);
  Move(a1, ExternalReference::Create(f));

  Handle<Code> code =
      CodeFactory::CEntry(isolate(), f->result_size, save_doubles);
  Call(code, RelocInfo::CODE_TARGET);
}

void MacroAssembler::JumpIfNotSmi(Register value, Label* not_smi_label) {
  JumpIfSmi(value, nullptr, not_smi_label);
}

void MacroAssembler::LoadWeakValue(Register out, Register in,
                                   Label* target_if_cleared) {
  Jump(target_if_cleared, eq, in, Operand(kClearedWeakHeapObjectLower32));
  And(out, in, Operand(~kWeakHeapObjectMask));
}

void MacroAssembler::GetInstanceType(Register object, Register map,
                                     Register type_reg) {
  LoadMap(map, object);
  Lhu(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
}

void MacroAssembler::PushCalleeSavedRegisters() {
  MultiPushFPU(kCalleeSavedDoubles);
  Push(ra, fp);
  constexpr RegList CalleeSaved =
      Register::ListOf(s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11);
  MultiPush(CalleeSaved);
}

void MacroAssembler::PopCalleeSavedRegisters() {
  constexpr RegList CalleeSaved =
      Register::ListOf(s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11);
  MultiPop(CalleeSaved);
  Pop(ra, fp);
  MultiPopFPU(kCalleeSavedDoubles);
}

void TurboAssembler::LoadRoot(Register destination, RootIndex index) {
  Ld(destination, MemOperand(s6, RootRegisterOffsetForRootIndex(index)));
}

void TurboAssembler::LoadRoot(Register destination, RootIndex index,
                              Condition cond, Register src1,
                              const Operand& src2) {
  Branch(8, NegateCondition(cond), src1, src2);
  Ld(destination, MemOperand(s6, RootRegisterOffsetForRootIndex(index)));
}

void TurboAssembler::LoadEntryFromBuiltinIndex(Register builtin_index) {
  STATIC_ASSERT(kSystemPointerSize == 8);
  STATIC_ASSERT(kSmiTagSize == 1);
  STATIC_ASSERT(kSmiTag == 0);

  // The builtin_index register contains the builtin index as a Smi.
  SmiUntag(builtin_index);
  GenScaledAddress(builtin_index, kRootRegister, builtin_index,
                   kSystemPointerSizeLog2);
  Ld(builtin_index, MemOperand(builtin_index));
  Ld(builtin_index,
     MemOperand(builtin_index, IsolateData::builtin_entry_table_offset()));
}

void TurboAssembler::Check(Condition cc, AbortReason reason, Register rs,
                           Operand rt) {
  Label L;
  Jump(&L, cc, rs, rt);
  Abort(reason);
  // Will not return here.
  bind(&L);
}

void MacroAssembler::AssertFunction(Register object) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(object != scratch);
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    And(scratch, object, Operand(kSmiTagMask));
    Check(ne, AbortReason::kOperandIsASmiAndNotAFunction, scratch,
          Operand(zero_reg));
    GetInstanceType(object, scratch, scratch);
    Check(eq, AbortReason::kOperandIsNotAFunction, scratch,
          Operand(JS_FUNCTION_TYPE));
  }
}

void MacroAssembler::AssertConstructor(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(object != scratch);
    And(scratch, object, Operand(kSmiTagMask));
    Check(ne, AbortReason::kOperandIsASmiAndNotAConstructor, scratch,
          Operand(zero_reg));
    LoadMap(scratch, object);
    Lbu(scratch, FieldMemOperand(scratch, Map::kBitFieldOffset));
    And(scratch, scratch, Operand(Map::Bits1::IsConstructorBit::kMask));
    Check(ne, AbortReason::kOperandIsNotAConstructor, scratch, Operand(zero_reg));
  }
}

void MacroAssembler::AssertBoundFunction(Register object) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(object != scratch);
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    And(scratch, object, Operand(kSmiTagMask));
    Check(ne, AbortReason::kOperandIsASmiAndNotABoundFunction, scratch,
          Operand(zero_reg));
    GetInstanceType(object, scratch, scratch);
    Check(eq, AbortReason::kOperandIsNotABoundFunction, scratch,
          Operand(JS_BOUND_FUNCTION_TYPE));
  }
}
void MacroAssembler::JumpIfRoot(const Register& obj, RootIndex index,
                                Label* if_equal) {
  UseScratchRegisterScope temps(this);
  Register temp = temps.Acquire();
  DCHECK(!AreAliased(obj, temp));
  LoadRoot(temp, index);
  Jump(if_equal, eq, temp, Operand(obj));
}

void MacroAssembler::JumpIfNotRoot(const Register& obj, RootIndex index,
                                   Label* if_not_equal) {
  UseScratchRegisterScope temps(this);
  Register temp = temps.Acquire();
  DCHECK(!AreAliased(obj, temp));
  LoadRoot(temp, index);
  Jump(if_not_equal, ne, temp, Operand(obj));
}

void MacroAssembler::InvokePrologue(Register expected_parameter_count,
                                    Register actual_parameter_count,
                                    Label* done, InvokeFlag flag) {
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. The registers
  // are set up according to contract with ArgumentsAdaptorTrampoline:
  //  a0: actual arguments count
  //  a1: function (passed through to callee)
  //  a2: expected arguments count

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract.

  DCHECK_EQ(actual_parameter_count, a0);
  DCHECK_EQ(expected_parameter_count, a2);

  Jump(&regular_invoke, eq, expected_parameter_count,
       Operand(actual_parameter_count));

  Handle<Code> adaptor = BUILTIN_CODE(isolate(), ArgumentsAdaptorTrampoline);
  if (flag == CALL_FUNCTION) {
    Call(adaptor);
    Jump(done);
  } else {
    Jump(adaptor, RelocInfo::CODE_TARGET);
  }

  bind(&regular_invoke);
}

void MacroAssembler::CheckDebugHook(Register fun, Register new_target,
                                    Register expected_parameter_count,
                                    Register actual_parameter_count) {
  Label skip_hook;

  li(t0, ExternalReference::debug_hook_on_function_call_address(isolate()));
  Lb(t0, MemOperand(t0));
  Jump(&skip_hook, eq, t0, Operand(zero_reg));

  {
    // Load receiver to pass it later to DebugOnFunctionCall hook.
    GenScaledAddress(t0, sp, actual_parameter_count, kPointerSizeLog2);
    //  t0 -> potiner to  receiver
    Ld(t0, MemOperand(t0));
    Ld(t0, MemOperand(t0));
    FrameScope frame(this,
                     has_frame() ? StackFrame::NONE : StackFrame::INTERNAL);
    SmiTag(expected_parameter_count);
    Push(expected_parameter_count);

    SmiTag(actual_parameter_count);
    Push(actual_parameter_count);

    if (new_target.is_valid()) {
      Push(new_target);
    }
    Push(fun);
    Push(fun);
    Push(t0);
    CallRuntime(Runtime::kDebugOnFunctionCall);
    Pop(fun);
    if (new_target.is_valid()) {
      Pop(new_target);
    }

    Pop(actual_parameter_count);
    SmiUntag(actual_parameter_count);

    Pop(expected_parameter_count);
    SmiUntag(expected_parameter_count);
  }
  bind(&skip_hook);
}

void MacroAssembler::InvokeFunctionCode(Register function, Register new_target,
                                        Register expected_parameter_count,
                                        Register actual_parameter_count,
                                        InvokeFlag flag) {
  // You can't call a function without a valid frame.
  DCHECK_IMPLIES(flag == CALL_FUNCTION, has_frame());
  DCHECK_EQ(function, a1);
  DCHECK_IMPLIES(new_target.is_valid(), new_target == a3);

  // On function call, call into the debugger if necessary.
  CheckDebugHook(function, new_target, expected_parameter_count,
                 actual_parameter_count);

  // Clear the new.target register if not given.
  if (!new_target.is_valid()) {
    LoadRoot(a3, RootIndex::kUndefinedValue);
  }

  Label done;
  InvokePrologue(expected_parameter_count, actual_parameter_count, &done, flag);
  // We call indirectly through the code field in the function to
  // allow recompilation to take effect without changing any of the
  // call sites.
  Register code = kJavaScriptCallCodeStartRegister;
  LoadTaggedPointerField(code,
                         FieldMemOperand(function, JSFunction::kCodeOffset));                     
  if (flag == CALL_FUNCTION) {
    CallCodeObject(code);
  } else {
    DCHECK(flag == JUMP_FUNCTION);
    JumpCodeObject(code);
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}

// If lr_status is kLRHasBeenSaved, lr will be clobbered.
//
// The register 'object' contains a heap object pointer. The heap object tag is
// shifted away.
void MacroAssembler::RecordWrite(Register object, Operand offset,
                                 Register value, RAStatus ra_status,
                                 SaveFPRegsMode fp_mode,
                                 RememberedSetAction remembered_set_action,
                                 SmiCheck smi_check) {
  ASM_LOCATION_IN_ASSEMBLER("MacroAssembler::RecordWrite");
  DCHECK(!AreAliased(object, value));
  UseScratchRegisterScope temps(this);
  Register temp = temps.Acquire();
  DCHECK(temp != object);
  DCHECK(temp != value);
  if (emit_debug_code()) {
    Dadd(temp, object, offset);
    // LoadTaggedPointerField(temp, MemOperand(temp));
    Ld(temp, MemOperand(temp));
    Assert(eq, AbortReason::kWrongAddressOrValuePassedToRecordWrite, temp,
           Operand(value));
  }

  if ((remembered_set_action == OMIT_REMEMBERED_SET &&
       !FLAG_incremental_marking) ||
      FLAG_disable_write_barriers) {
    return;
  }

  // First, check if a write barrier is even needed. The tests below
  // catch stores of smis and stores into the young generation.
  Label done;

  if (smi_check == INLINE_SMI_CHECK) {
    DCHECK_EQ(0, kSmiTag);
    JumpIfSmi(value, &done);
  }

  CheckPageFlag(value, temp, MemoryChunk::kPointersToHereAreInterestingMask, ne,
                &done);
  CheckPageFlag(object, temp, MemoryChunk::kPointersFromHereAreInterestingMask,
                ne, &done);

  // Record the actual write.
  if (ra_status == kRAHasNotBeenSaved) {
    Push(ra);
  }
  CallRecordWriteStub(object, offset, remembered_set_action, fp_mode);
  if (ra_status == kRAHasNotBeenSaved) {
    Pop(ra);
  }

  bind(&done);
}

// Clobbers object, dst, value, and ra, if (ra_status == kRAHasBeenSaved)
// The register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWriteField(Register object, int offset,
                                      Register value, RAStatus ra_status,
                                      SaveFPRegsMode save_fp,
                                      RememberedSetAction remembered_set_action,
                                      SmiCheck smi_check) {
  // First, check if a write barrier is even needed. The tests below
  // catch stores of Smis.
  Label done;

  // Skip the barrier if writing a smi.
  if (smi_check == INLINE_SMI_CHECK) {
    JumpIfSmi(value, &done);
  }

  // Although the object register is tagged, the offset is relative to the start
  // of the object, so offset must be a multiple of kTaggedSize.
  DCHECK(IsAligned(offset, kTaggedSize));

  if (emit_debug_code()) {
    Label ok;
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(scratch != object);
    Dadd(scratch, object, offset - kHeapObjectTag);
    Jump(&ok, eq, scratch, kTaggedSize - 1);
    DebugBreak();
    // Abort(AbortReason::kUnalignedCellInWriteBarrier);
    bind(&ok);
  }

  RecordWrite(object, Operand(offset - kHeapObjectTag), value, ra_status,
              save_fp, remembered_set_action, OMIT_SMI_CHECK);

  bind(&done);
}

void MacroAssembler::AssertNotSmi(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(scratch != object);
    andi(scratch, object, kSmiTagMask);
    Assert(ne, AbortReason::kOperandIsASmi, scratch, Operand(zero_reg));
  }
}

void MacroAssembler::AssertUndefinedOrAllocationSite(Register object,
                                                     Register scratch) {
  if (emit_debug_code()) {
    Label done_checking;
    AssertNotSmi(object);
    LoadRoot(scratch, RootIndex::kUndefinedValue);
    Jump(&done_checking, eq, object, Operand(scratch));
    GetInstanceType(object, scratch, scratch);
    Assert(eq, AbortReason::kExpectedUndefinedOrCell, scratch,
           Operand(ALLOCATION_SITE_TYPE));
    bind(&done_checking);
  }
}

void MacroAssembler::EnterExitFrame(bool save_doubles, const Register& scratch,
                                    int extra_space,
                                    StackFrame::Type frame_type) {
  DCHECK(frame_type == StackFrame::EXIT ||
         frame_type == StackFrame::BUILTIN_EXIT);
  // Set up the new stack frame.
  Push(ra, fp);
  Move(fp, sp);
  Move(scratch, StackFrame::TypeToMarker(frame_type));
  Push(scratch, zero_reg);
  //          fp[8]: CallerPC (lr)
  //    fp -> fp[0]: CallerFP (old fp)
  //          fp[-8]: STUB marker
  //    sp -> fp[-16]: Space reserved for SPOffset.
  STATIC_ASSERT((2 * kSystemPointerSize) ==
                ExitFrameConstants::kCallerSPOffset);
  STATIC_ASSERT((1 * kSystemPointerSize) ==
                ExitFrameConstants::kCallerPCOffset);
  STATIC_ASSERT((0 * kSystemPointerSize) ==
                ExitFrameConstants::kCallerFPOffset);
  STATIC_ASSERT((-2 * kSystemPointerSize) == ExitFrameConstants::kSPOffset);

  // Save the frame pointer and context pointer in the top frame.
  Move(scratch, ExternalReference::Create(IsolateAddressId::kCEntryFPAddress,
                                          isolate()));
  Sd(fp, MemOperand(scratch));
  Move(scratch,
       ExternalReference::Create(IsolateAddressId::kContextAddress, isolate()));
  Sd(cp, MemOperand(scratch));

  STATIC_ASSERT((-2 * kSystemPointerSize) ==
                ExitFrameConstants::kLastExitFrameField);
  if (save_doubles) {
    MultiPushFPU(kCalleeSavedDoubles);
  }

  // Round the number of space we need to claim to a multiple of two.
  int slots_to_claim = RoundUp(extra_space + 1, 2);

  // Reserve space for the return address and for user requested memory.
  // We do this before aligning to make sure that we end up correctly
  // aligned with the minimum of wasted space.
  uint64_t size = slots_to_claim * kXRegSize;
  Dsub(sp, sp, size);
  //         fp[8]: CallerPC (ra)
  //   fp -> fp[0]: CallerFP (old fp)
  //         fp[-8]: STUB marker
  //         fp[-16]: Space reserved for SPOffset.
  //         fp[-16 - fp_size]: Saved doubles (if save_doubles is true).
  //         sp[8]: Extra space reserved for caller (if extra_space != 0).
  //   sp -> sp[0]: Space reserved for the return address.

  // ExitFrame::GetStateForFramePointer expects to find the return address at
  // the memory address immediately below the pointer stored in SPOffset.
  // It is not safe to derive much else from SPOffset, because the size of the
  // padding can vary.
  Dadd(scratch, sp, kXRegSize);
  Sd(scratch, MemOperand(fp, ExitFrameConstants::kSPOffset));
}

// Leave the current exit frame.
void MacroAssembler::LeaveExitFrame(bool restore_doubles,
                                    const Register& scratch,
                                    const Register& scratch2, int extra_space) {
  //         fp[8]: CallerPC (ra)
  //   fp -> fp[0]: CallerFP (old fp)
  //         fp[-8]: STUB marker
  //         fp[-16]: Space reserved for SPOffset.
  //         fp[-16 - fp_size]: Saved doubles (if save_doubles is true).
  //         sp[8]: Extra space reserved for caller (if extra_space != 0).
  //   sp -> sp[0]: Space reserved for the return address.
  // free extra_space
  int slots_to_claim = RoundUp(extra_space + 1, 2);
  uint64_t size = slots_to_claim * kXRegSize;
  Dadd(sp, sp, size);
  // restore doubles register
  if (restore_doubles) {
    MultiPopFPU(kCalleeSavedDoubles);
  }

  //         fp[8]: CallerPC (ra)
  //   fp -> fp[0]: CallerFP (old fp)
  //         fp[-8]: STUB marker
  //   sp -> fp[-16]: Space reserved for SPOffset.
  // Restore the context pointer from the top frame.
  Move(scratch,
       ExternalReference::Create(IsolateAddressId::kContextAddress, isolate()));
  Ld(cp, MemOperand(scratch));

  if (emit_debug_code()) {
    // Also emit debug code to clear the cp in the top frame.
    Move(scratch2, Context::kInvalidContext);
    Move(scratch, ExternalReference::Create(IsolateAddressId::kContextAddress,
                                            isolate()));
    Sd(scratch2, MemOperand(scratch));
  }
  // Clear the frame pointer from the top frame.
  Move(scratch, ExternalReference::Create(IsolateAddressId::kCEntryFPAddress,
                                          isolate()));
  Sd(zero_reg, MemOperand(scratch));

  // Pop the exit frame.
  //         fp[8]: CallerPC (lr)
  //   fp -> fp[0]: CallerFP (old fp)
  //         fp[...]: The rest of the frame.
  Move(sp, fp);
  Pop(ra, fp);
}

void MacroAssembler::JumpIfIsInRange(Register value, unsigned lower_limit,
                                     unsigned higher_limit,
                                     Label* on_in_range) {
  if (lower_limit != 0) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    DCHECK(scratch != value);
    Dsub(scratch, value, Operand(lower_limit));
    Jump(on_in_range, ls, scratch, Operand(higher_limit - lower_limit));
  } else {
    Jump(on_in_range, ls, value, Operand(higher_limit - lower_limit));
  }
}

void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  DCHECK_GT(value, 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
    // This operation has to be exactly 32-bit wide in case the external
    // reference table redirects the counter to a uint32_t dummy_stats_counter_
    // field.
    li(scratch2, ExternalReference::Create(counter));
    Lw(scratch1, MemOperand(scratch2));
    Add(scratch1, scratch1, Operand(value));
    Sw(scratch1, MemOperand(scratch2));
  }
}

void MacroAssembler::InvokeFunctionWithNewTarget(
    Register function, Register new_target, Register actual_parameter_count,
    InvokeFlag flag) {
  // You can't call a function without a valid frame.
  DCHECK_IMPLIES(flag == CALL_FUNCTION, has_frame());

  // Contract with called JS functions requires that function is passed in a1.
  DCHECK_EQ(function, a1);
  Register expected_parameter_count = a2;
  Register temp_reg = t0;
  Ld(temp_reg, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  Ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));
  // The argument count is stored as uint16_t
  Lhu(expected_parameter_count,
      FieldMemOperand(temp_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));

  InvokeFunctionCode(a1, new_target, expected_parameter_count,
                     actual_parameter_count, flag);
}

void MacroAssembler::AssertGeneratorObject(Register object) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  DCHECK(scratch != object);
  if (!emit_debug_code()) return;
  STATIC_ASSERT(kSmiTag == 0);
  And(scratch, object, Operand(kSmiTagMask));
  Label L;
  Jump(&L, ne, scratch, Operand(zero_reg));
  Abort(AbortReason::kOperandIsASmiAndNotAGeneratorObject);
  bind(&L);

  GetInstanceType(object, scratch, scratch);

  Label done;

  // Check if JSGeneratorObject
  Jump(&done, eq, scratch, Operand(JS_GENERATOR_OBJECT_TYPE));

  // Check if JSAsyncFunctionObject (See MacroAssembler::CompareInstanceType)
  Jump(&done, eq, scratch, Operand(JS_ASYNC_FUNCTION_OBJECT_TYPE));

  // Check if JSAsyncGeneratorObject
  Jump(&done, eq, scratch, Operand(JS_ASYNC_GENERATOR_OBJECT_TYPE));

  Abort(AbortReason::kOperandIsNotAGeneratorObject);

  bind(&done);
}

void MacroAssembler::MaybeDropFrames() {
  // Check whether we need to drop frames to restart a function on the stack.
  li(a1, ExternalReference::debug_restart_fp_address(isolate()));
  Ld(a1, MemOperand(a1));
  Jump(BUILTIN_CODE(isolate(), FrameDropperTrampoline), RelocInfo::CODE_TARGET,
       ne, a1, Operand(zero_reg));
}

void MacroAssembler::InvokeFunction(Register function,
                                    Register expected_parameter_count,
                                    Register actual_parameter_count,
                                    InvokeFlag flag) {
  // You can't call a function without a valid frame.
  DCHECK_IMPLIES(flag == CALL_FUNCTION, has_frame());

  // Contract with called JS functions requires that function is passed in a1.
  DCHECK_EQ(function, a1);

  // Get the function and setup the context.
  Ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));

  InvokeFunctionCode(a1, no_reg, expected_parameter_count,
                     actual_parameter_count, flag);
}

void TurboAssembler::DecompressTaggedPointer(const Register& destination,
                                             const MemOperand& field_operand) {
  RecordComment("[ DecompressTaggedPointer");
  Lw(destination, field_operand);
  Dadd(destination, kRootRegister, destination);
  RecordComment("]");
}

void TurboAssembler::DecompressTaggedPointer(const Register& destination,
                                             const Register& source) {
  RecordComment("[ DecompressTaggedPointer");
  UNIMPLEMENTED();
  //Dadd(destination, kRootRegister, Operand(source, UXTW));
  RecordComment("]");
}

void TurboAssembler::LoadTaggedPointerField(const Register& destination,
                                            const MemOperand& field_operand) {
  if (COMPRESS_POINTERS_BOOL) {
    DecompressTaggedPointer(destination, field_operand);
  } else {
    Ld(destination, field_operand);
  }
}
// ---------------------------------------------------------------------------
// Exception handling.

}  // namespace internal
}  // namespace v8

#endif  // V8_TARGET_ARCH_RISCV64
