// Copyright 2017 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_WASM_BASELINE_RISCV64_LIFTOFF_ASSEMBLER_RISCV64_H_
#define V8_WASM_BASELINE_RISCV64_LIFTOFF_ASSEMBLER_RISCV64_H_

#include "src/codegen/assembler.h"
#include "src/wasm/baseline/liftoff-assembler.h"
#include "src/wasm/value-type.h"

namespace v8 {
namespace internal {
namespace wasm {

#define RETURN_FALSE_IF_MISSING_CPU_FEATURE(name)    \
  if (!CpuFeatures::IsSupported(name)) return false; \
  CpuFeatureScope feature(this, name);

namespace liftoff {

// plct: define like mips
constexpr int kInstanceOffset = 8;

}  // namespace liftoff

int LiftoffAssembler::PrepareStackFrame() {
  int offset = pc_offset();
  return offset;
}

inline void LiftoffAssembler::PatchPrepareStackFrame(int offset,
                                                     int frame_size) {}

inline void LiftoffAssembler::FinishCode() {}

inline void LiftoffAssembler::AbortCompilation() {}

constexpr int LiftoffAssembler::StaticStackFrameSize() {
  return liftoff::kInstanceOffset;
}

int LiftoffAssembler::SlotSizeForType(ValueType type) {
  return type.element_size_bytes();
}

bool LiftoffAssembler::NeedsAlignment(ValueType type) { return false; }

inline void LiftoffAssembler::LoadConstant(LiftoffRegister reg, WasmValue value,
                                           RelocInfo::Mode rmode) {}

inline void LiftoffAssembler::LoadFromInstance(Register dst, uint32_t offset,
                                               int size) {}

inline void LiftoffAssembler::LoadTaggedPointerFromInstance(Register dst,
                                                            uint32_t offset) {}

inline void LiftoffAssembler::SpillInstance(Register instance) {}

inline void LiftoffAssembler::FillInstanceInto(Register dst) {}

inline void LiftoffAssembler::LoadTaggedPointer(Register dst, Register src_addr,
                                                Register offset_reg,
                                                uint32_t offset_imm,
                                                LiftoffRegList pinned) {}

inline void LiftoffAssembler::Load(LiftoffRegister dst, Register src_addr,
                                   Register offset_reg, uint32_t offset_imm,
                                   LoadType type, LiftoffRegList pinned,
                                   uint32_t* protected_load_pc,
                                   bool is_load_mem) {}

inline void LiftoffAssembler::Store(Register dst_addr, Register offset_reg,
                                    uint32_t offset_imm, LiftoffRegister src,
                                    StoreType type, LiftoffRegList /* pinned */,
                                    uint32_t* protected_store_pc,
                                    bool is_store_mem) {}

void LiftoffAssembler::AtomicLoad(LiftoffRegister dst, Register src_addr,
                                  Register offset_reg, uint32_t offset_imm,
                                  LoadType type, LiftoffRegList pinned) {
  bailout(kAtomics, "AtomicLoad");
}

void LiftoffAssembler::AtomicStore(Register dst_addr, Register offset_reg,
                                   uint32_t offset_imm, LiftoffRegister src,
                                   StoreType type, LiftoffRegList pinned) {
  bailout(kAtomics, "AtomicStore");
}

void LiftoffAssembler::AtomicAdd(Register dst_addr, Register offset_reg,
                                 uint32_t offset_imm, LiftoffRegister value,
                                 StoreType type) {
  bailout(kAtomics, "AtomicAdd");
}

void LiftoffAssembler::AtomicSub(Register dst_addr, Register offset_reg,
                                 uint32_t offset_imm, LiftoffRegister value,
                                 StoreType type) {
  bailout(kAtomics, "AtomicSub");
}

void LiftoffAssembler::AtomicAnd(Register dst_addr, Register offset_reg,
                                 uint32_t offset_imm, LiftoffRegister value,
                                 StoreType type) {
  bailout(kAtomics, "AtomicAnd");
}

void LiftoffAssembler::AtomicOr(Register dst_addr, Register offset_reg,
                                uint32_t offset_imm, LiftoffRegister value,
                                StoreType type) {
  bailout(kAtomics, "AtomicOr");
}

void LiftoffAssembler::AtomicXor(Register dst_addr, Register offset_reg,
                                 uint32_t offset_imm, LiftoffRegister value,
                                 StoreType type) {
  bailout(kAtomics, "AtomicXor");
}

void LiftoffAssembler::AtomicExchange(Register dst_addr, Register offset_reg,
                                      uint32_t offset_imm,
                                      LiftoffRegister result, StoreType type) {}

void LiftoffAssembler::AtomicCompareExchange(
    Register dst_addr, Register offset_reg, uint32_t offset_imm,
    LiftoffRegister expected, LiftoffRegister new_value, LiftoffRegister value,
    StoreType type) {}

void LiftoffAssembler::AtomicFence() {}

inline void LiftoffAssembler::LoadCallerFrameSlot(LiftoffRegister dst,
                                                  uint32_t caller_slot_idx,
                                                  ValueType type) {}

inline void LiftoffAssembler::MoveStackValue(uint32_t dst_offset,
                                             uint32_t src_offset,
                                             ValueType type) {}

inline void LiftoffAssembler::Move(Register dst, Register src, ValueType type) {
}

inline void LiftoffAssembler::Move(DoubleRegister dst, DoubleRegister src,
                                   ValueType type) {}

inline void LiftoffAssembler::Spill(int offset, LiftoffRegister reg,
                                    ValueType type) {}

inline void LiftoffAssembler::Spill(int offset, WasmValue value) {}

inline void LiftoffAssembler::Fill(LiftoffRegister reg, int offset,
                                   ValueType type) {}

inline void LiftoffAssembler::FillI64Half(Register, int offset, RegPairHalf) {}

inline void LiftoffAssembler::FillStackSlotsWithZero(int start, int size) {}

inline void LiftoffAssembler::emit_i32_add(Register dst, Register lhs,
                                           Register rhs) {}

inline void LiftoffAssembler::emit_i32_addi(Register dst, Register lhs,
                                            int32_t imm) {}

inline void LiftoffAssembler::emit_i32_sub(Register dst, Register lhs,
                                           Register rhs) {}

inline void LiftoffAssembler::emit_i32_mul(Register dst, Register lhs,
                                           Register rhs) {}

inline void LiftoffAssembler::emit_i32_divs(Register dst, Register lhs,
                                            Register rhs,
                                            Label* trap_div_by_zero,
                                            Label* trap_div_unrepresentable) {}

inline void LiftoffAssembler::emit_i32_divu(Register dst, Register lhs,
                                            Register rhs,
                                            Label* trap_div_by_zero) {}

inline void LiftoffAssembler::emit_i32_rems(Register dst, Register lhs,
                                            Register rhs,
                                            Label* trap_div_by_zero) {}

inline void LiftoffAssembler::emit_i32_remu(Register dst, Register lhs,
                                            Register rhs,
                                            Label* trap_div_by_zero) {}

inline void LiftoffAssembler::emit_i32_and(Register dst, Register lhs,
                                           Register rhs) {}

inline void LiftoffAssembler::emit_i32_andi(Register dst, Register lhs,
                                            int32_t imm) {}

inline void LiftoffAssembler::emit_i32_or(Register dst, Register lhs,
                                          Register rhs) {}

inline void LiftoffAssembler::emit_i32_ori(Register dst, Register lhs,
                                           int32_t imm) {}

inline void LiftoffAssembler::emit_i32_xor(Register dst, Register lhs,
                                           Register rhs) {}

inline void LiftoffAssembler::emit_i32_xori(Register dst, Register lhs,
                                            int32_t imm) {}

inline void LiftoffAssembler::emit_i32_shl(Register dst, Register src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i32_shli(Register dst, Register src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i32_sar(Register dst, Register src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i32_sari(Register dst, Register src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i32_shr(Register dst, Register src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i32_shri(Register dst, Register src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i32_clz(Register dst, Register src) {}

inline void LiftoffAssembler::emit_i32_ctz(Register dst, Register src) {}

bool LiftoffAssembler::emit_i32_popcnt(Register dst, Register src) {
  return true;
}

inline void LiftoffAssembler::emit_i64_add(LiftoffRegister dst,
                                           LiftoffRegister lhs,
                                           LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_i64_addi(LiftoffRegister dst,
                                            LiftoffRegister lhs, int32_t imm) {}

inline void LiftoffAssembler::emit_i64_sub(LiftoffRegister dst,
                                           LiftoffRegister lhs,
                                           LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_i64_mul(LiftoffRegister dst,
                                           LiftoffRegister lhs,
                                           LiftoffRegister rhs) {}

bool LiftoffAssembler::emit_i64_divs(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero,
                                     Label* trap_div_unrepresentable) {
  return true;
}

bool LiftoffAssembler::emit_i64_divu(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  return true;
}

bool LiftoffAssembler::emit_i64_rems(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  return true;
}

bool LiftoffAssembler::emit_i64_remu(LiftoffRegister dst, LiftoffRegister lhs,
                                     LiftoffRegister rhs,
                                     Label* trap_div_by_zero) {
  return true;
}

inline void LiftoffAssembler::emit_i64_shl(LiftoffRegister dst,
                                           LiftoffRegister src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i64_shli(LiftoffRegister dst,
                                            LiftoffRegister src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i64_sar(LiftoffRegister dst,
                                           LiftoffRegister src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i64_sari(LiftoffRegister dst,
                                            LiftoffRegister src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i64_shr(LiftoffRegister dst,
                                           LiftoffRegister src,
                                           Register amount) {}

inline void LiftoffAssembler::emit_i64_shri(LiftoffRegister dst,
                                            LiftoffRegister src,
                                            int32_t amount) {}

inline void LiftoffAssembler::emit_i64_clz(LiftoffRegister dst,
                                           LiftoffRegister src) {}

inline void LiftoffAssembler::emit_i64_ctz(LiftoffRegister dst,
                                           LiftoffRegister src) {}

inline void LiftoffAssembler::emit_i64_and(LiftoffRegister dst,
                                           LiftoffRegister lhs,
                                           LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_i64_andi(LiftoffRegister dst,
                                            LiftoffRegister lhs, int32_t imm) {}

inline void LiftoffAssembler::emit_i64_or(LiftoffRegister dst,
                                          LiftoffRegister lhs,
                                          LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_i64_ori(LiftoffRegister dst,
                                           LiftoffRegister lhs, int32_t imm) {}

inline void LiftoffAssembler::emit_i64_xor(LiftoffRegister dst,
                                           LiftoffRegister lhs,
                                           LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_i64_xori(LiftoffRegister dst,
                                            LiftoffRegister lhs, int32_t imm) {}

bool LiftoffAssembler::emit_i64_popcnt(LiftoffRegister dst,
                                       LiftoffRegister src) {
  return true;
}

inline void LiftoffAssembler::emit_u32_to_intptr(Register dst, Register src) {}

inline void LiftoffAssembler::emit_f32_add(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_sub(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_mul(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_div(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_min(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_max(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_copysign(DoubleRegister dst,
                                                DoubleRegister lhs,
                                                DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f32_abs(DoubleRegister dst,
                                           DoubleRegister src) {}

inline void LiftoffAssembler::emit_f32_neg(DoubleRegister dst,
                                           DoubleRegister src) {}

bool LiftoffAssembler::emit_f32_ceil(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f32_floor(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f32_trunc(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f32_nearest_int(DoubleRegister dst,
                                            DoubleRegister src) {
  return true;
}

inline void LiftoffAssembler::emit_f32_sqrt(DoubleRegister dst,
                                            DoubleRegister src) {}

inline void LiftoffAssembler::emit_f64_add(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_sub(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_mul(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_div(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_min(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_copysign(DoubleRegister dst,
                                                DoubleRegister lhs,
                                                DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_max(DoubleRegister dst,
                                           DoubleRegister lhs,
                                           DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_abs(DoubleRegister dst,
                                           DoubleRegister src) {}

inline void LiftoffAssembler::emit_f64_neg(DoubleRegister dst,
                                           DoubleRegister src) {}

bool LiftoffAssembler::emit_f64_ceil(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f64_floor(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f64_trunc(DoubleRegister dst, DoubleRegister src) {
  return true;
}

bool LiftoffAssembler::emit_f64_nearest_int(DoubleRegister dst,
                                            DoubleRegister src) {
  return true;
}

inline void LiftoffAssembler::emit_f64_sqrt(DoubleRegister dst,
                                            DoubleRegister src) {}

bool LiftoffAssembler::emit_type_conversion(WasmOpcode opcode,
                                            LiftoffRegister dst,
                                            LiftoffRegister src, Label* trap) {
  return false;
}

inline void LiftoffAssembler::emit_i32_signextend_i8(Register dst,
                                                     Register src) {}

inline void LiftoffAssembler::emit_i32_signextend_i16(Register dst,
                                                      Register src) {}

inline void LiftoffAssembler::emit_i64_signextend_i8(LiftoffRegister dst,
                                                     LiftoffRegister src) {}

inline void LiftoffAssembler::emit_i64_signextend_i16(LiftoffRegister dst,
                                                      LiftoffRegister src) {}

inline void LiftoffAssembler::emit_i64_signextend_i32(LiftoffRegister dst,
                                                      LiftoffRegister src) {}

inline void LiftoffAssembler::emit_jump(Label* label) { jmp(label); }

inline void LiftoffAssembler::emit_jump(Register target) { jmp(target); }

inline void LiftoffAssembler::emit_cond_jump(Condition cond, Label* label,
                                             ValueType type, Register lhs,
                                             Register rhs) {}

inline void LiftoffAssembler::emit_i32_eqz(Register dst, Register src) {}

inline void LiftoffAssembler::emit_i32_set_cond(Condition cond, Register dst,
                                                Register lhs, Register rhs) {}

inline void LiftoffAssembler::emit_i64_eqz(Register dst, LiftoffRegister src) {}

inline void LiftoffAssembler::emit_i64_set_cond(Condition cond, Register dst,
                                                LiftoffRegister lhs,
                                                LiftoffRegister rhs) {}

inline void LiftoffAssembler::emit_f32_set_cond(Condition cond, Register dst,
                                                DoubleRegister lhs,
                                                DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_f64_set_cond(Condition cond, Register dst,
                                                DoubleRegister lhs,
                                                DoubleRegister rhs) {}

inline void LiftoffAssembler::emit_i8x16_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i16x8_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i32x4_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i64x2_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f32x4_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f64x2_splat(LiftoffRegister dst,
                                               LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i8x16_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i8x16_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_add_saturate_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_add_saturate_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_sub_saturate_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_sub_saturate_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_min_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_min_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_max_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_max_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i16x8_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_add_saturate_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_add_saturate_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_sub_saturate_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_sub_saturate_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_min_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_min_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_max_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_max_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i32x4_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_min_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_min_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_max_s(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i32x4_max_u(LiftoffRegister dst,
                                               LiftoffRegister lhs,
                                               LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i64x2_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i64x2_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i64x2_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i64x2_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f32x4_abs(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f32x4_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f32x4_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f32x4_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f32x4_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f64x2_abs(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f64x2_neg(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_f64x2_add(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f64x2_sub(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_f64x2_mul(LiftoffRegister dst,
                                             LiftoffRegister lhs,
                                             LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_rounding_average_u(
    LiftoffRegister dst, LiftoffRegister lhs, LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i16x8_rounding_average_u(
    LiftoffRegister dst, LiftoffRegister lhs, LiftoffRegister rhs) {}
inline void LiftoffAssembler::emit_i8x16_abs(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i16x8_abs(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i32x4_abs(LiftoffRegister dst,
                                             LiftoffRegister src) {}
inline void LiftoffAssembler::emit_i8x16_extract_lane_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i8x16_extract_lane_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i16x8_extract_lane_s(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i16x8_extract_lane_u(LiftoffRegister dst,
                                                        LiftoffRegister lhs,
                                                        uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i32x4_extract_lane(LiftoffRegister dst,
                                                      LiftoffRegister lhs,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i64x2_extract_lane(LiftoffRegister dst,
                                                      LiftoffRegister lhs,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_f32x4_extract_lane(LiftoffRegister dst,
                                                      LiftoffRegister lhs,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_f64x2_extract_lane(LiftoffRegister dst,
                                                      LiftoffRegister lhs,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i8x16_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i16x8_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i32x4_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_i64x2_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_f32x4_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}
inline void LiftoffAssembler::emit_f64x2_replace_lane(LiftoffRegister dst,
                                                      LiftoffRegister src1,
                                                      LiftoffRegister src2,
                                                      uint8_t imm_lane_idx) {}

inline void LiftoffAssembler::StackCheck(Label* ool_code,
                                         Register limit_address) {}

inline void LiftoffAssembler::CallTrapCallbackForTesting() {}

inline void LiftoffAssembler::AssertUnreachable(AbortReason reason) {}

inline void LiftoffAssembler::PushRegisters(LiftoffRegList regs) {}

inline void LiftoffAssembler::PopRegisters(LiftoffRegList regs) {}

inline void LiftoffAssembler::DropStackSlotsAndRet(uint32_t num_stack_slots) {}

inline void LiftoffAssembler::CallC(const FunctionSig* sig,
                                    const LiftoffRegister* args,
                                    const LiftoffRegister* rets,
                                    ValueType out_argument_type,
                                    int stack_bytes,
                                    ExternalReference ext_ref) {}

inline void LiftoffAssembler::CallNativeWasmCode(Address addr) {}

inline void LiftoffAssembler::CallIndirect(
    const FunctionSig* sig, compiler::CallDescriptor* call_descriptor,
    Register target) {}

inline void LiftoffAssembler::CallRuntimeStub(WasmCode::RuntimeStubId sid) {
  // A direct call to a wasm runtime stub defined in this module.
  // Just encode the stub index. This will be patched at relocation.
}

inline void LiftoffAssembler::AllocateStackSlot(Register addr, uint32_t size) {}

inline void LiftoffAssembler::DeallocateStackSlot(uint32_t size) {}

void LiftoffStackSlots::Construct() {
#if 1
  for (auto& slot : slots_) {
    const LiftoffAssembler::VarState& src = slot.src_;
    switch (src.loc()) {
      case LiftoffAssembler::VarState::kStack:
        if (src.type() == kWasmI32) {
          // Load i32 values to a register first to ensure they are zero
          // extended.
          // asm_->movl(kScratchRegister,
          // liftoff::GetStackSlot(slot.src_offset_));
          // asm_->pushq(kScratchRegister);
          asm_->push(kScratchRegister);
        } else {
          // For all other types, just push the whole (8-byte) stack slot.
          // This is also ok for f32 values (even though we copy 4 uninitialized
          // bytes), because f32 and f64 values are clearly distinguished in
          // Turbofan, so the uninitialized bytes are never accessed.
          // asm_->pushq(liftoff::GetStackSlot(slot.src_offset_));
        }
        break;
      case LiftoffAssembler::VarState::kRegister:
        // liftoff::push(asm_, src.reg(), src.type());
        break;
      case LiftoffAssembler::VarState::kIntConst:
        // asm_->pushq(Immediate(src.i32_const()));
        break;
    }
  }
#endif
}

#undef RETURN_FALSE_IF_MISSING_CPU_FEATURE

}  // namespace wasm
}  // namespace internal
}  // namespace v8

#endif  // V8_WASM_BASELINE_RISCV64_LIFTOFF_ASSEMBLER_RISCV64_H_
