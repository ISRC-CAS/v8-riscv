// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_INL_H_
#define V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_INL_H_

#include <type_traits>

#include "src/base/memory.h"
#include "src/codegen/assembler.h"
#include "src/codegen/riscv64/assembler-riscv64.h"
#include "src/debug/debug.h"
#include "src/objects/objects-inl.h"
#include "src/objects/smi.h"

namespace v8 {
namespace internal {

bool CpuFeatures::SupportsOptimizer() { return true; }

bool CpuFeatures::SupportsWasmSimd128() { return true; }

// -----------------------------------------------------------------------------
// Operand and MemOperand.

bool Operand::is_reg() const { return rm_.is_valid(); }

int64_t Operand::immediate() const {
  DCHECK(!is_reg());
  DCHECK(!IsHeapObjectRequest());
  return value_.immediate;
}

// -----------------------------------------------------------------------------
// RelocInfo.

void RelocInfo::apply(intptr_t delta) {}

HeapObject RelocInfo::target_object() {
  return HeapObject::cast(
      Object(Assembler::target_address_at(pc_, constant_pool_)));
}

// Read/Modify the code target address in the branch/call instruction at pc.
Address Assembler::target_address_at(Address pc, Address constant_pool) {
  return target_address_at(pc);
}

inline void Assembler::CheckBuffer() {
  DCHECK_LT(pc_, buffer_start_ + buffer_->size());
  if (buffer_space() < kGap) {
    GrowBuffer();
  }
}

Address RelocInfo::target_address() {
  DCHECK(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_) || IsWasmCall(rmode_));
  return Assembler::target_address_at(pc_, constant_pool_);
}

Address RelocInfo::target_external_reference() {
  DCHECK(rmode_ == RelocInfo::EXTERNAL_REFERENCE);
  return ReadUnalignedValue<Address>(pc_);
}

Address RelocInfo::target_internal_reference() {
  DCHECK(rmode_ == INTERNAL_REFERENCE);
  return ReadUnalignedValue<Address>(pc_);
}

Address RelocInfo::target_internal_reference_address() {
  DCHECK(rmode_ == INTERNAL_REFERENCE);
  return pc_;
}
Address RelocInfo::target_off_heap_target() {
  DCHECK(IsOffHeapTarget(rmode_));
  return Assembler::target_address_at(pc_, constant_pool_);
}

HeapObject RelocInfo::target_object_no_host(Isolate* isolate) {
  DCHECK(IsCodeTarget(rmode_) || IsEmbeddedObjectMode(rmode_));
  if (IsCompressedEmbeddedObject(rmode_)) {
    Tagged_t compressed = ReadUnalignedValue<Tagged_t>(pc_);
    DCHECK(!HAS_SMI_TAG(compressed));
    Object obj(DecompressTaggedPointer(isolate, compressed));
    return HeapObject::cast(obj);
  }
  return HeapObject::cast(Object(ReadUnalignedValue<Address>(pc_)));
}

Handle<HeapObject> RelocInfo::target_object_handle(Assembler* origin) {
  DCHECK(IsCodeTarget(rmode_) || IsEmbeddedObjectMode(rmode_));
  if (IsCodeTarget(rmode_)) {
    return origin->code_target_object_handle_at(pc_);
  } else {
    if (IsCompressedEmbeddedObject(rmode_)) {
      return origin->compressed_embedded_object_handle_at(pc_);
    }
    return Handle<HeapObject>::cast(ReadUnalignedValue<Handle<Object>>(pc_));
  }
}

Address RelocInfo::target_runtime_entry(Assembler* origin) {
  DCHECK(IsRuntimeEntry(rmode_));
  return origin->runtime_entry_at(pc_);
}

Address RelocInfo::constant_pool_entry_address() { UNREACHABLE(); }
void RelocInfo::set_target_object(Heap* heap, HeapObject target,
                                  WriteBarrierMode write_barrier_mode,
                                  ICacheFlushMode icache_flush_mode) {}

void RelocInfo::set_target_runtime_entry(Address target,
                                         WriteBarrierMode write_barrier_mode,
                                         ICacheFlushMode icache_flush_mode) {
  DCHECK(IsRuntimeEntry(rmode_));
  if (target_address() != target) {
    set_target_address(target, write_barrier_mode, icache_flush_mode);
  }
}

int RelocInfo::target_address_size() { return Assembler::kSpecialTargetSize; }

void RelocInfo::WipeOut() {}

void RelocInfo::set_target_external_reference(
    Address target, ICacheFlushMode icache_flush_mode) {}
Address RelocInfo::target_address_address() {
#if 0
  DCHECK(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_) || IsWasmCall(rmode_) ||
         IsWasmStubCall(rmode_) || IsFullEmbeddedObject(rmode_) ||
         IsCompressedEmbeddedObject(rmode_) || IsExternalReference(rmode_) ||
         IsOffHeapTarget(rmode_));
#endif
  return pc_;
}

void Assembler::deserialization_set_target_internal_reference_at(
    Address pc, Address target, RelocInfo::Mode mode) {
  //  WriteUnalignedValue(pc, target);
}

void Assembler::deserialization_set_special_target_at(
    Address instruction_payload, Code code, Address target) {
  set_target_address_at(instruction_payload,
                        !code.is_null() ? code.constant_pool() : kNullAddress,
                        target);
}

Handle<Code> Assembler::code_target_object_handle_at(Address pc) {
  return GetCodeTarget(ReadUnalignedValue<int32_t>(pc));
}
Handle<HeapObject> Assembler::compressed_embedded_object_handle_at(Address pc) {
  return GetEmbeddedObject(ReadUnalignedValue<uint32_t>(pc));
}

Address Assembler::runtime_entry_at(Address pc) {
  return ReadUnalignedValue<int32_t>(pc) + options().code_range_start;
}

}  // namespace internal
}  // namespace v8

#endif  // V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_INL_H_
