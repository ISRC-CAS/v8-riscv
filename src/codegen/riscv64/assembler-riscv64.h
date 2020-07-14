// Copyright 2013 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_H_
#define V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_H_

#include <deque>
#include <list>
#include <map>
#include <memory>
#include <vector>

#include "src/base/optional.h"
#include "src/codegen/assembler.h"
#include "src/codegen/constant-pool.h"
#include "src/codegen/riscv64/constants-riscv64.h"
#include "src/codegen/riscv64/register-riscv64.h"
#include "src/common/globals.h"
#include "src/utils/utils.h"

namespace v8 {
namespace internal {

enum RoundingMode {
  // Rounding Mode Mnemonic Meaning
  // 000 RNE Round to Nearest, ties to Even
  // 001 RTZ Round towards Zero
  // 010 RDN Round Down (towards −∞)
  // 011 RUP Round Up (towards +∞)
  // 100 RMM Round to Nearest, ties to Max Magnitude
  // 101 Invalid. Reserved for future use.
  // 110 Invalid. Reserved for future use.
  // 111 DYN In instruction’s rm field, selects dynamic rounding mode;
  // In Rounding Mode register, Invalid.
  kRNE = 0x0,
  kRTZ = 0x1,
  kRDN = 0x2,
  kRUP = 0x3,
  kRMM = 0x4,
  kDYN = 0x7
};

class SafepointTableBuilder;

// -----------------------------------------------------------------------------
// Class Operand represents a shifter operand in data processing instructions.
class Operand {
 public:
  // Immediate.
  V8_INLINE explicit Operand(int64_t immediate,
                             RelocInfo::Mode rmode = RelocInfo::NONE)
      : rm_(no_reg), rmode_(rmode) {
    value_.immediate = immediate;
  }
  V8_INLINE explicit Operand(const ExternalReference& f)
      : rm_(no_reg), rmode_(RelocInfo::EXTERNAL_REFERENCE) {
    value_.immediate = static_cast<int64_t>(f.address());
  }
  V8_INLINE explicit Operand(const char* s);
  explicit Operand(Handle<HeapObject> handle);
  V8_INLINE explicit Operand(Smi value) : rm_(no_reg), rmode_(RelocInfo::NONE) {
    value_.immediate = static_cast<intptr_t>(value.ptr());
  }

  static Operand EmbeddedNumber(double number);  // Smi or HeapNumber.
  static Operand EmbeddedStringConstant(const StringConstantBase* str);

  // Register.
  V8_INLINE explicit Operand(Register rm) : rm_(rm) {}

  // Return true if this is a register operand.
  V8_INLINE bool is_reg() const;

  inline int64_t immediate() const;

  bool IsImmediate() const { return !rm_.is_valid(); }

  HeapObjectRequest heap_object_request() const {
    DCHECK(IsHeapObjectRequest());
    return value_.heap_object_request;
  }

  bool IsHeapObjectRequest() const {
    DCHECK_IMPLIES(is_heap_object_request_, IsImmediate());
    DCHECK_IMPLIES(is_heap_object_request_,
                   rmode_ == RelocInfo::FULL_EMBEDDED_OBJECT ||
                       rmode_ == RelocInfo::CODE_TARGET);
    return is_heap_object_request_;
  }

  Register rm() const { return rm_; }

  RelocInfo::Mode rmode() const { return rmode_; }

 private:
  Register rm_;
  union Value {
    Value() {}
    HeapObjectRequest heap_object_request;  // if is_heap_object_request_
    int64_t immediate;                      // otherwise
  } value_;                                 // valid if rm_ == no_reg
  bool is_heap_object_request_ = false;
  RelocInfo::Mode rmode_;

  friend class Assembler;
  friend class MacroAssembler;
};

// On RISCV we have only one addressing mode MRI.
// Class MemOperand represents a memory operand in load and store instructions.
class V8_EXPORT_PRIVATE MemOperand : public Operand {
 public:
  // Immediate value attached to offset.
  enum OffsetAddend { offset_minus_one = -1, offset_zero = 0 };

  explicit MemOperand(Register rn, int32_t offset = 0);
  explicit MemOperand(Register rn, int32_t unit, int32_t multiplier,
                      OffsetAddend offset_addend = offset_zero);
  int32_t offset() const { return offset_; }

  bool OffsetIsInt16Encodable() const { return is_int16(offset_); }

 private:
  int32_t offset_;

  friend class Assembler;
};

#if 0
enum Condition {
  // any value < 0 is considered no_condition
  no_condition = -1,

  overflow = 0,
  no_overflow = 1,
  below = 2,
  above_equal = 3,
  equal = 4,
  not_equal = 5,
  below_equal = 6,
  above = 7,
  negative = 8,
  positive = 9,
  parity_even = 10,
  parity_odd = 11,
  less = 12,
  greater_equal = 13,
  less_equal = 14,
  greater = 15,

  // Fake conditions that are handled by the
  // opcodes using them.
  always = 16,
  never = 17,
  // aliases
  carry = below,
  not_carry = above_equal,
  zero = equal,
  not_zero = not_equal,
  sign = negative,
  not_sign = positive,
  last_condition = greater
};
#endif

// qiuji
#if 0
// Returns the equivalent of !cc.
// Negation of the default no_condition (-1) results in a non-default
// no_condition value (-2). As long as tests for no_condition check
// for condition < 0, this will work as expected.
inline Condition NegateCondition(Condition cc) {
  return static_cast<Condition>(cc ^ 1);
}
#endif
// -----------------------------------------------------------------------------
// Assembler.

class V8_EXPORT_PRIVATE Assembler : public AssemblerBase {
 public:
  // Create an assembler. Instructions and relocation information are emitted
  // into a buffer, with the instructions starting from the beginning and the
  // relocation information starting from the end of the buffer. See CodeDesc
  // for a detailed comment on the layout (globals.h).
  //
  // If the provided buffer is nullptr, the assembler allocates and grows its
  // own buffer. Otherwise it takes ownership of the provided buffer.
  explicit Assembler(const AssemblerOptions&,
                     std::unique_ptr<AssemblerBuffer> = {});

  ~Assembler() override = default;

  static constexpr int kSpecialTargetSize = 0;
  static constexpr SafepointTableBuilder* kNoSafepointTable = nullptr;
  static constexpr int kNoHandlerTable = 0;
  // Relocation information generation.
  // Each relocation is encoded as a variable size value.
  static constexpr int kMaxRelocSize = RelocInfoWriter::kMaxSize;
  RelocInfoWriter reloc_info_writer;

  void AbortedCodeGeneration() override {}
  void GetCode(Isolate* isolate, CodeDesc* desc,
               SafepointTableBuilder* safepoint_table_builder,
               int handler_table_offset);
  void GetCode(Isolate* isolate, CodeDesc* desc) {
    GetCode(isolate, desc, kNoSafepointTable, kNoHandlerTable);
  }
  static Address target_address_at(Address pc);
  static inline Address target_address_at(Address pc, Address constant_pool);
  static void set_target_value_at(
      Address pc, uint64_t target,
      ICacheFlushMode icache_flush_mode = FLUSH_ICACHE_IF_NEEDED);

  static constexpr int kTrampolineSlotsSize = 7 * kInstrSize;

  // Max offset for instructions with 12-bit offset field
static constexpr int kMaxBranchOffset = (1 << (13 - 1)) - 1;
#define ALL_OFFSET_SIZE(V) \
  V(26)                    \
  V(20)                    \
  V(13)                    \
  V(12)

  enum OffsetSize : int {
#define DECLARE_OFFSERSIZE(V) kOffset##V = V,
    ALL_OFFSET_SIZE(DECLARE_OFFSERSIZE) kLast = -1
#undef DECLARE_OFFSERSIZE
  };
  bool is_range(int offset, OffsetSize offsetsize, int bias) {
    switch (offsetsize) {
#define CASE(V)                                          \
  case kOffset##V:                                       \
    offset = offset > 0 ? offset + bias : offset - bias; \
    \                      
    return is_int##V(offset);                            \
    break;
      ALL_OFFSET_SIZE(CASE)
#undef CASE
      default:
        UNREACHABLE();
    }
  }
#undef ALL_OFFSET_SIZE

  RegList* GetScratchRegisterList() { return &scratch_register_list_; }

  // Record a deoptimization reason that can be used by a log or cpu profiler.
  // Use --trace-deopt to enable.
  void RecordDeoptReason(DeoptimizeReason reason, SourcePosition position,
                         int id);
  void MaybeEmitOutOfLineConstantPool() {}
  void CodeTargetAlign();
  void bind(Label* L);  // binds an unbound label L to the current code position
  void nop();
  void Align(int m);
  void DataAlign(int m);
  void CheckBuffer();
  void RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data = 0);
  int WriteCodeComments();
  // Emit 8 bits of data in the instruction stream.
  void dc8(uint8_t data) { EmitData(&data, sizeof(data)); }

  // Emit 32 bits of data in the instruction stream.
  void dc32(uint32_t data) { EmitData(&data, sizeof(data)); }

  // Emit 64 bits of data in the instruction stream.
  void dc64(uint64_t data) { EmitData(&data, sizeof(data)); }

  void db(uint8_t data);
  void dd(uint32_t data);
  void dq(uint64_t data);
  void dp(uintptr_t data) { dq(data); }
  void dd(Label* label);

  // Determines if Label is bound and near enough so that branch instruction
  // can be used to reach it, instead of jump instruction.

  bool is_near(Label* L);
  bool is_near(Label* L, OffsetSize bits);
  bool is_near_branch(Label* L);
  inline bool is_near_b(Label* L) {
    return is_range(pc_offset() - L->pos(), kOffset13, 4 * kInstrSize);
  }
  uint64_t branch_long_offset(Label* L);

  int BranchOffset(Instr instr);

  // Returns the branch offset to the given label from the current code
  // position. Links the label to the current position if it is still unbound.
  // Manages the jump elimination optimization if the second parameter is true.
  int32_t branch_offset_helper(Label* L, OffsetSize bits);
  // Difference between address of current opcode and target address offset.
  static constexpr int kBranchPCOffset = kInstrSize;
  // Difference between address of current opcode and target address offset,
  // when we are generatinga sequence of instructions for long relative PC
  // branches
  static constexpr int kLongBranchPCOffset = 2 * kInstrSize;
  // Determines the end of the Jump chain (a subset of the label link chain).
  const int kEndOfJumpChain = 0;
  inline int32_t branch_offset(Label* L) {
    return branch_offset_helper(L, OffsetSize::kOffset12);
  }
  inline int32_t shifted_branch_offset(Label* L) { return branch_offset(L); }
  static inline void set_target_address_at(
      Address pc, Address constant_pool, Address target,
      ICacheFlushMode icache_flush_mode = FLUSH_ICACHE_IF_NEEDED) {}
  inline Handle<Code> code_target_object_handle_at(Address pc);
  inline Handle<HeapObject> compressed_embedded_object_handle_at(Address pc);
  inline Address runtime_entry_at(Address pc);
  inline static void deserialization_set_special_target_at(
      Address instruction_payload, Code code, Address target);
  // This sets the internal reference at the pc.
  inline static void deserialization_set_target_internal_reference_at(
      Address pc, Address target,
      RelocInfo::Mode mode = RelocInfo::INTERNAL_REFERENCE);
  // plct add Emit function
  // Emit the instruction at pc_.
  void Emit(uint32_t instruction) {
    STATIC_ASSERT(sizeof(*pc_) == 1);
    STATIC_ASSERT(sizeof(instruction) == kInstrSize);
    DCHECK_LE(pc_ + sizeof(instruction), buffer_start_ + buffer_->size());
    memcpy(pc_, &instruction, sizeof(instruction));
    pc_ += sizeof(instruction);
    CheckBuffer();
  }
  // Emit data inline in the instruction stream.
  void EmitData(void const* data, unsigned size) {
    DCHECK_EQ(sizeof(*pc_), 1);
    DCHECK_LE(pc_ + size, buffer_start_ + buffer_->size());

    // TODO(all): Somehow register we have some data here. Then we can
    // disassemble it correctly.
    memcpy(pc_, data, size);
    pc_ += size;
    CheckBuffer();
  }

  // RV32I Base Instruction Set
  void lui(Register rd, int32_t j);
  void auipc(Register rd, int32_t j);
  void jal(Register rd, int32_t j);
  void jalr(Register rd, int32_t offset, Register rs1);
  void beq(Register rs1, Register rs2, int16_t j);
  void bne(Register rs1, Register rs2, int16_t j);
  void blt(Register rs1, Register rs2, int16_t j);
  void bge(Register rs1, Register rs2, int16_t j);
  void bltu(Register rs1, Register rs2, int16_t j);
  void bgeu(Register rs1, Register rs2, int16_t j);
  void lb(Register rd, const MemOperand& rs1);
  void lh(Register rd, const MemOperand& rs1);
  void lw(Register rd, const MemOperand& rs1);
  void lbu(Register rd, const MemOperand& rs1);
  void lhu(Register rd, const MemOperand& rs1);
  void sb(Register rs2, const MemOperand& rs1);
  void sh(Register rs2, const MemOperand& rs1);
  void sw(Register rs2, const MemOperand& rs1);
  void addi(Register rd, Register rs1, int32_t j);
  void slti(Register rd, Register rs1, int32_t j);
  void sltiu(Register rd, Register rs1, int32_t j);
  void xori(Register rd, Register rs1, int32_t j);
  void ori(Register rd, Register rs1, int32_t j);
  void andi(Register rd, Register rs1, int32_t j);
  void slli(Register rd, Register rs1, uint32_t shamt);
  void srli(Register rd, Register rs1, uint32_t shamt);
  void srai(Register rd, Register rs1, uint32_t shamt);
  void add(Register rd, Register rs1, Register rs2);
  void sub(Register rd, Register rs1, Register rs2);
  void sll(Register rd, Register rs1, Register rs2);
  void slt(Register rd, Register rs1, Register rs2);
  void sltu(Register rd, Register rs1, Register rs2);
  void xor_(Register rd, Register rs1, Register rs2);
  void srl(Register rd, Register rs1, Register rs2);
  void sra(Register rd, Register rs1, Register rs2);
  void or_(Register rd, Register rs1, Register rs2);
  void and_(Register rd, Register rs1, Register rs2);
  void fence(Register rd, Register rs1, uint32_t fm, uint32_t pred,
             uint32_t succ);
  void ecall();
  void ebreak();
  // RV64I Base Instruction Set (in addition to RV32I)
  void lwu(Register rd, const MemOperand& rs1);
  void ld(Register rd, const MemOperand& rs1);
  void sd(Register rs2, const MemOperand& rs1);
  void slli64(Register rd, Register rs1, uint32_t shamt);
  void srli64(Register rd, Register rs1, uint32_t shamt);
  void srai64(Register rd, Register rs1, uint32_t shamt);
  void addiw(Register rd, Register rs1, int32_t imm);
  void slliw(Register rd, Register rs1, int32_t shamt);
  void srliw(Register rd, Register rs1, uint32_t shamt);
  void sraiw(Register rd, Register rs1, uint32_t shamt);
  void addw(Register rd, Register rs1, Register rs2);
  void subw(Register rd, Register rs1, Register rs2);
  void sllw(Register rd, Register rs1, Register rs2);
  void srlw(Register rd, Register rs1, Register rs2);
  void sraw(Register rd, Register rs1, Register rs2);
  // RV32/RV64 Zifencei Standard Extension
  void fencei(Register rd, Register rs1, int32_t imm);
  // RV32/RV64 Zicsr Standard Extension
  void csrrw(Register rd, CSRRegister csr, Register rs1);
  void csrrs(Register rd, CSRRegister csr, Register rs1);
  void csrrc(Register rd, CSRRegister csr, Register rs1);
  void csrrwi(Register rd, CSRRegister csr, uint32_t uimm);
  void csrrsi(Register rd, CSRRegister csr, uint32_t uimm);
  void csrrci(Register rd, CSRRegister csr, uint32_t uimm);
  // RV32M Standard Extension
  void mul(Register rd, Register rs1, Register rs2);
  void mulh(Register rd, Register rs1, Register rs2);
  void mulhsu(Register rd, Register rs1, Register rs2);
  void mulhu(Register rd, Register rs1, Register rs2);
  void div(Register rd, Register rs1, Register rs2);
  void divu(Register rd, Register rs1, Register rs2);
  void rem(Register rd, Register rs1, Register rs2);
  void remu(Register rd, Register rs1, Register rs2);
  // RV64M Standard Extension (in addition to RV32M)
  void mulw(Register rd, Register rs1, Register rs2);
  void divw(Register rd, Register rs1, Register rs2);
  void divuw(Register rd, Register rs1, Register rs2);
  void remw(Register rd, Register rs1, Register rs2);
  void remuw(Register rd, Register rs1, Register rs2);
  // RV32A Standard Extension
  void lrw(Register rd, Register rs1, uint32_t aq, uint32_t rl);
  void scw(Register rd, Register rs1, Register rs2, uint32_t aq, uint32_t rl);
  void amoswapw(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);
  void amoaddw(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoxorw(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoandw(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoorw(Register rd, Register rs1, Register rs2, uint32_t aq,
              uint32_t rl);
  void amominw(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amomaxw(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amominuw(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);
  void amomaxuw(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);
  // RV64A Standard Extension (in addition to RV32A)
  void lrd(Register rd, Register rs1, uint32_t aq, uint32_t rl);
  void scd(Register rd, Register rs1, Register rs2, uint32_t aq, uint32_t rl);
  void amoswapd(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);
  void amoaddd(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoxord(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoandd(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amoord(Register rd, Register rs1, Register rs2, uint32_t aq,
              uint32_t rl);
  void amomind(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amomaxd(Register rd, Register rs1, Register rs2, uint32_t aq,
               uint32_t rl);
  void amominud(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);
  void amomaxud(Register rd, Register rs1, Register rs2, uint32_t aq,
                uint32_t rl);

  // Read/patch instructions.
  static Instr instr_at(Address pc) { return *reinterpret_cast<Instr*>(pc); }
  static void instr_at_put(Address pc, Instr instr) {
    *reinterpret_cast<Instr*>(pc) = instr;
  }
  Instr instr_at(int pos) {
    return *reinterpret_cast<Instr*>(buffer_start_ + pos);
  }

  void instr_at_put(int pos, Instr instr) {
    *reinterpret_cast<Instr*>(buffer_start_ + pos) = instr;
  }

  static bool IsJal(Instr instr);
  static bool IsJalr(Instr instr);
  static bool IsLui(Instr instr);
  static bool IsOri(Instr instr);
  static bool IsMv(Instr instr, Register rd, Register rs1);
  static bool IsEmittedConstant(Instr instr);

  static uint32_t GetRd(Instr instr);
  static uint32_t GetRs1(Instr instr);
  static uint32_t GetItype_imm(Instr instr);
  static uint32_t GetLabelConst(Instr instr);
  // Instruction generation.

  // R-type rs2 field, i.e., bit[24:20] is a 5-bit register index
  // ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR_/AND_
  // ADDW/SUBW/SLLW/SRLW/SRAW
  // MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU
  // MULW/DIVW/DIVUW/REMW/REMUW
  // GIR1(GenInstr R-type 1)
  void GenInstrRType(Funct7 func7, Register rs2, Register rs1, Funct3 func3,
                     Register rd, Opcode opcode);

  // I-type rs2 filed ,
  // i.e. bit[24:20] is a imm, can be a 5 bit shamt or fixed value
  // SLLI/SRLI/SRAI
  // SLLIW/SRLIW/SRAIW
  // GII0(GenInstr I-type 0)
  // imm is a 5 bit shamt or other
  void GenInstrIType(Funct7 func7, int32_t imm, Register rs1, Funct3 func3,
                     Register rd, Opcode opcode);

  // RV64I SHIFT IMM has 6 bit shamt
  // I-type. bit[25:20] is a 6 bit shamt
  // SLLI64/SRLI64/SRAI64
  // GISI64(GenInstr Shift I type 64)
  void GenInstrShiftI64(Funct6 func6, int32_t imm, Register rs1, Funct3 func3,
                        Register rd, Opcode opcode);

  // R-type atomic
  // LR.W/SC.W/AMOSWAP.W/AMOADD.W/AMOXOR.W/AMOAND.W/AMOOR.W/AMOMIN.W/AMOMAX.W/AMOMINU.W/AMOMAXU.W
  // LR.D/SC.D/AMOSWAP.D/AMOADD.D/AMOXOR.D/AMOAND.D/AMOOR.D/AMOMIN.D/AMOMAX.D/AMOMINU.D/AMOMAXU.D
  // GIRA(GenInstr R-type Atomic_Instructions)
  void GenInstrRA(Funct5 R_A, uint32_t aq, uint32_t rl, Register rs2,
                  Register rs1, Funct3 func3, Register rd, Opcode opcode);

  // I-type, bit[31:20] is a 12-bit imm
  // JALR
  // LB/LH/LW/LBU/LHU/ADDI/SLTI/SLTIU/XORI/ORI/ANDI/ECALL/EBREAK
  // LWU/LD/ADDIW
  // FENCEi
  // GII1(GenInstr I-type 1)
  void GenInstrIType(int32_t imm, Register rs1, Funct3 func3, Register rd,
                     Opcode opcode);

  // I-type fence
  // FENCE
  // GIIF(GenInstr I-type Fence_Instructions)
  void GenInstrIFence(uint32_t fm, uint32_t pred, uint32_t succ, Register rs1,
                      Funct3 func3, Register rd, Opcode opcode);

  // I-type Zicsr
  // csrrw/csrrs/csrrc
  // GII3(GenInstr I-type Zicsr_Instructions)
  void GenInstrIType(CSRRegister csr, Register rs1, Funct3 func3, Register rd,
                     Opcode opcode);
  // I-type Zicsr bit[19:15] is a 5-bit zimm
  // csrrwi/csrrsi/csrrci
  // GII2(GenInstr I-type 2)
  void GenInstrIType(CSRRegister csr, uint32_t uimm, Funct3 func3, Register rd,
                     Opcode opcode);

  // S-type, Store
  // SB/SH/SW
  // SD
  // GIS1(GenInstr S-type 1)
  void GenInstrSType(int32_t offset, Register rs2, Register rs1, Funct3 func3,
                     Opcode opcode);

  // B-type, Branch
  // BEQ/BNE/BLT/BGE/BLTU/BGEU
  // GIB(GenInstr B-type)
  void GenInstrBType(int16_t offset, Register rs2, Register rs1, Funct3 func3,
                     Opcode opcode);

  // U-type, bit[31:12] is a 20-bit imm
  // LUI/AUIPC
  // GIU(GenInstr U-type)
  void GenInstrUType(int32_t offset, Register rd, Opcode opcode);

  // J-type JAL Deprecated because PIC jump
  // offset is a 20-bit address which has been scaled by 2,
  // so the actually layout in jal is:
  // [19][9:0][10][18:11]
  // JAL
  // GIJ(GenInstr J-type)
  void GenInstrJType(int32_t offset, Register rd, Opcode opcode);

  //  floating-point computational instructions
  // R-type bit[24:20] is rs2 & bit[14:12] is Funct3
  // FSGNJS/FSGNJNS/FSGNJXS/FMINS/FMAXS/FMVXW/FCLASSS/FMVWX
  // FSGNJD/FSGNJND/FSGNJXD/FMIND/FMAXD/FCLASSD/FMVXD/FMVDX
  // GIFPR1(GenInstr FloatPoint R type 1)
  void GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                      Funct3 func3, FPURegister rd, Opcode opcode);
  // R-type bit[24:20] is rs2 & bit[14:12] is Funct3
  // feqs/flts/fles/feqd/fltd/fled
  // GIFPR1_1(GenInstr FloatPoint R type 1)
  // rd is Register
  // fled/fltd/feqd
  void GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                      Funct3 func3, Register rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is Funct3
  // GIFPR2(GenInstr FloatPoint R type 2)
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                      Funct3 func3, Register rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is Funct3
  // rd is Register
  // GIFPR2_1(GenInstr FloatPoint R type 2)
  // fmvxw/fclasss/fclassd/fmvxd
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1,
                      Funct3 func3, FPURegister rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is Funct3
  // rs1 is Register
  // GIFPR2_2(GenInstr FloatPoint R type 2)
  // fmvwx/fmvdx
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, Register rs1, Funct3 func3,
                      FPURegister rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is rm
  // fsqrts/fcvtsl/fsqrtd/fcvtsd/fcvtds
  // GIFPR3(GenInstr FloatPoint R type 3)
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1, uint32_t rm,
                      FPURegister rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is rm
  // rs1 is Register
  // fcvtsw/fcvtswu/fcvtslu/fcvtdw/fcvtdwu/fcvtdl/fcvtdlu
  // GIFPR3_1(GenInstr FloatPoint R type 3)
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, Register rs1, uint32_t rm,
                      FPURegister rd, Opcode opcode);
  // R-type bit[24:20] is imm & bit[14:12] is rm
  // rd is Register
  // fcvtws/fcvtwus/fcvtls/fcvtlus/fcvtwd/fcvtwud/fcvtld/fcvtlud
  // GIFPR3_2(GenInstr FloatPoint R type 3)
  void GenInstrFRType(Funct7 func7, uint32_t rs2i, FPURegister rs1, uint32_t rm,
                      Register rd, Opcode opcode);
  // R-type bit[24:20] is rs2 & bit[14:12] is rm
  // FADDS/FSUBS/FMULS/FDIVS/FSQRTS/FCVTWS/FCVTSW
  // FADDD/FSUBD/FMULD/FDIVD/FSQRTD/FCVTDS/FCVTWD/FCVTDW
  // GIFPR4(GenInstr FloatPoint R type 4)
  void GenInstrFRType(Funct7 func7, FPURegister rs2, FPURegister rs1,
                      uint32_t rm, FPURegister rd, Opcode opcode);
  // I-type
  // FLW
  // FLD
  // GII4(GenInstr I-type 4)
  void GenInstrIType(int32_t imm, Register rs1, Funct3 func3, FPURegister rd,
                     Opcode opcode);
  // S-type
  // FSW
  // FSD
  // GIS2(GenInstr S-type 2)
  void GenInstrSType(FPURegister rs2, Register rs1, Funct3 func3, int32_t imm,
                     Opcode opcode);
  // R4-type bit[14:12] is funct3
  // GIFPR41(GenInstr FloatPoint R4 type 1)
  void GenInstrFR4Type(FPURegister rs3, Funct2 func2, FPURegister rs2,
                       FPURegister rs1, Funct3 func3, FPURegister rd,
                       Opcode opcode);
  // R4-type bit[14:12] is rm
  // FMADDS/FMSUBS/FNMSUBS/FNMADDS
  // FMADDD/FMSUBD/FNMSUBD/FNMADDD
  // GIFPR42(GenInstr FloatPoint R4 type 2)
  void GenInstrFR4Type(FPURegister rs3, Funct2 func2, FPURegister rs2,
                       FPURegister rs1, uint32_t rm, FPURegister rd,
                       Opcode opcode);
  // RV32F Standard Extension
  void flw(FPURegister rd, const MemOperand& rs1);
  void fsw(FPURegister rs2, const MemOperand& rs1);
  void fmadds(FPURegister rd, FPURegister rs1, FPURegister rs2, FPURegister rs3,
              uint32_t rm);
  void fmsubs(FPURegister rd, FPURegister rs1, FPURegister rs2, FPURegister rs3,
              uint32_t rm);
  void fnmsubs(FPURegister rd, FPURegister rs1, FPURegister rs2,
               FPURegister rs3, uint32_t rm);
  void fnmadds(FPURegister rd, FPURegister rs1, FPURegister rs2,
               FPURegister rs3, uint32_t rm);
  void fadds(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fsubs(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fmuls(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fdivs(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fsqrts(FPURegister rd, FPURegister rs1, uint32_t rm);
  void fsgnjs(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fsgnjns(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fsgnjxs(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fmins(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fmaxs(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fcvtws(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtwus(Register rd, FPURegister rs1, uint32_t rm);
  void fmvxw(Register rd, FPURegister rs1);
  void feqs(Register rd, FPURegister rs1, FPURegister rs2);
  void flts(Register rd, FPURegister rs1, FPURegister rs2);
  void fles(Register rd, FPURegister rs1, FPURegister rs2);
  void fclasss(Register rd, FPURegister rs1);
  void fcvtsw(FPURegister rd, Register rs1, uint32_t rm);
  void fcvtswu(FPURegister rd, Register rs1, uint32_t rm);
  void fmvwx(FPURegister rd, Register rs1);
  // RV64F Standard Extension (in addition to RV32F)
  void fcvtls(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtlus(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtsl(FPURegister rd, Register rs1, uint32_t rm);
  void fcvtslu(FPURegister rd, Register rs1, uint32_t rm);
  // RV32D Standard Extension
  void fld(FPURegister rd, const MemOperand& rs1);
  void fsd(FPURegister rs2, const MemOperand& rs1);
  void fmaddd(FPURegister rd, FPURegister rs1, FPURegister rs2, FPURegister rs3,
              uint32_t rm);
  void fmsubd(FPURegister rd, FPURegister rs1, FPURegister rs2, FPURegister rs3,
              uint32_t rm);
  void fnmsubd(FPURegister rd, FPURegister rs1, FPURegister rs2,
               FPURegister rs3, uint32_t rm);
  void fnmaddd(FPURegister rd, FPURegister rs1, FPURegister rs2,
               FPURegister rs3, uint32_t rm);
  void faddd(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fsubd(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fmuld(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fdivd(FPURegister rd, FPURegister rs1, FPURegister rs2, uint32_t rm);
  void fsqrtd(FPURegister rd, FPURegister rs1, uint32_t rm);
  void fsgnjd(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fsgnjnd(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fsgnjxd(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fmind(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fmaxD(FPURegister rd, FPURegister rs1, FPURegister rs2);
  void fcvtsd(FPURegister rd, FPURegister rs1, uint32_t rm);
  void fcvtds(FPURegister rd, FPURegister rs1, uint32_t rm);
  void feqd(Register rd, FPURegister rs1, FPURegister rs2);
  void fltd(Register rd, FPURegister rs1, FPURegister rs2);
  void fled(Register rd, FPURegister rs1, FPURegister rs2);
  void fclassd(Register rd, FPURegister rs1);
  void fcvtwd(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtwud(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtdw(FPURegister rd, Register rs1, uint32_t rm);
  void fcvtdwu(FPURegister rd, Register rs1, uint32_t rm);
  // RV64D Standard Extension (in addition to RV32D)
  void fcvtld(Register rd, FPURegister rs1, uint32_t rm);
  void fcvtlud(Register rd, FPURegister rs1, uint32_t rm);
  void fmvxd(Register rd, FPURegister rs1);
  void fcvtdl(FPURegister rd, Register rs1, uint32_t rm);
  void fcvtdlu(FPURegister rd, Register rs1, uint32_t rm);
  void fmvdx(FPURegister rd, Register rs1);
// RV32Q Standard Extension
#if 0
  void flq(FPURegister rs1, FPURegister rd, uint32_t imm);
  void fsq(FPURegister rs2, FPURegister rs1, uint32_t imm);
  void fmaddq(FPURegister rs3, FPURegister rs2, FPURegister rs1,
               FPURegister rd, uint32_t rm);
  void fmsubq(FPURegister rs3, FPURegister rs2, FPURegister rs1,
                FPURegister rd, uint32_t rm);
  void fnmsubq(FPURegister rs3, FPURegister rs2, FPURegister rs1,
                FPURegister rd, uint32_t rm);
  void fnmaddq(FPURegister rs3, FPURegister rs2, FPURegister rs1,
                FPURegister rd, uint32_t rm);
  void faddq(FPURegister rs2, FPURegister rs1, FPURegister rd, uint32_t rm);
  void fsubq(FPURegister rs2, FPURegister rs1, FPURegister rd, uint32_t rm);
  void fmulq(FPURegister rs2, FPURegister rs1, FPURegister rd, uint32_t rm);
  void fdivq(FPURegister rs2, FPURegister rs1, FPURegister rd, uint32_t rm);
  void fsqrtq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fsgnjq(FPURegister rs2, FPURegister rs1);
  void fsgnjnq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fsgnjxq(FPURegister rs2, FPURegister rs1, FPURegister rd, uint32_t rm);
  void fminq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fmaxq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fcvtsq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtqs(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtdq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtqd(FPURegister rs1, FPURegister rd, uint32_t rm);
  void feqq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fltq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fleq(FPURegister rs2, FPURegister rs1, FPURegister rd);
  void fclassq(FPURegister rs1, FPURegister rd);
  void fcvtwq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtwuq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtqw(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtqwu(FPURegister rs1, FPURegister rd, uint32_t rm);
  // RV64Q Standard Extension (in addition to RV32Q)
  void fcvtlq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtluq(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtql(FPURegister rs1, FPURegister rd, uint32_t rm);
  void fcvtqlu(FPURegister rs1, FPURegister rd, uint32_t rm);
#endif

  void b(int16_t offset);
  inline void b(Label* L) { b(shifted_branch_offset(L)); }
  // Pesudo instruction

  void mv(Register rd, Register rs);
  void not_(Register rd, Register rs);
  void neg(Register rd, Register rs);
  void negw(Register rd, Register rs);
  void sextw(Register rd, Register rs);
  void seqz(Register rd, Register rs);
  void snez(Register rd, Register rs);
  void sltz(Register rd, Register rs);
  void sgtz(Register rd, Register rs);

  void fmvs(FPURegister rd, FPURegister rs);
  void fabss(FPURegister rd, FPURegister rs);
  void fnegs(FPURegister rd, FPURegister rs);
  void fmvd(FPURegister rd, FPURegister rs);
  void fabsd(FPURegister rd, FPURegister rs);
  void fnegd(FPURegister rd, FPURegister rs);

  void beqz(Register rs, int16_t offset);
  void bnez(Register rs, int16_t offset);
  void blez(Register rs, int16_t offset);
  void bgez(Register rs, int16_t offset);
  void bltz(Register rs, int16_t offset);
  void bgtz(Register rs, int16_t offset);

  void bgt(Register rs, Register rt, int16_t offset);
  void ble(Register rs, Register rt, int16_t offset);
  void bgtu(Register rs, Register rt, int16_t offset);
  void bleu(Register rs, Register rt, int16_t offset);

  void jr(Register rs);
  void jalr(Register rs);
  void ret(Register rs);

  void rdinstreth(Register rd);
  void rdcycleh(Register rd);
  void rdtimeh(Register rd);

  void csrr(Register rd, CSRRegister csr);
  void csrw(CSRRegister csr, Register rs);
  void csrs(CSRRegister csr, Register rs);
  void csrc(CSRRegister csr, Register rs);

  void csrwi(CSRRegister csr, uint32_t uimm);
  void csrsi(CSRRegister csr, uint32_t uimm);
  void csrci(CSRRegister csr, uint32_t uimm);

  void frcsr(Register rd);
  void frcsr(Register rd, Register rs);
  void fscsr(Register rs);

  void frrm(Register rd);
  void fsrm(Register rd, Register rs);
  void fsrm(Register rs);

  void frflags(Register rd);
  void fsflags(Register rd, Register rs);
  void fsflags(Register rs);
  // Load local Address Instruction, Pseudoinstruction
  void lla(Register rd, Label* l);
  void lla(Register rd, int offset);
  void call(int offset);
  // Check the code size generated from label to here.
  int SizeOfCodeGeneratedSince(Label* label) {
    return pc_offset() - label->pos();
  }

  // Check the number of instructions generated from label to here.
  int InstructionsGeneratedSince(Label* label) {
    return SizeOfCodeGeneratedSince(label) / kInstrSize;
  }

  static uint32_t GetOpcodeField(Instr instr);
  static uint32_t GetFunct3Filed(Instr instr);
  // Check if an instruction is a branch of some kind.
  static bool IsBranch(Instr instr);
  static bool IsAUIPC(Instr instr);

  // Decode branch instruction at pos and return branch target pos.
  int target_at(int pos, bool is_internal);

  // Patch branch instruction at pos to branch to given branch target pos.
  void target_at_put(int pos, int target_pos, bool is_internal);
  // Say if we need to relocate with this mode.
  bool MustUseReg(RelocInfo::Mode rmode);
  int64_t buffer_space() const { return reloc_info_writer.pos() - pc_; }
  std::deque<int> internal_reference_positions_;

  void CheckTrampolinePool();
  bool IsPrevInstrCompactBranch() { return prev_instr_compact_branch_; }
  bool prev_instr_compact_branch_ = false;
  // Class for scoping postponing the trampoline pool generation.
  class BlockTrampolinePoolScope {
   public:
    explicit BlockTrampolinePoolScope(Assembler* assem) : assem_(assem) {
      // assem_->StartBlockTrampolinePool();
    }
    ~BlockTrampolinePoolScope() {
      // assem_->EndBlockTrampolinePool();
    }

   private:
    Assembler* assem_;

    DISALLOW_IMPLICIT_CONSTRUCTORS(BlockTrampolinePoolScope);
  };

  void GetBranchlongArg(int32_t offset, int32_t& auipc_arg, int32_t& jalr_arg) {
    jalr_arg = offset << 20;
    jalr_arg = jalr_arg >> 20;
    auipc_arg = (offset - jalr_arg) >> 12;
  }

 protected:
  // Readable constants for base and offset adjustment helper, these indicate if
  // aside from offset, another value like offset + 4 should fit into int12.
  enum class OffsetAccessType : bool {
    SINGLE_ACCESS = false,
    TWO_ACCESSES = true
  };

  // Helper function for memory load/store using base register and offset.
  void AdjustBaseAndOffset(
      MemOperand* src,
      OffsetAccessType access_type = OffsetAccessType::SINGLE_ACCESS,
      int second_access_add_to_offset = 4);

  void EndBlockTrampolinePool() {
    trampoline_pool_blocked_nesting_--;
    if (trampoline_pool_blocked_nesting_ == 0) {
      CheckTrampolinePoolQuick(1);
    }
  }

  void StartBlockTrampolinePool() { trampoline_pool_blocked_nesting_++; }
  void CheckTrampolinePoolQuick(int extra_instructions = 0) {
    if (pc_offset() >= next_buffer_check_ - extra_instructions * kInstrSize) {
      CheckTrampolinePool();
    }
  }

  bool is_trampoline_pool_blocked() const {
    return trampoline_pool_blocked_nesting_ > 0;
  }

  bool is_trampoline_emitted() const { return trampoline_emitted_; }

 private:
  static constexpr int kGap = 64;
  static const int kMaximalBufferSize = 512 * MB;

  // Emission of the trampoline pool may be blocked in some code sequences.
  int trampoline_pool_blocked_nesting_;  // Block emission if this is not zero.
  int no_trampoline_pool_before_;  // Block emission before this pc offset.
  int next_buffer_check_;          // pc offset of next buffer check.

  // Labels.
  void print(const Label* L);
  void bind_to(Label* L, int pos);
  void next(Label* L, bool is_internal);
  int unbound_labels_count_;
  // One trampoline consists of:
  // - space for trampoline slots,
  // - space for labels.
  //
  // Space for trampoline slots is equal to slot_count * 2 * kInstrSize.
  // Space for trampoline slots precedes space for labels. Each label is of one
  // instruction size, so total amount for labels is equal to
  // label_count *  kInstrSize.
  class Trampoline {
   public:
    Trampoline() {
      start_ = 0;
      next_slot_ = 0;
      free_slot_count_ = 0;
      end_ = 0;
    }
    Trampoline(int start, int slot_count) {
      start_ = start;
      next_slot_ = start;
      free_slot_count_ = slot_count;
      end_ = start + slot_count * kTrampolineSlotsSize;
    }
    int start() { return start_; }
    int end() { return end_; }
    int take_slot() {
      int trampoline_slot = kInvalidSlotPos;
      if (free_slot_count_ <= 0) {
        // We have run out of space on trampolines.
        // Make sure we fail in debug mode, so we become aware of each case
        // when this happens.
        DCHECK(0);
        // Internal exception will be caught.
      } else {
        trampoline_slot = next_slot_;
        free_slot_count_--;
        next_slot_ += kTrampolineSlotsSize;
      }
      return trampoline_slot;
    }

   private:
    int start_;
    int end_;
    int next_slot_;
    int free_slot_count_;
  };

  int32_t get_trampoline_entry(int32_t pos);

  // After trampoline is emitted, long branches are used in generated code for
  // the forward branches whose target offsets could be beyond reach of branch
  // instruction. We use this information to trigger different mode of
  // branch instruction generation, where we use jump instructions rather
  // than regular branch instructions.
  bool trampoline_emitted_;
  Trampoline trampoline_;
  bool internal_trampoline_exception_;

  RegList scratch_register_list_;

  static constexpr int kInvalidSlotPos = -1;
  // Internal reference positions, required for unbounded internal reference
  // labels.
  bool is_internal_reference(Label* L) {
    return std::find(internal_reference_positions_.begin(),
                     internal_reference_positions_.end(),
                     L->pos()) != internal_reference_positions_.end();
  }

  // The bound position, before this we cannot do instruction elimination.
  int last_bound_pos_;

  void GrowBuffer();
  void AllocateAndInstallRequestedHeapObjects(Isolate* isolate);
  friend class BlockTrampolinePoolScope;
};
// Helper class that ensures that there is enough space for generating
// instructions and relocation information.  The constructor makes
// sure that there is enough space and (in debug mode) the destructor
// checks that we did not generate too much.
class EnsureSpace {
 public:
  explicit EnsureSpace(Assembler* assembler) : assembler_(assembler) {
    assembler_->CheckBuffer();
  }

 private:
  Assembler* assembler_;
};

class V8_EXPORT_PRIVATE UseScratchRegisterScope {
 public:
  explicit UseScratchRegisterScope(Assembler* assembler);
  ~UseScratchRegisterScope();

  Register Acquire();
  bool hasAvailable() const;

 private:
  RegList* available_;
  RegList old_available_;
};

}  // namespace internal
}  // namespace v8

#endif  // V8_CODEGEN_RISCV64_ASSEMBLER_RISCV64_H_
