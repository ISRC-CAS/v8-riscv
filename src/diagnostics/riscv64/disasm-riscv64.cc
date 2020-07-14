// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// A Disassembler object is used to disassemble a block of code instruction by
// instruction. The default implementation of the NameConverter object can be
// overriden to modify register names or to do symbol lookup on addresses.
//
// The example below will disassemble a block of code and print it to stdout.
//
//   NameConverter converter;
//   Disassembler d(converter);
//   for (byte* pc = begin; pc < end;) {
//     v8::internal::EmbeddedVector<char, 256> buffer;
//     byte* prev_pc = pc;
//     pc += d.InstructionDecode(buffer, pc);
//     printf("%p    %08x      %s\n",
//            prev_pc, *reinterpret_cast<int32_t*>(prev_pc), buffer);
//   }
//
// The Disassembler class also has a convenience method to disassemble a block
// of code into a FILE*, meaning that the above functionality could also be
// achieved by just calling Disassembler::Disassemble(stdout, begin, end);

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if V8_TARGET_ARCH_RISCV64

#include "src/base/platform/platform.h"
#include "src/codegen/macro-assembler.h"
#include "src/codegen/riscv64/constants-riscv64.h"
#include "src/codegen/riscv64/instructions-riscv64.h"
#include "src/codegen/riscv64/register-riscv64.h"
#include "src/diagnostics/disasm.h"

#define ILLEGAL_RISCV() \
  out_buffer_pos_ +=    \
      SNPrintF(out_buffer_ + out_buffer_pos_, "ILLEGAL Instructions");

namespace v8 {
namespace internal {

typedef const char* InstrName;
//------------------------------------------------------------------------------

// Decoder decodes and disassembles instructions into an output buffer.
// It uses the converter to convert register names and call destinations into
// more informative description.
class Decoder {
 public:
  Decoder(const disasm::NameConverter& converter,
          v8::internal::Vector<char> out_buffer)
      : converter_(converter), out_buffer_(out_buffer), out_buffer_pos_(0) {
    out_buffer_[out_buffer_pos_] = '\0';
  }

  ~Decoder() {}

  // Writes one disassembled instruction into 'buffer' (0-terminated).
  // Returns the length of the disassembled machine instruction in bytes.
  int InstructionDecode(byte* instruction);

  const disasm::NameConverter& converter_;
  v8::internal::Vector<char> out_buffer_;
  int out_buffer_pos_;

  void DecodeTypeLoad(Instruction* instr);
  void DecodeTypeLoadFp(Instruction* instr);
  void DecodeTypeMiscMem(Instruction* instr);
  void DecodeTypeOpImm(Instruction* instr);
  void DecodeTypeAuipc(Instruction* instr);
  void DecodeTypeJalrWithAddres(Instruction* instr_auipc,
                                Instruction* instr_jalr);
  void DecodeTypeOpImm32(Instruction* instr);
  void DecodeTypeStore(Instruction* instr);
  void DecodeTypeStoreFp(Instruction* instr);
  void DecodeTypeAmo(Instruction* instr);
  void DecodeTypeOp(Instruction* instr);
  void DecodeTypeLui(Instruction* instr);
  void DecodeTypeOp32(Instruction* instr);
  void DecodeTypeMadd(Instruction* instr);
  void DecodeTypeMsub(Instruction* instr);
  void DecodeTypeNmsub(Instruction* instr);
  void DecodeTypeNmadd(Instruction* instr);
  void DecodeTypeOpFp(Instruction* instr);
  void DecodeTypeBranch(Instruction* instr);
  void DecodeTypeJalr(Instruction* instr);
  void DecodeTypeJal(Instruction* instr);
  void DecodeTypeSystem(Instruction* instr);

  bool DecodePseudoInstruction(Instruction* instr);

  void format(const char* msg);
  void format(InstrName instrName, Register r1, Register r2, Register rd);
  void format(InstrName instrName, Register r1, uint32_t imm,
              bool isHex = true);
  void formatOffset(InstrName instrName, Register r1, Register r2,
                    uint32_t imm);
  void formatOffset(InstrName instrName, Register r1, Register r2, Register r3,
                    uint32_t imm);
  void formatOffset(InstrName instrName, Register r1, Register r2, uint32_t imm,
                    const char* target);
  void formatImm(InstrName instrName, Register r1, Register r2, uint32_t imm,
                 bool isHex = true);
  void formatBranch(InstrName instrName, Register r1, Register r2, uint32_t imm,
                    const char* target);
  void formatFloat(InstrName instrName, FPURegister r1, FPURegister r2,
                   FPURegister r3, FPURegister r4);
  void formatFloat(InstrName instrName, FPURegister r1, FPURegister r2,
                   FPURegister r3);
  void formatFloatImm(InstrName instrName, FPURegister r1, FPURegister r2,
                      uint32_t imm);
  void formatFloatImm(InstrName instrName, FPURegister r1, uint32_t imm);
  void formatFloatOffset(InstrName instrName, FPURegister r1, FPURegister rd,
                         uint32_t imm);
  void formatCsr(InstrName instrName, Register r1, Register r2);
  void formatCsrI(InstrName instrName, Register r1, uint32_t imm);

  DISALLOW_COPY_AND_ASSIGN(Decoder);

  static bool branch_long_flag;
};
bool Decoder::branch_long_flag = false;
void Decoder::DecodeTypeLoad(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  DCHECK(funct3Value >= LB_F3 && funct3Value <= LWU_F3);
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rd = ToRegister(instr->RdValue());
  InstrName names[] = {"lb", "lh", "lw", "ld", "lbu", "lhu", "lwu"};
  InstrName name = names[funct3Value];
  formatOffset(name, rd, rs1, instr->ImmValueIType());
}

void Decoder::DecodeTypeLoadFp(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  DCHECK(funct3Value >= FLW_F3 && funct3Value <= FLQ_F3);
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  InstrName names[] = {"flw", "fld", "flq"};
  InstrName name = names[funct3Value - FLW_F3];
  formatFloatOffset(name, rd, rs1, instr->ImmValueIType());
}

void Decoder::DecodeTypeMiscMem(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  DCHECK(funct3Value == FENCE_F3 || funct3Value == FENCEi_F3);
  switch (funct3Value) {
    case FENCE_F3: {
      int succ = instr->Bits(Fence4Shift + Fence20Shift - 1, Fence20Shift);
      int pred = instr->Bits(Fence4Shift + Fence24Shift - 1, Fence24Shift);
      const char* actions[] = {"",   "w",   "r",   "rw",  "o",  "ow",
                               "or", "orw", "i",   "iw",  "ir", "irw",
                               "io", "iow", "ior", "iorw"};
      out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "fence %s, %s",
                                  actions[succ], actions[pred]);
      break;
    }
    case FENCEi_F3:
      format("fence.i");
      break;
    default:
      UNSUPPORTED_RISCV();
  }
}

void Decoder::DecodeTypeOpImm(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rd = ToRegister(instr->RdValue());
  InstrName name = "";
  switch (funct3Value) {
    case ADDI_F3:
    case SLTI_F3:
    case SLTIU_F3:
    case XORI_F3:
    case ORI_F3:
    case ANDI_F3: {
      DCHECK(funct3Value != SLLI_F3 && funct3Value != SRAI_F3 &&
             funct3Value != SRLI_F3);
      InstrName names[] = {"addi", "", "slti", "sltiu",
                           "xori", "", "ori",  "andi"};
      name = names[funct3Value];
      formatImm(name, rd, rs1, instr->ImmValueIType(), false);
      break;
    }
    case SLLI_F3:
    case SRAI_F3: {  // same as SRLI_F3
      if (funct3Value == SLLI_F3) {
        name = "slli";
      } else if (instr->Bit(30)) {  // this bit will be 1 for srai 0 for srli
        name = "srai";
      } else {
        name = "srli";
      }
      // take shamt6 but ignoring shamt5 in rv32
      // because shamt[5] must be 0 in rv32
      formatImm(name, rd, rs1, instr->Shamt6());
      break;
    }
  }
}

void Decoder::DecodeTypeAuipc(Instruction* instr) {
  Register rd = ToRegister(instr->RdValue());
  format("auipc", rd, (int32_t)(instr->ImmValueUType() << kImm12Shift), false);
}

void Decoder::DecodeTypeOpImm32(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Register rd = ToRegister(instr->RdValue());
  Register rs1 = ToRegister(instr->Rs1Value());

  DCHECK(funct3Value == ADDIW_F3 || funct3Value == SLLIW_F3 ||
         funct3Value == SRLIW_F3 || funct3Value == SRAIW_F3);
  switch (funct3Value) {
    case ADDIW_F3:
      formatImm("addiw", rd, rs1, instr->ImmValueIType(), false);
      break;
    case SLLIW_F3:
      formatImm("slliw", rd, rs1, instr->Shamt5());
      break;
    case SRAIW_F3: {
      // same as SRLIW_F3
      InstrName name = instr->Bit(30) ? "sraiw" : "srliw";
      formatImm(name, rd, rs1, instr->Shamt5());
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }
}

void Decoder::DecodeTypeStore(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rs2 = ToRegister(instr->Rs2Value());
  DCHECK(SB_F3 <= funct3Value && funct3Value <= SD_F3);
  switch (funct3Value) {
    case SB_F3:
    case SH_F3:
    case SW_F3:
    case SD_F3: {
      InstrName names[] = {"sb", "sh", "sw", "sd"};
      InstrName name = names[funct3Value];
      formatOffset(name, rs2, rs1, instr->ImmValueSType());
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }
}

void Decoder::DecodeTypeStoreFp(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  DCHECK(FSW_F3 <= funct3Value && funct3Value <= FSQ_F3);
  switch (funct3Value) {
    case FSW_F3:
    case FSD_F3:
    case FSQ_F3: {
      InstrName names[] = {"", "", "fsw", "fsd", "fsq"};
      InstrName name = names[funct3Value];
      formatFloatOffset(name, rs2, rs1, instr->ImmValueSType());
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }
}

void Decoder::DecodeTypeAmo(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Funct7 funct7Value = instr->Funct7Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rs2 = ToRegister(instr->Rs2Value());
  Register rd = ToRegister(instr->RdValue());
  DCHECK(LRW_F3 <= funct3Value && funct3Value <= LRD_F3);
  InstrName name = "";
  InstrName postFix = "";
  switch (funct3Value) {
    case LRW_F3: {
      postFix = ".w";
      break;
    }
    case LRD_F3: {
      postFix = ".d";
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }

  switch (funct7Value >> kImm2Shift) {
    case 0:
      name = "amoadd";
      break;
    case 1:
      name = "amoswap";
      break;
    case 2: {
      DCHECK_EQ(rs2.code(), 0);
      name = "lr";
      std::string fullName = std::string(name) + std::string(postFix);
      formatOffset(fullName.c_str(), rd, rs1, 0);
      return;
    }
    case 3:
      name = "sc";
      break;
    case 4:
      name = "amoxor";
      break;
    case 8:
      name = "amoor";
      break;
    case 12:
      name = "amoand";
      break;
    case 16:
      name = "amomin";
      break;
    case 20:
      name = "amomax";
      break;
    case 24:
      name = "amominu";
      break;
    case 28:
      name = "amomaxu";
      break;
    default:
      UNSUPPORTED_RISCV();
  }
  std::string fullName = std::string(name) + std::string(postFix);
  formatOffset(fullName.c_str(), rd, rs2, rs1, 0);
}

void Decoder::DecodeTypeOp(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Funct7 funct7Value = instr->Funct7Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rs2 = ToRegister(instr->Rs2Value());
  Register rd = ToRegister(instr->RdValue());
  InstrName name = "";
  DCHECK(funct7Value == MUL_F7 || funct7Value == ADD_F7 ||
         funct7Value == SUB_F7);
  switch (funct7Value) {
    case ADD_F7: {
      InstrName names[] = {"add", "sll", "slt", "sltu",
                           "xor", "srl", "or",  "and"};
      name = names[funct3Value];
      break;
    }
    case SUB_F7: {
      DCHECK(funct3Value == SUB_F3 || funct3Value == SRA_F3);
      switch (funct3Value) {
        case SUB_F3:
          name = "sub_f3";
          break;
        case SRA_F3:
          name = "sra_f3";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    }
    case MUL_F7: {
      InstrName names[] = {"mul", "mulh", "mulhsu", "mulhu",
                           "div", "divu", "rem",    "remu"};
      name = names[funct3Value];
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }
  format(name, rd, rs1, rs2);
}

void Decoder::DecodeTypeLui(Instruction* instr) {
  Register rd = ToRegister(instr->RdValue());
  format("lui", rd, instr->ImmValueUType());
}

void Decoder::DecodeTypeOp32(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Funct7 funct7Value = instr->Funct7Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rs2 = ToRegister(instr->Rs2Value());
  Register rd = ToRegister(instr->RdValue());
  InstrName name = "";
  DCHECK(funct7Value == ADDW_F7 || funct7Value == SUBW_F7 ||
         funct7Value == MULW_F7);
  switch (funct7Value) {
    case ADDW_F7:
      switch (funct3Value) {
        case ADDW_F3:
          name = "addw";
          break;
        case SLLW_F3:
          name = "sllw";
          break;
        case SRLW_F3:
          name = "srlw";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    case SUBW_F7:
      switch (funct3Value) {
        case SUBW_F3:
          name = "subw";
          break;
        case SRAW_F3:
          name = "sraw";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    case MULW_F7:
      switch (funct3Value) {
        case MULW_F3:
          name = "mulw";
          break;
        case DIVW_F3:
          name = "divw";
          break;
        case DIVUW_F3:
          name = "divuw";
          break;
        case REMW_F3:
          name = "remw";
          break;
        case REMUW_F3:
          name = "remuw";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    default:
      UNSUPPORTED_RISCV()
  }
  format(name, rd, rs1, rs2);
}

void Decoder::DecodeTypeMadd(Instruction* instr) {
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  FPURegister rs3 = ToFPURegister(instr->Rs3Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  Funct2 funct2Value = instr->Funct2Value();
  DCHECK(funct2Value == FMADDS_F2 || funct2Value == FMADDD_F2 ||
         funct2Value == FMADDQ_F2);
  InstrName names[] = {"fmadd.s", "fmadd.d", "", "fmadd.q"};
  InstrName name = names[funct2Value];
  formatFloat(name, rd, rs1, rs2, rs3);
}

void Decoder::DecodeTypeMsub(Instruction* instr) {
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  FPURegister rs3 = ToFPURegister(instr->Rs3Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  Funct2 funct2Value = instr->Funct2Value();
  DCHECK(funct2Value == FMSUBS_F2 || funct2Value == FMSUBD_F2 ||
         funct2Value == FMSUBQ_F2);
  InstrName names[] = {"fmsub.s", "fmsub.d", "", "fmsub.q"};
  InstrName name = names[funct2Value];
  formatFloat(name, rd, rs1, rs2, rs3);
}

void Decoder::DecodeTypeNmsub(Instruction* instr) {
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  FPURegister rs3 = ToFPURegister(instr->Rs3Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  Funct2 funct2Value = instr->Funct2Value();
  DCHECK(funct2Value == FNMSUBS_F2 || funct2Value == FNMSUBD_F2 ||
         funct2Value == FNMSUBQ_F2);
  InstrName names[] = {"fnmsub.s", "fnmsub.d", "", "fnmsub.q"};
  InstrName name = names[funct2Value];
  formatFloat(name, rd, rs1, rs2, rs3);
}

void Decoder::DecodeTypeNmadd(Instruction* instr) {
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  FPURegister rs3 = ToFPURegister(instr->Rs3Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  Funct2 funct2Value = instr->Funct2Value();
  DCHECK(funct2Value == FNMADDS_F2 || funct2Value == FNMADDD_F2 ||
         funct2Value == FNMADDQ_F2);
  InstrName names[] = {"fnmadd.s", "fnmadd.d", "", "fnmadd.q"};
  InstrName name = names[funct2Value];
  formatFloat(name, rd, rs1, rs2, rs3);
}

void Decoder::DecodeTypeOpFp(Instruction* instr) {
  FPURegister rs1 = ToFPURegister(instr->Rs1Value());
  FPURegister rs2 = ToFPURegister(instr->Rs2Value());
  FPURegister rd = ToFPURegister(instr->RdValue());
  Funct3 funct3Value = instr->Funct3Value();
  Funct7 funct7Value = instr->Funct7Value();
  uint32_t funct7_1_0 = funct3Value & kImm3Shift;
  DCHECK(funct7_1_0 == kInstrPostFixS || funct7_1_0 == kInstrPostFixD ||
         funct7_1_0 == kInstrPostFixQ);
  const char* postFixNames[] = {"s", "d", "", "q"};
  const char* postFix = postFixNames[funct7_1_0];
  InstrName name = "";
  uint32_t funct7_6_2 = funct7Value >> kImm2Shift;
  switch (funct7_6_2) {
    case (FADDS_F7 >> kImm2Shift):    // FADD.S FADD.D FADD.Q
    case (FSUBS_F7 >> kImm2Shift):    // FSUB.S FSUB.D FSUB.Q
    case (FMULS_F7 >> kImm2Shift):    // FMUL.S FMUL.D FMUL.Q
    case (FDIVS_F7 >> kImm2Shift): {  // FDIV.S FDIV.D FDIV.Q
      DCHECK((FADDS_F7 >> kImm2Shift) <= funct7_6_2 &&
             funct7_6_2 <= (FDIVS_F7 >> kImm2Shift));
      InstrName names[] = {"fadd", "fsub", "fmul", "fdiv"};
      name = names[funct7_6_2];
      break;
    }
    case (FSQRTS_F7 >> kImm2Shift):  // FSQRT.S FSQRT.D FSQRT.Q
      DCHECK_EQ(rs2, f0);
      name = "fsqrt";
      return;
    case (FSGNJS_F7 >> kImm2Shift):
      switch (funct3Value) {
        case FSGNJS_F3:  // FSGNJ.S FSGNJ.D FSGNJ.Q
          name = "fsgnj";
          break;
        case FSGNJNS_F3:  // FSGNJN.S FSGNJN.D FSGNJN.Q
          name = "fsgnjn";
          break;
        case FSGNJXS_F3:  // FSGNJX.S FSGNJX.D FSGNJX.Q
          name = "fsgnjx";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    case (FMINS_F7 >> kImm2Shift): {  // FMIN FMAX
      DCHECK(FMINS_F3 <= funct3Value && funct3Value <= FMAXS_F3);
      InstrName names[] = {"fmin", "fmax"};
      name = names[funct3Value];
      break;
    }
    case (FCVTWS_F7 >> kImm2Shift): {
      uint32_t rs2Code = rs2.code();
      DCHECK(kImm0Shift <= rs2Code && rs2Code <= kImm3Shift);
      InstrName names[] = {"fcvt.w", "fcvt.wu", "fcvt.l", "fcvt.lu"};
      name = names[rs2Code];
      break;
    }
    case (FMVXW_F7 >> kImm2Shift): {
      switch (funct3Value) {
        case FMVXW_F3:
          switch (funct7_1_0) {
            case 0:
              formatFloat("fmv.x.w", rd, rs1, rs2);
              return;
            case 1:
              formatFloat("fmv.x.d", rd, rs1, rs2);
              return;
            default:
              UNSUPPORTED_RISCV()
          }
          break;
        case FCLASSS_F3: {
          name = "fclass";
          break;
        }
        default:
          UNSUPPORTED_RISCV()
      }
      break;
    }
    case (FEQS_F7 >> kImm2Shift): {
      InstrName names[] = {"feq", "flt", "fle"};
      DCHECK(FLES_F3 <= funct3Value && funct3Value <= FEQS_F3);
      name = names[funct3Value];
      break;
    }
    case (FCVTSW_F7 >> kImm2Shift): {
      uint32_t rs2Code = rs2.code();
      DCHECK(kImm0Shift <= rs2Code && rs2Code <= kImm3Shift);
      InstrName tailNames[] = {"w", "wu", "l", "lu"};
      std::string fullName = std::string("fcvt.") + std::string(postFix) +
                             std::string(".") + std::string(tailNames[rs2Code]);
      formatFloat(fullName.c_str(), rd, rs1, rs2);
      return;
    }
    case (FMVWX_F7 >> kImm2Shift): {
      DCHECK(rs2 == f0 && funct3Value == kImm0Shift);
      switch (funct7_1_0) {
        case 0:
          name = "fmv.w.x";
          break;
        case 1:
          name = "fmv.d.x";
          break;
        default:
          UNSUPPORTED_RISCV()
      }
      formatFloat(name, rd, rs1, rs2);
      return;
    }
    default:
      UNSUPPORTED_RISCV()
  }

  std::string fullName =
      std::string(name) + std::string(".") + std::string(postFix);
  formatFloat(fullName.c_str(), rd, rs1, rs2);
}

void Decoder::DecodeTypeBranch(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  DCHECK_NE(funct3Value, 2);
  DCHECK_NE(funct3Value, 3);
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rs2 = ToRegister(instr->Rs2Value());
  InstrName names[] = {"beq", "bne", "", "", "blt", "bge", "bltu", "bgeu"};
  InstrName name = names[funct3Value];
  const char* target = converter_.NameOfAddress(
      reinterpret_cast<byte*>(instr) + int32_t(instr->ImmValueBType()));
  formatBranch(name, rs1, rs2, instr->ImmValueBType(), target);
}

void Decoder::DecodeTypeJalr(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  DCHECK_EQ(funct3Value, JALR_F3);
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rd = ToRegister(instr->RdValue());
  formatOffset("jalr", rd, rs1, instr->ImmValueIType());
}

void Decoder::DecodeTypeJalrWithAddres(Instruction* instr_auipc,
                                       Instruction* instr_jalr) {
  Funct3 funct3Value = instr_jalr->Funct3Value();
  DCHECK_EQ(funct3Value, JALR_F3);
  Register auipc_rd = ToRegister(instr_auipc->RdValue());
  Register jalr_rs1 = ToRegister(instr_jalr->Rs1Value());
  Register jalr_rd = ToRegister(instr_jalr->RdValue());
  if (auipc_rd == jalr_rs1) {
    int32_t auipc_arg = (int32_t)(instr_auipc->ImmValueUType() << kImm12Shift);
    int32_t jalr_args = instr_jalr->ImmValueIType();
    int32_t offset = auipc_arg + jalr_args;
    const char* target =
        converter_.NameOfAddress(reinterpret_cast<byte*>(instr_auipc) + offset);
    formatOffset("jalr", jalr_rd, jalr_rs1, jalr_args, target);
  } else {
    DecodeTypeJalr(instr_jalr);
  }
}

void Decoder::DecodeTypeJal(Instruction* instr) {
  Register rd = ToRegister(instr->RdValue());
  format("jal", rd, instr->ImmValueJType());
}

void Decoder::DecodeTypeSystem(Instruction* instr) {
  Funct3 funct3Value = instr->Funct3Value();
  Register rs1 = ToRegister(instr->Rs1Value());
  Register rd = ToRegister(instr->RdValue());
  switch (funct3Value) {
    case ECALL_F3:  // EBREAK
      DCHECK_EQ(rs1, x0);
      DCHECK_EQ(rd, x0);
      if (instr->Bit(kImm20Shift)) {
        format("ebreak");
      } else {
        format("ecall");
      }
      break;
    case CSRRW_F3:
    case CSRRS_F3:
    case CSRRC_F3: {
      DCHECK(CSRRW_F3 <= funct3Value && funct3Value <= CSRRC_F3);
      InstrName names[] = {"", "csrrw", "csrrs", "csrrc"};
      InstrName name = names[funct3Value];
      formatCsr(name, rd, rs1);
      break;
    }
    case CSRRWI_F3:
    case CSRRSI_F3:
    case CSRRCI_F3: {
      DCHECK(CSRRWI_F3 <= funct3Value && funct3Value <= CSRRCI_F3);
      InstrName names[] = {"csrrwi", "csrrsi", "csrrci"};
      InstrName name = names[funct3Value - CSRRWI_F3];
      formatCsrI(name, rd, instr->ZImmValue());
      break;
    }
    default:
      UNSUPPORTED_RISCV()
  }
}

bool Decoder::DecodePseudoInstruction(Instruction* instr) { return false; }

// Disassemble the instruction at *instr_ptr into the output buffer.
int Decoder::InstructionDecode(byte* instr_ptr) {
  Instruction* instr = Instruction::At(instr_ptr);
  // Print raw instruction bytes.
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%08x       ",
                              instr->InstructionBits());
  // 16bit compressed instruction
  if (instr->isCompressed()) {
    format("unsupported Compressed Instructions");
    return kInstrCompressedSize;
  }

  // decode pseudo-instruction
  if (DecodePseudoInstruction(instr)) {
    return kInstrSize;
  }
  // 32bit RISCV-G instruction
  switch (instr->InstructionType()) {
    case Instruction::kLoadType:
      DecodeTypeLoad(instr);
      break;
    case Instruction::kLoadFpType:
      DecodeTypeLoadFp(instr);
      break;
    case Instruction::kCustom0Type:
      format("customize instruction");
      break;
    case Instruction::kMiscMemType:
      DecodeTypeMiscMem(instr);
      break;
    case Instruction::kOpImmType:
      DecodeTypeOpImm(instr);
      break;
    case Instruction::kAuipcType:
      DecodeTypeAuipc(instr);
      branch_long_flag = false;
      if (Instruction::At(instr_ptr + 4)->InstructionType() ==
          InstructionBase::kJalrType) {
        branch_long_flag = true;
      } 
      break;
    case Instruction::kOpImm32Type:
      DecodeTypeOpImm32(instr);
      break;
    case Instruction::kUnSupport48Type:
      format("unsupported 48bit instruction");
      break;
    case Instruction::kStoreType:
      DecodeTypeStore(instr);
      break;
    case Instruction::kStoreFpType:
      DecodeTypeStoreFp(instr);
      break;
    case Instruction::kCustom1Type:
      format("customize instruction");
      break;
    case Instruction::kAmoType:
      DecodeTypeAmo(instr);
      break;
    case Instruction::kOpType:
      DecodeTypeOp(instr);
      break;
    case Instruction::kLuiType:
      DecodeTypeLui(instr);
      break;
    case Instruction::kOp32Type:
      DecodeTypeOp32(instr);
      break;
    case Instruction::kUnSupport64Type:
      format("unsupport 64bit instruction");
      break;
    case Instruction::kMaddType:
      DecodeTypeMadd(instr);
      break;
    case Instruction::kMsubType:
      DecodeTypeMsub(instr);
      break;
    case Instruction::kNmsubType:
      DecodeTypeNmsub(instr);
      break;
    case Instruction::kNmaddType:
      DecodeTypeNmadd(instr);
      break;
    case Instruction::kOpFpType:
      DecodeTypeOpFp(instr);
      break;
    case Instruction::kReversed0Type:
      format("reversed type instruction");
      break;
    case Instruction::kCustom2Type:
      format("customize instruction");
      break;
    case Instruction::kAnotherUnSupport48Type:
      format("unsupported 48bit instruction");
      break;
    case Instruction::kBranchType:
      DecodeTypeBranch(instr);
      break;
    case Instruction::kJalrType:
      if (branch_long_flag) {
        DecodeTypeJalrWithAddres(Instruction::At(instr_ptr - 4), instr);
        branch_long_flag = false;
      } else {
        DecodeTypeJalr(instr);
      }
      break;
    case Instruction::kReversed1Type:
      format("reversed type instruction");
      break;
    case Instruction::kJalType:
      DecodeTypeJal(instr);
      break;
    case Instruction::kSystemType:
      DecodeTypeSystem(instr);
      break;
    case Instruction::kReversed2Type:
      format("reversed type instruction");
      break;
    case Instruction::kCustom3Type:
      format("customize instruction");
      break;
    case Instruction::kUnSupport80Type:
      format("unsupported 80bit instruction");
      break;
    default:
      UNSUPPORTED_RISCV()
  }
  return kInstrSize;
}

void Decoder::format(const char* msg) {
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s", msg);
}

void Decoder::format(InstrName instrName, Register r1, Register r2,
                     Register r3) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));
  const char* r3Name = converter_.NameOfCPURegister(ToNumber(r3));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %s",
                              instrName, r1Name, r2Name, r3Name);
}
void Decoder::formatOffset(InstrName instrName, Register r1, Register r2,
                           uint32_t imm) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %d(%s)",
                              instrName, r1Name, imm, r2Name);
}

void Decoder::formatOffset(InstrName instrName, Register r1, Register r2,
                           uint32_t imm, const char* target) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));
  out_buffer_pos_ +=
      SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %d(%s) -> %s", instrName,
               r1Name, imm, r2Name, target);
}

void Decoder::formatOffset(InstrName instrName, Register r1, Register r2,
                           Register r3, uint32_t imm) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));
  const char* r3Name = converter_.NameOfCPURegister(ToNumber(r3));
  out_buffer_pos_ +=
      SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %d(%s)", instrName,
               r1Name, r2Name, imm, r3Name);
}

void Decoder::formatImm(InstrName instrName, Register r1, Register r2,
                        uint32_t imm, bool isHex) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));
  const char* immHex = "%s %s, %s, 0x%x";
  const char* immDec = "%s %s, %s, %d";
  out_buffer_pos_ +=
      SNPrintF(out_buffer_ + out_buffer_pos_, isHex ? immHex : immDec,
               instrName, r1Name, r2Name, imm);
}

void Decoder::formatBranch(InstrName instrName, Register r1, Register r2,
                           uint32_t imm, const char* target) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfCPURegister(ToNumber(r2));

  out_buffer_pos_ +=
      SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %d -> %s", instrName,
               r1Name, r2Name, imm, target);
}

void Decoder::format(InstrName instrName, Register r1, uint32_t imm,
                     bool isHex) {
  const char* r1Name = converter_.NameOfCPURegister(ToNumber(r1));
  const char* immHex = "%s %s, 0x%x";
  const char* immDec = "%s %s, %d";
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_,
                              isHex ? immHex : immDec, instrName, r1Name, imm);
}

void Decoder::formatFloat(InstrName instrName, FPURegister r1, FPURegister r2,
                          FPURegister r3) {
  const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfXMMRegister(ToNumber(r2));
  const char* r3Name = converter_.NameOfXMMRegister(ToNumber(r3));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %s",
                              instrName, r1Name, r2Name, r3Name);
}

void Decoder::formatFloat(InstrName instrName, FPURegister r1, FPURegister r2,
                          FPURegister r3, FPURegister r4) {
  const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfXMMRegister(ToNumber(r2));
  const char* r3Name = converter_.NameOfXMMRegister(ToNumber(r3));
  const char* r4Name = converter_.NameOfXMMRegister(ToNumber(r4));
  out_buffer_pos_ +=
      SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %s, %s", instrName,
               r1Name, r2Name, r3Name, r4Name);
}

void Decoder::formatFloatOffset(InstrName instrName, FPURegister r1,
                                FPURegister r2, uint32_t imm) {
  const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfXMMRegister(ToNumber(r2));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %d(%s)",
                              instrName, r1Name, imm, r2Name);
}

// void Decoder::formatFloatImm(InstrName instrName, FPURegister r1, uint32_t
// imm) {
//   const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
//   out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, 0x%x",
//                               instrName, r1Name, imm);
// }
// void Decoder::formatFloatImm(InstrName instrName, FPURegister r1, FPURegister
// r2, uint32_t imm) {
//   const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
//   const char* r2Name = converter_.NameOfXMMRegister(ToNumber(r2));
//   out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, %s, %f",
//                               instrName, r1Name, r2Name, imm);
// }

void Decoder::formatCsr(InstrName instrName, Register r1, Register r2) {
  const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
  const char* r2Name = converter_.NameOfXMMRegister(ToNumber(r2));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, csr, %s",
                              instrName, r1Name, r2Name);
}

void Decoder::formatCsrI(InstrName instrName, Register r1, uint32_t imm) {
  const char* r1Name = converter_.NameOfXMMRegister(ToNumber(r1));
  out_buffer_pos_ += SNPrintF(out_buffer_ + out_buffer_pos_, "%s %s, csr, 0x%x",
                              instrName, r1Name, imm);
}

}  // namespace internal
}  // namespace v8

//------------------------------------------------------------------------------

namespace disasm {

static const char* const cpu_regs[v8::internal::kNumRegisters] = {
    "zero", "ra", "sp", "gp", "tp",  "t0",  "t1", "t2", "fp", "s1", "a0",
    "a1",   "a2", "a3", "a4", "a5",  "a6",  "a7", "s2", "s3", "s4", "s5",
    "s6",   "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"};

static const char* const fpu_regs[v8::internal::kNumFPURegisters] = {
    "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7",
    "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
    "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7",
    "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"};

const char* NameConverter::NameOfAddress(byte* addr) const {
  v8::internal::SNPrintF(tmp_buffer_, "%p", static_cast<void*>(addr));
  return tmp_buffer_.begin();
}

const char* NameConverter::NameOfConstant(byte* addr) const {
  return NameOfAddress(addr);
}

const char* NameConverter::NameOfCPURegister(int reg) const {
  if (0 <= reg && reg < v8::internal::kNumRegisters) return cpu_regs[reg];
  return "noreg";
}

const char* NameConverter::NameOfXMMRegister(int reg) const {
  if (0 <= reg && reg < v8::internal::kNumFPURegisters) return fpu_regs[reg];
  return "noreg";
}

const char* NameConverter::NameOfByteCPURegister(int reg) const {
  return "nobytereg";
}

const char* NameConverter::NameInCode(byte* addr) const {
  // The default name converter is called for unknown code. So we will not try
  // to access any memory.
  return "";
}

//------------------------------------------------------------------------------

int Disassembler::InstructionDecode(v8::internal::Vector<char> buffer,
                                    byte* instruction) {
  v8::internal::Decoder d(converter_, buffer);
  return d.InstructionDecode(instruction);
}

// The RISCV64 assembler does not currently use constant pools.
int Disassembler::ConstantPoolSizeAt(byte* instruction) { return -1; }

void Disassembler::Disassemble(FILE* f, byte* begin, byte* end,
                               UnimplementedOpcodeAction unimplemented_action) {
  NameConverter converter;
  Disassembler d(converter, unimplemented_action);
  for (byte* pc = begin; pc < end;) {
    v8::internal::EmbeddedVector<char, 128> buffer;
    buffer[0] = '\0';
    byte* prev_pc = pc;
    pc += d.InstructionDecode(buffer, pc);
    v8::internal::PrintF(f, "%p    %08x      %s\n", static_cast<void*>(prev_pc),
                         *reinterpret_cast<int32_t*>(prev_pc), buffer.begin());
  }
}

#undef STRING_STARTS_WITH

}  // namespace disasm

#endif  // V8_TARGET_ARCH_RISCV64
