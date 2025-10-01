///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// File Name  : cpu64_defs.vh                                                                   //
// Description: Header containing definitions used throughout the CPU64 core.                  //
//              This file defines all constants, opcodes, and control signals for the          //
//              RISC-V RV64IMAC_Zicsr_Zifencei processor implementation.                        //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
`ifndef CPU64_VH
`define CPU64_VH

//===================== Development and Debug Defines =====================//
`define TODO_DUMMY '0 // Dummy value for development - remove before release
// `define CPU64_RVFI  // Uncomment to enable RISC-V Formal Interface

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Architectural Constants                                   //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Core architectural parameters and alignment requirements

`define XLEN            64        // Data width: 64-bit RISC-V architecture
`define IALIGN_MASK     ( ~('b1) ) // Instruction alignment mask: instructions must be 2-byte aligned

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Load Store Defines                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Memory access width, sign extension, and phase control definitions

//===================== Memory Access Width (func3[1:0]) ==================//
// Standard 2-bit encoding for memory access width
`define MEM_WIDTH_BYTE      2'b00    // 8-bit memory access (LB, LBU, SB)
`define MEM_WIDTH_HALF      2'b01    // 16-bit memory access (LH, LHU, SH)
`define MEM_WIDTH_WORD      2'b10    // 32-bit memory access (LW, LWU, SW)
`define MEM_WIDTH_DOUBLE    2'b11    // 64-bit memory access (LD, SD)

//===================== Memory Width One-Hot Encoding =====================//
// One-hot encoding for memory width selection in control logic
`define MEM_WIDTH_1H_BYTE   4'b0001  // Byte access (1-hot)
`define MEM_WIDTH_1H_HALF   4'b0010  // Half-word access (1-hot)
`define MEM_WIDTH_1H_WORD   4'b0100  // Word access (1-hot)
`define MEM_WIDTH_1H_DOUBLE 4'b1000  // Double-word access (1-hot)

//===================== Memory Sign Extension (func3[2]) ==================//
// Sign extension control for load operations
`define MEM_SIGNED          1'b0     // Sign-extend loaded data
`define MEM_UNSIGNED        1'b1     // Zero-extend loaded data

//===================== Memory Transaction Phases ========================//
// Memory interface transaction phase indicators
`define MEM_PHASE_ADDR      1'b0     // Address phase of memory transaction
`define MEM_PHASE_RESP      1'b1     // Response phase of memory transaction

///////////////////////////////////////////////////////////////////////////////////////////////////
//                             Register File Writeback Source Defines                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
// One-hot encoding for selecting the source of data written back to register file

//===================== Writeback Source One-Hot Encoding ================//
// 5-bit one-hot encoding for writeback source multiplexer
`define WB_SRC_1H_I_ALU     5'b00001  // Integer ALU result (ADD, SUB, AND, OR, etc.)
`define WB_SRC_1H_MEM       5'b00010  // Memory load data (LB, LH, LW, LD)
`define WB_SRC_1H_PC_PLUS_4 5'b00100  // PC + 4 (JAL, JALR return address)
`define WB_SRC_1H_M_ALU     5'b01000  // Multiply/Divide ALU result (MUL, DIV, etc.)
`define WB_SRC_CSR          5'b10000  // CSR read data (CSRRW, CSRRS, etc.)


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Branch Condition Code Defines                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Branch condition codes for conditional branch instructions (BEQ, BNE, BLT, etc.)

//===================== Branch Condition Codes (func3) ====================//
// Standard 3-bit encoding for branch conditions
`define BR_OP_BEQ           3'b000    // Branch if Equal (rs1 == rs2)
`define BR_OP_BNE           3'b001    // Branch if Not Equal (rs1 != rs2)
`define BR_OP_BLT           3'b100    // Branch if Less Than (rs1 < rs2, signed)
`define BR_OP_BGE           3'b101    // Branch if Greater or Equal (rs1 >= rs2, signed)
`define BR_OP_BLTU          3'b110    // Branch if Less Than Unsigned (rs1 < rs2, unsigned)
`define BR_OP_BGEU          3'b111    // Branch if Greater or Equal Unsigned (rs1 >= rs2, unsigned)

//===================== Branch Condition One-Hot Encoding =================//
// 6-bit one-hot encoding for branch condition selection in control logic
`define BR_OP_1H_BEQ        6'b000001  // BEQ condition (1-hot)
`define BR_OP_1H_BNE        6'b000010  // BNE condition (1-hot)
`define BR_OP_1H_BLT        6'b000100  // BLT condition (1-hot)
`define BR_OP_1H_BGE        6'b001000  // BGE condition (1-hot)
`define BR_OP_1H_BLTU       6'b010000  // BLTU condition (1-hot)
`define BR_OP_1H_BGEU       6'b100000  // BGEU condition (1-hot)


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                          OPCODE Defines                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
// RISC-V instruction opcodes for all supported instruction types

//===================== Base RV32I Opcodes ================================//
// Standard 7-bit opcodes for base RISC-V instruction set
`define OPCODE_LUI          7'b0110111  // Load Upper Immediate
`define OPCODE_AUIPC        7'b0010111  // Add Upper Immediate to PC
`define OPCODE_JAL          7'b1101111  // Jump and Link
`define OPCODE_JALR         7'b1100111  // Jump and Link Register
`define OPCODE_BRANCH       7'b1100011  // Conditional Branch (BEQ, BNE, BLT, etc.)
`define OPCODE_LOAD         7'b0000011  // Load instructions (LB, LH, LW, LD)
`define OPCODE_STORE        7'b0100011  // Store instructions (SB, SH, SW, SD)
`define OPCODE_OP_IMM       7'b0010011  // Immediate ALU operations (ADDI, ANDI, etc.)
`define OPCODE_OP           7'b0110011  // Register-register ALU operations (ADD, SUB, etc.)
`define OPCODE_SYSTEM       7'b1110011  // System instructions (CSR, ECALL, EBREAK, etc.)

//===================== RV64I Extension Opcodes ===========================//
// 64-bit specific opcodes for word operations
`define OPCODE_OP_IMM_W     7'b0011011  // Immediate word operations (ADDIW, SLLIW, etc.)
`define OPCODE_OP_W         7'b0111011  // Register-register word operations (ADDW, SUBW, etc.)

//===================== Zifencei Extension Opcode =========================//
// Fence instruction opcode
`define OPCODE_MISC_MEM     7'b0001111  // Miscellaneous memory operations (FENCE, FENCE.I)


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       System Op Defines                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
// System instruction function codes and CSR operation definitions

//===================== System Function Codes (func3) =====================//
// Function codes for system instructions (opcode = SYSTEM)
`define SYSTEM_OP_PRIV      3'b000    // Privileged instructions (ECALL, EBREAK, MRET, etc.)
`define SYSTEM_OP_CSRRW     3'b001    // CSR Read/Write (CSRRW)
`define SYSTEM_OP_CSRRS     3'b010    // CSR Read and Set bits (CSRRS)
`define SYSTEM_OP_CSRRC     3'b011    // CSR Read and Clear bits (CSRRC)
`define SYSTEM_OP_CSRRWI    3'b101    // CSR Read/Write Immediate (CSRRWI)
`define SYSTEM_OP_CSRRSI    3'b110    // CSR Read and Set bits Immediate (CSRRSI)
`define SYSTEM_OP_CSRRCI    3'b111    // CSR Read and Clear bits Immediate (CSRRCI)

//===================== CSR ALU Operation One-Hot Encoding ================//
// One-hot encoding for CSR ALU operations
`define CSR_ALU_OP_1H_RW    3'b001    // Read/Write operation (1-hot)
`define CSR_ALU_OP_1H_RS    3'b010    // Read and Set operation (1-hot)
`define CSR_ALU_OP_1H_RC    3'b100    // Read and Clear operation (1-hot)

//===================== Privileged Instruction Function Codes =============//
// 12-bit function codes for privileged instructions (func12 field)
`define FUNC12_ECALL        12'b0000000_00000  // Environment Call
`define FUNC12_EBREAK       12'b0000000_00001  // Environment Break
`define FUNC12_MRET         12'b0011000_00010  // Machine Return
`define FUNC12_SRET         12'b0001000_00010  // Supervisor Return
`define FUNC12_WFI          12'b0001000_00101  // Wait for Interrupt


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Compressed Op Defines                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
// RISC-V compressed instruction set (RV64C) definitions

//===================== Compressed Instruction Quadrants ==================//
// 2-bit quadrant field determines compressed instruction format
`define C0 2'b00    // Quadrant 0: Stack pointer based operations
`define C1 2'b01    // Quadrant 1: Immediate operations and branches
`define C2 2'b10    // Quadrant 2: Stack pointer operations and register ops

//===================== Quadrant C0 Function Codes (func3) ================//
// Stack pointer based operations (C.ADDI4SPN, C.LW, C.LD, C.SW, C.SD)
`define C_FUNC3_ADDI4SPN  3'b000    // C.ADDI4SPN: Add immediate to SP
`define C_FUNC3_LW        3'b010    // C.LW: Load word from stack
`define C_FUNC3_LD        3'b011    // C.LD: Load double from stack
`define C_FUNC3_SW        3'b110    // C.SW: Store word to stack
`define C_FUNC3_SD        3'b111    // C.SD: Store double to stack

//===================== Quadrant C1 Function Codes (func3) ================//
// Immediate operations, jumps, and branches
`define C_FUNC3_ADDI      3'b000    // C.ADDI: Add immediate
`define C_FUNC3_ADDIW     3'b001    // C.ADDIW: Add immediate word
`define C_FUNC3_LI        3'b010    // C.LI: Load immediate
`define C_FUNC3_ADDI16SPN 3'b011    // C.ADDI16SPN: Add immediate to SP (if rd == 2)
`define C_FUNC3_LUI       3'b011    // C.LUI: Load upper immediate (if rd != 2)
`define C_FUNC3_MISC_ALU  3'b100    // C.MISC_ALU: Miscellaneous ALU operations
`define C_FUNC3_J         3'b101    // C.J: Jump
`define C_FUNC3_BEQZ      3'b110    // C.BEQZ: Branch if equal to zero
`define C_FUNC3_BNEZ      3'b111    // C.BNEZ: Branch if not equal to zero

//===================== Quadrant C2 Function Codes (func3) ================//
// Stack pointer operations and register operations
`define C_FUNC3_SLLI      3'b000    // C.SLLI: Shift left logical immediate
`define C_FUNC3_LWSP      3'b010    // C.LWSP: Load word from SP
`define C_FUNC3_LDSP      3'b011    // C.LDSP: Load double from SP
`define C_FUNC3_JR_MV_ADD 3'b100    // C.JR/MV/ADD: Jump register/Move/Add
`define C_FUNC3_SWSP      3'b110    // C.SWSP: Store word to SP
`define C_FUNC3_SDSP      3'b111    // C.SDSP: Store double to SP

//===================== Compressed Function 4 Codes =======================//
// 4-bit function codes for specific compressed instructions
`define C_FUNC4_JR        4'b1000   // C.JR: Jump register
`define C_FUNC4_MV        4'b1000   // C.MV: Move register
`define C_FUNC4_JALR      4'b1001   // C.JALR: Jump and link register
`define C_FUNC4_ADD       4'b1001   // C.ADD: Add registers
`define C_FUNC4_EBREAK    4'b1001   // C.EBREAK: Environment break

//===================== Compressed Function 6 Codes =======================//
// 6-bit function codes for miscellaneous ALU operations
`define C_FUNC6_SRLI64    6'b100000 // C.SRLI64: Shift right logical immediate (64-bit)
`define C_FUNC6_SRAI64    6'b100001 // C.SRAI64: Shift right arithmetic immediate (64-bit)
`define C_FUNC6_ANDI      6'b100010 // C.ANDI: And immediate
`define C_FUNC6_SUB       6'b100011 // C.SUB: Subtract registers
`define C_FUNC6_XOR       6'b100011 // C.XOR: Exclusive or registers
`define C_FUNC6_OR        6'b100011 // C.OR: Or registers
`define C_FUNC6_AND       6'b100011 // C.AND: And registers
`define C_FUNC6_SUBW      6'b100111 // C.SUBW: Subtract word registers
`define C_FUNC6_ADDW      6'b100111 // C.ADDW: Add word registers


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Zifencei Defines                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Fence instruction definitions for instruction cache synchronization

//===================== Miscellaneous Memory Function Codes ===============//
// Function codes for fence instructions (opcode = MISC_MEM)
`define MISC_MEM_FENCE_I         3'b001    // FENCE.I: Instruction cache synchronization

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ALU Defines                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
// ALU operation codes and operand source definitions

//===================== Base ALU Operations ===============================//
// 6-bit ALU operation codes: {func7[5], func7[0], func3, opcode[3]}
`define ALU_OP_ADD          6'b0_0_000_0    // ADD: Add registers
`define ALU_OP_SLL          6'b0_0_001_0    // SLL: Shift left logical
`define ALU_OP_SLT          6'b0_0_010_0    // SLT: Set less than (signed)
`define ALU_OP_SLTU         6'b0_0_011_0    // SLTU: Set less than unsigned
`define ALU_OP_XOR          6'b0_0_100_0    // XOR: Exclusive or
`define ALU_OP_SRL          6'b0_0_101_0    // SRL: Shift right logical
`define ALU_OP_OR           6'b0_0_110_0    // OR: Or registers
`define ALU_OP_AND          6'b0_0_111_0    // AND: And registers
`define ALU_OP_SUB          6'b1_0_000_0    // SUB: Subtract registers
`define ALU_OP_SRA          6'b1_0_101_0    // SRA: Shift right arithmetic
`define ALU_OP_PASS         6'b1_0_111_0    // PASS: Pass through (LUI)

//===================== RV64I Word Operations =============================//
// 32-bit word operations for RV64I extension
`define ALU_OP_ADDW         6'b0_0_000_1    // ADDW: Add word registers
`define ALU_OP_SLLW         6'b0_0_001_1    // SLLW: Shift left logical word
`define ALU_OP_SRLW         6'b0_0_101_1    // SRLW: Shift right logical word
`define ALU_OP_SUBW         6'b1_0_000_1    // SUBW: Subtract word registers
`define ALU_OP_SRAW         6'b1_0_101_1    // SRAW: Shift right arithmetic word

//===================== M Extension Operations =============================//
// Multiply/Divide operations for M extension
`define ALU_OP_MUL          6'b0_1_000_0    // MUL: Multiply (low 64 bits)
`define ALU_OP_MULH         6'b0_1_001_0    // MULH: Multiply high (signed × signed)
`define ALU_OP_MULHSU       6'b0_1_010_0    // MULHSU: Multiply high (signed × unsigned)
`define ALU_OP_MULHU        6'b0_1_011_0    // MULHU: Multiply high (unsigned × unsigned)
`define ALU_OP_DIV          6'b0_1_100_0    // DIV: Divide (signed)
`define ALU_OP_DIVU         6'b0_1_101_0    // DIVU: Divide (unsigned)
`define ALU_OP_REM          6'b0_1_110_0    // REM: Remainder (signed)
`define ALU_OP_REMU         6'b0_1_111_0    // REMU: Remainder (unsigned)
`define ALU_OP_MULW         6'b0_1_000_1    // MULW: Multiply word (low 32 bits)
`define ALU_OP_DIVW         6'b0_1_100_1    // DIVW: Divide word (signed)
`define ALU_OP_DIVUW        6'b0_1_101_1    // DIVUW: Divide word (unsigned)
`define ALU_OP_REMW         6'b0_1_110_1    // REMW: Remainder word (signed)
`define ALU_OP_REMUW        6'b0_1_111_1    // REMUW: Remainder word (unsigned)

//===================== ALU Function Codes ================================//
// Function codes for shift operations
`define FUNC3_ALU_SHIFT_L   3'b001         // Shift left operations
`define FUNC3_ALU_SHIFT_R   3'b101         // Shift right operations

//===================== ALU Operand Source Selection =====================//
// ALU Operand A source selection
`define ALU_A_SRC_RS1       3'b000         // Source register 1
`define ALU_A_SRC_U_IMMED   3'b001         // Upper immediate (LUI, AUIPC)
`define ALU_A_SRC_J_IMMED   3'b010         // Jump immediate (JAL)
`define ALU_A_SRC_B_IMMED   3'b011         // Branch immediate (BEQ, BNE, etc.)
`define ALU_A_SRC_CSR_IMMED 3'b100         // CSR immediate (CSRRWI, CSRRSI, etc.)

// ALU Operand B source selection
`define ALU_B_SRC_RS2       2'b00          // Source register 2
`define ALU_B_SRC_I_IMMED   2'b01          // I-type immediate (ADDI, ANDI, etc.)
`define ALU_B_SRC_S_IMMED   2'b10          // S-type immediate (SW, SH, etc.)
`define ALU_B_SRC_PC        2'b11          // Program counter (AUIPC, JAL)

//===================== Division Constants ================================//
// Special values for division operations
`define NEGATIVE_1          64'hFFFFFFFFFFFFFFFF  // -1 in 64-bit two's complement
`define NEGATIVE_1_W        32'hFFFFFFFF          // -1 in 32-bit two's complement
`define DIV_MOST_NEG_INT    64'h8000000000000000  // Most negative 64-bit integer
`define DIV_MOST_NEG_INT_W  32'h80000000          // Most negative 32-bit integer


`endif // CPU64_VH

