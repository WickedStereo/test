///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Module Name: alu_decoder                                                                      //
// Description: ALU decoder decodes ALU operations and operands from 32-bit RV64IM
//              instructions. Generates ALU control signals, immediate values,
//              and operand selection signals. Handles all arithmetic, logical,
//              and shift operations with proper immediate generation.
//              instruction.                                                                     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
`include "cpu64_defs.vh"


module cpu64_alu_decoder #(parameter VADDR = 39) (
    //===================== Instruction Input =====================//
    input      [31:0]       inst_i,              // 32-bit instruction to decode

    //===================== Register Data Inputs ==================//
    input [`XLEN-1:0]       rs1_data_i,          // Source register 1 data
    input [`XLEN-1:0]       rs2_data_i,          // Source register 2 data

    //===================== Program Counter Input =================//
    input [VADDR-1:0]       pc_i,                // Current program counter

    //===================== Immediate Output ======================//
    output wire [`XLEN-1:0] csr_imm_ao,         // CSR immediate value (5-bit zero-extended)

    //===================== ALU Operand Outputs ===================//
    output reg [`XLEN-1:0]  alu_op_a_ao,        // ALU operand A (first operand)
    output reg [`XLEN-1:0]  alu_op_b_ao,        // ALU operand B (second operand)
    output reg              alu_uses_rs1_ao,    // ALU uses rs1 as operand A
    output reg              alu_uses_rs2_ao,    // ALU uses rs2 as operand B

    //===================== ALU Control Outputs ===================//
    output reg  [5:0]       alu_operation_ao,    // ALU operation code (6-bit)
    output reg              legal_alu_op_ao     // ALU operation is legal
);

    //===================== Instruction Field Extraction =============//
    wire [2:0] func3  = inst_i[14:12];           // Function 3 field (bits 14:12)
    wire [6:0] opcode = inst_i[6:0];             // Opcode field (bits 6:0)


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                      Immediate Generator                                  //
    /////////////////////////////////////////////////////////////////////////////////////////////// 
    // Generates all possible immediate values from the 32-bit instruction
    // Each immediate type has a specific bit layout and sign extension pattern

    reg [`XLEN-1:0] csr_imm, i_imm, s_imm, u_imm, b_imm, j_imm;

    always @(*) begin
        // CSR Immediate: 5-bit zero-extended immediate for CSR instructions
        // Used in CSRRWI, CSRRSI, CSRRCI instructions
        csr_imm = { 59'b0, inst_i[19:15] };
        
        // I-Type Immediate: 12-bit sign-extended immediate
        // Used in ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI, JALR, LOAD instructions
        i_imm = { { 52{inst_i[31]} }, inst_i[31:20] } ;
        
        // S-Type Immediate: 12-bit sign-extended immediate
        // Used in STORE instructions (SB, SH, SW, SD)
        s_imm = { { 52{inst_i[31]} }, inst_i[31:25], inst_i[11:7]};
        
        // U-Type Immediate: 20-bit immediate shifted left by 12 bits
        // Used in LUI, AUIPC instructions
        u_imm = { { 32{inst_i[31]} }, inst_i[31:12], 12'b0};
        
        // B-Type Immediate: 13-bit sign-extended immediate (shifted left by 1)
        // Used in BRANCH instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
        b_imm = { { 51{inst_i[31]} }, inst_i[31], inst_i[7], inst_i[30:25], inst_i[11:8], 1'b0};
        
        // J-Type Immediate: 21-bit sign-extended immediate (shifted left by 1)
        // Used in JAL instruction
        j_imm = { { 43{inst_i[31]} }, inst_i[31], inst_i[19:12], inst_i[20], inst_i[30:21], 1'b0};
    end

    assign csr_imm_ao = csr_imm;
    

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                    ALU Operand Decoders                                   //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Determines ALU operands A and B based on instruction type
    // Also determines which source registers are actually used by the ALU

    // CSR immediate operations: CSRRWI, CSRRSI, CSRRCI use immediate instead of rs1
    wire csr_op = (func3 == `SYSTEM_OP_CSRRWI) || 
                  (func3 == `SYSTEM_OP_CSRRSI) || 
                  (func3 == `SYSTEM_OP_CSRRCI);
    
    // Program counter extended to full XLEN width
    wire [`XLEN-1:0] pc = { {`XLEN-VADDR{1'b0}},  pc_i};

    always @(*) begin : alu_operand_decoder
        //===================== ALU Operand A Selection ===================//
        // Determines the first ALU operand based on instruction type
        case (opcode)
            `OPCODE_LUI     : alu_op_a_ao = u_imm;        // LUI: upper immediate
            `OPCODE_AUIPC   : alu_op_a_ao = u_imm;        // AUIPC: upper immediate
            `OPCODE_JAL     : alu_op_a_ao = j_imm;        // JAL: jump immediate
            `OPCODE_BRANCH  : alu_op_a_ao = b_imm;        // BRANCH: branch immediate
            `OPCODE_SYSTEM  : alu_op_a_ao = (csr_op) ? csr_imm : rs1_data_i;  // CSR: immediate or rs1
            default         : alu_op_a_ao = rs1_data_i;   // Default: rs1 data
        endcase

        //===================== ALU Operand B Selection ===================//
        // Determines the second ALU operand based on instruction type
        case (opcode)
            `OPCODE_AUIPC   : alu_op_b_ao = pc;           // AUIPC: PC + immediate
            `OPCODE_JAL     : alu_op_b_ao = pc;           // JAL: PC + immediate
            `OPCODE_BRANCH  : alu_op_b_ao = pc;           // BRANCH: PC + immediate
            `OPCODE_STORE   : alu_op_b_ao = s_imm;        // STORE: store immediate
            `OPCODE_LOAD    : alu_op_b_ao = i_imm;        // LOAD: load immediate
            `OPCODE_JALR    : alu_op_b_ao = i_imm;        // JALR: immediate + rs1
            `OPCODE_OP_IMM  : alu_op_b_ao = i_imm;        // OP-IMM: immediate operations
            `OPCODE_OP_IMM_W: alu_op_b_ao = i_imm;        // OP-IMM-W: immediate word operations
            default         : alu_op_b_ao = rs2_data_i;   // Default: rs2 data
        endcase
                           
        //===================== Register Usage Detection ==================//
        // Determines which source registers are actually used by the ALU
        
        // ALU uses rs2 only for register-register operations (OP, OP_W)
        alu_uses_rs2_ao = (opcode == `OPCODE_OP_W) || (opcode == `OPCODE_OP);
        
        // ALU uses rs1 for most operations except:
        // - LUI, AUIPC, JAL, BRANCH (use immediates instead)
        // - CSR immediate operations (use immediate instead of rs1)
        alu_uses_rs1_ao = ((opcode == `OPCODE_SYSTEM) && csr_op) ||
                          ((opcode != `OPCODE_LUI)   && 
                           (opcode != `OPCODE_AUIPC) && 
                           (opcode != `OPCODE_JAL)   &&
                           (opcode != `OPCODE_BRANCH));
    end


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                      Legality Checkers                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Validates that the instruction represents a legal ALU operation
    // Checks func7 and func3 fields for valid combinations according to RISC-V spec

    always @(*) begin
        //===================== OP (Register-Register) Instructions =============//
        if (opcode == `OPCODE_OP) begin
            // For ADD/SUB and SRL/SRA operations, func7 can be 0x00 or 0x20
            if ( (func3 == '0) || (func3 == 3'b101) ) 
                legal_alu_op_ao = (inst_i[31:25] == '0        ) ||  // 0x00: ADD, SRL
                                  (inst_i[31:25] == 7'b0100000);    // 0x20: SUB, SRA
            else
                // For other operations (SLT, SLTU, XOR, OR, AND), func7 must be 0x00
                legal_alu_op_ao = (inst_i[31:25] == '0);

        //===================== OP_W (Register-Register Word) Instructions ======//
        end else if (opcode == `OPCODE_OP_W) begin
            // For ADDW/SUBW and SRLW/SRAW operations, func7 can be 0x00 or 0x20
            if ( (func3 == '0) || (func3 == 3'b101) )
                legal_alu_op_ao = (inst_i[31:25] == '0        ) ||  // 0x00: ADDW, SRLW
                                  (inst_i[31:25] == 7'b0100000);    // 0x20: SUBW, SRAW
            else
                // For SLLW, func7 must be 0x00 and func3 must be 3'b001
                legal_alu_op_ao = (inst_i[31:25] == '0) && (func3 == 3'b001);

        //===================== OP_IMM (Immediate) Instructions =================//
        end else if (opcode == `OPCODE_OP_IMM) begin
            if      (func3 == `FUNC3_ALU_SHIFT_R)  // SRLI, SRAI
                legal_alu_op_ao = ( (inst_i[31:26] == '0       ) ||  // 0x00: SRLI
                                    (inst_i[31:26] == 6'b010000) );  // 0x20: SRAI

            else if (func3 == `FUNC3_ALU_SHIFT_L)  // SLLI
                legal_alu_op_ao = (inst_i[31:26] == '0);             // 0x00: SLLI

            else
                // For ADDI, SLTI, XORI, ORI, ANDI, any func7 is legal
                legal_alu_op_ao = 1'b1;

        //===================== OP_IMM_W (Immediate Word) Instructions ==========//
        end else if (opcode == `OPCODE_OP_IMM_W) begin
            if      (func3 == `FUNC3_ALU_SHIFT_R)  // SRLIW, SRAIW
                legal_alu_op_ao = ( (inst_i[31:25] == '0        ) ||  // 0x00: SRLIW
                                    (inst_i[31:25] == 7'b0100000) );  // 0x20: SRAIW

            else if (func3 == `FUNC3_ALU_SHIFT_L)  // SLLIW
                legal_alu_op_ao = (inst_i[31:25] == '0);             // 0x00: SLLIW

            else
                // For ADDIW, func7 must be 0x00 and func3 must be 3'b000
                legal_alu_op_ao = (func3 == '0);

        //===================== Other Instructions ==============================//
        end else begin
            // Only LUI and AUIPC are legal for ALU operations
            legal_alu_op_ao = ( (opcode == `OPCODE_LUI  ) ||
                                (opcode == `OPCODE_AUIPC) );
        end

    end


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                  ALU Operation Decoders                                   //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Generates the 6-bit ALU operation code based on instruction fields
    // Handles the complex encoding of RISC-V ALU operations

    // Extract func7 bits for ALU operation encoding
    wire func7_5 = inst_i[30];  // func7[5] - distinguishes ADD/SUB, SRL/SRA, etc.
    wire func7_0 = inst_i[25];  // func7[0] - used for M-extension operations

    reg [5:0] alu_op_code;      // 6-bit ALU operation code
    reg alu_op_arith, non_shift_imm, op_imm;

    always @ (*) begin
        //===================== Operation Type Detection =====================//
        // Determine if this is an arithmetic/logical operation
        alu_op_arith = ( opcode == `OPCODE_OP_IMM || opcode == `OPCODE_OP_IMM_W ||
                         opcode == `OPCODE_OP     || opcode == `OPCODE_OP_W );

        // Determine if this is an immediate operation
        op_imm = ( opcode == `OPCODE_OP_IMM || opcode == `OPCODE_OP_IMM_W );
        
        // Determine if this is a non-shift immediate operation
        // For non-shift immediates, func7[5] is actually imm[10] and must be ignored
        non_shift_imm = op_imm && (func3 != `FUNC3_ALU_SHIFT_R);
        
        //===================== ALU Operation Code Generation ================//
        // Construct 6-bit ALU operation code: {func7[5], func7[0], func3, opcode[3]}
        // This encoding allows the ALU to distinguish between all operation types
        alu_op_code = { (func7_5 & ~non_shift_imm),  // func7[5] (ignored for non-shift immediates)
                        (func7_0 & ~op_imm),         // func7[0] (ignored for all immediates)
                        func3,                       // func3 field (3 bits)
                        opcode[3] };                 // opcode[3] (distinguishes 32/64-bit)

        //===================== ALU Operation Selection ======================//
        if      (alu_op_arith)          alu_operation_ao = alu_op_code;  // Use generated code
        else if (opcode == `OPCODE_LUI) alu_operation_ao = `ALU_OP_PASS; // LUI: pass through
        else                            alu_operation_ao = `ALU_OP_ADD;  // Default: ADD
        
    end

endmodule

