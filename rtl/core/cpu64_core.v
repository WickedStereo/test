///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Module Name: cpu64_core                                                                      //
// Description: Top-level 5-stage RISC-V RV64IMAC processor core with compressed instruction   //
//              support. Implements a complete in-order pipeline with hazard detection,        //
//              data forwarding, and comprehensive exception/interrupt handling.                //
//                                                                                               //
// Architecture: 5-Stage Pipeline (Fetch → Decode → Execute → Memory → Writeback)              //
// ISA Support:  RV64IMAC_Zicsr_Zifencei (64-bit, Integer, Multiply/Divide, Atomic,            //
//              Compressed, CSR, Fence.i extensions)                                            //
// Pipeline:     In-order execution with hazard resolution and data forwarding                  //
// Memory:       Separate instruction and data memories via OBI interface                      //
// CSRs:         Full machine-mode CSR implementation with interrupt controller                 //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
// `include "cpu64_defs.vh"


module cpu64_core #(parameter VADDR = 39, parameter RESET_ADDR = 0) (
    //===================== Clock and Reset =====================//
    input                   clk_i,              // System clock
    input                   rst_ni,             // Active-low reset

    //================== Machine-Mode Interrupts ================//
    input                   m_ext_inter_i,      // Machine External Interrupt
    input                   m_soft_inter_i,     // Machine Software Interrupt  
    input                   m_timer_inter_i,    // Machine Timer Interrupt

    //===================== Time Input ==========================//
    input [`XLEN-1:0]       time_i,             // Wall-clock time for mtime CSR

    //===================== Fence.i Output ======================//
    output                  fencei_flush_ao,    // Instruction cache flush signal

    //========== Instruction Memory Interface (OBI) =============//
    // Read-only interface for instruction fetching
    output wire             imem_req_o,         // Instruction memory request
    input                   imem_gnt_i,         // Instruction memory grant
    output wire [VADDR-1:0] imem_addr_ao,       // Instruction memory address
    input                   imem_rvalid_i,      // Instruction memory read valid
    input        [31:0]     imem_rdata_i,       // Instruction memory read data

    //============ Data Memory Interface (OBI) ==================//
    // Read-write interface for load/store operations
    output wire             dmem_req_o,         // Data memory request
    input                   dmem_gnt_i,         // Data memory grant
    output wire [VADDR-1:0] dmem_addr_ao,       // Data memory address
    output wire             dmem_we_ao,         // Data memory write enable
    output wire  [7:0]      dmem_be_ao,         // Data memory byte enable
    output wire [`XLEN-1:0] dmem_wdata_ao,      // Data memory write data
    input                   dmem_rvalid_i,      // Data memory read valid
    input       [`XLEN-1:0] dmem_rdata_i,       // Data memory read data

    //============ Debug/Trace Signals ==========================//
    output wire             ecall_o,            // ECALL exception (for testbench halt)
    output wire             WB_valid_o          // Writeback stage valid (for monitoring)

`ifdef CPU64_RVFI
    ,
    output reg                   rvfi_valid,
    output reg [  64 - 1 : 0]    rvfi_order,
    output reg [  32 - 1 : 0]    rvfi_insn,
    output reg                   rvfi_trap,
    output reg                   rvfi_halt,
    output reg                   rvfi_intr,
    output reg [2    - 1 : 0]    rvfi_mode,
    output reg [2    - 1 : 0]    rvfi_ixl,
    output reg [   5 - 1 : 0]    rvfi_rs1_addr,
    output reg [   5 - 1 : 0]    rvfi_rs2_addr,
    output reg [`XLEN - 1 : 0]   rvfi_rs1_rdata,
    output reg [`XLEN - 1 : 0]   rvfi_rs2_rdata,
    output reg [   5 - 1 : 0]    rvfi_rd_addr,
    output reg [`XLEN - 1 : 0]   rvfi_rd_wdata,
    output reg [`XLEN - 1 : 0]   rvfi_pc_rdata,
    output reg [`XLEN - 1 : 0]   rvfi_pc_wdata,
    output reg [`XLEN   - 1 : 0] rvfi_mem_addr,
    output reg [`XLEN/8 - 1 : 0] rvfi_mem_rmask,
    output reg [`XLEN/8 - 1 : 0] rvfi_mem_wmask,
    output reg [`XLEN   - 1 : 0] rvfi_mem_rdata,
    output reg [`XLEN   - 1 : 0] rvfi_mem_wdata
`endif
    );

    //===================== Pipeline Control Signals =====================//
    // Stage validity, stall, bubble, and squash signals for each pipeline stage
    wire FCH_squash, DCD_squash, EXE_squash, MEM_squash, WB_squash;  // Squash signals
    wire FCH_bubble, DCD_bubble, EXE_bubble, MEM_bubble, WB_bubble;  // Bubble signals  
    wire FCH_stall,  DCD_stall,  EXE_stall,  MEM_stall,  WB_stall;   // Stall signals
    wire FCH_valid,  DCD_valid,  EXE_valid,  MEM_valid,  WB_valid;   // Validity signals

    //===================== Control Flow Signals ========================//
    wire mret, wait_for_int, fencei_flush, WB_inst_retired;           // Control flow
    wire [VADDR-1:0] trap_ret_addr;                                   // Trap return address

    //===================== Hazard Detection Signals ====================//
    // Various hazard conditions that require pipeline control
    wire load_use_haz, csr_load_use_haz, imem_stall, dmem_stall, alu_stall, fencei;

    //===================== Exception Signals ===========================//
    // Exception conditions detected in different pipeline stages
    wire illegal_inst_ex, ecall_ex, ebreak_ex, unalign_load_ex, unalign_store_ex, 
         csr_wr_ex, csr_rd_ex;

    //===================== Interrupt Signals ===========================//
    // Interrupt handling and trap vector generation
    wire [VADDR-1:0] csr_branch_addr;                                 // Trap vector address
    wire             pipeline_interrupt, csr_interrupt;               // Interrupt signals

    //===================== Register File Signals =======================//
    // Register file interface signals for reading and writing registers
    wire [4:0]       rs1_idx,  rs2_idx,  rd_idx;    // Register indices
    wire [`XLEN-1:0] rs1_data, rs2_data, rd_data;   // Register data
    wire             rd_wr_en;                       // Register write enable

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //            ____               _                ____  _                                    //
    //           |  _ \(_)_ __   ___| (_)_ __   ___  / ___|| |_ __ _  __ _  ___  ___             //
    //           | |_) | | '_ \ / _ \ | | '_ \ / _ \ \___ \| __/ _` |/ _` |/ _ \/ __|            //
    //           |  __/| | |_) |  __/ | | | | |  __/  ___) | || (_| | (_| |  __/\__ \            //
    //           |_|   |_| .__/ \___|_|_|_| |_|\___| |____/ \__\__,_|\__, |\___||___/            //
    //                   |_|                                         |___/                       //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                            Fetch                                          //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Fetch Stage: Manages program counter and instruction memory access
    // - Handles branch targets and PC updates
    // - Manages compressed instruction feedback
    // - Interfaces with instruction memory via OBI protocol
    // - Responds to pipeline control signals (stall/bubble/squash)
    
    wire             EXE_branch, DCD_compress_instr;           // Branch and compression signals
    wire [VADDR-1:0] EXE_pc_target_addr, FCH_pc, FCH_next_pc;  // PC-related signals
    wire [31:0]      FCH_inst;                                   // Fetched instruction

    // cpu64_fetch_stage #(.VADDR(VADDR), .RESET_ADDR(RESET_ADDR)) FCH (
    //     //======= Clocks, Resets, and Stage Controls ========//
    //     .clk_i              (clk_i),
    //     .rst_ni             (rst_ni),

    //     .squash_i           (FCH_squash),
    //     .bubble_i           (FCH_bubble),
    //     .stall_i            (FCH_stall),
    //     .fencei_i           (fencei_flush),
    //     .compressed_instr_i (DCD_compress_instr),

    //     //============== Branch Control Inputs ==============//
    //     .branch_i           (EXE_branch),
    //     .target_addr_i      (EXE_pc_target_addr),
        
    //     .csr_branch_i       (pipeline_interrupt),
    //     .csr_branch_addr_i  (csr_branch_addr),

    //     .trap_ret_i         (mret),
    //     .trap_ret_addr_i    (trap_ret_addr),
    
    //     //========== Instruction Memory Interface ===========//
    //     .imem_req_o         (imem_req_o),
    //     .imem_addr_ao       (imem_addr_ao),
    //     .imem_gnt_i         (imem_gnt_i),
    //     .imem_rvalid_i      (imem_rvalid_i),

    //     .imem_stall_ao      (imem_stall),

    //     //================ Pipeline Outputs =================//
    //     .valid_o            (FCH_valid),
    //     .pc_o               (FCH_pc),
    //     .next_pc_o          (FCH_next_pc)
    // );
    cpu64_fetch_stage #(
  .VADDR      (VADDR),
  .RESET_ADDR (RESET_ADDR),
  .MODE       (0)            // ICACHE behavior (for ICache integration)
) FCH (
  .clk_i              (clk_i),
  .rst_ni             (rst_ni),

  .squash_i           (FCH_squash),
  .bubble_i           (FCH_bubble),
  .stall_i            (FCH_stall),
  .fencei_i           (fencei_flush),

  .ret_is16_i         (1'b0),              // tie off on CPU path
  .compressed_instr_i (DCD_compress_instr),

  .branch_i           (EXE_branch),
  .target_addr_i      (EXE_pc_target_addr),
  .csr_branch_i       (pipeline_interrupt),
  .csr_branch_addr_i  (csr_branch_addr),
  .trap_ret_i         (mret),
  .trap_ret_addr_i    (trap_ret_addr),

  .imem_req_o         (imem_req_o),
  .imem_addr_ao       (imem_addr_ao),
  .imem_gnt_i         (imem_gnt_i),
  .imem_rvalid_i      (imem_rvalid_i),
  .imem_rdata_i       (imem_rdata_i),
  .imem_stall_ao      (imem_stall),

  .valid_o            (FCH_valid),
  .pc_o               (FCH_pc),
  .next_pc_o          (FCH_next_pc),
  .inst_o             (FCH_inst)
);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                            Decode                                         //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Register File Signals
    wire [4:0]       DCD_rs1_idx,  DCD_rs2_idx,  DCD_rd_idx;
    wire             DCD_rs1_used, DCD_rs2_used, DCD_rd_wr_en;
    wire [`XLEN-1:0] DCD_rs1_data, DCD_rs2_data;
    wire [4:0]       DCD_rd_wr_src_1h;
    // ALU Signals
    wire [5:0]       DCD_alu_operation;
    wire [`XLEN-1:0] DCD_alu_op_a,       DCD_alu_op_b;
    wire             DCD_alu_uses_rs1,   DCD_alu_uses_rs2;
    // Memory Signals
    wire [3:0]       DCD_mem_width_1h;
    wire             DCD_mem_rd,   DCD_mem_wr,   DCD_mem_sign;
    // Flow Control Signals
    wire             DCD_branch;
    wire [VADDR-1:0] DCD_next_pc, DCD_pc;
    wire [5:0]       DCD_br_cond_1h;
    // CSR Control Signals
    wire             DCD_csr_wr_en, EXE_csr_wr_en, csr_rd_en;
    wire [2:0]       DCD_csr_op_1h;
    wire [11:0]      DCD_csr_addr, csr_rd_addr;
    wire [11:0]      csr_wr_addr;
    // Traps and Exceptions
    wire DCD_ecall_ex, DCD_ebreak_ex, DCD_mret, DCD_wait_for_int, DCD_illegal_inst_ex, DCD_fencei;
    
`ifdef CPU64_RVFI
    wire                    DCD_rvfi_trap;
    wire [31:0]             DCD_rvfi_insn;
    wire [`XLEN-1:0]        DCD_rvfi_pc_rdata, DCD_rvfi_pc_wdata;
`endif

    cpu64_decode_stage #(.VADDR(VADDR)) DCD (
        //======= Clocks, Resets, and Stage Controls ========//
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        
        .squash_i           (DCD_squash),
        .bubble_i           (DCD_bubble),
        .stall_i            (DCD_stall),
        //=============== Fetch Stage Inputs ================//
        .pc_i               (FCH_pc),
        .next_pc_i          (FCH_next_pc),
        .valid_i            (FCH_valid),
        // Instruction from fetch stage
        .inst_i             (FCH_inst),
        // ============== Fetch Stage Feedback ==============//
        .compressed_instr_ao(DCD_compress_instr),
        //======= Register File Async Read Interface ========//
        .rs1_idx_ao         (rs1_idx),
        .rs2_idx_ao         (rs2_idx),
        .rs1_data_i         (rs1_data),
        .rs2_data_i         (rs2_data),
        //=============== CSR Read Interface ================//
        .csr_addr_ao        (csr_rd_addr),
        .csr_rd_en_ao       (csr_rd_en),
        .csr_rd_ex_i        (csr_rd_ex),
        // CSR Load Use Hazard Inputs
        .EXE_csr_addr_i     (csr_wr_addr),
        .EXE_csr_wr_en_i    (EXE_csr_wr_en),
        .csr_load_use_haz_ao(csr_load_use_haz),
        //================ Pipeline Outputs =================//
        .valid_o            (DCD_valid),
        // Register Source 1 (rs1)
        .rs1_idx_o          (DCD_rs1_idx),
        .rs1_used_o         (DCD_rs1_used),
        .rs1_data_o         (DCD_rs1_data),
        // Register Source 2 (rs2)
        .rs2_idx_o          (DCD_rs2_idx),
        .rs2_used_o         (DCD_rs2_used),
        .rs2_data_o         (DCD_rs2_data),
        // Destination Register (rd)
        .rd_idx_o           (DCD_rd_idx),
        .rd_wr_en_o         (DCD_rd_wr_en),
        .rd_wr_src_1h_o     (DCD_rd_wr_src_1h),
        // ALU Operation and Operands
        .alu_operation_o    (DCD_alu_operation),
        .alu_op_a_o         (DCD_alu_op_a),
        .alu_op_b_o         (DCD_alu_op_b),
        .alu_uses_rs1_o     (DCD_alu_uses_rs1),
        .alu_uses_rs2_o     (DCD_alu_uses_rs2),
        // Load/Store
        .mem_width_1h_o     (DCD_mem_width_1h),
        .mem_rd_o           (DCD_mem_rd),
        .mem_wr_o           (DCD_mem_wr),
        .mem_sign_o         (DCD_mem_sign),
        // Flow Control
        .branch_o           (DCD_branch),
        .pc_o               (DCD_pc),
        .next_pc_o          (DCD_next_pc),
        .br_cond_1h_o       (DCD_br_cond_1h),
        // CSR Control
        .csr_wr_en_o        (DCD_csr_wr_en),
        .csr_op_1h_o        (DCD_csr_op_1h),
        .csr_addr_o         (DCD_csr_addr),
        // Traps and Exceptions
        .ecall_ex_o         (DCD_ecall_ex),
        .ebreak_ex_o        (DCD_ebreak_ex),
        .illegal_inst_ex_o  (DCD_illegal_inst_ex),
        .mret_o             (DCD_mret),
        .wait_for_int_o     (DCD_wait_for_int),
        .fencei_o           (DCD_fencei)

`ifdef CPU64_RVFI
        ,
        .rvfi_insn_o          (DCD_rvfi_insn),
        .rvfi_trap_o          (DCD_rvfi_trap),
        .rvfi_pc_rdata_o      (DCD_rvfi_pc_rdata),
        .rvfi_pc_wdata_o      (DCD_rvfi_pc_wdata)
`endif
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                           Execute                                         //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    wire             EXE_rd_wr_en, EXE_rd_wr_src_load;
    wire [`XLEN-1:0] EXE_rs2_data,  EXE_rd_data, EXE_csr_wdata, EXE_csr_rdata; 
    wire [VADDR-1:0] EXE_pc, EXE_dmem_addr;
    wire [4:0]       EXE_rd_idx;
    wire [3:0]       EXE_mem_width_1h;
    wire             EXE_rd, EXE_wr, EXE_sign, EXE_wr_a;
    
`ifdef CPU64_RVFI
    wire                    EXE_rvfi_trap;
    wire [31:0]             EXE_rvfi_insn;
    wire [4:0]              EXE_rvfi_rs1_addr,  EXE_rvfi_rs2_addr;
    wire [`XLEN-1:0]        EXE_rvfi_rs1_rdata, EXE_rvfi_rs2_rdata;
    wire [`XLEN-1:0]        EXE_rvfi_pc_rdata,  EXE_rvfi_pc_wdata;
`endif

    cpu64_execute_stage #(.VADDR(VADDR)) EXE (
        //======= Clocks, Resets, and Stage Controls ========//
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        
        .squash_i           (EXE_squash),
        .bubble_i           (EXE_bubble),
        .stall_i            (EXE_stall),

        //============== Decode Stage Inputs ================//
        .valid_i            (DCD_valid),
        // Register Source 1 (rs1)
        .rs1_idx_i          (DCD_rs1_idx),
        .rs1_used_i         (DCD_rs1_used),
        .rs1_data_i         (DCD_rs1_data),
        // Register Source 2 (rs2)
        .rs2_idx_i          (DCD_rs2_idx),
        .rs2_used_i         (DCD_rs2_used),
        .rs2_data_i         (DCD_rs2_data),
        // Destination Register (rd)
        .rd_idx_i           (DCD_rd_idx),
        .rd_wr_en_i         (DCD_rd_wr_en),
        .rd_wr_src_1h_i     (DCD_rd_wr_src_1h),
        // ALU Operation and Operands
        .alu_operation_i    (DCD_alu_operation),
        .alu_op_a_i         (DCD_alu_op_a),
        .alu_op_b_i         (DCD_alu_op_b),
        .alu_uses_rs1_i     (DCD_alu_uses_rs1),
        .alu_uses_rs2_i     (DCD_alu_uses_rs2),
        // Load/Store
        .mem_width_1h_i     (DCD_mem_width_1h),
        .mem_rd_i           (DCD_mem_rd),
        .mem_wr_i           (DCD_mem_wr),
        .mem_sign_i         (DCD_mem_sign),
        // Flow Control
        .branch_i           (DCD_branch),
        .pc_i               (DCD_pc),
        .next_pc_i          (DCD_next_pc),
        .br_cond_1h_i       (DCD_br_cond_1h),
        // CSR Control
        .csr_wr_en_i        (DCD_csr_wr_en),
        .csr_op_1h_i        (DCD_csr_op_1h),
        .csr_wr_addr_i      (DCD_csr_addr),
        // Traps and Exceptions
        .ecall_ex_i         (DCD_ecall_ex),
        .ebreak_ex_i        (DCD_ebreak_ex),
        .illegal_inst_ex_i  (DCD_illegal_inst_ex),
        .mret_i             (DCD_mret),
        .wait_for_int_i     (DCD_wait_for_int),
        .fencei_i           (DCD_fencei),

        //============= Forwarded Register Data =============//
        .MEM_rd_wr_en_i     (rd_wr_en),
        .MEM_valid_i        (WB_valid),
        .MEM_rd_idx_i       (rd_idx),
        .MEM_rd_data_i      (rd_data),
        
        .WB_rd_wr_en_i      (rd_wr_en),
        .WB_valid_i         (WB_valid),
        .WB_rd_idx_i        (rd_idx),
        .WB_rd_data_i       (rd_data),

        //================== CSR Interface ==================//
        .csr_wr_ex_i        (csr_wr_ex),
        .csr_rdata_i        (EXE_csr_rdata),
        .csr_wdata_ao       (EXE_csr_wdata),
        .csr_wr_addr_ao     (csr_wr_addr),
        .csr_wr_en_ao       (EXE_csr_wr_en),

        //===============Traps and Exceptions================//
        .ecall_ex_ao        (ecall_ex),
        .ebreak_ex_ao       (ebreak_ex),
        .illegal_inst_ex_ao (illegal_inst_ex),
        .mret_ao            (mret),
        .wait_for_int_ao    (wait_for_int),
        .fencei_ao          (fencei),

        //============== Flow Control Outputs ===============//
        .branch_o           (EXE_branch),
        .target_addr_o      (EXE_pc_target_addr),

        .load_use_haz_ao    (load_use_haz),
        .alu_stall_o        (alu_stall),
        .mem_wr_ao          (EXE_wr_a),

        //================ Pipeline Outputs =================//
        .valid_o            (EXE_valid),
        .dmem_addr_o        (EXE_dmem_addr),
        .rs2_data_o         (EXE_rs2_data),
        // Destination Register (rd)
        .rd_data_o          (EXE_rd_data),
        .rd_idx_o           (EXE_rd_idx),
        .rd_wr_en_o         (EXE_rd_wr_en),
        .rd_wr_src_load_o   (EXE_rd_wr_src_load),
        // Load/Store
        .mem_width_1h_o     (EXE_mem_width_1h),
        .mem_rd_o           (EXE_rd),
        .mem_wr_o           (EXE_wr),
        .mem_sign_o         (EXE_sign),
        // Program Counter
        .pc_o               (EXE_pc)

`ifdef CPU64_RVFI
        ,
        .rvfi_insn_i          (DCD_rvfi_insn),
        .rvfi_trap_i          (DCD_rvfi_trap),
        .rvfi_pc_rdata_i      (DCD_rvfi_pc_rdata),
        .rvfi_pc_wdata_i      (DCD_rvfi_pc_wdata),

        .rvfi_insn_o          (EXE_rvfi_insn),
        .rvfi_trap_o          (EXE_rvfi_trap),
        .rvfi_rs1_addr_o      (EXE_rvfi_rs1_addr),
        .rvfi_rs2_addr_o      (EXE_rvfi_rs2_addr),
        .rvfi_rs1_rdata_o     (EXE_rvfi_rs1_rdata),
        .rvfi_rs2_rdata_o     (EXE_rvfi_rs2_rdata),
        .rvfi_pc_rdata_o      (EXE_rvfi_pc_rdata),
        .rvfi_pc_wdata_o      (EXE_rvfi_pc_wdata)
`endif
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                            Memory                                         //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    wire [VADDR-1:0] MEM_pc;
    wire             MEM_rd_wr_en, MEM_rd_wr_src_load;
    wire [3:0]       MEM_mem_width_1h;
    wire [4:0]       MEM_rd_idx;
    wire [`XLEN-1:0] MEM_rd_data;
    wire [2:0]       MEM_byte_addr;
    wire             MEM_sign;
    
`ifdef CPU64_RVFI
    wire                    MEM_rvfi_trap;
    wire [31:0]             MEM_rvfi_insn;
    wire [4:0]              MEM_rvfi_rs1_addr,  MEM_rvfi_rs2_addr;
    wire [`XLEN-1:0]        MEM_rvfi_rs1_rdata, MEM_rvfi_rs2_rdata;
    wire [`XLEN-1:0]        MEM_rvfi_pc_rdata,  MEM_rvfi_pc_wdata;
    wire [`XLEN-1:0]        MEM_rvfi_mem_addr;
    wire [`XLEN/8 - 1 : 0]  MEM_rvfi_mem_rmask, MEM_rvfi_mem_wmask;
    wire [`XLEN-1:0]        MEM_rvfi_mem_wdata;
`endif


    cpu64_memory_stage #(.VADDR(VADDR)) MEM (
        //======= Clocks, Resets, and Stage Controls ========//
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),

        .squash_i           (MEM_squash),
        .bubble_i           (MEM_bubble),
        .stall_i            (MEM_stall),

        //============== Execute Stage Inputs ===============//
        .valid_i            (EXE_valid),
        .pc_i               (EXE_pc),
        .dmem_full_addr_i   (EXE_dmem_addr),
        .rs2_data_i         (EXE_rs2_data),
        // Destination Register (rd)
        .rd_data_i          (EXE_rd_data),
        .rd_idx_i           (EXE_rd_idx),
        .rd_wr_en_i         (EXE_rd_wr_en),
        .rd_wr_src_load_i   (EXE_rd_wr_src_load),
        // Load/Store
        .mem_width_1h_i     (EXE_mem_width_1h),
        .mem_rd_i           (EXE_rd),
        .mem_wr_i           (EXE_wr),
        .mem_sign_i         (EXE_sign),
        
        //============== Data Memory Interface ==============//
        .dmem_req_o         (dmem_req_o),
        .dmem_gnt_i         (dmem_gnt_i),
        .dmem_addr_ao       (dmem_addr_ao),
        .dmem_we_ao         (dmem_we_ao),
        .dmem_be_ao         (dmem_be_ao),
        .dmem_wdata_ao      (dmem_wdata_ao),
        .dmem_rvalid_i      (dmem_rvalid_i),

        .dmem_stall_ao      (dmem_stall),
        .unalign_store_ex_ao(unalign_store_ex),
        .unalign_load_ex_ao (unalign_load_ex),

        //================ Pipeline Outputs =================//
        .valid_o            (MEM_valid),
        .pc_o               (MEM_pc),
        // Destination Register (rd)
        .rd_data_o          (MEM_rd_data),
        .rd_idx_o           (MEM_rd_idx),
        .rd_wr_en_o         (MEM_rd_wr_en),
        .rd_wr_src_load_o   (MEM_rd_wr_src_load),
        // Load/Store
        .mem_width_1h_o     (MEM_mem_width_1h),
        .mem_sign_o         (MEM_sign),
        .byte_addr_o        (MEM_byte_addr)

`ifdef CPU64_RVFI
        ,
        .rvfi_insn_i          (EXE_rvfi_insn),
        .rvfi_trap_i          (EXE_rvfi_trap),
        .rvfi_rs1_addr_i      (EXE_rvfi_rs1_addr),
        .rvfi_rs2_addr_i      (EXE_rvfi_rs2_addr),
        .rvfi_rs1_rdata_i     (EXE_rvfi_rs1_rdata),
        .rvfi_rs2_rdata_i     (EXE_rvfi_rs2_rdata),
        .rvfi_pc_rdata_i      (EXE_rvfi_pc_rdata),
        .rvfi_pc_wdata_i      (EXE_rvfi_pc_wdata),

        .rvfi_insn_o          (MEM_rvfi_insn),
        .rvfi_trap_o          (MEM_rvfi_trap),
        .rvfi_rs1_addr_o      (MEM_rvfi_rs1_addr),
        .rvfi_rs2_addr_o      (MEM_rvfi_rs2_addr),
        .rvfi_rs1_rdata_o     (MEM_rvfi_rs1_rdata),
        .rvfi_rs2_rdata_o     (MEM_rvfi_rs2_rdata),
        .rvfi_pc_rdata_o      (MEM_rvfi_pc_rdata),
        .rvfi_pc_wdata_o      (MEM_rvfi_pc_wdata),
        .rvfi_mem_addr_o      (MEM_rvfi_mem_addr),
        .rvfi_mem_rmask_o     (MEM_rvfi_mem_rmask),
        .rvfi_mem_wmask_o     (MEM_rvfi_mem_wmask),
        .rvfi_mem_wdata_o     (MEM_rvfi_mem_wdata)
`endif
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Writeback                                        //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cpu64_writeback_stage WB (
        //======= Clocks, Resets, and Stage Controls ========//
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),

        .squash_i           (WB_squash),
        .bubble_i           (WB_bubble),
        .stall_i            (WB_stall),

        //============= Memory Pipeline Inputs ==============//
        .valid_i            (MEM_valid),
        .pc_i               (MEM_pc),
        // Destination Register (rd)
        .rd_data_i          (MEM_rd_data),
        .rd_idx_i           (MEM_rd_idx),
        .rd_wr_en_i         (MEM_rd_wr_en),
        .rd_wr_src_load_i   (MEM_rd_wr_src_load),
        // Data Memory Load Inputs
        .dmem_rdata_i       (dmem_rdata_i),
        .mem_width_1h_i     (MEM_mem_width_1h),
        .mem_sign_i         (MEM_sign),
        .byte_addr_i        (MEM_byte_addr),

        //============= Register File Controls ==============//
        .rd_data_ao         (rd_data),
        .rd_idx_ao          (rd_idx),
        .rd_wr_en_ao        (rd_wr_en),

        .inst_retired_ao    (WB_inst_retired),
        .valid_ao           (WB_valid)

`ifdef CPU64_RVFI
        ,
        .rvfi_insn_i          (MEM_rvfi_insn),
        .rvfi_trap_i          (MEM_rvfi_trap),
        .rvfi_rs1_addr_i      (MEM_rvfi_rs1_addr),
        .rvfi_rs2_addr_i      (MEM_rvfi_rs2_addr),
        .rvfi_rs1_rdata_i     (MEM_rvfi_rs1_rdata),
        .rvfi_rs2_rdata_i     (MEM_rvfi_rs2_rdata),
        .rvfi_pc_rdata_i      (MEM_rvfi_pc_rdata),
        .rvfi_pc_wdata_i      (MEM_rvfi_pc_wdata),
        .rvfi_mem_addr_i      (MEM_rvfi_mem_addr),
        .rvfi_mem_rmask_i     (MEM_rvfi_mem_rmask),
        .rvfi_mem_wmask_i     (MEM_rvfi_mem_wmask),
        .rvfi_mem_wdata_i     (MEM_rvfi_mem_wdata),

        .rvfi_valid_o         (rvfi_valid),
        .rvfi_order_o         (rvfi_order),
        .rvfi_insn_o          (rvfi_insn),
        .rvfi_trap_o          (rvfi_trap),
        .rvfi_halt_o          (rvfi_halt),
        .rvfi_intr_o          (rvfi_intr),
        .rvfi_mode_o          (rvfi_mode),
        .rvfi_ixl_o           (rvfi_ixl),
        .rvfi_rs1_addr_o      (rvfi_rs1_addr),
        .rvfi_rs2_addr_o      (rvfi_rs2_addr),
        .rvfi_rs1_rdata_o     (rvfi_rs1_rdata),
        .rvfi_rs2_rdata_o     (rvfi_rs2_rdata),
        .rvfi_rd_addr_o       (rvfi_rd_addr),
        .rvfi_rd_wdata_o      (rvfi_rd_wdata),
        .rvfi_pc_rdata_o      (rvfi_pc_rdata),
        .rvfi_pc_wdata_o      (rvfi_pc_wdata),
        .rvfi_mem_addr_o      (rvfi_mem_addr),
        .rvfi_mem_rmask_o     (rvfi_mem_rmask),
        .rvfi_mem_wmask_o     (rvfi_mem_wmask),
        .rvfi_mem_rdata_o     (rvfi_mem_rdata),
        .rvfi_mem_wdata_o     (rvfi_mem_wdata)
`endif
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                ____                                             _                         //
    //               / ___|___  _ __ ___  _ __   ___  _ __   ___ _ __ | |_ ___                   //
    //              | |   / _ \| '_ ` _ \| '_ \ / _ \| '_ \ / _ \ '_ \| __/ __|                  //
    //              | |__| (_) | | | | | | |_) | (_) | | | |  __/ | | | |_\__ \                  //
    //               \____\___/|_| |_| |_| .__/ \___/|_| |_|\___|_| |_|\__|___/                  //
    //                                   |_|                                                     //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                    Pipeline Controller                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cpu64_pipeline_controller pipeline_controller_inst (
        //================= Clocks, Resets ==================//
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),

        //================= Hazard Inputs ===================//
        .imem_stall_i       (imem_stall),
        .dmem_stall_i       (dmem_stall),
        .load_use_haz_i     (load_use_haz),
        .csr_load_use_haz_i (csr_load_use_haz),
        .alu_stall_i        (alu_stall),
        .fencei_i           (fencei),
        .mem_exc_i          (unalign_store_ex || unalign_load_ex),
        .trap_i             (mret || ecall_ex || ebreak_ex || illegal_inst_ex),

        //============= Pipeline State Inputs ===============//
        .EXE_branch_i       (EXE_branch),
        .EXE_write_i        (EXE_wr_a),
        .MEM_write_i        (dmem_we_ao),
        .valid_inst_in_pipe_i(FCH_valid | DCD_valid | EXE_valid | MEM_valid | WB_valid),

        //============= Interrupt Controls ==================//
        .wait_for_int_i     (wait_for_int),
        .interrupt_i        (csr_interrupt),
        .interrupt_o        (pipeline_interrupt),

        //============= Stage Control Outputs ===============//
        .FCH_squash_o       (FCH_squash),
        .FCH_bubble_o       (FCH_bubble),
        .FCH_stall_o        (FCH_stall),

        .DCD_squash_o       (DCD_squash),
        .DCD_bubble_o       (DCD_bubble),
        .DCD_stall_o        (DCD_stall),

        .EXE_squash_o       (EXE_squash),
        .EXE_bubble_o       (EXE_bubble),
        .EXE_stall_o        (EXE_stall),

        .MEM_squash_o       (MEM_squash),
        .MEM_bubble_o       (MEM_bubble),
        .MEM_stall_o        (MEM_stall),

        .WB_squash_o        (WB_squash),
        .WB_bubble_o        (WB_bubble),
        .WB_stall_o         (WB_stall),

        .fencei_flush_ao    (fencei_flush)
    );

    assign fencei_flush_ao = fencei_flush;


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                       Register File                                       //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Gate register file write by writeback validity to prevent invalid instructions from writing
    wire rd_wr_en_gated = rd_wr_en && WB_valid;

    cpu64_register_file #(.XLEN(`XLEN)) i_register_file (
        .clk_i        (clk_i),

        .rs1_idx_i    (rs1_idx),
        .rs2_idx_i    (rs2_idx),

        .rd_idx_i     (rd_idx),
        .wr_data_i    (rd_data),
        .wr_en_i      (rd_wr_en_gated),  // Use gated write enable

        .rs1_data_ao  (rs1_data),
        .rs2_data_ao  (rs2_data) 
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                            Control and Status Registers (CSRs)                            //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    wire [VADDR-1:0] EXE_exc_pc = DCD_pc;
    wire [VADDR-1:0] MEM_exc_pc = EXE_pc;

    cpu64_csr #(.VADDR(VADDR)) i_csr (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),

        .rd_en_i            (csr_rd_en),
        .rd_addr_i          (csr_rd_addr),
        .rdata_o            (EXE_csr_rdata),
        .csr_rd_ex_ao       (csr_rd_ex),

        .wr_en_i            (EXE_csr_wr_en),
        .wr_addr_i          (csr_wr_addr),
        .wdata_i            (EXE_csr_wdata),
        .csr_wr_ex_ao       (csr_wr_ex),

        .unalign_load_ex_i  (unalign_load_ex),
        .unalign_store_ex_i (unalign_store_ex),
        .ecall_ex_i         (ecall_ex),
        .ebreak_ex_i        (ebreak_ex),
        .illegal_inst_ex_i  (illegal_inst_ex),

        .m_ext_inter_i      (m_ext_inter_i),
        .m_soft_inter_i     (m_soft_inter_i),
        .m_timer_inter_i    (m_timer_inter_i),

        .mret_i             (mret),
        .instr_retired_i    (WB_inst_retired),
        .time_i             (time_i),

        .EXE_exc_pc_i       (EXE_exc_pc),
        .MEM_exc_pc_i       (MEM_exc_pc),
        .load_store_bad_addr(EXE_dmem_addr),
        .csr_interrupt_ao   (csr_interrupt),
        .csr_branch_addr_o  (csr_branch_addr),

        .trap_ret_addr_o    (trap_ret_addr)
    );

    //===================== Debug/Trace Signal Assignments ==================//
    assign ecall_o = ecall_ex || ebreak_ex;  // Export ECALL/EBREAK for testbench halt
    assign WB_valid_o = WB_valid;    // Export WB valid for monitoring

endmodule
