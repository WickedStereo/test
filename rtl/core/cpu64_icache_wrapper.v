// `include "cpu64_fetch_stage.v"
// `include "cpu64_icache_top.v"


// `timescale 1ns/1ps
// `define 

//`timescale 1ns/1ps
`ifndef CPU64_IFETCH_WRAPPER_V
`define CPU64_IFETCH_WRAPPER_V
/* verilator lint_off MULTITOP */
// ============================================================================
// cpu64_ifetch_wrapper.v  (Verilog-2001)
// - Connects icache instr_is_16_o → fetch_stage.ret_is16_i.
// - No speculative compressed flag.
// ============================================================================

module cpu64_icache_wrapper
#(
  parameter VADDR      = 39,
  parameter RESET_ADDR = 0
)(
  input                   clk_i,
  input                   rst_ni,
  input                   squash_i,
  input                   bubble_i,
  input                   stall_i,
  input                   fencei_i,
  input                   branch_i,
  input      [VADDR-1:0]  target_addr_i,
  input                   csr_branch_i,
  input      [VADDR-1:0]  csr_branch_addr_i,
  input                   trap_ret_i,
  input      [VADDR-1:0]  trap_ret_addr_i,
  output                  instr_valid_o,
  output     [31:0]       instr_o,
  output                  instr_is_16_o,
  output                  valid_o,
  output     [VADDR-1:0]  pc_o,
  output     [VADDR-1:0]  next_pc_o,
  output                  mem_req_o,
  output                  mem_we_o,
  output      [7:0]       mem_be_o,
  output     [63:0]       mem_addr_o,
  output     [63:0]       mem_wdata_o,
  input                   mem_gnt_i,
  input                   mem_rvalid_i,
  input      [63:0]       mem_rdata_i
);

  wire                    imem_req_w;
  wire [VADDR-1:0]        imem_addr_w;
  wire                    imem_gnt_w;
  wire                    imem_rvalid_w;
  wire [31:0]             imem_rdata_w;

  wire                    ic_fetch_gnt_w;
  wire                    ic_instr_valid_w;
  wire [31:0]             ic_instr_w;
  wire                    ic_is16_w;

  cpu64_fetch_stage #(
    .VADDR      (VADDR),
    .RESET_ADDR (RESET_ADDR)
  ) u_fetch (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .squash_i           (squash_i),
    .bubble_i           (bubble_i),
    .stall_i            (stall_i),
    .fencei_i           (fencei_i),
    .ret_is16_i         (ic_is16_w), // returned-length feed
  .compressed_instr_i (1'b0),
    .branch_i           (branch_i),
    .target_addr_i      (target_addr_i),
    .csr_branch_i       (csr_branch_i),
    .csr_branch_addr_i  (csr_branch_addr_i),
    .trap_ret_i         (trap_ret_i),
    .trap_ret_addr_i    (trap_ret_addr_i),
    .imem_req_o         (imem_req_w),
    .imem_addr_ao       (imem_addr_w),
    .imem_gnt_i         (imem_gnt_w),
    .imem_rvalid_i      (imem_rvalid_w),
    .imem_rdata_i       (imem_rdata_w),
    .imem_stall_ao      (/*unused*/),
    .valid_o            (valid_o),
    .pc_o               (pc_o),
    .next_pc_o          (next_pc_o),
    .inst_o             (instr_o)
  );

  cpu64_icache_top u_icache (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .invalidate_all_i   (fencei_i),
    .fetch_req_i        (imem_req_w),
    .fetch_pc_i         ( { {(64-VADDR){1'b0}}, imem_addr_w } ),
    .fetch_gnt_o        (ic_fetch_gnt_w),
    .instr_valid_o      (ic_instr_valid_w),
    .instr_o            (ic_instr_w),
    .instr_is_16_o      (ic_is16_w),
    .req_o              (mem_req_o),
    .we_o               (mem_we_o),
    .be_o               (mem_be_o),
    .addr_o             (mem_addr_o),
    .wdata_o            (mem_wdata_o),
    .gnt_i              (mem_gnt_i),
    .rvalid_i           (mem_rvalid_i),
    .rdata_i            (mem_rdata_i)
  );

  assign imem_gnt_w    = ic_fetch_gnt_w;
  assign imem_rvalid_w = ic_instr_valid_w;
  assign imem_rdata_w  = ic_instr_w;

  assign instr_valid_o = ic_instr_valid_w;
  assign instr_is_16_o = ic_is16_w;

endmodule
/* verilator lint_on MULTITOP */
`endif
