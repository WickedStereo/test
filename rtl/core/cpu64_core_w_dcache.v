// cpu64_core_w_dcache.v - Wrapper: cpu64_core + OBI receiver + cache stack
// `timescale 1ns/1ps

// `include "cpu64_defs.vh"

module cpu64_core_w_dcache #(parameter VADDR = 39, parameter RESET_ADDR = 0) (
    //===================== Clock and Reset =====================//
    input                   clk_i,
    input                   rst_ni,

    //================== Machine-Mode Interrupts ================//
    input                   m_ext_inter_i,
    input                   m_soft_inter_i,
    input                   m_timer_inter_i,

    //===================== Time Input ==========================//
    input [`XLEN-1:0]       time_i,

    //===================== Fence.i Output ======================//
    output                  fencei_flush_ao,

    //========== Instruction Memory Interface (OBI) =============//
    output                  imem_req_o,
    input                   imem_gnt_i,
    output [VADDR-1:0]      imem_addr_ao,
    input                   imem_rvalid_i,
    input        [31:0]     imem_rdata_i,

    //============ Data Memory Interface to External Mem ========//
    // These ports are the external memory interface behind the cache stack
    output                  dmem_req_o,
    input                   dmem_gnt_i,
    output [VADDR-1:0]      dmem_addr_ao,
    output                  dmem_we_ao,
    output  [7:0]           dmem_be_ao,
    output [`XLEN-1:0]      dmem_wdata_ao,
    input                   dmem_rvalid_i,
    input       [`XLEN-1:0] dmem_rdata_i,

    //===================== Cache Maintenance ===================//
    input                   dcache_invalidate_all_i
);

    //===================== Core Instance =======================//
    wire             core_dmem_req;
    wire [VADDR-1:0] core_dmem_addr;
    wire             core_dmem_we;
    wire  [7:0]      core_dmem_be;
    wire [`XLEN-1:0] core_dmem_wdata;
    wire             core_dmem_gnt;
    wire             core_dmem_rvalid;
    wire [`XLEN-1:0] core_dmem_rdata;

    cpu64_core #(.VADDR(VADDR), .RESET_ADDR(RESET_ADDR)) u_core (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .m_ext_inter_i      (m_ext_inter_i),
        .m_soft_inter_i     (m_soft_inter_i),
        .m_timer_inter_i    (m_timer_inter_i),
        .time_i             (time_i),
        .fencei_flush_ao    (fencei_flush_ao),
        // IMEM
        .imem_req_o         (imem_req_o),
        .imem_gnt_i         (imem_gnt_i),
        .imem_addr_ao       (imem_addr_ao),
        .imem_rvalid_i      (imem_rvalid_i),
        .imem_rdata_i       (imem_rdata_i),
        // DMEM (to receiver)
        .dmem_req_o         (core_dmem_req),
        .dmem_gnt_i         (core_dmem_gnt),
        .dmem_addr_ao       (core_dmem_addr),
        .dmem_we_ao         (core_dmem_we),
        .dmem_be_ao         (core_dmem_be),
        .dmem_wdata_ao      (core_dmem_wdata),
        .dmem_rvalid_i      (core_dmem_rvalid),
        .dmem_rdata_i       (core_dmem_rdata)
    );

    //===================== OBI Receiver ========================//
    wire             l1_req;
    wire             l1_we;
    wire [7:0]       l1_be;
    wire [63:0]      l1_addr;
    wire [63:0]      l1_wdata;
    wire             l1_gnt;
    wire             l1_rvalid;
    wire [63:0]      l1_rdata;

    cpu64_obi_receiver #(.CORE_ADDR_W(VADDR), .DATA_W(`XLEN)) u_obi_recv (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        // Core-side
        .req_i        (core_dmem_req),
        .we_i         (core_dmem_we),
        .be_i         (core_dmem_be),
        .addr_i       (core_dmem_addr),
        .wdata_i      (core_dmem_wdata),
        .gnt_o        (core_dmem_gnt),
        .rvalid_o     (core_dmem_rvalid),
        .rdata_o      (core_dmem_rdata),
        // L1-side
        .l1_req_o     (l1_req),
        .l1_we_o      (l1_we),
        .l1_be_o      (l1_be),
        .l1_addr_o    (l1_addr),
        .l1_wdata_o   (l1_wdata),
        .l1_gnt_i     (l1_gnt),
        .l1_rvalid_i  (l1_rvalid),
        .l1_rdata_i   (l1_rdata)
    );

    //===================== Cache Stack =========================//
    wire             c_req;
    wire             c_we;
    wire [7:0]       c_be;
    wire [63:0]      c_addr;
    wire [63:0]      c_wdata;
    wire             c_gnt;
    wire             c_rvalid;
    wire [63:0]      c_rdata;

    cpu64_cache_stack u_cache_stack (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .invalidate_all_i   (dcache_invalidate_all_i),
        // CPU ↔ L1 interface
        .req_i              (l1_req),
        .we_i               (l1_we),
        .be_i               (l1_be),
        .addr_i             (l1_addr),
        .wdata_i            (l1_wdata),
        .gnt_o              (l1_gnt),
        .rvalid_o           (l1_rvalid),
        .rdata_o            (l1_rdata),
        // L3 ↔ External memory
        .req_o              (c_req),
        .we_o               (c_we),
        .be_o               (c_be),
        .addr_o             (c_addr),
        .wdata_o            (c_wdata),
        .gnt_i              (dmem_gnt_i),
        .rvalid_i           (dmem_rvalid_i),
        .rdata_i            (dmem_rdata_i)
    );

    //===================== External Mem Ports ==================//
    assign dmem_req_o    = c_req;
    assign dmem_we_ao    = c_we;
    assign dmem_be_ao    = c_be;
    assign dmem_wdata_ao = c_wdata;
    assign dmem_addr_ao  = c_addr[VADDR-1:0];

endmodule


