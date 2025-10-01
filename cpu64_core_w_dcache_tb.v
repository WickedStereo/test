// cpu64_core_w_dcache_tb_stripped.v - Minimal functional testbench
`timescale 1ns/1ps

`include "cpu64_defs.vh"

module cpu64_core_w_dcache_tb_stripped(
    input clk,
    input rst_n
);

    //===================== Parameters ======================//
    localparam VADDR = 39;

    //===================== IMEM Signals ====================//
    wire                 imem_req_o;
    wire [VADDR-1:0]     imem_addr_ao;
    wire                 imem_gnt_i;
    reg                  imem_rvalid_i;
    reg  [31:0]          imem_rdata_i;

    //===================== DMEM Signals ====================//
    wire                 dmem_req_o;
    wire                 dmem_we_ao;
    wire [7:0]           dmem_be_ao;
    wire [VADDR-1:0]     dmem_addr_ao;
    wire [`XLEN-1:0]     dmem_wdata_ao;
    wire                 dmem_rvalid_i;
    wire [`XLEN-1:0]     dmem_rdata_i;
    wire                 dmem_gnt_i;

    //===================== External Signals ================//
    reg                  m_ext_inter_i;
    reg                  m_soft_inter_i;
    reg                  m_timer_inter_i;
    reg  [`XLEN-1:0]     time_i;
    wire                 fencei_flush_ao;

    //===================== DUT =============================//
    cpu64_core_w_dcache #(.VADDR(VADDR), .RESET_ADDR(0)) dut (
        .clk_i                 (clk),
        .rst_ni                (rst_n),
        .m_ext_inter_i         (m_ext_inter_i),
        .m_soft_inter_i        (m_soft_inter_i),
        .m_timer_inter_i       (m_timer_inter_i),
        .time_i                (time_i),
        .fencei_flush_ao       (fencei_flush_ao),
        // IMEM
        .imem_req_o            (imem_req_o),
        .imem_gnt_i            (imem_gnt_i),
        .imem_addr_ao          (imem_addr_ao),
        .imem_rvalid_i         (imem_rvalid_i),
        .imem_rdata_i          (imem_rdata_i),
        // DMEM (external behind cache stack)
        .dmem_req_o            (dmem_req_o),
        .dmem_gnt_i            (dmem_gnt_i),
        .dmem_addr_ao          (dmem_addr_ao),
        .dmem_we_ao            (dmem_we_ao),
        .dmem_be_ao            (dmem_be_ao),
        .dmem_wdata_ao         (dmem_wdata_ao),
        .dmem_rvalid_i         (dmem_rvalid_i),
        .dmem_rdata_i          (dmem_rdata_i),
        // Cache maintenance
        .dcache_invalidate_all_i(1'b0)
    );

    //===================== IMEM Model ======================//

    // Simple OBI-like instruction memory model with hex loader
    // Always-accepting grant with single outstanding request, fixed read latency
    localparam integer IMEM_BYTES = (1<<20);
    localparam integer IMEM_WORDS = IMEM_BYTES / 4;
    localparam integer IMEM_READ_LATENCY = 2;

    // 32-bit word-addressable instruction memory
    reg [31:0] instruction_memory [0:IMEM_WORDS-1];

    // Track last accepted address for debug/program-finish detection
    reg [VADDR-1:0] imem_last_addr;

    // Accept at most one outstanding read; grant only when not pending
    reg imem_pending;
    reg [7:0] imem_delay_cnt;

    // Grant is combinational based on pending status
    assign imem_gnt_i = imem_req_o && !imem_pending;

    // Optional: widen address for indexing (zero-extend virtual address)
    wire [63:0] imem_addr_64 = { {(64-VADDR){1'b0}}, imem_addr_ao };

    // Manually load store_demo.hex program
    initial begin
        integer i;
        // Initialize all memory with NOPs
        for (i = 0; i < IMEM_WORDS; i = i + 1) begin
            instruction_memory[i] = 32'h00000013; // NOP (addi x0,x0,0)
        end
        
        // Load store_demo.hex program manually
        instruction_memory[0]  = 32'h00010537; // lui  a0, 0x10          ; a0 = 0x0000000000010000
        instruction_memory[1]  = 32'h00053283; // ld   x5,  0(a0)        ; x5  = [0x10000] = DEADBEEF_DEADBEEF
        instruction_memory[2]  = 32'h00853303; // ld   x6,  8(a0)        ; x6  = [0x10008] = CAFEBABE_CAFEBABE
        instruction_memory[3]  = 32'h01053683; // ld   x13, 16(a0)       ; x13 = [0x10010] = 11111111_11111111
        instruction_memory[4]  = 32'h01853703; // ld   x14, 24(a0)       ; x14 = [0x10018] = 22222222_22222222
        instruction_memory[5]  = 32'h000205B7; // lui  a1, 0x20          ; a1 = 0x0000000000020000
        instruction_memory[6]  = 32'h0005B383; // ld   x7,  0(a1)        ; x7  = [0x20000] = 12345678_12345678
        instruction_memory[7]  = 32'h0085B403; // ld   x8,  8(a1)        ; x8  = [0x20008] = 87654321_87654321
        instruction_memory[8]  = 32'h00030637; // lui  a2, 0x30          ; a2 = 0x0000000000030000
        instruction_memory[9]  = 32'h00063483; // ld   x9,  0(a2)        ; x9  = [0x30000] = AAAAAAAAAAAAAAAA
        instruction_memory[10] = 32'h00863503; // ld   x10, 8(a2)        ; x10 = [0x30008] = BBBBBBBBBBBBBBBB
        instruction_memory[11] = 32'h00100073; // ebreak
    end

    // IMEM read response timing
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imem_rvalid_i   <= 1'b0;
            imem_rdata_i    <= 32'd0;
            imem_last_addr  <= {VADDR{1'b0}};
            imem_pending    <= 1'b0;
            imem_delay_cnt  <= 8'd0;
        end else begin
            // Default deassert rvalid unless we hit response point this cycle
            imem_rvalid_i <= 1'b0;

            // Service outstanding request first
            if (imem_pending) begin
                if (imem_delay_cnt != 0) begin
                    imem_delay_cnt <= imem_delay_cnt - 1'b1;
                end else begin
                    // Issue read data
                    imem_rdata_i  <= instruction_memory[imem_last_addr[31:2]];
                    imem_rvalid_i <= 1'b1;
                    imem_pending  <= 1'b0;
                end
            end
            
            // Accept a new request ONLY if no request is pending
            // This prevents overwriting imem_last_addr while a request is in flight
            if (imem_req_o && imem_gnt_i && !imem_pending) begin
                imem_last_addr <= imem_addr_ao;
                imem_pending   <= 1'b1;
                imem_delay_cnt <= IMEM_READ_LATENCY[7:0];
            end
        end
    end
    //===================== DMEM Model ======================//
    wire [63:0] dmem_addr_64 = { {(64-VADDR){1'b0}}, dmem_addr_ao };

    obi_mem_model_adv #(
        .MEM_BYTES    (1<<20),
        .READ_LATENCY (2),
        .WRITE_ACCEPT (1),
        .STALL_PATTERN(16'h0000)
    ) i_dmem_model (
        .clk_i    (clk),
        .rst_ni   (rst_n),
        .req_i    (dmem_req_o),
        .we_i     (dmem_we_ao),
        .be_i     (dmem_be_ao),
        .addr_i   (dmem_addr_64),
        .wdata_i  (dmem_wdata_ao),
        .gnt_o    (dmem_gnt_i),
        .rvalid_o (dmem_rvalid_i),
        .rdata_o  (dmem_rdata_i)
    );


endmodule


