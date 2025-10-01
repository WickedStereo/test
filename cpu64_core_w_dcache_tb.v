// cpu64_core_w_dcache_tb_stripped.v - Minimal functional testbench
`timescale 1ns/1ps

`include "cpu64_defs.vh"

module cpu64_core_w_dcache_tb(
    input clk,
    input rst_n,
    // DUT interface ports
    input                  m_ext_inter_i,
    input                  m_soft_inter_i,
    input                  m_timer_inter_i,
    input  [`XLEN-1:0]     time_i,
    output                 fencei_flush_ao,
    // IMEM interface
    output                 imem_req_o,
    output [VADDR-1:0]     imem_addr_ao,
    input                  imem_gnt_i,
    input                  imem_rvalid_i,
    input  [31:0]          imem_rdata_i,
    // DMEM interface
    output                 dmem_req_o,
    output                 dmem_we_ao,
    output [7:0]           dmem_be_ao,
    output [VADDR-1:0]     dmem_addr_ao,
    output [`XLEN-1:0]     dmem_wdata_ao,
    input                  dmem_rvalid_i,
    input  [`XLEN-1:0]     dmem_rdata_i,
    input                  dmem_gnt_i
);

    //===================== Parameters ======================//
    localparam VADDR = 39;

    //===================== Internal Signals ====================//
    reg                  imem_rvalid_i;
    reg  [31:0]          imem_rdata_i;


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

    // Inline OBI memory model implementation
    localparam integer DMEM_BYTES = (1<<20);
    localparam integer DMEM_WORDS = DMEM_BYTES/8;
    localparam integer DMEM_READ_LATENCY = 2;
    localparam [15:0] DMEM_STALL_PATTERN = 16'h0000;
    
    reg [63:0] dmem_memory [0:DMEM_WORDS-1];
    
    // Pre-initialize memory with known test patterns
    initial begin
        integer i;
        // Initialize all memory to zero first
        for (i = 0; i < DMEM_WORDS; i = i + 1) begin
            dmem_memory[i] = 64'd0;
        end
        
        // Pre-load specific test addresses with known patterns
        // Address 0x10000 (word index 0x2000) = 0xDEADBEEF_DEADBEEF
        dmem_memory[20'h02000] = 64'hDEADBEEF_DEADBEEF;
        
        // Address 0x10008 (word index 0x2001) = 0xCAFEBABE_CAFEBABE
        dmem_memory[20'h02001] = 64'hCAFEBABE_CAFEBABE;
        
        // Address 0x10010 (word index 0x2002) = 0x11111111_11111111
        dmem_memory[20'h02002] = 64'h11111111_11111111;
        
        // Address 0x10018 (word index 0x2003) = 0x22222222_22222222
        dmem_memory[20'h02003] = 64'h22222222_22222222;
        
        // Address 0x20000 (word index 0x4000) = 0x12345678_12345678
        dmem_memory[20'h04000] = 64'h12345678_12345678;
        
        // Address 0x20008 (word index 0x4001) = 0x87654321_87654321
        dmem_memory[20'h04001] = 64'h87654321_87654321;
        
        // Address 0x30000 (word index 0x6000) = 0xAAAAAAAA_AAAAAAAA
        dmem_memory[20'h06000] = 64'hAAAAAAAA_AAAAAAAA;
        
        // Address 0x30008 (word index 0x6001) = 0xBBBBBBBB_BBBBBBBB
        dmem_memory[20'h06001] = 64'hBBBBBBBB_BBBBBBBB;
    end

    // Simple grant: accept if allowed by stall pattern
    wire dmem_stall = DMEM_STALL_PATTERN[dmem_addr_64[5:2]]; // small pseudo-stall by low addr bits
    assign dmem_gnt_i = dmem_req_o && (!dmem_stall);

    reg        dmem_pending;
    reg [7:0]  dmem_delay_cnt;
    reg [63:0] dmem_latched_addr;
    reg [7:0]  dmem_latched_be;
    reg [63:0] dmem_latched_wdata;
    reg        dmem_latched_we;

    integer dmem_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmem_rvalid_i     <= 1'b0;
            dmem_rdata_i      <= 64'd0;
            dmem_pending      <= 1'b0;
            dmem_delay_cnt    <= 8'd0;
            dmem_latched_addr <= 64'd0;
            dmem_latched_be   <= 8'd0;
            dmem_latched_wdata<= 64'd0;
            dmem_latched_we   <= 1'b0;
        end else begin
            dmem_rvalid_i <= 1'b0;

            // Accept a new transaction
            if (dmem_req_o && dmem_gnt_i && !dmem_pending) begin
                dmem_latched_addr  <= dmem_addr_64;
                dmem_latched_we    <= dmem_we_ao;
                dmem_latched_be    <= dmem_be_ao;
                dmem_latched_wdata <= dmem_wdata_ao;
                dmem_pending       <= 1'b1;
                dmem_delay_cnt     <= DMEM_READ_LATENCY[7:0];
            end

            if (dmem_pending) begin
                if (dmem_delay_cnt != 0) begin
                    dmem_delay_cnt <= dmem_delay_cnt - 8'd1;
                end else begin
                    if (dmem_latched_we) begin
                        // Perform write with byte enables
                        reg [63:0] dmem_tmp;
                        dmem_tmp = dmem_memory[dmem_latched_addr[63:3]];
                        for (dmem_b = 0; dmem_b < 8; dmem_b = dmem_b + 1) begin
                            if (dmem_latched_be[dmem_b]) begin
                                dmem_tmp[8*dmem_b +: 8] = dmem_latched_wdata[8*dmem_b +: 8];
                            end
                        end
                        dmem_memory[dmem_latched_addr[63:3]] <= dmem_tmp;
                        // For writes, no rvalid_o asserted
                        dmem_rvalid_i <= 1'b0;
                    end else begin
                        // Read
                        dmem_rdata_i  <= dmem_memory[dmem_latched_addr[63:3]];
                        dmem_rvalid_i <= 1'b1;
                    end
                    dmem_pending <= 1'b0;
                end
            end
        end
    end



endmodule
