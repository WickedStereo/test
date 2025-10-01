// Simple OBI-like memory model with byte enables, latency, and optional stalls
`timescale 1ns/1ps

module obi_mem_model_adv #(
    parameter integer MEM_BYTES     = (1<<20),
    parameter integer READ_LATENCY  = 2,
    parameter integer WRITE_ACCEPT  = 1,
    parameter [15:0] STALL_PATTERN  = 16'h0000
)(
    input  wire         clk_i,
    input  wire         rst_ni,

    input  wire         req_i,
    input  wire         we_i,
    input  wire  [7:0]  be_i,
    input  wire [63:0]  addr_i,
    input  wire [63:0]  wdata_i,
    output wire         gnt_o,
    output reg          rvalid_o,
    output reg  [63:0]  rdata_o
);

    // QUIET plusarg to suppress verbose prints from memory model
    reg quiet_mode;
    initial begin
        quiet_mode = 1'b0;
        void'($value$plusargs("QUIET=%d", quiet_mode));
    end

    localparam integer MEM_WORDS = MEM_BYTES/8;
    reg [63:0] mem [0:MEM_WORDS-1];

    // Pre-initialize memory with known test patterns
    initial begin
        integer i;
        // Initialize all memory to zero first
        for (i = 0; i < MEM_WORDS; i = i + 1) begin
            mem[i] = 64'd0;
        end
        
        // Pre-load specific test addresses with known patterns
        // Address 0x10000 (word index 0x2000) = 0xDEADBEEF_DEADBEEF
        mem[20'h02000] = 64'hDEADBEEF_DEADBEEF;
        
        // Address 0x10008 (word index 0x2001) = 0xCAFEBABE_CAFEBABE
        mem[20'h02001] = 64'hCAFEBABE_CAFEBABE;
        
        // Address 0x10010 (word index 0x2002) = 0x11111111_11111111
        mem[20'h02002] = 64'h11111111_11111111;
        
        // Address 0x10018 (word index 0x2003) = 0x22222222_22222222
        mem[20'h02003] = 64'h22222222_22222222;
        
        // Address 0x20000 (word index 0x4000) = 0x12345678_12345678
        mem[20'h04000] = 64'h12345678_12345678;
        
        // Address 0x20008 (word index 0x4001) = 0x87654321_87654321
        mem[20'h04001] = 64'h87654321_87654321;
        
        // Address 0x30000 (word index 0x6000) = 0xAAAAAAAA_AAAAAAAA
        mem[20'h06000] = 64'hAAAAAAAA_AAAAAAAA;
        
        // Address 0x30008 (word index 0x6001) = 0xBBBBBBBB_BBBBBBBB
        mem[20'h06001] = 64'hBBBBBBBB_BBBBBBBB;
        
        if (!quiet_mode) begin
            $display("DMEM: Pre-initialized with test patterns:");
            $display("  [0x10000] = 0x%016h", mem[20'h02000]);
            $display("  [0x10008] = 0x%016h", mem[20'h02001]);
            $display("  [0x20000] = 0x%016h", mem[20'h04000]);
            $display("  [0x30000] = 0x%016h", mem[20'h06000]);
        end
    end

    // Simple grant: accept if allowed by stall pattern
    wire stall = STALL_PATTERN[addr_i[5:2]]; // small pseudo-stall by low addr bits
    assign gnt_o = req_i && (!stall);

    reg        pending;
    reg [7:0]  delay_cnt;
    reg [63:0] latched_addr;
    reg [7:0]  latched_be;
    reg [63:0] latched_wdata;
    reg        latched_we;

    integer b;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rvalid_o     <= 1'b0;
            rdata_o      <= 64'd0;
            pending      <= 1'b0;
            delay_cnt    <= 8'd0;
            latched_addr <= 64'd0;
            latched_be   <= 8'd0;
            latched_wdata<= 64'd0;
            latched_we   <= 1'b0;
        end else begin
            rvalid_o <= 1'b0;

            // Accept a new transaction
            if (req_i && gnt_o && !pending) begin
                latched_addr  <= addr_i;
                latched_we    <= we_i;
                latched_be    <= be_i;
                latched_wdata <= wdata_i;
                pending       <= 1'b1;
                delay_cnt     <= READ_LATENCY[7:0];
            end

            if (pending) begin
                if (delay_cnt != 0) begin
                    delay_cnt <= delay_cnt - 8'd1;
                end else begin
                    if (latched_we) begin
                        // Perform write with byte enables
                        reg [63:0] tmp;
                        tmp = mem[latched_addr[63:3]];
                        for (b = 0; b < 8; b = b + 1) begin
                            if (latched_be[b]) begin
                                tmp[8*b +: 8] = latched_wdata[8*b +: 8];
                            end
                        end
                        mem[latched_addr[63:3]] <= tmp;
                        // For writes, no rvalid_o asserted
                        rvalid_o <= 1'b0;
                    end else begin
                        // Read
                        rdata_o  <= mem[latched_addr[63:3]];
                        rvalid_o <= 1'b1;
                    end
                    pending <= 1'b0;
                end
            end
        end
    end

endmodule


