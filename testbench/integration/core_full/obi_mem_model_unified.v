// Unified OBI memory model supporting both instruction and data memory
// Provides separate address spaces for Harvard architecture simulation
// `timescale 1ns/1ps

module obi_mem_model_unified #(
    parameter integer IMEM_BYTES     = (1<<20),     // 1MB instruction memory
    parameter integer DMEM_BYTES     = (1<<20),     // 1MB data memory
    parameter integer IMEM_READ_LATENCY  = 8,       // ICache refill latency (8 beats)
    parameter integer DMEM_READ_LATENCY  = 2,       // DCache latency
    parameter integer WRITE_ACCEPT   = 1
)(
    input  wire         clk_i,
    input  wire         rst_ni,

    // Instruction Memory Port
    input  wire         imem_req_i,
    input  wire         imem_we_i,
    input  wire  [7:0]  imem_be_i,
    input  wire [63:0]  imem_addr_i,
    input  wire [63:0]  imem_wdata_i,
    output wire         imem_gnt_o,
    output reg          imem_rvalid_o,
    output reg  [63:0]  imem_rdata_o,

    // Data Memory Port
    input  wire         dmem_req_i,
    input  wire         dmem_we_i,
    input  wire  [7:0]  dmem_be_i,
    input  wire [63:0]  dmem_addr_i,
    input  wire [63:0]  dmem_wdata_i,
    output wire         dmem_gnt_o,
    output reg          dmem_rvalid_o,
    output reg  [63:0]  dmem_rdata_o
);

    // QUIET plusarg to suppress verbose prints
    reg quiet_mode;
    initial begin
        quiet_mode = 1'b0;
        void'($value$plusargs("QUIET=%d", quiet_mode));
    end

    // Separate memory arrays for instruction and data
    localparam integer IMEM_WORDS = IMEM_BYTES/8;
    localparam integer DMEM_WORDS = DMEM_BYTES/8;
    
    reg [63:0] imem [0:IMEM_WORDS-1];
    reg [63:0] dmem [0:DMEM_WORDS-1];

    // Initialize memories
    initial begin
        integer i;
        // Initialize instruction memory to NOPs (addi x0,x0,0 = 0x00000013)
        for (i = 0; i < IMEM_WORDS; i = i + 1) begin
            imem[i] = {32'h00000013, 32'h00000013};  // Two NOPs per 64-bit word
        end
        
        // Initialize data memory to zeros
        for (i = 0; i < DMEM_WORDS; i = i + 1) begin
            dmem[i] = 64'd0;
        end
        
        // Pre-load data memory with test patterns
        dmem[20'h02000] = 64'hDEADBEEF_DEADBEEF;  // 0x10000
        dmem[20'h02001] = 64'hCAFEBABE_CAFEBABE;  // 0x10008
        dmem[20'h04000] = 64'h12345678_12345678;  // 0x20000
        dmem[20'h06000] = 64'hAAAAAAAA_AAAAAAAA;  // 0x30000
    end

    // ===== Instruction Memory Logic (Burst Mode) =====
    // ICache requests 64-byte cache lines (8 beats of 8 bytes each)
    assign imem_gnt_o = imem_req_i;  // Always grant (for simplicity)

    reg        imem_pending;
    reg [7:0]  imem_delay_cnt;
    reg [63:0] imem_latched_addr;
    reg [2:0]  imem_beat_cnt;  // 0-7 for 8 beats

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            imem_rvalid_o    <= 1'b0;
            imem_rdata_o     <= 64'd0;
            imem_pending     <= 1'b0;
            imem_delay_cnt   <= 8'd0;
            imem_latched_addr<= 64'd0;
            imem_beat_cnt    <= 3'd0;
        end else begin
            imem_rvalid_o <= 1'b0;

            // Accept new IMEM request - start burst
            if (imem_req_i && imem_gnt_o && !imem_pending) begin
                imem_latched_addr <= imem_addr_i;
                imem_delay_cnt    <= IMEM_READ_LATENCY[7:0];
                imem_pending      <= 1'b1;
                imem_beat_cnt     <= 3'd0;
                
                if (!quiet_mode) begin
                    $display("[%0t] IMEM: Accept burst read addr=0x%h", $time, imem_addr_i);
                end
            end

            // Process pending burst read
            if (imem_pending) begin
                if (imem_delay_cnt > 8'd0) begin
                    imem_delay_cnt <= imem_delay_cnt - 8'd1;
                end else begin
                    // Return current beat
                    imem_rdata_o  <= imem[(imem_latched_addr[19:3]) + {17'd0, imem_beat_cnt}];
                    imem_rvalid_o <= 1'b1;
                    
                    if (!quiet_mode) begin
                        $display("[%0t] IMEM: Return beat %0d data=0x%h for addr=0x%h",
                                 $time, imem_beat_cnt,
                                 imem[(imem_latched_addr[19:3]) + {17'd0, imem_beat_cnt}],
                                 imem_latched_addr + {61'd0, imem_beat_cnt, 3'd0});
                    end
                    
                    // Check if burst complete
                    if (imem_beat_cnt == 3'd7) begin
                        imem_pending <= 1'b0;  // Burst done
                    end else begin
                        imem_beat_cnt <= imem_beat_cnt + 3'd1;
                        imem_delay_cnt <= 8'd1;  // 1-cycle delay between beats
                    end
                end
            end
        end
    end

    // ===== Data Memory Logic =====
    assign dmem_gnt_o = dmem_req_i;  // Always grant

    reg        dmem_pending;
    reg [7:0]  dmem_delay_cnt;
    reg [63:0] dmem_latched_addr;
    reg [7:0]  dmem_latched_be;
    reg [63:0] dmem_latched_wdata;
    reg        dmem_latched_we;

    integer b;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            dmem_rvalid_o     <= 1'b0;
            dmem_rdata_o      <= 64'd0;
            dmem_pending      <= 1'b0;
            dmem_delay_cnt    <= 8'd0;
            dmem_latched_addr <= 64'd0;
            dmem_latched_be   <= 8'd0;
            dmem_latched_wdata<= 64'd0;
            dmem_latched_we   <= 1'b0;
        end else begin
            dmem_rvalid_o <= 1'b0;

            // Accept new DMEM request
            if (dmem_req_i && dmem_gnt_o && !dmem_pending) begin
                dmem_latched_addr  <= dmem_addr_i;
                dmem_latched_be    <= dmem_be_i;
                dmem_latched_wdata <= dmem_wdata_i;
                dmem_latched_we    <= dmem_we_i;
                dmem_delay_cnt     <= dmem_we_i ? WRITE_ACCEPT[7:0] : DMEM_READ_LATENCY[7:0];
                dmem_pending       <= 1'b1;
                
                if (!quiet_mode) begin
                    $display("[%0t] DMEM: Accept %s addr=0x%h data=0x%h be=0x%h",
                             $time, dmem_we_i ? "write" : "read",
                             dmem_addr_i, dmem_wdata_i, dmem_be_i);
                end
            end

            // Process pending operation
            if (dmem_pending) begin
                if (dmem_delay_cnt > 8'd0) begin
                    dmem_delay_cnt <= dmem_delay_cnt - 8'd1;
                end else begin
                    if (dmem_latched_we) begin
                        // Write operation
                        for (b = 0; b < 8; b = b + 1) begin
                            if (dmem_latched_be[b]) begin
                                dmem[(dmem_latched_addr[19:3])][b*8 +: 8] <= dmem_latched_wdata[b*8 +: 8];
                            end
                        end
                        
                        if (!quiet_mode) begin
                            $display("[%0t] DMEM: Write complete addr=0x%h data=0x%h",
                                     $time, dmem_latched_addr, dmem_latched_wdata);
                        end
                    end else begin
                        // Read operation
                        dmem_rdata_o  <= dmem[(dmem_latched_addr[19:3])];
                        dmem_rvalid_o <= 1'b1;
                        
                        if (!quiet_mode) begin
                            $display("[%0t] DMEM: Return data=0x%h for addr=0x%h",
                                     $time, dmem[(dmem_latched_addr[19:3])], dmem_latched_addr);
                        end
                    end
                    dmem_pending <= 1'b0;
                end
            end
        end
    end

    // Task to load instruction memory from hex file
    task load_imem_hex;
        input [1023:0] filename;
        reg [31:0] temp_mem [0:IMEM_BYTES/4-1];  // 32-bit word array for $readmemh
        integer i;
    begin
        // Read 32-bit words
        $readmemh(filename, temp_mem);
        
        // Pack into 64-bit words (little-endian)
        for (i = 0; i < IMEM_WORDS; i = i + 1) begin
            imem[i] = {temp_mem[i*2 + 1], temp_mem[i*2]};
        end
        
        $display("IMEM: Loaded hex file: %s", filename);
        
        // Show first few instructions
        if (!quiet_mode) begin
            $display("IMEM: First 8 words loaded:");
            for (i = 0; i < 8; i = i + 1) begin
                $display("  [0x%h] = 0x%h", i*8, imem[i]);
            end
        end
    end
    endtask

endmodule

