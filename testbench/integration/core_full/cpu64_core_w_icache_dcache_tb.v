// cpu64_core_w_icache_dcache_tb.v - Testbench for full core with icache and dcache
`timescale 1ns/1ps

`include "cpu64_defs.vh"

module cpu64_core_w_icache_dcache_tb(
    input clk,
    input rst_n
);

    //===================== Parameters ======================//
    localparam VADDR = 39;
    localparam RESET_ADDR = 0;

    //===================== Timing Controls ====================//
    localparam integer RESET_IDLE_CYCLES     = 32;
    localparam integer POST_RUN_IDLE_CYCLES  = 64;

    // QUIET plusarg to suppress verbose console output
    reg quiet_mode;
    initial begin
        quiet_mode = 1'b0;
        void'($value$plusargs("QUIET=%d", quiet_mode));
    end

    //===================== IMEM Signals ====================//
    wire                 imem_req_o;
    wire [63:0]          imem_addr_o;
    wire                 imem_we_o;
    wire [7:0]           imem_be_o;
    wire [63:0]          imem_wdata_o;
    wire                 imem_gnt_i;
    wire                 imem_rvalid_i;
    wire [63:0]          imem_rdata_i;

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

    // Trace file dumping removed; using console-only reporting

    //===================== ECALL Detection =================//
    wire ecall_detected;
    
    // Monitor ECALL/EBREAK signal
    always @(posedge clk) begin
        if (rst_n && ecall_detected) begin
            $display("\n=== ECALL Detected - Program Complete ===");
        end
    end

    //===================== DUT =============================//
    cpu64_core_w_icache_dcache #(
        .VADDR(VADDR),
        .RESET_ADDR(RESET_ADDR)
    ) dut (
        .clk_i                     (clk),
        .rst_ni                    (rst_n),
        .m_ext_inter_i             (m_ext_inter_i),
        .m_soft_inter_i            (m_soft_inter_i),
        .m_timer_inter_i           (m_timer_inter_i),
        .time_i                    (time_i),
        .fencei_flush_ao           (fencei_flush_ao),
        // IMEM
        .imem_req_o                (imem_req_o),
        .imem_gnt_i                (imem_gnt_i),
        .imem_addr_o               (imem_addr_o),
        .imem_we_o                 (imem_we_o),
        .imem_be_o                 (imem_be_o),
        .imem_wdata_o              (imem_wdata_o),
        .imem_rvalid_i             (imem_rvalid_i),
        .imem_rdata_i              (imem_rdata_i),
        // DMEM
        .dmem_req_o                (dmem_req_o),
        .dmem_gnt_i                (dmem_gnt_i),
        .dmem_addr_ao              (dmem_addr_ao),
        .dmem_we_ao                (dmem_we_ao),
        .dmem_be_ao                (dmem_be_ao),
        .dmem_wdata_ao             (dmem_wdata_ao),
        .dmem_rvalid_i             (dmem_rvalid_i),
        .dmem_rdata_i              (dmem_rdata_i),
        // Cache maintenance
        .icache_invalidate_all_i   (1'b0),
        .dcache_invalidate_all_i   (1'b0),
        // ECALL detection
        .ecall_o                   (ecall_detected),
        .WB_valid_o                ()
    );

    //===================== Unified Memory Model ============//
    obi_mem_model_unified #(
        .IMEM_BYTES(1<<20),
        .DMEM_BYTES(1<<20),
        .IMEM_READ_LATENCY(8),     // 8-beat icache refill
        .DMEM_READ_LATENCY(2),
        .WRITE_ACCEPT(1)
    ) u_mem (
        .clk_i              (clk),
        .rst_ni             (rst_n),
        // IMEM
        .imem_req_i         (imem_req_o),
        .imem_we_i          (imem_we_o),
        .imem_be_i          (imem_be_o),
        .imem_addr_i        (imem_addr_o),
        .imem_wdata_i       (imem_wdata_o),
        .imem_gnt_o         (imem_gnt_i),
        .imem_rvalid_o      (imem_rvalid_i),
        .imem_rdata_o       (imem_rdata_i),
        // DMEM
        .dmem_req_i         (dmem_req_o),
        .dmem_we_i          (dmem_we_ao),
        .dmem_be_i          (dmem_be_ao),
        .dmem_addr_i        ({{(64-VADDR){1'b0}}, dmem_addr_ao}),
        .dmem_wdata_i       (dmem_wdata_ao),
        .dmem_gnt_o         (dmem_gnt_i),
        .dmem_rvalid_o      (dmem_rvalid_i),
        .dmem_rdata_o       (dmem_rdata_i)
    );

    //===================== Cycle Counter ===================//
    reg [63:0] cycle_count;
    reg program_finished;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count <= 64'd0;
        end else begin
            cycle_count <= cycle_count + 64'd1;
        end
    end

    // Traces to file removed

    //===================== Statistics ======================//
    reg [31:0] stat_instructions_fetched;
    reg [31:0] stat_dmem_accesses;
    reg [31:0] stat_ecall_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_instructions_fetched <= 32'd0;
            stat_dmem_accesses <= 32'd0;
            stat_ecall_count <= 32'd0;
        end else begin
            // Track instruction fetches
            if (imem_rvalid_i) begin
                stat_instructions_fetched <= stat_instructions_fetched + 32'd1;
            end
            
            // Detect ECALL when it actually executes (using core's ecall_o signal)
            if (ecall_detected) begin
                stat_ecall_count <= stat_ecall_count + 32'd1;
            end
            
            // Track data memory accesses
            if (dmem_req_o && dmem_gnt_i) begin
                stat_dmem_accesses <= stat_dmem_accesses + 32'd1;
            end
        end
    end
    
    //===================== ECALL Halt Logic =================//
    always @(posedge clk) begin
        // Halt simulation on ECALL (program completion)
        if (stat_ecall_count >= 1) begin
            repeat (10) @(posedge clk);  // Drain pipeline
            
            // Final state to console
            dump_register_file();
            cache_summary();
            
            $finish;
        end
    end

    //===================== Hex Loading =====================//
    reg [1023:0] hex_path;
    
    initial begin
        // Initialize control signals
        m_ext_inter_i  = 1'b0;
        m_soft_inter_i = 1'b0;
        m_timer_inter_i= 1'b0;
        time_i         = 64'd0;
        program_finished = 1'b0;

        // File dumping removed

        // Load hex file - DISABLED: Program is pre-loaded in icache arrays
        // The icache arrays module loads the hex file directly during initialization
        // This avoids duplication and ensures the core runs from pre-loaded icache
        if (!$value$plusargs("HEX=%s", hex_path)) begin
            hex_path = "hex/simple_calculator.hex";
        end

        // Wait for reset
        @(posedge rst_n);
        
        // Run simulation
        repeat (20000) @(posedge clk);
        
        // Finish
        $display("\n=== Simulation Complete (Timeout) ===");
        $display("Cycles: %0d | Instructions: %0d | DMEM: %0d", 
                 cycle_count, stat_instructions_fetched, stat_dmem_accesses);
        
        // Final state to console
        dump_register_file();
        cache_summary();
        
        $finish;
    end

    //===================== Instruction Monitor =============//
    // Track instruction fetches for statistics only
    always @(posedge clk) begin
        if (rst_n && imem_rvalid_i) begin
            stat_instructions_fetched <= stat_instructions_fetched + 32'd1;
        end
    end

    //===================== Data Memory Monitor =============//
    // Track data memory accesses for statistics only
    always @(posedge clk) begin
        if (rst_n && dmem_req_o && dmem_gnt_i) begin
            stat_dmem_accesses <= stat_dmem_accesses + 32'd1;
        end
    end

    // Note: For detailed pipeline traces, internal signals would need to be
    // exposed via debug ports in cpu64_core. This simplified testbench
    // focuses on instruction/data cache integration testing.

    //===================== Dump Tasks ==========================//
    
    // Dump Register File to console
    task dump_register_file;
        integer i;
    begin
        $display("\n==============================================");
        $display("   CPU64 Register Dump - Final State");
        $display("==============================================\n");
        $display("Reg  | Value (Hex)          | Value (Dec)");
        $display("-----+----------------------+-------------");
        $display("x00  | 0x%016h | %0d", 64'h0, 64'd0);
        for (i = 1; i < 32; i = i + 1) begin
            $display("x%02d  | 0x%016h | %0d", i,
                dut.u_core.i_register_file.RF[i], $signed(dut.u_core.i_register_file.RF[i]));
        end
    end
    endtask
    
    // Cache summary (console): occupancy only
    task cache_summary;
        integer si, wi;
        integer l1i_valid_lines;
        integer l1d_valid_lines;
        integer l2_valid_lines;
        integer l3_valid_lines;
    begin
        l1i_valid_lines = 0;
        for (si = 0; si < 64; si = si + 1) begin
            for (wi = 0; wi < 8; wi = wi + 1) begin
                if (dut.u_icache.u_arrays.valid_q[wi][si]) l1i_valid_lines = l1i_valid_lines + 1;
            end
        end
        l1d_valid_lines = 0;
        for (si = 0; si < 64; si = si + 1) begin
            for (wi = 0; wi < 8; wi = wi + 1) begin
                if (dut.u_cache_stack.u_l1.u_arrays.valid_q[wi][si]) l1d_valid_lines = l1d_valid_lines + 1;
            end
        end
        l2_valid_lines = 0;
        for (si = 0; si < 256; si = si + 1) begin
            for (wi = 0; wi < 16; wi = wi + 1) begin
                if (dut.u_cache_stack.u_l2.u_arrays.valid_q[wi][si]) l2_valid_lines = l2_valid_lines + 1;
            end
        end
        l3_valid_lines = 0;
        for (si = 0; si < 2048; si = si + 1) begin
            for (wi = 0; wi < 16; wi = wi + 1) begin
                if (dut.u_cache_stack.u_l3.u_arrays.valid_q[wi][si]) l3_valid_lines = l3_valid_lines + 1;
            end
        end
        $display("\n=== Cache Occupancy Summary ===");
        $display("L1 I$: %0d/512 (%.1f%%) | L1 D$: %0d/512 (%.1f%%)", 
                 l1i_valid_lines, (l1i_valid_lines * 100.0) / 512.0,
                 l1d_valid_lines, (l1d_valid_lines * 100.0) / 512.0);
        $display("L2 D$: %0d/4096 (%.1f%%) | L3 D$: %0d/32768 (%.1f%%)", 
                 l2_valid_lines, (l2_valid_lines * 100.0) / 4096.0,
                 l3_valid_lines, (l3_valid_lines * 100.0) / 32768.0);
    end
    endtask
    // Detailed cache dumps removed

endmodule

