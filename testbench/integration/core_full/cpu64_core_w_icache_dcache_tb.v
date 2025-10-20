// cpu64_core_w_icache_dcache_tb.v - Testbench for full core with icache and dcache
// `timescale 1ns/1ps

// `include "cpu64_defs.vh"

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

    // Trace file descriptors
    integer              fd_wb;
    integer              fd_dec;
    integer              fd_br;
    integer              fd_pipe;
    integer              fd_flush;
    integer              fd_ifetch;

    //===================== ECALL Detection =================//
    wire ecall_detected;
    
        // Debug: Monitor ECALL/EBREAK signal and execute stage
    always @(posedge clk) begin
        if (rst_n && ecall_detected) begin
            $display("[%0t cyc=%0d] *** ECALL/EBREAK SIGNAL ASSERTED ***", $time, cycle_count);
        end
        // Monitor execute stage when EBREAK instruction is present
        if (rst_n && dut.u_core.EXE.pc_i == 64'h30) begin
            $display("[cyc=%0d] EBREAK@EXE: PC=%h valid=%b ebreak_ex_i=%b ecall_o=%b", 
                     cycle_count, dut.u_core.EXE.pc_i, dut.u_core.EXE.valid, 
                     dut.u_core.EXE.ebreak_ex_i, ecall_detected);
        end
        // Monitor ALU stall signal
        if (rst_n && dut.u_core.alu_stall) begin
            $display("[cyc=%0d] ALU_STALL: alu_stall=%b PC_e=%h", 
                     cycle_count, dut.u_core.alu_stall, dut.u_core.EXE.pc_i);
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

    //===================== Fetch/ICache Trace =============//
    always @(posedge clk) begin
        if (rst_n && fd_ifetch) begin
            if (cycle_count < 512) begin
                $fwrite(fd_ifetch,
                    "%0d %0b %0b %0b %0b %0b %0b 0x%010h 0x%010h %0d %0d 0x%08h\n",
                    cycle_count,
                    dut.u_core.imem_req_o,
                    dut.u_core.imem_gnt_i,
                    dut.u_core.imem_rvalid_i,
                    dut.u_icache.fetch_req_i,
                    dut.u_icache.fetch_gnt_o,
                    dut.u_icache.instr_valid_o,
                    {25'd0, dut.u_core.FCH.pc_o},
                    dut.u_icache.req_pc_q,
                    dut.u_icache.req_word_sel_q,
                    dut.u_icache.word_off_core,
                    dut.u_icache.instr_o);
            end
        end
    end

    //===================== Pipeline Valid Trace ===========//
    always @(posedge clk) begin
        if (rst_n && fd_pipe) begin
            if (cycle_count < 512) begin
                $fwrite(fd_pipe,
                    "%0d 0x%010h 0x%010h 0x%010h 0x%010h 0x%010h %0b %0b %0b %0b %0b rd=%0d wr_en=%0b gated=%0b\n",
                    cycle_count,
                    {25'd0, dut.u_core.FCH_pc},
                    {25'd0, dut.u_core.DCD_pc},
                    {25'd0, dut.u_core.EXE_pc},
                    64'h0,
                    64'h0,
                    dut.u_core.FCH_valid,
                    dut.u_core.DCD_valid,
                    dut.u_core.EXE_valid,
                    dut.u_core.MEM_valid,
                    dut.u_core.WB_valid,
                    dut.u_core.rd_idx,
                    dut.u_core.rd_wr_en,
                    dut.u_core.rd_wr_en_gated);
            end
        end
    end

    //===================== Decode Stage Trace =============//
    always @(posedge clk) begin
        if (rst_n && fd_dec) begin
            if (dut.u_core.DCD_valid && !dut.u_core.DCD_stall) begin
                $fwrite(fd_dec,
                    "0x%010h %0d %0d %0d %0b 0x%016h 0x%016h 0x%02h\n",
                    {25'd0, dut.u_core.DCD_pc},
                    dut.u_core.DCD_rs1_idx,
                    dut.u_core.DCD_rs2_idx,
                    dut.u_core.DCD_rd_idx,
                    dut.u_core.DCD_rs2_used,
                    dut.u_core.DCD_alu_op_a,
                    dut.u_core.DCD_alu_op_b,
                    dut.u_core.DCD_alu_operation);
            end
        end
    end

    //===================== Writeback Stage Trace =============//
    always @(posedge clk) begin
        if (rst_n && fd_wb) begin
            if (dut.u_core.rd_wr_en) begin
                $fwrite(fd_wb,
                    "%0d rd=x%0d data=0x%016h\n",
                    cycle_count,
                    dut.u_core.rd_idx,
                    dut.u_core.rd_data);
            end
        end
    end

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
                if (!quiet_mode) begin
                    $display("[%0t cyc=%0d] *** ECALL DETECTED (core signal) *** count=%0d",
                             $time, cycle_count, stat_ecall_count + 32'd1);
                end
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
            $display("\n=== HALTING: ECALL Detected (Program Complete) ===");
            $display("ECALL count:        %0d", stat_ecall_count);
            $display("Cycles:             %0d", cycle_count);
            $display("Instructions Fetched: %0d", stat_instructions_fetched);
            if (stat_instructions_fetched > 0) begin
                $display("Avg Cycles/Fetch:   %0d.%02d",
                         cycle_count / stat_instructions_fetched,
                         ((cycle_count % stat_instructions_fetched) * 100) / stat_instructions_fetched);
            end
            
            // Dump final state
            dump_register_file();
            dump_l1_icache();
            dump_l1_dcache();
            dump_l2_dcache();
            dump_l3_dcache();
            
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

        // Open trace files
        fd_wb = $fopen("obj_dir/wb_trace.txt", "w");
        if (fd_wb) $fwrite(fd_wb, "rd data (WB)\n");
        
        fd_dec = $fopen("obj_dir/decode_trace.txt", "w");
        if (fd_dec) $fwrite(fd_dec, "pc rs1 rs2 rd uses_rs2 alu_op_a alu_op_b alu_op\n");
        
        fd_br = $fopen("obj_dir/branch_trace.txt", "w");
        if (fd_br) $fwrite(fd_br, "type pc inst br_cond d_branch exe_valid exe_branch taken target rs1 rs2 eq\n");
        
        fd_pipe = $fopen("obj_dir/pipeline_trace.txt", "w");
        if (fd_pipe) $fwrite(fd_pipe, "cycle pc_f pc_d pc_e pc_m pc_wb v_f v_d v_e v_m v_wb\n");
        
    fd_flush = $fopen("obj_dir/flush_trace.txt", "w");
    if (fd_flush) $fwrite(fd_flush, "cycle event stage details\n");

    fd_ifetch = $fopen("obj_dir/icache_core_fetch_trace.txt", "w");
    if (fd_ifetch) $fwrite(fd_ifetch, "cycle core_req core_gnt core_rvalid fetch_req fetch_gnt instr_valid pc ic_pc word_sel word_off instr_to_core\n");

        // Load hex file
        if (!$value$plusargs("HEX=%s", hex_path)) begin
            hex_path = "hex/simple_calculator.hex";
        end
        u_mem.load_imem_hex(hex_path);
        $display("Loaded program: %s", hex_path);

        // Wait for reset
        @(posedge rst_n);
        
        // Run simulation
        repeat (20000) @(posedge clk);
        
        // Finish
        $display("\n=== Simulation Complete ===");
        $display("Cycles:             %0d", cycle_count);
        $display("Instructions Fetched: %0d", stat_instructions_fetched);
        $display("DMEM Accesses:      %0d", stat_dmem_accesses);
        if (stat_instructions_fetched > 0) begin
            $display("Avg Cycles/Fetch:   %0d.%02d",
                     cycle_count / stat_instructions_fetched,
                     ((cycle_count % stat_instructions_fetched) * 100) / stat_instructions_fetched);
        end
        
        // Dump final state
        dump_register_file();
        dump_l1_icache();
        dump_l1_dcache();
        dump_l2_dcache();
        dump_l3_dcache();
        
    if (fd_wb)    $fclose(fd_wb);
    if (fd_dec)   $fclose(fd_dec);
    if (fd_br)    $fclose(fd_br);
    if (fd_pipe)  $fclose(fd_pipe);
    if (fd_flush) $fclose(fd_flush);
    if (fd_ifetch)$fclose(fd_ifetch);
        
        $finish;
    end

    //===================== Instruction Monitor =============//
    // Debug: Track all IMEM transactions
    always @(posedge clk) begin
        if (rst_n && !quiet_mode) begin
            if (imem_req_o) begin
                $display("[%0t cyc=%0d] IMEM_REQ: addr=0x%h gnt=%b",
                         $time, cycle_count, imem_addr_o, imem_gnt_i);
            end
            if (imem_rvalid_i) begin
                $display("[%0t cyc=%0d] IMEM_RVALID: data=0x%h",
                         $time, cycle_count, imem_rdata_i);
            end
        end
    end
    
    // Debug: Track reset sequence
    reg [7:0] reset_dbg_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_dbg_counter <= 8'd0;
        end else begin
            if (reset_dbg_counter < 8'd10) begin
                reset_dbg_counter <= reset_dbg_counter + 8'd1;
                if (!quiet_mode) begin
                    $display("[%0t cyc=%0d] POST_RESET: counter=%0d imem_req=%b",
                             $time, cycle_count, reset_dbg_counter, imem_req_o);
                end
            end
        end
    end

    //===================== Data Memory Monitor =============//
    always @(posedge clk) begin
        if (rst_n && dmem_req_o && dmem_gnt_i && !quiet_mode) begin
            $display("[%0t cyc=%0d] DMEM: %s addr=0x%h data=0x%h",
                     $time, cycle_count,
                     dmem_we_ao ? "WRITE" : "READ",
                     dmem_addr_ao, dmem_wdata_ao);
        end
    end

    // Note: For detailed pipeline traces, internal signals would need to be
    // exposed via debug ports in cpu64_core. This simplified testbench
    // focuses on instruction/data cache integration testing.

    //===================== Dump Tasks ==========================//
    
    // Dump Register File (matching cpu64_core_w_dcache_tb.v format)
    task dump_register_file;
        integer i;
        integer fd;
    begin
        fd = $fopen("obj_dir/register_dump.txt", "w");
        if (fd) begin
            $fwrite(fd, "==============================================\n");
            $fwrite(fd, "   CPU64 Register Dump - Final State\n");
            $fwrite(fd, "==============================================\n\n");
            $fwrite(fd, "Reg  | Value (Hex)          | Value (Dec)\n");
            $fwrite(fd, "-----+----------------------+-------------\n");
            
            // x0 is hardwired to zero
            $fwrite(fd, "x00  | 0x%016h | %0d\n", 64'h0, 64'd0);
            
            // x1-x31 are in the RF array
            for (i = 1; i < 32; i = i + 1) begin
                $fwrite(fd, "x%02d  | 0x%016h | %0d\n", i,
                    dut.u_core.i_register_file.RF[i], $signed(dut.u_core.i_register_file.RF[i]));
            end
            
            $fclose(fd);
            
            if (!quiet_mode) begin
                $display("\n=== Register File Dump ===");
                $display("  x00 (zero): 0x%016h | %0d", 64'h0, 64'd0);
                $display("  x01 (ra):   0x%016h | %0d", dut.u_core.i_register_file.RF[1], $signed(dut.u_core.i_register_file.RF[1]));
                $display("  x02 (sp):   0x%016h | %0d", dut.u_core.i_register_file.RF[2], $signed(dut.u_core.i_register_file.RF[2]));
                $display("  x05 (t0):   0x%016h | %0d", dut.u_core.i_register_file.RF[5], $signed(dut.u_core.i_register_file.RF[5]));
                $display("  x10 (a0):   0x%016h | %0d", dut.u_core.i_register_file.RF[10], $signed(dut.u_core.i_register_file.RF[10]));
                $display("Register dump written to obj_dir/register_dump.txt");
            end
        end
    end
    endtask
    
    // Dump L1 ICache (detailed, matching cpu64_core_w_dcache_tb.v format)
    task dump_l1_icache;
        integer si, wi, wordi;
        integer fd;
        reg [51:0] tag;
        reg valid;
        reg [63:0] data_word;
        reg [63:0] addr_line, addr_word;
        integer valid_lines;
    begin
        fd = $fopen("obj_dir/cache_dump_L1_ICache.txt", "w");
        if (fd) begin
            $fwrite(fd, "set way word valid tag addr data\n");
            
            valid_lines = 0;
            
            for (si = 0; si < 64; si = si + 1) begin
                for (wi = 0; wi < 8; wi = wi + 1) begin
                    valid = dut.u_icache.u_arrays.valid_q[wi][si];
                    tag = dut.u_icache.u_arrays.tag_q[wi][si];
                    addr_line = {tag, si[5:0], 6'b0};
                    
                    if (valid) valid_lines = valid_lines + 1;
                    
                    for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                        data_word = dut.u_icache.u_arrays.data_q[wi][si*8 + wordi];
                        addr_word = addr_line + (wordi << 3);
                        $fwrite(fd, "%0d %0d %0d %0d 0x%013h 0x%016h 0x%016h\n",
                            si, wi, wordi,
                            valid,
                            tag,
                            addr_word, data_word);
                    end
                end
            end
            
            $fclose(fd);
            
            if (!quiet_mode) begin
                $display("\n=== L1 ICache Dump ===");
                $display("  Configuration: 64 sets x 8 ways, 64B lines");
                $display("  Valid lines:   %0d / 512 (%.1f%% occupancy)",
                         valid_lines, (valid_lines * 100.0) / 512.0);
                $display("L1 ICache dump written to obj_dir/cache_dump_L1_ICache.txt");
            end
        end
    end
    endtask
    
    // Dump L1 DCache (detailed, matching cpu64_core_w_dcache_tb.v format)
    task dump_l1_dcache;
        integer si, wi, wordi;
        integer fd;
        reg [51:0] tag;
        reg valid, dirty;
        reg [63:0] data_word;
        reg [63:0] addr_line, addr_word;
        integer valid_lines, dirty_lines;
    begin
        fd = $fopen("obj_dir/cache_dump_L1_DCache.txt", "w");
        if (fd) begin
            $fwrite(fd, "set way word valid dirty tag addr data\n");
            
            valid_lines = 0;
            dirty_lines = 0;
            
            for (si = 0; si < 64; si = si + 1) begin
                for (wi = 0; wi < 8; wi = wi + 1) begin
                    valid = dut.u_cache_stack.u_l1.u_arrays.valid_q[wi][si];
                    dirty = dut.u_cache_stack.u_l1.u_arrays.dirty_q[wi][si];
                    tag = dut.u_cache_stack.u_l1.u_arrays.tag_q[wi][si];
                    addr_line = {tag, si[5:0], 6'b0};
                    
                    if (valid) begin
                        valid_lines = valid_lines + 1;
                        if (dirty) dirty_lines = dirty_lines + 1;
                    end
                    
                    for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                        data_word = dut.u_cache_stack.u_l1.u_arrays.data_q[wi][si*8 + wordi];
                        addr_word = addr_line + (wordi << 3);
                        $fwrite(fd, "%0d %0d %0d %0d %0d 0x%013h 0x%016h 0x%016h\n",
                            si, wi, wordi,
                            valid, dirty,
                            tag,
                            addr_word, data_word);
                    end
                end
            end
            
            $fclose(fd);
            
            if (!quiet_mode) begin
                $display("\n=== L1 DCache Dump ===");
                $display("  Configuration: 64 sets x 8 ways, 64B lines");
                $display("  Valid lines:   %0d / 512 (%.1f%% occupancy)",
                         valid_lines, (valid_lines * 100.0) / 512.0);
                $display("  Dirty lines:   %0d / %0d valid (%.1f%%)",
                         dirty_lines, valid_lines, 
                         valid_lines > 0 ? (dirty_lines * 100.0) / valid_lines : 0.0);
                $display("L1 DCache dump written to obj_dir/cache_dump_L1_DCache.txt");
            end
        end
    end
    endtask

    // Dump L2 DCache
    task dump_l2_dcache;
        integer si, wi, wordi;
        integer fd;
        reg [43:0] tag;
        reg valid, dirty;
        reg [63:0] data_word;
        reg [63:0] addr_line, addr_word;
        integer valid_lines, dirty_lines;
    begin
        fd = $fopen("obj_dir/cache_dump_L2.txt", "w");
        if (fd) begin
            $fwrite(fd, "set way word valid dirty tag addr data\n");
            
            valid_lines = 0;
            dirty_lines = 0;
            
            for (si = 0; si < 256; si = si + 1) begin
                for (wi = 0; wi < 16; wi = wi + 1) begin
                    valid = dut.u_cache_stack.u_l2.u_arrays.valid_q[wi][si];
                    dirty = dut.u_cache_stack.u_l2.u_arrays.dirty_q[wi][si];
                    tag = dut.u_cache_stack.u_l2.u_arrays.tag_q[wi][si];
                    addr_line = {tag, si[7:0], 6'b0};
                    
                    if (valid) begin
                        valid_lines = valid_lines + 1;
                        if (dirty) dirty_lines = dirty_lines + 1;
                    end
                    
                    for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                        data_word = dut.u_cache_stack.u_l2.u_arrays.data_q[wi][si*8 + wordi];
                        addr_word = addr_line + (wordi << 3);
                        $fwrite(fd, "%0d %0d %0d %0d %0d 0x%011h 0x%016h 0x%016h\n",
                            si, wi, wordi,
                            valid, dirty,
                            tag,
                            addr_word, data_word);
                    end
                end
            end
            
            $fclose(fd);
            
            if (!quiet_mode) begin
                $display("\n=== L2 DCache Dump ===");
                $display("  Configuration: 256 sets x 16 ways, 64B lines");
                $display("  Valid lines:   %0d / 4096 (%.1f%% occupancy)",
                         valid_lines, (valid_lines * 100.0) / 4096.0);
                $display("  Dirty lines:   %0d / %0d valid (%.1f%%)",
                         dirty_lines, valid_lines, 
                         valid_lines > 0 ? (dirty_lines * 100.0) / valid_lines : 0.0);
                $display("L2 DCache dump written to obj_dir/cache_dump_L2.txt");
            end
        end
    end
    endtask

    // Dump L3 DCache
    task dump_l3_dcache;
        integer si, wi, wordi;
        integer fd;
        reg [38:0] tag;
        reg valid, dirty;
        reg [63:0] data_word;
        reg [63:0] addr_line, addr_word;
        integer valid_lines, dirty_lines;
    begin
        fd = $fopen("obj_dir/cache_dump_L3.txt", "w");
        if (fd) begin
            $fwrite(fd, "set way word valid dirty tag addr data\n");
            
            valid_lines = 0;
            dirty_lines = 0;
            
            for (si = 0; si < 2048; si = si + 1) begin
                for (wi = 0; wi < 16; wi = wi + 1) begin
                    valid = dut.u_cache_stack.u_l3.u_arrays.valid_q[wi][si];
                    dirty = dut.u_cache_stack.u_l3.u_arrays.dirty_q[wi][si];
                    tag = dut.u_cache_stack.u_l3.u_arrays.tag_q[wi][si];
                    addr_line = {tag, si[10:0], 6'b0};
                    
                    if (valid) begin
                        valid_lines = valid_lines + 1;
                        if (dirty) dirty_lines = dirty_lines + 1;
                    end
                    
                    for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                        data_word = dut.u_cache_stack.u_l3.u_arrays.data_q[wi][si*8 + wordi];
                        addr_word = addr_line + (wordi << 3);
                        $fwrite(fd, "%0d %0d %0d %0d %0d 0x%010h 0x%016h 0x%016h\n",
                            si, wi, wordi,
                            valid, dirty,
                            tag,
                            addr_word, data_word);
                    end
                end
            end
            
            $fclose(fd);
            
            if (!quiet_mode) begin
                $display("\n=== L3 DCache Dump ===");
                $display("  Configuration: 2048 sets x 16 ways, 64B lines");
                $display("  Valid lines:   %0d / 32768 (%.1f%% occupancy)",
                         valid_lines, (valid_lines * 100.0) / 32768.0);
                $display("  Dirty lines:   %0d / %0d valid (%.1f%%)",
                         dirty_lines, valid_lines, 
                         valid_lines > 0 ? (dirty_lines * 100.0) / valid_lines : 0.0);
                $display("L3 DCache dump written to obj_dir/cache_dump_L3.txt");
            end
        end
    end
    endtask

endmodule

