// cpu64_core_w_dcache_tb.v - Verilator-friendly TB: clock/reset driven by C++
`timescale 1ns/1ps

`include "cpu64_defs.vh"

module cpu64_core_w_dcache_tb(
    input clk,
    input rst_n
);

    //===================== Parameters ======================//
    localparam VADDR = 39;

    // QUIET plusarg to suppress verbose console output
    reg quiet_mode;
    initial begin
        quiet_mode = 1'b0;
        void'($value$plusargs("QUIET=%d", quiet_mode));
    end

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

    // Hex program loading
    reg [1023:0] hex_path;

    // Accept at most one outstanding read; grant only when not pending
    reg imem_pending;
    reg [7:0] imem_delay_cnt;

    // Grant is combinational based on pending status
    assign imem_gnt_i = imem_req_o && !imem_pending;

    // Optional: widen address for indexing (zero-extend virtual address)
    wire [63:0] imem_addr_64 = { {(64-VADDR){1'b0}}, imem_addr_ao };

    // Load program from +HEX=<path> (default to hex/comprehensive_program.hex)
    initial begin
        integer i;
        for (i = 0; i < IMEM_WORDS; i = i + 1) begin
            instruction_memory[i] = 32'h00000013; // NOP (addi x0,x0,0)
        end
        if (!$value$plusargs("HEX=%s", hex_path)) begin
            hex_path = "hex/comprehensive_program.hex";
        end
        if (!quiet_mode) $display("IMEM: Loading hex file: %0s", hex_path);
        $readmemh(hex_path, instruction_memory);
        
        // Debug: Display first 20 loaded instructions
        if (!quiet_mode) begin
            $display("IMEM: First 20 instructions loaded:");
            for (i = 0; i < 20; i = i + 1) begin
                $display("  [%2d] addr=0x%02h: 0x%08h", i, i*4, instruction_memory[i]);
            end
        end
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
                    if (!quiet_mode) $display("[Cycle %0d] IMEM: Response addr=0x%0h word_idx=%0d inst=0x%08h", 
                             time_i, imem_last_addr, imem_last_addr[31:2], instruction_memory[imem_last_addr[31:2]]);
                end
            end
            
            // Accept a new request ONLY if no request is pending
            // This prevents overwriting imem_last_addr while a request is in flight
            if (imem_req_o && imem_gnt_i && !imem_pending) begin
                imem_last_addr <= imem_addr_ao;
                imem_pending   <= 1'b1;
                imem_delay_cnt <= IMEM_READ_LATENCY[7:0];
                if (!quiet_mode) $display("[Cycle %0d] IMEM: Fetch request addr=0x%0h (word_idx=%0d)", 
                         time_i, imem_addr_ao, imem_addr_ao[31:2]);
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

    //===================== Defaults/Time ===================//
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_ext_inter_i  <= 1'b0;
            m_soft_inter_i <= 1'b0;
            m_timer_inter_i<= 1'b0;
            time_i         <= '0;
        end else begin
            time_i         <= time_i + 1'b1;
        end
    end

    //===================== Program Completion Monitor ===================//
    reg program_finished = 0;
    reg [7:0] finish_delay = 0;
    
    always @(posedge clk) begin
        // Monitor for ebreak instruction (0x00100073)
        if (imem_rvalid_i && instruction_memory[imem_last_addr[31:2]] == 32'h00100073 && !program_finished) begin
            $display("PROGRAM: ebreak instruction executed - program finished");
            program_finished <= 1;
            finish_delay <= 10;
        end
        
        // Wait a few cycles before dumping final state
        if (program_finished && finish_delay > 0) begin
            finish_delay <= finish_delay - 1;
        end else if (program_finished && finish_delay == 0) begin
            integer fd, i;
            integer fd_regs, ri;
            
            
            // Write to file
            fd = $fopen("obj_dir/memory_dump.txt", "w");
            $fwrite(fd, "==============================================\n");
            $fwrite(fd, "   CPU64 Memory Dump - Final State\n");
            $fwrite(fd, "==============================================\n\n");
            $fwrite(fd, "=== EXTERNAL MEMORY (Pre-initialized) ===\n");
            $fwrite(fd, "Address  | Value (Hex)          | Value (Dec)\n");
            $fwrite(fd, "---------+----------------------+-------------\n");
            $fwrite(fd, "0x10000 | 0x%016h | %0d\n", i_dmem_model.mem[20'h02000], $signed(i_dmem_model.mem[20'h02000]));
            $fwrite(fd, "0x10008 | 0x%016h | %0d\n", i_dmem_model.mem[20'h02001], $signed(i_dmem_model.mem[20'h02001]));
            $fwrite(fd, "0x20000 | 0x%016h | %0d\n", i_dmem_model.mem[20'h04000], $signed(i_dmem_model.mem[20'h04000]));
            $fwrite(fd, "0x20008 | 0x%016h | %0d\n", i_dmem_model.mem[20'h04001], $signed(i_dmem_model.mem[20'h04001]));
            $fwrite(fd, "0x30000 | 0x%016h | %0d\n", i_dmem_model.mem[20'h06000], $signed(i_dmem_model.mem[20'h06000]));
            $fwrite(fd, "\n==============================================\n");
            $fwrite(fd, "Total execution cycles: %0d\n", time_i);
            $fwrite(fd, "==============================================\n");
            $fclose(fd);
            $display("INFO: Memory dump written to memory_dump.txt");

            // Register dump: x0..x31 → obj_dir/register_dump.txt
            fd_regs = $fopen("obj_dir/register_dump.txt", "w");
            if (fd_regs) begin
                $fwrite(fd_regs, "==============================================\n");
                $fwrite(fd_regs, "   CPU64 Register Dump - Final State\n");
                $fwrite(fd_regs, "==============================================\n\n");
                $fwrite(fd_regs, "Reg  | Value (Hex)          | Value (Dec)\n");
                $fwrite(fd_regs, "-----+----------------------+-------------\n");
                // x0 is hardwired to zero
                $fwrite(fd_regs, "x00  | 0x%016h | %0d\n", 64'h0, 64'd0);
                for (ri = 1; ri <= 31; ri = ri + 1) begin
                    $fwrite(fd_regs, "x%02d  | 0x%016h | %0d\n", ri,
                        dut.u_core.i_register_file.RF[ri], $signed(dut.u_core.i_register_file.RF[ri]));
                end
                $fclose(fd_regs);
                $display("INFO: Register dump written to register_dump.txt");
            end
            
            // Cache dumps: L1/L2/L3 → obj_dir/cache_dump_*.txt
            begin : cache_dumps_block
                integer fd_l1, fd_l2, fd_l3;
                integer si, wi, wordi;
                reg [63:0] data_word;
                reg [63:0] addr_line, addr_word;
                // L1
                fd_l1 = $fopen("obj_dir/cache_dump_L1.txt", "w");
                if (fd_l1) begin
                    $fwrite(fd_l1, "set way word valid dirty tag addr data\n");
                    for (si = 0; si < 64; si = si + 1) begin
                        for (wi = 0; wi < 8; wi = wi + 1) begin
                            addr_line = { dut.u_cache_stack.u_l1.u_arrays.tag_q[wi][si], si[5:0], 6'b0 };
                            for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                                data_word = dut.u_cache_stack.u_l1.u_arrays.data_q[wi][si*8 + wordi];
                                addr_word = addr_line + (wordi << 3);
                                $fwrite(fd_l1, "%0d %0d %0d %0d %0d 0x%013h 0x%016h 0x%016h\n",
                                    si, wi, wordi,
                                    dut.u_cache_stack.u_l1.u_arrays.valid_q[wi][si],
                                    dut.u_cache_stack.u_l1.u_arrays.dirty_q[wi][si],
                                    dut.u_cache_stack.u_l1.u_arrays.tag_q[wi][si],
                                    addr_word, data_word);
                            end
                        end
                    end
                    $fclose(fd_l1);
                end
                // L2
                fd_l2 = $fopen("obj_dir/cache_dump_L2.txt", "w");
                if (fd_l2) begin
                    $fwrite(fd_l2, "set way word valid dirty tag addr data\n");
                    for (si = 0; si < 256; si = si + 1) begin
                        for (wi = 0; wi < 16; wi = wi + 1) begin
                            addr_line = { dut.u_cache_stack.u_l2.u_arrays.tag_q[wi][si], si[7:0], 6'b0 };
                            for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                                data_word = dut.u_cache_stack.u_l2.u_arrays.data_q[wi][si*8 + wordi];
                                addr_word = addr_line + (wordi << 3);
                                $fwrite(fd_l2, "%0d %0d %0d %0d %0d 0x%013h 0x%016h 0x%016h\n",
                                    si, wi, wordi,
                                    dut.u_cache_stack.u_l2.u_arrays.valid_q[wi][si],
                                    dut.u_cache_stack.u_l2.u_arrays.dirty_q[wi][si],
                                    dut.u_cache_stack.u_l2.u_arrays.tag_q[wi][si],
                                    addr_word, data_word);
                            end
                        end
                    end
                    $fclose(fd_l2);
                end
                // L3
                fd_l3 = $fopen("obj_dir/cache_dump_L3.txt", "w");
                if (fd_l3) begin
                    $fwrite(fd_l3, "set way word valid dirty tag addr data\n");
                    for (si = 0; si < 2048; si = si + 1) begin
                        for (wi = 0; wi < 16; wi = wi + 1) begin
                            addr_line = { dut.u_cache_stack.u_l3.u_arrays.tag_q[wi][si], si[10:0], 6'b0 };
                            for (wordi = 0; wordi < 8; wordi = wordi + 1) begin
                                data_word = dut.u_cache_stack.u_l3.u_arrays.data_q[wi][si*8 + wordi];
                                addr_word = addr_line + (wordi << 3);
                                $fwrite(fd_l3, "%0d %0d %0d %0d %0d 0x%013h 0x%016h 0x%016h\n",
                                    si, wi, wordi,
                                    dut.u_cache_stack.u_l3.u_arrays.valid_q[wi][si],
                                    dut.u_cache_stack.u_l3.u_arrays.dirty_q[wi][si],
                                    dut.u_cache_stack.u_l3.u_arrays.tag_q[wi][si],
                                    addr_word, data_word);
                            end
                        end
                    end
                    $fclose(fd_l3);
                end
            end
            

            $finish;
        end
    end


endmodule


