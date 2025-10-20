// -------------------------------------------------------------
// Arrays: cpu64_l1i_arrays — data/tag/valid only
// -------------------------------------------------------------
// `timescale 1ns/1ps
module cpu64_l1i_arrays (
  input               clk_i,
  input               rst_ni,
  input               invalidate_all_i,

  // Access controls
  input       [5:0]   index_i,      // set index (64 sets)
  input       [2:0]   word_sel_i,   // word within line (8 words)
  input       [2:0]   way_sel_i,    // selected way for write
  input               write_en_i,   // write enable for selected way/word
  input               set_valid_i,
  input      [51:0]   tag_in_i,
  input      [63:0]   wdata_i,

  // Outputs for current index/word
  output     [63:0]   rdata_selected_o,
  output     [51:0]   tag_selected_o,
  output              valid_selected_o,

  output [8*64-1:0]   rdata_way_flat_o,
  output [8*52-1:0]   tag_way_flat_o,
  output      [7:0]   valid_way_o
);
  localparam integer DATA_W          = 64;
  localparam integer TAG_W           = 52;
  localparam integer WORDS_PER_LINE  = 8;  // 64B / 8B
  localparam integer WAYS            = 8;
  localparam integer SETS            = 64;
  localparam integer LINE_ADDR_W     = 9;  // 6 (index) + 3 (word)

  // data_q[way][{index,word}]
  reg [DATA_W-1:0] data_q [0:WAYS-1][0:SETS*WORDS_PER_LINE-1];
  reg [TAG_W-1:0]  tag_q  [0:WAYS-1][0:SETS-1];
  reg              valid_q[0:WAYS-1][0:SETS-1];

  wire [LINE_ADDR_W-1:0] line_idx = {index_i, word_sel_i};

  assign rdata_selected_o = data_q[way_sel_i][line_idx];
  assign tag_selected_o   = tag_q[way_sel_i][index_i];
  assign valid_selected_o = valid_q[way_sel_i][index_i];

  genvar w; generate
    for (w = 0; w < WAYS; w = w + 1) begin : g_flat
      assign rdata_way_flat_o[(w+1)*DATA_W-1 : w*DATA_W] = data_q[w][line_idx];
      assign tag_way_flat_o[(w+1)*TAG_W-1    : w*TAG_W]  = tag_q[w][index_i];
      assign valid_way_o[w] = valid_q[w][index_i];
    end
  endgenerate

  // -----------------------------
  // Hex file loading for icache
  // -----------------------------
  reg [1023:0] hex_path;
  reg [31:0] temp_mem [0:1023];   // 32-bit word array for $readmemh (4KB - enough for 1000+ instructions)
  integer hex_i, hex_j, hex_word_idx, hex_set_idx, hex_way_idx;
  // File dumping removed; we keep console prints only
  integer v_si, v_wi, valid_lines_total;
  
  initial begin
    // Load hex file into instruction cache
    if (!$value$plusargs("HEX=%s", hex_path)) begin
      hex_path = "hex/simple_calculator.hex";
    end
    
    // Read 32-bit words from hex file
    $readmemh(hex_path, temp_mem);
    
    // Load into icache arrays (way 0, starting from set 0)
    hex_way_idx = 0;  // Use way 0 for pre-loading
    for (hex_i = 0; hex_i < SETS; hex_i = hex_i + 1) begin  // 64 sets
      hex_set_idx = hex_i;
      // Check if this set has any program data
      hex_word_idx = hex_i * WORDS_PER_LINE;
      if (hex_word_idx < 1024) begin   // Within loaded program size (4KB)
        // Load 8 words per cache line (64 bytes)
        for (hex_j = 0; hex_j < WORDS_PER_LINE; hex_j = hex_j + 1) begin
          hex_word_idx = hex_i * WORDS_PER_LINE + hex_j;
          if (hex_word_idx < 1024) begin   // Within loaded program size (4KB)
            // Pack two 32-bit instructions into one 64-bit word
            data_q[hex_way_idx][hex_set_idx * WORDS_PER_LINE + hex_j] = {temp_mem[hex_word_idx * 2 + 1], temp_mem[hex_word_idx * 2]};
          end else begin
            // Fill remaining words with zeros
            data_q[hex_way_idx][hex_set_idx * WORDS_PER_LINE + hex_j] = 64'd0;
          end
        end
        // Only set valid and tag for sets that contain program data
        valid_q[hex_way_idx][hex_set_idx] = 1'b1;
        tag_q[hex_way_idx][hex_set_idx] = 52'd0;  // All instructions start at address 0x0
      end else begin
        // Set invalid for sets beyond program data
        valid_q[hex_way_idx][hex_set_idx] = 1'b0;
        tag_q[hex_way_idx][hex_set_idx] = 52'd0;
      end
    end
    
    $display("ICACHE: Loaded hex file: %s", hex_path);
    // Count valid lines after preloading (all ways/sets)
    valid_lines_total = 0;
    for (v_si = 0; v_si < SETS; v_si = v_si + 1) begin
      for (v_wi = 0; v_wi < WAYS; v_wi = v_wi + 1) begin
        if (valid_q[v_wi][v_si]) valid_lines_total = valid_lines_total + 1;
      end
    end
    $display("ICACHE: Valid lines after preload: %0d / %0d (%.1f%%)",
             valid_lines_total, SETS*WAYS, (valid_lines_total * 100.0) / (SETS*WAYS));
    
    // Detailed cache dump of all valid lines
    $display("\n=== ICACHE Detailed Dump - All Valid Lines ===");
    $display("Way | Set | Tag (Hex)        | Valid | Data (8 words)");
    $display("----+-----+------------------+-------+----------------------------------------------------------------");
    
    for (v_wi = 0; v_wi < WAYS; v_wi = v_wi + 1) begin
      for (v_si = 0; v_si < SETS; v_si = v_si + 1) begin
        if (valid_q[v_wi][v_si]) begin
          $display("%2d  | %2d  | 0x%012h |   %b   | %016h %016h %016h %016h %016h %016h %016h %016h",
                   v_wi, v_si, tag_q[v_wi][v_si], valid_q[v_wi][v_si],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 0],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 1],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 2],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 3],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 4],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 5],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 6],
                   data_q[v_wi][v_si * WORDS_PER_LINE + 7]);
        end
      end
    end
    $display("=== End ICACHE Detailed Dump ===\n");
  end

  integer i, j, k;
  reg reset_done;
  
  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      reset_done <= 1'b0;
      // Clear valid bits during reset
      for (i = 0; i < WAYS; i = i + 1) begin
        for (j = 0; j < SETS; j = j + 1) begin
          valid_q[i][j] <= 1'b0;
        end
        // Don't clear data arrays - preserve pre-loaded instructions
      end
    end else begin
      // After reset, restore valid bits for pre-loaded cache lines
      if (!reset_done) begin
        reset_done <= 1'b1;
        // Restore valid bits for way 0, sets 0-63 (only 64 sets exist)
        // Only restore valid bits for sets that were pre-loaded with program data
        for (j = 0; j < SETS; j = j + 1) begin  // Only 64 sets (0-63)
          // Check if this set was pre-loaded with program data
          if (j * WORDS_PER_LINE < 1024) begin  // Within 4KB program size
            valid_q[0][j] <= 1'b1;
          end else begin
            valid_q[0][j] <= 1'b0;
          end
        end
      end
      
      if (invalidate_all_i) begin
        for (i = 0; i < WAYS; i = i + 1) begin
          for (j = 0; j < SETS; j = j + 1) begin
            valid_q[i][j] <= 1'b0;
          end
        end
      end else if (write_en_i) begin
        // Full-beat writes during refill
        data_q[way_sel_i][line_idx] <= wdata_i;
        tag_q[way_sel_i][index_i]   <= tag_in_i;
        valid_q[way_sel_i][index_i] <= set_valid_i;
      end
    end
  end
endmodule


