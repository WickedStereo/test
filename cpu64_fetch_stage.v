///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Module Name: fetch_stage                                                                      //
// Description: This stage manages the program counter, drives the instruction memory port, and  //
//              keeps the program counter synchronized during any pipeline hazards.              //                                                  //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
`include "cpu64_defs.vh"


module cpu64_fetch_stage #(parameter VADDR = 39, parameter RESET_ADDR = 0) (
    //===================== Clock and Reset =====================//
    input                   clk_i,              // System clock
    input                   rst_ni,             // Active-low reset signal
    
    //===================== Pipeline Control Inputs ============//
    input                   squash_i,           // Squash fetch stage (branch misprediction)
    input                   bubble_i,           // Insert bubble (load-use hazard)
    input                   stall_i,            // Stall fetch stage (memory not ready)
    input                   fencei_i,           // Fence.i instruction (cache flush)

    //===================== Compressed Instruction Feedback ====//
    input                   compressed_instr_i, // Previous instruction was 16-bit compressed

    //===================== Branch Control Inputs ==============//
    input                   branch_i,           // Branch taken (from execute stage)
    input       [VADDR-1:0] target_addr_i,      // Branch target address
    // Interrupt branch
    input                   csr_branch_i,       // CSR interrupt/trap branch
    input       [VADDR-1:0] csr_branch_addr_i,  // CSR branch target address
    // xRET branch
    input                   trap_ret_i,         // Trap return (mret, sret, uret)
    input       [VADDR-1:0] trap_ret_addr_i,    // Trap return address

    //===================== Instruction Memory Interface =======//
    output wire             imem_req_o,         // Instruction memory request
    output wire [VADDR-1:0] imem_addr_ao,       // Instruction memory address
    input                   imem_gnt_i,         // Instruction memory grant
    input                   imem_rvalid_i,      // Instruction memory read valid

    output wire             imem_stall_ao,      // Instruction memory stall signal
    
    //===================== Pipeline Outputs ===================//
    output reg              valid_o,            // Fetch stage valid output
    output reg [VADDR-1:0]  pc_o,               // Current PC output
    output reg [VADDR-1:0]  next_pc_o           // Next PC output
);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                      Validity Tracker                                     //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Manages the validity of the fetch stage considering pipeline control signals
    // Handles stall, bubble, and squash conditions for proper pipeline state management

    wire valid;

    cpu64_validity_tracker FCH_validity_tracker (
        .clk_i          (clk_i),              // System clock
        .rst_ni         (rst_ni),             // Active-low reset

        .valid_i        (1'b1),               // Fetch stage always tries to fetch
        
        .squash_i       (squash_i),           // Squash signal (branch misprediction)
        .bubble_i       (bubble_i),           // Bubble signal (load-use hazard)
        .stall_i        (stall_i),            // Stall signal (memory not ready)

        .valid_ao       (valid)               // Output validity considering all control signals
    );


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                            Program Counter Target Synchronizer                            //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // This complex logic handles PC updates during stalls and compressed instruction feedback
    // It ensures proper PC synchronization when the pipeline is stalled and when compressed
    // instructions require PC correction

    //===================== Control and State Registers =====================//
    reg stall_delayed, branch_taken_r, branch_taken_saved;  // Control state tracking
    reg [VADDR-1:0] pc, next_pc, next_pc_current, next_pc_saved, pc_updated;  // PC state

    //===================== Branch Detection ================================//
    // Detect any type of branch: trap return, regular branch, or CSR branch
    wire branch_taken = (trap_ret_i || branch_i || csr_branch_i);

    //===================== Stall Delay Logic ===============================//
    // Create a one-cycle delayed stall signal for PC target synchronization
    // This is needed to save PC targets during the first stall cycle and
    // restore them during the first cycle after stall ends
    always @(posedge clk_i) begin
        if (~rst_ni)
            stall_delayed <= 1'b0;
        else 
            stall_delayed <= stall_i;  // One cycle delayed stall signal
    end

    //===================== PC Target Saving Logic ==========================//
    // Save PC targets and branch state during stalls for proper restoration
    always @(posedge clk_i) begin
        if (~rst_ni) begin
            next_pc_saved      <= 'b0;
            branch_taken_saved <= 'b0;
        end else if (~stall_delayed) begin
            // Save current PC target when not stalled
            next_pc_saved      <= next_pc_current;
            branch_taken_saved <= branch_taken;
        end else if (branch_i) begin
            // Update saved target if branch occurs during stall
            next_pc_saved      <= target_addr_i;
        end
    end

    //===================== Current PC Target Generation ===================//
    // Generate the current PC target based on control signals
    always @(*) begin
        if      (csr_branch_i) next_pc_current = csr_branch_addr_i;  // CSR interrupt/trap
        else if (branch_i)     next_pc_current = target_addr_i;      // Regular branch
        else if (trap_ret_i)   next_pc_current = trap_ret_addr_i;    // Trap return
        else                   next_pc_current = (pc + 4);           // Sequential (32-bit)
        
        // Note: During stalls, the previous instruction is not yet decoded, so it is not known
        // if the instruction is compressed. When saving PC targets, we speculate that the
        // increment is 4, then update when retrieving the saved value.
    end

    //===================== PC Update Logic with Compressed Correction =====//
    // Handle PC updates with proper compressed instruction correction
    always @(*) begin
        // Choose between saved PC (during stall) or current PC target
        pc_updated = stall_delayed ? next_pc_saved : next_pc_current;

        if (branch_i) begin
            // Update branch target when the branch target is a load-use dependency
            next_pc = target_addr_i;
        end else if (~branch_taken || (stall_delayed && ~branch_taken_saved)) begin
            // Update PC with corrected increment for compressed instructions if necessary
            // If previous instruction was compressed (16-bit), subtract 2 from the +4 increment
            next_pc = compressed_instr_i ? (pc_updated - 2) : pc_updated;
        end else
            next_pc = pc_updated;
    end


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                      Program Counter                                      //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Manages the actual program counter register and its output
    // Handles PC updates, branch tracking, and compressed instruction correction

    wire [VADDR-1:0] pc_out;    // Final PC output with compressed instruction correction

    //===================== Program Counter Register =============//
    // Update the program counter register based on control signals
    always @(posedge clk_i) begin
        if      (~rst_ni)   pc <= RESET_ADDR;                    // Reset: start at reset address
        else if (~stall_i)  pc <= next_pc & (~('b1));           // Normal: update PC, ensure 2-byte alignment
    end

    //===================== Branch Taken Register ================//
    // Track whether a branch was taken in the previous cycle
    // This is used for compressed instruction correction
    always @(posedge clk_i) begin
        if      (~rst_ni)   branch_taken_r <= 'b0;              // Reset: clear branch taken flag
        else if (~stall_i)  branch_taken_r <= branch_taken;     // Normal: update branch taken flag
    end

    //===================== PC Output with Compressed Correction =//
    // Generate final PC output with compressed instruction correction
    // If previous instruction was compressed and no branch was taken, subtract 2 from PC
    assign pc_out = (compressed_instr_i && ~branch_taken_r) ? (pc - 2) : pc;


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                               Instruction Memory Interface                                //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Manages the instruction memory interface using Open Bus Interface (OBI)
    // Handles instruction fetch requests, grants, and stall conditions

    //===================== Memory Interface Signals =============//
    wire [31:0] wdata;    // Write data (unused for instruction memory)
    wire [3:0]  be;       // Byte enable (unused for instruction memory)
    wire        we;       // Write enable (unused for instruction memory)
    wire        read = valid && ~fencei_i;  // Read enable: valid instruction and not fence.i

    //===================== OBI Host Driver Instantiation ========//
    // Instantiate Open Bus Interface host driver for instruction memory
    // This module handles the OBI protocol for instruction fetching
    cpu64_obi_host_driver #(.DATA_W(32), .ADDR_W(VADDR)) imem_obi_host_driver 
    (
        //===================== Clock and Reset =================//
        .clk_i    (clk_i),        // System clock
        .rst_ni   (rst_ni),       // Active-low reset

        //===================== Memory Interface ================//
        .gnt_i    (imem_gnt_i),   // Memory grant signal
        .rvalid_i (imem_rvalid_i), // Memory read valid signal

        //===================== Pipeline Control ================//
        .stall_i  (stall_i),      // Pipeline stall signal

        //===================== Memory Access ===================//
        .be_i     (4'b0),         // Byte enable (not used for instruction memory)
        .addr_i   (pc_out),       // Memory address (current PC)
        .wdata_i  (32'b0),        // Write data (not used for instruction memory)
        .rd_i     (read),         // Read enable
        .wr_i     (1'b0),         // Write enable (not used for instruction memory)

        //===================== Outputs =========================//
        .stall_ao (imem_stall_ao), // Memory stall output
        .req_o    (imem_req_o),    // Memory request output
        .we_ao    (we),            // Write enable output (unused)
        .be_ao    (be),            // Byte enable output (unused)
        .addr_ao  (imem_addr_ao),  // Memory address output
        .wdata_ao (wdata)          // Write data output (unused)
    );

    //===================== Verilator Unused Signal =============//
    // Prevent synthesis warnings for unused signals
`ifdef VERILATOR
    wire _unused = &{we, be, wdata};
`endif


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                    Pipeline Outputs                                        //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Generate the final pipeline outputs for the fetch stage
    // These outputs are registered and only update when not stalled

    //===================== Valid Output Register ================//
    // Register the valid output signal  
    // Only assert validity when we have actually received valid instruction data from memory
    // This prevents stale/uninitialized instructions from being marked as valid
    always @(posedge clk_i) begin
        if (~rst_ni)
            valid_o <= 1'b0;                    // Reset: clear valid output
        else if (~stall_i)
            valid_o <= valid && imem_rvalid_i;  // Normal: valid only when memory returns valid data
    end

    //===================== PC Tracking for Instruction Responses =//
    // Track the PC of the last instruction request to properly match
    // instruction responses with their corresponding PC values
    // This prevents PC/instruction data misalignment when IMEM has latency
    reg [VADDR-1:0] fetched_pc;  // PC of the instruction request
    
    always @(posedge clk_i) begin
        if (~rst_ni)
            fetched_pc <= RESET_ADDR;                        // Reset: set to reset address
        else if (imem_req_o && imem_gnt_i)
            // Save PC when request is granted, even during stalls
            // The address output (imem_addr_ao) is what was actually requested
            fetched_pc <= imem_addr_ao;                      // Save requested address
    end
    
    //===================== PC Output Registers ==================//
    // Register the PC outputs for the pipeline
    // Output the PC that corresponds to the instruction data received
    // Update whenever valid data arrives, even during stalls, to maintain PC-instruction sync
    always @(posedge clk_i) begin
        if (~rst_ni) begin
            pc_o      <= RESET_ADDR;                         // Reset: set PC to reset address
            next_pc_o <= RESET_ADDR + 4;                     // Reset: set next PC to reset address + 4
        end else if (imem_rvalid_i) begin
            // When instruction data arrives, output the PC it corresponds to
            // Do this even during stalls to maintain synchronization
            pc_o      <= fetched_pc;                         // PC of the fetched instruction
            next_pc_o <= fetched_pc + 4;                     // Next PC
        end
    end


endmodule
