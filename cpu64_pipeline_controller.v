///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Module Name: cpu64_pipeline_controller                                                       //
// Description: Centralized pipeline controller managing hazard detection and resolution.
//              Handles RAW, load-use, and CSR load-use hazards. Generates
//              stall, bubble, and squash signals for each pipeline stage.
//              Coordinates branch misprediction recovery and memory stalls.
//              This is the central control unit that manages all pipeline flow control
//              and hazard resolution for the 5-stage CPU64 pipeline.
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
`include "cpu64_defs.vh"


module cpu64_pipeline_controller (
    //===================== Clock and Reset =====================//
    input               clk_i,              // System clock
    input               rst_ni,             // Active-low reset

    //===================== Hazard Detection Inputs =============//
    input               imem_stall_i,       // Instruction memory stall
    input               dmem_stall_i,       // Data memory stall
    input               load_use_haz_i,     // Load-use hazard detected
    input               csr_load_use_haz_i, // CSR load-use hazard detected
    input               alu_stall_i,        // ALU stall (multi-cycle operations)
    input               fencei_i,           // Fence.i instruction
    input               mem_exc_i,          // Memory exception
    input               trap_i,             // Trap condition

    //===================== Pipeline State Inputs ===============//
    input               EXE_branch_i,       // Branch instruction in execute stage
    input               EXE_write_i,        // Write operation in execute stage
    input               MEM_write_i,        // Write operation in memory stage
    input               valid_inst_in_pipe_i, // Valid instruction in pipeline

    //===================== Interrupt Controls ==================//
    input               wait_for_int_i,     // Wait for interrupt signal
    input               interrupt_i,        // Interrupt request
    output wire         interrupt_o,        // Interrupt output

    //===================== Fetch Stage Controls ================//
    output wire         FCH_squash_o,       // Fetch stage squash
    output wire         FCH_bubble_o,       // Fetch stage bubble
    output wire         FCH_stall_o,        // Fetch stage stall

    //===================== Decode Stage Controls ===============//
    output wire         DCD_squash_o,       // Decode stage squash
    output wire         DCD_bubble_o,       // Decode stage bubble
    output wire         DCD_stall_o,        // Decode stage stall

    //===================== Execute Stage Controls ==============//
    output wire         EXE_squash_o,       // Execute stage squash
    output wire         EXE_bubble_o,       // Execute stage bubble
    output wire         EXE_stall_o,        // Execute stage stall

    //===================== Memory Stage Controls ===============//
    output wire         MEM_squash_o,       // Memory stage squash
    output wire         MEM_bubble_o,       // Memory stage bubble
    output wire         MEM_stall_o,        // Memory stage stall

    //===================== Writeback Stage Controls ============//
    output wire         WB_squash_o,        // Writeback stage squash
    output wire         WB_bubble_o,        // Writeback stage bubble
    output wire         WB_stall_o,         // Writeback stage stall

    //===================== Core External Outputs ===============//
    output wire         fencei_flush_ao     // Fence.i flush output
);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                    Control Signal Generation                               //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Generate control signals for branch handling and fence.i operations
    // These signals are used throughout the pipeline controller logic

    //===================== Branch Control Signals ===============//
    wire branch_taken   = EXE_branch_i || trap_i;              // Branch taken or trap occurred
    wire fencei_flush_a = (EXE_write_i || MEM_write_i) && fencei_i;  // Fence.i flush when write operations present
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                        Interrupts                                         //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Interrupt handling state machine
    // Manages interrupt requests and pipeline flushing for interrupt service

    //===================== Interrupt State Registers ============//
    reg flush_in_progress;    // Pipeline flush is in progress due to interrupt
    reg waiting_for_int;      // Waiting for interrupt to be serviced

    //===================== Interrupt Flush State Machine ========//
    // Manage pipeline flush during interrupt handling
    always @(posedge clk_i) begin
        if (~rst_ni) begin
            flush_in_progress <= 'b0;        // Reset: clear flush
        end else if (~valid_inst_in_pipe_i) begin
            flush_in_progress <= 'b0;        // No valid instruction: clear flush
        end else if (interrupt_i) begin
            flush_in_progress <= 'b1;        // Interrupt request: start flush
        end
    end

    //===================== Interrupt Wait State Machine =========//
    // Manage waiting state for interrupt service
    always @(posedge clk_i) begin
        if (~rst_ni) begin
            waiting_for_int <= 'b0;          // Reset: clear wait
        end else if (interrupt_i) begin
            waiting_for_int <= 'b0;          // Interrupt: clear wait
        end else if (wait_for_int_i) begin
            waiting_for_int <= 'b1;          // Wait request: set wait state
        end
    end


    ///////////////////////////////////////////////////////////////////////////////////////////////
    //                                          Outputs                                          //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Generate stage control signals for all pipeline stages
    // Each stage has three control signals: squash, bubble, and stall
    // These signals coordinate hazard resolution and pipeline flow control

    //===================== Fetch Stage Controls ================//
    // Squash: Invalidate fetched instruction due to branch, exception, or interrupt
    assign FCH_squash_o = branch_taken || mem_exc_i || flush_in_progress || ~rst_ni;
    // Bubble: Never bubble fetch stage (always fetch new instruction)
    assign FCH_bubble_o = 'b0;
    // Stall: Stall fetch due to various hazards and memory stalls
    assign FCH_stall_o  = imem_stall_i   || dmem_stall_i || load_use_haz_i || waiting_for_int ||
                          fencei_flush_a || alu_stall_i  || csr_load_use_haz_i;

    //===================== Decode Stage Controls ===============//
    // Squash: Invalidate decoded instruction due to branch or exception
    assign DCD_squash_o = branch_taken || mem_exc_i || ~rst_ni;
    // Bubble: Insert bubble due to fence.i or CSR hazard only when instruction can advance
    assign DCD_bubble_o = fencei_flush_a || csr_load_use_haz_i;
    // Stall: Stall decode due to instruction/data memory, load-use, ALU, or interrupt wait
    assign DCD_stall_o  = dmem_stall_i || load_use_haz_i || alu_stall_i || waiting_for_int;

    //===================== Execute Stage Controls ==============//
    // Squash: Invalidate executed instruction due to memory exception
    assign EXE_squash_o = mem_exc_i || ~rst_ni;
    // Bubble: Insert bubble due to load-use hazard or interrupt wait
    assign EXE_bubble_o = load_use_haz_i || waiting_for_int;
    // Stall: Stall execute due to data memory or ALU stalls
    assign EXE_stall_o  = dmem_stall_i || alu_stall_i;

    //===================== Memory Stage Controls ===============//
    // Squash: Invalidate memory instruction due to reset only
    assign MEM_squash_o = ~rst_ni;
    // Bubble: Insert bubble due to ALU stall
    assign MEM_bubble_o = alu_stall_i;
    // Stall: Stall memory due to data memory stall
    assign MEM_stall_o  = dmem_stall_i;

    //===================== Writeback Stage Controls ============//
    // Squash: Invalidate writeback instruction due to reset only
    assign WB_squash_o  = ~rst_ni;
    // Bubble: Never bubble writeback stage
    assign WB_bubble_o  = 'b0;
    // Stall: Stall writeback due to data memory stall
    assign WB_stall_o   = dmem_stall_i;

    //===================== External Outputs ====================//
    // Fence.i flush signal for external use
    assign fencei_flush_ao = fencei_flush_a;
    // Interrupt output signal
    assign interrupt_o = flush_in_progress;


endmodule
