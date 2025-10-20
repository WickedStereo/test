// Top includes file for CPU64 RTL
// This file includes all Verilog modules in the cpu64rtl_new directory

// Definition files (must be included first)
`include "rtl/core/cpu64_defs.vh"
`include "rtl/core/cpu64_csr_defs.vh"

// Control modules
`include "rtl/control/cpu64_pipeline_controller.v"

// Core modules
`include "rtl/core/cpu64_core.v"
`include "rtl/core/cpu64_core_w_icache_dcache.v"

// Memory cache modules
`include "rtl/memory/cache/cpu64_cache_stack.v"

// Instruction cache modules
`include "rtl/memory/cache/icache/cpu64_icache_top.v"
`include "rtl/memory/cache/icache/cpu64_l1i_arrays.v"
`include "rtl/memory/cache/icache/cpu64_l1i_plru.v"
`include "rtl/memory/cache/icache/cpu64_obi_host_driver_icache.v"

// L1 Data cache modules
`include "rtl/memory/cache/l1/cpu64_l1_arrays.v"
`include "rtl/memory/cache/l1/cpu64_l1_dcache.v"
`include "rtl/memory/cache/l1/cpu64_l1_plru.v"

// L2 Data cache modules
`include "rtl/memory/cache/l2/cpu64_l2_arrays.v"
`include "rtl/memory/cache/l2/cpu64_l2_dcache.v"
`include "rtl/memory/cache/l2/cpu64_l2_plru.v"

// L3 Data cache modules
`include "rtl/memory/cache/l3/cpu64_l3_arrays.v"
`include "rtl/memory/cache/l3/cpu64_l3_dcache.v"
`include "rtl/memory/cache/l3/cpu64_l3_plru.v"

// Pipeline decode modules
`include "rtl/pipeline/decode/cpu64_alu_decoder.v"
`include "rtl/pipeline/decode/cpu64_compressed_decoder.v"
`include "rtl/pipeline/decode/cpu64_decode_stage.v"
`include "rtl/pipeline/decode/cpu64_decoder.v"

// Pipeline execute modules
`include "rtl/pipeline/execute/cpu64_I_alu.v"
`include "rtl/pipeline/execute/cpu64_M_alu.v"
`include "rtl/pipeline/execute/cpu64_bypass_unit.v"
`include "rtl/pipeline/execute/cpu64_divider.v"
`include "rtl/pipeline/execute/cpu64_execute_stage.v"

// Pipeline fetch modules
`include "rtl/pipeline/fetch/cpu64_fetch_stage.v"
`include "rtl/pipeline/fetch/cpu64_obi_host_driver.v"

// Pipeline memory modules
`include "rtl/pipeline/memory/cpu64_memory_stage.v"
`include "rtl/pipeline/memory/cpu64_obi_receiver.v"

// Pipeline writeback modules
`include "rtl/pipeline/writeback/cpu64_writeback_stage.v"

// Register modules
`include "rtl/registers/cpu64_csr.v"
`include "rtl/registers/cpu64_register_file.v"

// Utility modules
`include "rtl/utils/cpu64_validity_tracker.v"

