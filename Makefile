# CPU64 Makefile
# RISC-V RV64IMAC Processor

# Tool configuration
SIMULATOR ?= modelsim
COMPILER ?= vlog
SIMULATOR_CMD ?= vsim
VERILATOR ?= verilator

# Directories
RTL_DIR = rtl
TB_DIR = testbench
SIM_DIR = sim
LOG_DIR = sim/logs

# Source files
CORE_SOURCES = $(wildcard $(RTL_DIR)/core/*.v)
PIPELINE_SOURCES = $(wildcard $(RTL_DIR)/pipeline/*/*.v)
MEMORY_SOURCES = $(wildcard $(RTL_DIR)/memory/*/*.v)
EXECUTION_SOURCES = $(wildcard $(RTL_DIR)/execution/*/*.v)
CONTROL_SOURCES = $(wildcard $(RTL_DIR)/control/*.v)
REGISTER_SOURCES = $(wildcard $(RTL_DIR)/registers/*.v)
INTERFACE_SOURCES = $(wildcard $(RTL_DIR)/interfaces/*.v)
UTIL_SOURCES = $(wildcard $(RTL_DIR)/utils/*.v)

ALL_SOURCES = $(CORE_SOURCES) $(PIPELINE_SOURCES) $(MEMORY_SOURCES) \
              $(EXECUTION_SOURCES) $(CONTROL_SOURCES) $(REGISTER_SOURCES) \
              $(INTERFACE_SOURCES) $(UTIL_SOURCES)

# Testbench files (recursive)
TB_SOURCES = $(shell find $(TB_DIR) -type f -name '*.v')

# Default target
all: compile

# Compile all sources
compile: $(LOG_DIR)
	@echo "Compiling CPU64 design..."
	$(COMPILER) +incdir+$(RTL_DIR)/core +incdir+$(RTL_DIR) $(ALL_SOURCES)
	@echo "Compilation complete."

# Compile testbench
compile_tb: compile
	@echo "Compiling testbench..."
	$(COMPILER) +incdir+$(RTL_DIR)/core +incdir+$(RTL_DIR) $(TB_SOURCES)
	@echo "Testbench compilation complete."

# Run simulation
sim: compile_tb
	@echo "Running simulation..."
	$(SIMULATOR_CMD) -c -do "run -all; quit" cpu64_core_tb
	@echo "Simulation complete."

# Run tests
test: compile_tb
	@echo "Running tests..."
	$(SIMULATOR_CMD) -c -do "run -all; quit" cpu64_core_tb
	@echo "Tests complete."

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(LOG_DIR)/*
	rm -rf work/
	rm -rf transcript
	rm -rf *.wlf
	@echo "Clean complete."

# Create log directory
$(LOG_DIR):
	mkdir -p $(LOG_DIR)

# Verilator unit test for L1 D-cache
L1_TB_DIR := $(TB_DIR)/unit/memory/cache/l1
L1_TB := $(L1_TB_DIR)/cpu64_l1_dcache_tb.v
L1_MEM_MODEL := $(L1_TB_DIR)/cpu64_l1_mem_model.v
L1_CPP := $(L1_TB_DIR)/sim_main.cpp
L1_RTL := \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_dcache.v \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_arrays.v \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_plru.v

DBG_DEFINE := $(if $(filter 1,$(DBG)),-DCPU64_DBG_TRACE,)

VERILATOR_FLAGS := -cc -exe -Wall -Wno-fatal -trace $(DBG_DEFINE)
VERILATOR_INCLUDES := -I$(RTL_DIR) -I$(RTL_DIR)/core

# CPU64 Core sources for Verilator
CPU64_CORE_RTL := \
	$(RTL_DIR)/core/cpu64_defs.vh \
	$(RTL_DIR)/core/cpu64_csr_defs.vh \
	$(RTL_DIR)/core/cpu64_core.v \
	$(RTL_DIR)/core/cpu64_core_w_dcache.v \
	$(RTL_DIR)/utils/cpu64_validity_tracker.v \
	$(RTL_DIR)/control/cpu64_pipeline_controller.v \
	$(RTL_DIR)/registers/cpu64_register_file.v \
	$(RTL_DIR)/registers/cpu64_csr.v \
	$(RTL_DIR)/pipeline/fetch/cpu64_fetch_stage.v \
	$(RTL_DIR)/pipeline/fetch/cpu64_obi_host_driver.v \
	$(RTL_DIR)/pipeline/decode/cpu64_decode_stage.v \
	$(RTL_DIR)/pipeline/decode/cpu64_decoder.v \
	$(RTL_DIR)/pipeline/decode/cpu64_alu_decoder.v \
	$(RTL_DIR)/pipeline/decode/cpu64_compressed_decoder.v \
	$(RTL_DIR)/pipeline/execute/cpu64_execute_stage.v \
	$(RTL_DIR)/pipeline/execute/cpu64_I_alu.v \
	$(RTL_DIR)/pipeline/execute/cpu64_M_alu.v \
	$(RTL_DIR)/pipeline/execute/cpu64_bypass_unit.v \
	$(RTL_DIR)/pipeline/execute/cpu64_divider.v \
	$(RTL_DIR)/pipeline/memory/cpu64_memory_stage.v \
	$(RTL_DIR)/pipeline/writeback/cpu64_writeback_stage.v
# ===================== Core+DCACHE Integration TB (Verilator) ===================== #
CORE_DCACHE_TB_DIR := $(TB_DIR)/integration/core_dcache
CORE_DCACHE_TB := $(CORE_DCACHE_TB_DIR)/cpu64_core_w_dcache_tb.v
CORE_DCACHE_CPP := $(CORE_DCACHE_TB_DIR)/sim_main_core_dcache.cpp
CORE_DCACHE_MEM := $(CORE_DCACHE_TB_DIR)/obi_mem_model_adv.v

# ===================== Core+ICACHE Integration TB (Verilator) ===================== #
CORE_ICACHE_TB_DIR := $(TB_DIR)/integration/core_icache
CORE_ICACHE_TB := $(CORE_ICACHE_TB_DIR)/cpu64_core_w_icache_tb.v
CORE_ICACHE_CPP := $(CORE_ICACHE_TB_DIR)/sim_main_core_icache.cpp

# ===================== Core+ICACHE+DCACHE Full Integration TB ===================== #
CORE_FULL_TB_DIR := $(TB_DIR)/integration/core_full
CORE_FULL_TB := $(CORE_FULL_TB_DIR)/cpu64_core_w_icache_dcache_tb.v
CORE_FULL_CPP := $(CORE_FULL_TB_DIR)/sim_main_core_full.cpp
CORE_FULL_MEM := $(CORE_FULL_TB_DIR)/obi_mem_model_unified.v

# ICache RTL modules
ICACHE_RTL := \
	$(RTL_DIR)/memory/cache/icache/cpu64_icache_top.v \
	$(RTL_DIR)/memory/cache/icache/cpu64_l1i_arrays.v \
	$(RTL_DIR)/memory/cache/icache/cpu64_l1i_plru.v \
	$(RTL_DIR)/memory/cache/icache/cpu64_obi_host_driver_icache.v

# DCache stack RTL modules
DCACHE_RTL := \
	$(RTL_DIR)/memory/cache/cpu64_cache_stack.v \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_dcache.v \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_arrays.v \
	$(RTL_DIR)/memory/cache/l1/cpu64_l1_plru.v \
	$(RTL_DIR)/memory/cache/l2/cpu64_l2_dcache.v \
	$(RTL_DIR)/memory/cache/l2/cpu64_l2_arrays.v \
	$(RTL_DIR)/memory/cache/l2/cpu64_l2_plru.v \
	$(RTL_DIR)/memory/cache/l3/cpu64_l3_dcache.v \
	$(RTL_DIR)/memory/cache/l3/cpu64_l3_arrays.v \
	$(RTL_DIR)/memory/cache/l3/cpu64_l3_plru.v

# Combined cache RTL (backward compatibility)
CACHE_RTL := $(DCACHE_RTL) $(ICACHE_RTL)

OBI_RX_RTL := $(RTL_DIR)/pipeline/memory/cpu64_obi_receiver.v

# Standalone icache wrapper test RTL
CORE_ICACHE_RTL := \
	$(RTL_DIR)/core/cpu64_icache_wrapper.v \
	$(ICACHE_RTL)

# Core + dcache RTL
CORE_DCACHE_RTL := \
	$(CPU64_CORE_RTL) \
	$(DCACHE_RTL) \
	$(OBI_RX_RTL)

# Core + icache + dcache RTL (full integration)
CORE_FULL_RTL := \
	$(RTL_DIR)/core/cpu64_core_w_icache_dcache.v \
	$(CPU64_CORE_RTL) \
	$(ICACHE_RTL) \
	$(DCACHE_RTL) \
	$(OBI_RX_RTL)

.PHONY: lint_core_dcache verilate_core_dcache build_core_dcache run_core_dcache

lint_core_dcache:
	@echo "Running Verilator lint check on core+dcache TB..."
	$(VERILATOR) --lint-only -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_core_w_dcache_tb \
		$(CORE_DCACHE_TB) $(CORE_DCACHE_MEM) $(CORE_DCACHE_RTL)
	@echo "Lint complete."

verilate_core_dcache: clean_verilator
	$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INCLUDES) \
		$(CORE_DCACHE_CPP) $(CORE_DCACHE_TB) $(CORE_DCACHE_MEM) $(CORE_DCACHE_RTL)

build_core_dcache: verilate_core_dcache
	$(MAKE) -C obj_dir -f Vcpu64_core_w_dcache_tb.mk Vcpu64_core_w_dcache_tb

# Usage: make run_core_dcache HEX=hex/comprehensive_program.hex [TRACE=1]
run_core_dcache: build_core_dcache
	@HEX_FILE=$(HEX); \
	if [ -z "$$HEX_FILE" ]; then HEX_FILE=hex/simple_calculator.hex; fi; \
	CMD="./obj_dir/Vcpu64_core_w_dcache_tb +TRACE"; \
	mkdir -p $(LOG_DIR); \
	LOG=$(LOG_DIR)/run_core_dcache.log; \
	$$CMD +HEX=$$HEX_FILE $(PLUSARGS) | tee $$LOG; \
	echo ""; \
	echo "=== Generating Pipeline Visualization ==="; \
	python3 tools/pipeline_visualizer.py && \
	echo "Pipeline visualization saved to: obj_dir/pipeline_visualization.txt"



# Note: Standalone icache wrapper test (no C++ driver yet)
# .PHONY: lint_core_icache verilate_core_icache build_core_icache run_core_icache
# lint_core_icache:
# 	@echo "Running Verilator lint check on icache wrapper TB..."
# 	$(VERILATOR) --lint-only -timing -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_core_w_icache_tb \
# 		$(CORE_ICACHE_TB) $(CORE_ICACHE_RTL)
# 	@echo "Lint complete."

# ===================== Core+ICACHE+DCACHE Full Integration Targets ===================== #
.PHONY: lint_core_full verilate_core_full build_core_full run_core_full

lint_core_full:
	@echo "Running Verilator lint check on core+icache+dcache TB..."
	$(VERILATOR) --lint-only --timing -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_core_w_icache_dcache_tb \
		$(CORE_FULL_TB) $(CORE_FULL_MEM) $(CORE_FULL_RTL)
	@echo "Lint complete."

verilate_core_full: clean_verilator
	$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INCLUDES) --timing \
		$(CORE_FULL_CPP) $(CORE_FULL_TB) $(CORE_FULL_MEM) $(CORE_FULL_RTL)

build_core_full: verilate_core_full
	$(MAKE) -C obj_dir -f Vcpu64_core_w_icache_dcache_tb.mk Vcpu64_core_w_icache_dcache_tb

# Usage: make run_core_full HEX=hex/simple_calculator.hex [TRACE=1]
run_core_full: build_core_full
	@HEX_FILE=$(HEX); \
	if [ -z "$$HEX_FILE" ]; then HEX_FILE=hex/simple_calculator.hex; fi; \
	CMD="./obj_dir/Vcpu64_core_w_icache_dcache_tb"; \
	if [ "$(TRACE)" = "1" ]; then CMD="$$CMD +TRACE"; fi; \
	mkdir -p $(LOG_DIR); \
	LOG=$(LOG_DIR)/run_core_full.log; \
	$$CMD +HEX=$$HEX_FILE $(PLUSARGS) | tee $$LOG








clean_verilator:
	rm -rf obj_dir obj_dir/wave.vcd

# L1 Cache Verilator targets
lint_l1:
	$(VERILATOR) --lint-only -Wall -Wno-fatal $(L1_RTL) $(L1_TB) $(L1_MEM_MODEL)

verilate_l1: clean_verilator
	$(VERILATOR) $(VERILATOR_FLAGS) $(L1_CPP) $(L1_TB) $(L1_MEM_MODEL) $(L1_RTL)

build_l1: verilate_l1
	$(MAKE) -C obj_dir -f Vcpu64_l1_dcache_tb.mk Vcpu64_l1_dcache_tb

run_l1: build_l1
	obj_dir/Vcpu64_l1_dcache_tb

# CPU64 Core Verilator targets
lint_core:
	@echo "Running Verilator lint check on CPU64 core..."
	$(VERILATOR) --lint-only -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_core $(CPU64_CORE_RTL)
	@echo "Verilator lint check completed."

lint_memory:
	@echo "Running Verilator lint check on memory stage..."
	$(VERILATOR) --lint-only -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_memory_stage \
		$(RTL_DIR)/core/cpu64_defs.vh \
		$(RTL_DIR)/utils/cpu64_validity_tracker.v \
		$(RTL_DIR)/pipeline/fetch/cpu64_obi_host_driver.v \
		$(RTL_DIR)/pipeline/memory/cpu64_memory_stage.v
	@echo "Memory stage lint check completed."

lint_execute:
	@echo "Running Verilator lint check on execute stage..."
	$(VERILATOR) --lint-only -Wall -Wno-fatal $(VERILATOR_INCLUDES) --top-module cpu64_execute_stage \
		$(RTL_DIR)/core/cpu64_defs.vh \
		$(RTL_DIR)/utils/cpu64_validity_tracker.v \
		$(RTL_DIR)/pipeline/execute/cpu64_I_alu.v \
		$(RTL_DIR)/pipeline/execute/cpu64_M_alu.v \
		$(RTL_DIR)/pipeline/execute/cpu64_bypass_unit.v \
		$(RTL_DIR)/pipeline/execute/cpu64_divider.v \
		$(RTL_DIR)/pipeline/execute/cpu64_execute_stage.v
	@echo "Execute stage lint check completed."

# Help
help:
	@echo "Available targets:"
	@echo "  compile       - Compile RTL sources"
	@echo "  compile_tb    - Compile testbench"
	@echo "  sim           - Run simulation"
	@echo "  test          - Run tests"
	@echo ""
	@echo "Verilator Integration Tests:"
	@echo "  lint_core_dcache   - Lint core+dcache integration"
	@echo "  build_core_dcache  - Build core+dcache integration"
	@echo "  run_core_dcache    - Run core+dcache (HEX=<file> TRACE=1)"
	@echo ""
	@echo "  lint_core_full     - Lint core+icache+dcache integration"
	@echo "  build_core_full    - Build core+icache+dcache integration"
	@echo "  run_core_full      - Run core+icache+dcache (HEX=<file> TRACE=1)"
	@echo ""
	@echo "Unit Tests:"
	@echo "  lint_l1       - Verilator lint for L1 unit TB"
	@echo "  build_l1      - Build L1 unit TB"
	@echo "  run_l1        - Run L1 unit TB"
	@echo ""
	@echo "Lint Only:"
	@echo "  lint_core     - Verilator lint for CPU64 core"
	@echo "  lint_memory   - Verilator lint for memory stage"
	@echo "  lint_execute  - Verilator lint for execute stage"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean         - Clean build artifacts"
	@echo "  clean_verilator - Remove Verilator build outputs (obj_dir, wave.vcd)"

.PHONY: all compile compile_tb sim test clean help lint_l1 verilate_l1 build_l1 run_l1 clean_verilator lint_core lint_memory lint_execute
