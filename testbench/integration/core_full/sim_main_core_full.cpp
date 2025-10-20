#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vcpu64_core_w_icache_dcache_tb.h"

static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    bool trace_enable = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "+TRACE") { trace_enable = true; }
    }

    Vcpu64_core_w_icache_dcache_tb* tb = new Vcpu64_core_w_icache_dcache_tb;

    VerilatedVcdC* tfp = nullptr;
    if (trace_enable) {
        Verilated::traceEverOn(true);
        tfp = new VerilatedVcdC;
        tb->trace(tfp, 99);
        tfp->open("obj_dir/cpu64_core_w_icache_dcache_tb.vcd");
    }

    // Drive clock and reset
    tb->clk = 0;
    tb->rst_n = 0;

    auto tick = [&]() {
        tb->clk = !tb->clk;
        tb->eval();
        // Dump all VCD cycles (changed from limited window)
        if (tfp) tfp->dump(main_time);
        main_time++;
    };

    // Reset for a few cycles
    for (int i = 0; i < 10; i++) tick();
    tb->rst_n = 1;

    // Run for some cycles or until $finish (increased for slow pipeline - observing ~16K cycles/iteration)
    const vluint64_t max_cycles = 500000;  // Allow 250K clock cycles
    while (!Verilated::gotFinish() && main_time < max_cycles) {
        tick();
    }

    if (tfp) {
        tfp->close();
        delete tfp;
    }
    delete tb;
    return 0;
}

