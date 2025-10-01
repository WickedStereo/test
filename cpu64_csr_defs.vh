///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// File Name  : cpu64_csr_defs.vh                                                               //
// Description: This header contains definitions used for the Control and Status Register (CSR)s //
//              implemented here. The CSRs defined are in compliance with the RISC-V Machine ISA //
//              v1.12, and further documentation can be found in the The RISC-V Instruction Set  //
//              Manual Volume II: Privileged Architecture. This file defines all CSR addresses, //
//              privilege modes, trap causes, and machine-specific constants for the CPU64 core. //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
`ifndef CSR_VH
`define CSR_VH



///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Privilege Modes                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
// RISC-V privilege levels for instruction execution and memory access control
// Higher privilege levels can access lower privilege level resources

`define USER_MODE           2'b00    // User mode (U-mode): Lowest privilege level
`define SUPERVISOR_MODE     2'b10    // Supervisor mode (S-mode): Middle privilege level
`define MACHINE_MODE        2'b11    // Machine mode (M-mode): Highest privilege level

`define LOWEST_PRIVILEGE    `MACHINE_MODE  // CPU64 implements only machine mode


///////////////////////////////////////////////////////////////////////////////////////////////////
//       __  __            _     _              __  __          _         ___  ___  ___          //
//      |  \/  | __ _  __ | |_  (_) _ _   ___  |  \/  | ___  __| | ___   / __|/ __|| _ \ ___     //
//      | |\/| |/ _` |/ _|| ' \ | || ' \ / -_) | |\/| |/ _ \/ _` |/ -_) | (__ \__ \|   /(_-<     //
//      |_|  |_|\__,_|\__||_||_||_||_||_|\___| |_|  |_|\___/\__,_|\___|  \___||___/|_|_\/__/     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Register Addresses                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
// 12-bit CSR addresses as defined in RISC-V Privileged Architecture Specification
// CSRs are organized by functional groups for better understanding

//===================== Machine Information Registers (Read-Only) ===============//
// These registers provide information about the machine implementation
`define CSR_MVENDORID       12'hF11     // Vendor ID: Identifies the vendor
`define CSR_MARCHID         12'hF12     // Architecture ID: Identifies the architecture
`define CSR_MIMPID          12'hF13     // Implementation ID: Identifies the implementation
`define CSR_MHARTID         12'hF14     // Hardware thread ID: Identifies the hardware thread
`define CSR_MCONFIGPTR      12'hF15     // Configuration pointer: Points to configuration data

//===================== Machine Trap Setup (Read-Write) ========================//
// These registers control trap handling and interrupt behavior
`define CSR_MSTATUS         12'h300     // Machine status: Global interrupt enable and status
`define CSR_MISA            12'h301     // Machine ISA: Supported ISA extensions and XLEN
`define CSR_MEDELEG         12'h302     // Machine exception delegation (S extension only)
`define CSR_MIDELEG         12'h303     // Machine interrupt delegation (S extension only)
`define CSR_MIE             12'h304     // Machine interrupt-enable: Interrupt enable bits
`define CSR_MTVEC           12'h305     // Machine trap vector: Trap handler base address
`define CSR_MCOUNTEREN      12'h306     // Machine counter enable: Counter access control

//===================== Machine Counter Setup (Read-Write) =====================//
// These registers control performance monitoring counters
`define CSR_MCOUNTINHIBIT   12'h320     // Machine counter inhibit: Counter enable/disable
`define CSR_MHPMEVENT_3     12'h323     // Machine performance monitoring event selector 3
                                        // Range: (CSR_MHPMEVENT_3) to (CSR_MHPMEVENT_3 + 28)

//===================== Machine Trap Handling (Read-Write) =====================//
// These registers are used during trap handling
`define CSR_MSCRATCH        12'h340     // Machine scratch: Scratch register for trap handlers
`define CSR_MEPC            12'h341     // Machine exception PC: PC when exception occurred
`define CSR_MCAUSE          12'h342     // Machine cause: Exception/interrupt cause code
`define CSR_MTVAL           12'h343     // Machine trap value: Bad address or instruction
`define CSR_MIP             12'h344     // Machine interrupt pending: Pending interrupt bits
`define CSR_MTINST          12'h34A     // Machine trap instruction: Transformed instruction
`define CSR_MTVAL2          12'h34B     // Machine trap value 2: Bad guest physical address

//===================== Machine Configuration (Read-Write) =====================//
// These registers control machine configuration
`define CSR_MENVCFG         12'h30A     // Machine environment configuration
`define CSR_MSECCFG         12'h747     // Machine security configuration

//===================== Machine Memory Protection (Read-Write) =================//
// These registers control physical memory protection
`define CSR_PMPCFG_BASE     12'h3A0     // PMP configuration: Memory protection settings
                                        // Range: (CSR_PMPCFG_BASE) to (CSR_PMPCFG_BASE + 15)
                                        // Note: Odd numbered registers only in RV32
`define CSR_PMPADDR_BASE    12'h3B0     // PMP address: Memory protection address ranges
                                        // Range: (CSR_PMPADDR_BASE) to (CSR_PMPADDR_BASE + 63)

//===================== Machine Hardware Performance Counters ==================//
// These registers provide performance monitoring capabilities
`define CSR_MCYCLE          12'hB00     // Machine cycle counter: CPU cycle count
`define CSR_MINSTRET        12'hB02     // Machine instruction retired: Instructions executed

//===================== Performance Counter Configuration ======================//
`define NUM_CSR_MHPM_COUNTERS 32        // Number of hardware performance monitoring counters

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                 Machine Information Registers                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Default values for machine information registers
// These values identify the CPU64 implementation

`define VENDOR_ID_VAL       'b0    // Vendor ID: 0 = Unknown vendor
`define MARCH_ID_VAL        'b0    // Architecture ID: 0 = Unknown architecture
`define IMP_ID_VAL          'b0    // Implementation ID: 0 = Unknown implementation
`define HART_ID_VAL         'b0    // Hardware thread ID: 0 = Single-threaded
`define CONFIG_PTR_VAL      'b0    // Configuration pointer: 0 = No configuration data


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Machine Trap Setup                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
// Constants for machine trap setup and handling registers

//===================== Machine ISA Register (misa) =============================//
// Hardware supported ISA extensions: ZY_XWVU_TSRQ_PONM_LKJI_HGFE_DCBA
// Bit positions correspond to RISC-V extension letters
`define MISA_SUPPORTED      26'b00_0000_0000_0001_0001_0000_0101 // IMAC extensions
`define MXLEN               2'd2                                 // XLEN = 64 (RV64)

//===================== Machine Trap Vector (mtvec) Modes ======================//
// Trap vector addressing modes
`define MTVEC_MODE_DIRECT   2'b0    // Direct mode: All traps go to base address
`define MTVEC_MODE_VECTORED 2'b1    // Vectored mode: Traps go to base + (cause * 4)

//===================== Machine Cause (mcause) Interrupt Codes =================//
// Interrupt cause codes (bit 63 = 1 for interrupts)
`define MCAUSE_SUPERV_SOFT_INT          63'd1    // Supervisor software interrupt
`define MCAUSE_MACH_SOFT_INT            63'd3    // Machine software interrupt
`define MCAUSE_SUPERV_TIMER_INT         63'd5    // Supervisor timer interrupt
`define MCAUSE_MACH_TIMER_INT           63'd7    // Machine timer interrupt
`define MCAUSE_SUPERV_EXT_INT           63'd9    // Supervisor external interrupt
`define MCAUSE_MACH_EXT_INT             63'd11   // Machine external interrupt

//===================== Machine Cause (mcause) Exception Codes =================//
// Exception cause codes (bit 63 = 0 for exceptions)
`define MCAUSE_INST_ADDR_MISALIGNED     63'd0    // Instruction address misaligned
`define MCAUSE_INST_ACCESS_FAULT        63'd1    // Instruction access fault
`define MCAUSE_ILLEGAL_INST             63'd2    // Illegal instruction
`define MCAUSE_BREAKPOINT               63'd3    // Breakpoint
`define MCAUSE_LOAD_ADDR_MISALIGNED     63'd4    // Load address misaligned
`define MCAUSE_LOAD_ACCESS_FAULT        63'd5    // Load access fault
`define MCAUSE_STORE_AMO_MISALIGNED     63'd6    // Store/AMO address misaligned
`define MCAUSE_STORE_AMO_ACCESS_FAULT   63'd7    // Store/AMO access fault
`define MCAUSE_ECALL_FROM_U_MODE        63'd8    // Environment call from U-mode
`define MCAUSE_ECALL_FROM_S_MODE        63'd9    // Environment call from S-mode
`define MCAUSE_ECALL_FROM_M_MODE        63'd11   // Environment call from M-mode
`define MCAUSE_INST_PAGE_FAULT          63'd12   // Instruction page fault
`define MCAUSE_LOAD_PAGE_FAULT          63'd13   // Load page fault
`define MCAUSE_STORE_AMO_PAGE_FAULT     63'd15   // Store/AMO page fault

//===================== Machine Status (mstatus) Endianness ====================//
// Data endianness configuration
`define MSTATUS_LITTLE_ENDIAN           1'b0    // Little-endian data access
`define MSTATUS_BIG_ENDIAN              1'b1    // Big-endian data access


///////////////////////////////////////////////////////////////////////////////////////////////////
//               _   _                  __  __          _         ___  ___  ___                  //
//              | | | | ___  ___  _ _  |  \/  | ___  __| | ___   / __|/ __|| _ \ ___             //
//              | |_| |(_-< / -_)| '_| | |\/| |/ _ \/ _` |/ -_) | (__ \__ \|   /(_-<             //
//               \___/ /__/ \___||_|   |_|  |_|\___/\__,_|\___|  \___||___/|_|_\/__/             //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      User Counter Registers                                   //
///////////////////////////////////////////////////////////////////////////////////////////////////
// User-accessible counter registers (read-only in user mode)
// These provide performance monitoring capabilities to user programs

`define CSR_CYCLE       12'hC00    // Cycle counter: CPU cycle count (user accessible)
`define CSR_TIME        12'hC01    // Time counter: Real-time counter (user accessible)
`define CSR_INSTRET     12'hC02    // Instruction retired: Instructions executed (user accessible)


`endif // CSR_VH
