// =============================================================================
//  npu_defines.vh – Preprocessor macro definitions for the NPU
// =============================================================================
`ifndef NPU_DEFINES_VH
`define NPU_DEFINES_VH

// ---------------------------------------------------------------------------
// Version
// ---------------------------------------------------------------------------
`define NPU_VERSION_MAJOR  2
`define NPU_VERSION_MINOR  0
`define NPU_VERSION_PATCH  0
`define NPU_VERSION        32'h02000000

// ---------------------------------------------------------------------------
// Vendor / IP identification
// ---------------------------------------------------------------------------
`define NPU_VENDOR_ID      16'hAB12
`define NPU_DEVICE_ID      16'h4E50   // ASCII "NP"

// ---------------------------------------------------------------------------
// Default parameters (can be overridden per-module via parameters)
// ---------------------------------------------------------------------------
`define PE_COUNT    256
`define DWIDTH      8
`define ACC_WIDTH   32
`define BRAM_DEPTH  4096
`define BUS_WIDTH   64
`define FREQ_MHZ    200
`define BANK_ADDR_W 12   // clog2(4096)

// ---------------------------------------------------------------------------
// Memory bank assignments
// ---------------------------------------------------------------------------
`define BANK_WEIGHTS   2'b00   // Bank 0: Weights (read-only)
`define BANK_ACT       2'b01   // Bank 1: Activations (read/write)
`define BANK_INTER     2'b10   // Bank 2: Intermediate
`define BANK_OUTPUT    2'b11   // Bank 3: Outputs (write-only)

// ---------------------------------------------------------------------------
// AXI response codes
// ---------------------------------------------------------------------------
`define AXI_OKAY    2'b00
`define AXI_EXOKAY  2'b01
`define AXI_SLVERR  2'b10
`define AXI_DECERR  2'b11

// ---------------------------------------------------------------------------
// AXI burst type
// ---------------------------------------------------------------------------
`define AXI_BURST_FIXED  2'b00
`define AXI_BURST_INCR   2'b01
`define AXI_BURST_WRAP   2'b10

// ---------------------------------------------------------------------------
// CSR base addresses
// ---------------------------------------------------------------------------
`define CSR_CFG_BASE   16'h0000
`define CSR_PERF_BASE  16'h0100
`define CSR_STAT_BASE  16'h0200
`define CSR_DATA_BASE  16'h0300

// ---------------------------------------------------------------------------
// Specific CSR offsets (from their base)
// ---------------------------------------------------------------------------
// Config registers (0x0000 base)
`define CSR_MODE_SEL       12'h000   // [1:0] precision mode
`define CSR_FUSE_EN        12'h004   // [0] accumulation enable
`define CSR_ACT_SEL        12'h008   // [3:0] activation function
`define CSR_BIAS           12'h00C   // [31:0] bias value
`define CSR_SCALE          12'h010   // [3:0] scale factor
`define CSR_SHIFT          12'h014   // [3:0] right shift
`define CSR_GATE_EN        12'h018   // [0] global gate enable
`define CSR_THRESHOLD      12'h01C   // [7:0] sparsity threshold
`define CSR_DYN_SCHED      12'h020   // [0] dynamic scheduler enable
`define CSR_SLICE_EN       12'h024   // [0] bit-slicing enable
`define CSR_MSB_THRES      12'h028   // [3:0] MSB stat threshold
`define CSR_RELU_EN        12'h02C   // [0] ReLU enable
`define CSR_DATA_SEL       12'h030   // [0] data source: 0=DMA, 1=CSR
`define CSR_PE_COUNT_CFG   12'h034   // [11:0] active PE count
`define CSR_MAC_RESULT     12'h038   // [31:0] (read-only) last MAC result

// Performance monitors (0x0100 base)
`define CSR_PERF_CTRL      12'h100   // [0] start, [1] clear
`define CSR_PERF_CYC_LO    12'h104   // [31:0] cycle counter lo
`define CSR_PERF_CYC_HI    12'h108   // [31:0] cycle counter hi
`define CSR_PERF_OPS_LO    12'h10C   // [31:0] op counter lo
`define CSR_PERF_OPS_HI    12'h110   // [31:0] op counter hi
`define CSR_PERF_PE_SUM    12'h114   // [31:0] active PE sum
`define CSR_PERF_LATENCY   12'h118   // [31:0] last batch latency

// Status & interrupts (0x0200 base)
`define CSR_STATUS         12'h200   // [0] busy, [1] done, [7] error
`define CSR_IRQ_STATUS     12'h204   // IRQ source bits
`define CSR_IRQ_ENABLE     12'h208   // IRQ enable mask
`define CSR_IRQ_CLEAR      12'h20C   // write-1-to-clear
`define CSR_ECC_STATUS     12'h210   // [0] single-bit, [1] double-bit
`define CSR_BIST_STATUS    12'h214   // [0] running, [1] pass, [2] fail

// DMA registers (0x0240 base relative to CSR_STAT)
`define CSR_DMA_CTRL       12'h240   // [0]=start, [1]=clear, [3]=INT4
`define CSR_DMA_SRC_ADDR   12'h244   // source byte address
`define CSR_DMA_DST_ADDR   12'h248   // destination byte address
`define CSR_DMA_LEN        12'h24C   // byte length
`define CSR_DMA_STATUS     12'h250   // [0]=done, [1]=busy, [31]=error

// ---------------------------------------------------------------------------
// Synthesis pragmas (Xilinx / Vivado)
// ---------------------------------------------------------------------------
`define BRAM_ATTR  (* ram_style = "block" *)
`define DIST_ATTR  (* ram_style = "distributed" *)
`define DSP_ATTR   (* use_dsp = "yes" *)
`define NO_DSP     (* use_dsp = "no" *)
`define KEEP_HIER  (* keep_hierarchy = "yes", dont_touch = "yes" *)

`endif // NPU_DEFINES_VH
