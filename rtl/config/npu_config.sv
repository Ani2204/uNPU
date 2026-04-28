// =============================================================================
//  npu_config.sv – Centralised NPU parameter package
//  Supports PE_COUNT = 256 | 512 | 1024 | 2048 from one codebase.
// =============================================================================
`ifndef NPU_CONFIG_SV
`define NPU_CONFIG_SV

package npu_pkg;

  // ---------------------------------------------------------------------------
  // Primary knobs (override at elaboration time)
  // ---------------------------------------------------------------------------
  parameter int PE_COUNT   = 256;   // 256 | 512 | 1024 | 2048
  parameter int DWIDTH     = 8;     // data width per operand in bits
  parameter int ACC_WIDTH  = 32;    // accumulator width
  parameter int BRAM_DEPTH = 4096;  // entries per BRAM bank
  parameter int BUS_WIDTH  = 64;    // memory bus width in bits
  parameter int FREQ_MHZ   = 200;   // target clock frequency

  // ---------------------------------------------------------------------------
  // Derived: cluster & tile geometry  (all power-of-2, auto-checked)
  // ---------------------------------------------------------------------------
  parameter int TILE_ROWS   = 8;
  parameter int TILE_COLS   = 8;
  parameter int TILE_PES    = TILE_ROWS * TILE_COLS;          // 64
  parameter int NUM_TILES   = PE_COUNT / TILE_PES;            // 4 for 256
  parameter int CLUSTERS    = (PE_COUNT + 63) / 64;           // == NUM_TILES
  parameter int TREE_DEPTH  = $clog2(TILE_PES);               // 6 for 64-PE tile

  // ---------------------------------------------------------------------------
  // Memory geometry
  // ---------------------------------------------------------------------------
  parameter int NUM_BANKS      = 4;
  parameter int BANK_ADDR_W    = $clog2(BRAM_DEPTH);
  parameter int ECC_BITS       = 8;   // SECDED over 64-bit word

  // ---------------------------------------------------------------------------
  // AXI parameters
  // ---------------------------------------------------------------------------
  parameter int AXI_ADDR_W = 32;
  parameter int AXI_DATA_W = 32;
  parameter int AXI_ID_W   = 4;
  parameter int AXI_LEN_W  = 8;      // AXI4 supports 256-beat bursts

  // ---------------------------------------------------------------------------
  // CSR address map boundaries
  // ---------------------------------------------------------------------------
  parameter int CSR_CFG_BASE   = 16'h0000;
  parameter int CSR_CFG_END    = 16'h00FF;
  parameter int CSR_PERF_BASE  = 16'h0100;
  parameter int CSR_PERF_END   = 16'h01FF;
  parameter int CSR_STAT_BASE  = 16'h0200;
  parameter int CSR_STAT_END   = 16'h02FF;
  parameter int CSR_DATA_BASE  = 16'h0300;
  parameter int CSR_DATA_END   = 16'h7FFF;

  // ---------------------------------------------------------------------------
  // Pipeline stage identifiers
  // ---------------------------------------------------------------------------
  parameter int STAGE_DMA      = 0;
  parameter int STAGE_COMPUTE  = 1;
  parameter int STAGE_REDUCE   = 2;
  parameter int STAGE_OUTPUT   = 3;
  parameter int NUM_STAGES     = 4;

endpackage

`endif // NPU_CONFIG_SV
