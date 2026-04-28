// =============================================================================
//  npu_typedefs.sv – SystemVerilog types used throughout the NPU
// =============================================================================
`ifndef NPU_TYPEDEFS_SV
`define NPU_TYPEDEFS_SV

`include "npu_defines.vh"

package npu_types_pkg;

  // ---------------------------------------------------------------------------
  // Precision mode
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    MODE_INT8 = 2'b00,
    MODE_INT4 = 2'b01,
    MODE_INT2 = 2'b10,
    MODE_AUTO = 2'b11
  } mode_e;

  // ---------------------------------------------------------------------------
  // Micro-instruction opcodes
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    OP_NOP     = 3'd0,
    OP_LOAD    = 3'd1,
    OP_COMPUTE = 3'd2,
    OP_REDUCE  = 3'd3,
    OP_STORE   = 3'd4,
    OP_SYNC    = 3'd5
  } opcode_e;

  // ---------------------------------------------------------------------------
  // Micro-instruction record
  // ---------------------------------------------------------------------------
  typedef struct packed {
    opcode_e         opcode;
    logic [4:0]      flags;
    logic [23:0]     operand;
  } uinstr_t;

  // ---------------------------------------------------------------------------
  // DMA job descriptor
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0] src_addr;
    logic [31:0] dst_addr;
    logic [15:0] byte_len;
    logic [1:0]  mode;      // 00=INT8, 01=INT4, 10=weight, 11=activation
    logic        valid;
  } dma_job_t;

  // ---------------------------------------------------------------------------
  // Memory bank request
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [`BANK_ADDR_W-1:0] addr;
    logic [`BUS_WIDTH-1:0]   wdata;
    logic [`BUS_WIDTH/8-1:0] wstrb;
    logic                    wen;
    logic                    ren;
    logic [2:0]              bank_sel;
    logic                    valid;
  } mem_req_t;

  // ---------------------------------------------------------------------------
  // Memory bank response
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [`BUS_WIDTH-1:0]   rdata;
    logic                    ecc_err_single;
    logic                    ecc_err_double;
    logic                    valid;
  } mem_rsp_t;

  // ---------------------------------------------------------------------------
  // Pipeline stage handshake
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic valid;
    logic ready;
    logic last;
  } hs_t;

  // ---------------------------------------------------------------------------
  // PE state
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PE_IDLE   = 2'b00,
    PE_LOAD   = 2'b01,
    PE_EXEC   = 2'b10,
    PE_DRAIN  = 2'b11
  } pe_state_e;

  // ---------------------------------------------------------------------------
  // Interrupt sources
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic dma_done;
    logic compute_done;
    logic ecc_single;
    logic ecc_double;
    logic watchdog;
    logic bist_done;
    logic bist_fail;
  } irq_src_t;

endpackage

`endif // NPU_TYPEDEFS_SV
