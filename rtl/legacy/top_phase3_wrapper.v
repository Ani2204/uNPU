// =============================================================================
//  top_phase3_wrapper.v – Backward Compatibility Wrapper
//
//  Preserves the exact port interface of the original top_phase3.v while
//  delegating all functionality to the new npu_top.v.
//
//  Migration note:
//    - AXI-Lite ports (no awlen/awsize/awburst) are expanded to AXI4-Full
//      with single-beat burst defaults (awlen=0, awsize=2, awburst=INCR).
//    - irq_out is a new output (tie to open if unused).
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module top_phase3_wrapper #(
    parameter integer PE_COUNT   = 256,
    parameter integer DWIDTH     = 8,
    parameter integer ACC_WIDTH  = 32,
    parameter integer BRAM_DEPTH = 4096,
    parameter integer ADDR_WIDTH = 12,
    parameter integer DATA_WIDTH = 32,
    parameter integer WIDTH      = 2048   // PE_COUNT * DWIDTH
)(
    input  wire                      clk,
    input  wire                      resetn,

    // AXI-Lite write (legacy)
    input  wire [ADDR_WIDTH-1:0]     awaddr,
    input  wire                      awvalid,
    output wire                      awready,
    input  wire [DATA_WIDTH-1:0]     wdata,
    input  wire [DATA_WIDTH/8-1:0]   wstrb,
    input  wire                      wvalid,
    output wire                      wready,
    output wire [1:0]                bresp,
    output wire                      bvalid,
    input  wire                      bready,

    // AXI-Lite read (legacy)
    input  wire [ADDR_WIDTH-1:0]     araddr,
    input  wire                      arvalid,
    output wire                      arready,
    output wire [DATA_WIDTH-1:0]     rdata,
    output wire [1:0]                rresp,
    output wire                      rvalid,
    input  wire                      rready,

    // Interrupt (new output – was not in v1)
    output wire                      irq_out
);

    // Expand AXI-Lite to AXI4-Full (single-beat default)
    npu_top #(
        .PE_COUNT   (PE_COUNT),
        .DWIDTH     (DWIDTH),
        .ACC_WIDTH  (ACC_WIDTH),
        .BRAM_DEPTH (BRAM_DEPTH),
        .AXI_AW     (ADDR_WIDTH),
        .AXI_DW     (DATA_WIDTH),
        .AXI_IDW    (1)
    ) u_npu (
        .clk          (clk),
        .resetn_async (resetn),

        // Write address channel – single beat
        .s_awid       (1'b0),
        .s_awaddr     (awaddr),
        .s_awlen      (8'd0),       // length = 1 beat
        .s_awsize     (3'd2),       // 4 bytes per beat
        .s_awburst    (2'b01),      // INCR
        .s_awvalid    (awvalid),
        .s_awready    (awready),

        // Write data channel
        .s_wdata      (wdata),
        .s_wstrb      (wstrb),
        .s_wlast      (1'b1),       // always last for single beat
        .s_wvalid     (wvalid),
        .s_wready     (wready),

        // Write response channel
        .s_bid        (),
        .s_bresp      (bresp),
        .s_bvalid     (bvalid),
        .s_bready     (bready),

        // Read address channel – single beat
        .s_arid       (1'b0),
        .s_araddr     (araddr),
        .s_arlen      (8'd0),
        .s_arsize     (3'd2),
        .s_arburst    (2'b01),
        .s_arvalid    (arvalid),
        .s_arready    (arready),

        // Read data channel
        .s_rid        (),
        .s_rdata      (rdata),
        .s_rresp      (rresp),
        .s_rlast      (),
        .s_rvalid     (rvalid),
        .s_rready     (rready),

        .irq_out      (irq_out)
    );

endmodule
