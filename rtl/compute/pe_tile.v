// =============================================================================
//  pe_tile.v – Configurable 8×8 (or ROWS×COLS) PE tile with reduction tree
//
//  - Instantiates ROWS×COLS pe_core instances
//  - Instantiates a reduction_tree to sum all partial products
//  - Supports independent clock gating via tile_en
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module pe_tile #(
    parameter integer DWIDTH    = 8,
    parameter integer ACC_WIDTH = 32,
    parameter integer ROWS      = 8,
    parameter integer COLS      = 8,
    parameter integer NUM_PES   = ROWS * COLS   // must be power of 2
)(
    input  wire                           clk,
    input  wire                           resetn,
    input  wire                           tile_en,         // clock-gate enable

    // Precision / control
    input  wire [1:0]                     mode_sel,
    input  wire                           acc_clear,
    input  wire                           slice_en,

    // Operands (flat packed): [PE_i] at bits [(i+1)*DWIDTH-1 : i*DWIDTH]
    input  wire [NUM_PES*DWIDTH-1:0]      A_flat,
    input  wire [NUM_PES*DWIDTH-1:0]      B_flat,
    input  wire [NUM_PES-1:0]             gate_en,         // per-PE enable

    // Outputs
    output wire signed [ACC_WIDTH-1:0]    tile_sum,        // reduced sum from tree
    output wire                           tile_valid,
    output wire [NUM_PES-1:0]             pe_busy          // per-PE busy vector
);

    // -------------------------------------------------------------------------
    // Gated clock (ICG model – works in simulation; synthesis maps to ICG cell)
    // -------------------------------------------------------------------------
    wire gated_clk;
    `KEEP_HIER
    clock_gate u_cg (
        .clk_in  (clk),
        .enable  (tile_en),
        .test_en (1'b0),
        .clk_out (gated_clk)
    );

    // -------------------------------------------------------------------------
    // PE array: ROWS × COLS pe_core instances
    // -------------------------------------------------------------------------
    wire signed [ACC_WIDTH-1:0] partial [0:NUM_PES-1];
    wire [NUM_PES*ACC_WIDTH-1:0] partial_flat;

    genvar r, c;
    generate
        for (r = 0; r < ROWS; r = r + 1) begin : gen_row
            for (c = 0; c < COLS; c = c + 1) begin : gen_col
                localparam integer idx = r * COLS + c;

                pe_core #(
                    .DWIDTH    (DWIDTH),
                    .ACC_WIDTH (ACC_WIDTH)
                ) u_pe (
                    .clk        (gated_clk),
                    .resetn     (resetn),
                    .mode_sel   (mode_sel),
                    .gate_en    (gate_en[idx]),
                    .acc_clear  (acc_clear),
                    .slice_en   (slice_en),
                    .A_in       (A_flat[(idx+1)*DWIDTH-1 -: DWIDTH]),
                    .B_in       (B_flat[(idx+1)*DWIDTH-1 -: DWIDTH]),
                    .result     (partial[idx]),
                    .busy       (pe_busy[idx]),
                    .valid_out  ()              // not used at tile level
                );

                // Flatten partial products for reduction tree
                assign partial_flat[(idx+1)*ACC_WIDTH-1 -: ACC_WIDTH] = partial[idx];
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Reduction tree
    // -------------------------------------------------------------------------
    reduction_tree #(
        .NUM_PES   (NUM_PES),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_tree (
        .clk          (gated_clk),
        .resetn       (resetn),
        .partial_sums (partial_flat),
        .tree_sum     (tile_sum),
        .tree_valid   (tile_valid)
    );

endmodule
