// =============================================================================
//  pe_cluster.v – Hierarchical PE cluster
//
//  Contains N_TILES pe_tile instances.  A second-level adder tree reduces the
//  individual tile sums into a single cluster_sum output.
//
//  Scalable: set N_TILES = PE_COUNT / (TILE_ROWS * TILE_COLS) at elaboration.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module pe_cluster #(
    parameter integer DWIDTH    = 8,
    parameter integer ACC_WIDTH = 32,
    parameter integer TILE_ROWS = 8,
    parameter integer TILE_COLS = 8,
    parameter integer N_TILES   = 4    // number of tiles; must be power of 2
)(
    input  wire                                          clk,
    input  wire                                          resetn,

    // Precision / control
    input  wire [1:0]                                    mode_sel,
    input  wire                                          acc_clear,
    input  wire                                          slice_en,

    // Per-tile enables
    input  wire [N_TILES-1:0]                            tile_en,

    // Operands (flat across all PEs in cluster)
    input  wire [N_TILES*TILE_ROWS*TILE_COLS*DWIDTH-1:0] A_flat,
    input  wire [N_TILES*TILE_ROWS*TILE_COLS*DWIDTH-1:0] B_flat,
    input  wire [N_TILES*TILE_ROWS*TILE_COLS-1:0]        gate_en,

    // Outputs
    output reg  signed [ACC_WIDTH-1:0]                   cluster_sum,
    output reg                                           cluster_valid,
    output wire [N_TILES*TILE_ROWS*TILE_COLS-1:0]        pe_busy
);

    localparam integer TILE_PES  = TILE_ROWS * TILE_COLS;
    localparam integer TOTAL_PES = N_TILES * TILE_PES;

    // -------------------------------------------------------------------------
    // Tile instantiation
    // -------------------------------------------------------------------------
    wire signed [ACC_WIDTH-1:0] tile_sums  [0:N_TILES-1];
    wire                        tile_valid [0:N_TILES-1];

    genvar t;
    generate
        for (t = 0; t < N_TILES; t = t + 1) begin : gen_tile
            pe_tile #(
                .DWIDTH    (DWIDTH),
                .ACC_WIDTH (ACC_WIDTH),
                .ROWS      (TILE_ROWS),
                .COLS      (TILE_COLS),
                .NUM_PES   (TILE_PES)
            ) u_tile (
                .clk       (clk),
                .resetn    (resetn),
                .tile_en   (tile_en[t]),
                .mode_sel  (mode_sel),
                .acc_clear (acc_clear),
                .slice_en  (slice_en),
                .A_flat    (A_flat   [(t+1)*TILE_PES*DWIDTH-1 -: TILE_PES*DWIDTH]),
                .B_flat    (B_flat   [(t+1)*TILE_PES*DWIDTH-1 -: TILE_PES*DWIDTH]),
                .gate_en   (gate_en  [(t+1)*TILE_PES-1 -: TILE_PES]),
                .tile_sum  (tile_sums[t]),
                .tile_valid(tile_valid[t]),
                .pe_busy   (pe_busy  [(t+1)*TILE_PES-1 -: TILE_PES])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Second-level reduction: sum all tile sums
    // Simple pipelined adder tree (registered, balanced)
    // -------------------------------------------------------------------------
    // For N_TILES=4: two-stage binary tree
    // For N_TILES=8: three-stage binary tree
    // Generalise with a generate loop
    localparam integer TREE_DEPTH = $clog2(N_TILES);

    // Flattened tile sums for use in generate
    wire [N_TILES*ACC_WIDTH-1:0] tile_sums_flat;
    generate
        for (t = 0; t < N_TILES; t = t + 1) begin : gen_flat
            assign tile_sums_flat[(t+1)*ACC_WIDTH-1 -: ACC_WIDTH] = tile_sums[t];
        end
    endgenerate

    // Reduction tree on tile sums
    wire signed [ACC_WIDTH-1:0] reduced_sum;
    wire                        reduced_valid;

    reduction_tree #(
        .NUM_PES   (N_TILES),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_cluster_tree (
        .clk          (clk),
        .resetn       (resetn),
        .partial_sums (tile_sums_flat),
        .tree_sum     (reduced_sum),
        .tree_valid   (reduced_valid)
    );

    // Final register
    always @(posedge clk) begin
        if (!resetn) begin
            cluster_sum   <= {ACC_WIDTH{1'b0}};
            cluster_valid <= 1'b0;
        end else begin
            cluster_sum   <= reduced_sum;
            cluster_valid <= reduced_valid;
        end
    end

endmodule
