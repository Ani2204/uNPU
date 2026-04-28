// =============================================================================
//  reduction_tree.v – Balanced, pipelined log-depth adder tree
//
//  Reduces NUM_PES partial products (each ACC_WIDTH bits) to a single sum.
//  Pipeline depth = ceil(log2(NUM_PES)).
//  NUM_PES must be a power of 2.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module reduction_tree #(
    parameter integer NUM_PES   = 64,
    parameter integer ACC_WIDTH = 32
)(
    input  wire                              clk,
    input  wire                              resetn,
    input  wire [NUM_PES*ACC_WIDTH-1:0]      partial_sums,   // flat input vector
    output reg  signed [ACC_WIDTH-1:0]       tree_sum,       // pipelined output
    output reg                               tree_valid      // output valid (delayed)
);

    // -------------------------------------------------------------------------
    // Compute tree depth = log2(NUM_PES)
    // -------------------------------------------------------------------------
    localparam integer DEPTH = $clog2(NUM_PES);

    // -------------------------------------------------------------------------
    // Elaboration-time sanity check
    // -------------------------------------------------------------------------
    initial begin
        if ((NUM_PES & (NUM_PES - 1)) != 0) begin
            $error("reduction_tree: NUM_PES=%0d must be a power of 2.", NUM_PES);
            $finish;
        end
    end

    // -------------------------------------------------------------------------
    // Build tree as a flattened register array
    //   level[0] = NUM_PES entries  (input latch)
    //   level[1] = NUM_PES/2 entries
    //   ...
    //   level[DEPTH] = 1 entry      (final sum)
    //
    // Total storage: NUM_PES + NUM_PES/2 + ... + 1 = 2*NUM_PES - 1 entries
    // -------------------------------------------------------------------------

    // Unpack input into level-0 registers
    reg signed [ACC_WIDTH-1:0] lvl [0:2*NUM_PES-2];

    genvar d, k;
    integer ii;

    // Level-0 latch
    always @(posedge clk) begin
        if (!resetn) begin
            for (ii = 0; ii < NUM_PES; ii = ii + 1)
                lvl[ii] <= {ACC_WIDTH{1'b0}};
        end else begin
            for (ii = 0; ii < NUM_PES; ii = ii + 1)
                lvl[ii] <= partial_sums[(ii+1)*ACC_WIDTH-1 -: ACC_WIDTH];
        end
    end

    // Reduction levels 1..DEPTH
    generate
        for (d = 1; d <= DEPTH; d = d + 1) begin : gen_level
            localparam integer IN_CNT  = NUM_PES >> (d-1);  // nodes at previous level
            localparam integer OUT_CNT = NUM_PES >> d;       // nodes at this level
            // Offset into lvl array for previous and current level
            localparam integer PREV_OFF = (1 << (d-1)) > 1 ?
                                          NUM_PES * 2 - (NUM_PES >> (d-2)) : 0;
            // Simple formula: offset[d] = 2*NUM_PES - 2*(NUM_PES >> (d-1))
            localparam integer CUR_OFF  = 2*NUM_PES - 2*(NUM_PES >> d);

            for (k = 0; k < OUT_CNT; k = k + 1) begin : gen_node
                localparam integer PREV_L = 2*NUM_PES - 2*(NUM_PES >> (d-1)) + 2*k;
                localparam integer PREV_R = PREV_L + 1;
                localparam integer CUR    = CUR_OFF + k;

                always @(posedge clk) begin
                    if (!resetn)
                        lvl[CUR] <= {ACC_WIDTH{1'b0}};
                    else
                        lvl[CUR] <= lvl[PREV_L] + lvl[PREV_R];
                end
            end
        end
    endgenerate

    // Output register
    always @(posedge clk) begin
        if (!resetn) begin
            tree_sum   <= {ACC_WIDTH{1'b0}};
            tree_valid <= 1'b0;
        end else begin
            tree_sum   <= lvl[2*NUM_PES-2];
            tree_valid <= 1'b1;
        end
    end

endmodule
