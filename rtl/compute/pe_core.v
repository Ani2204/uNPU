// =============================================================================
//  pe_core.v – Individual Pipelined Processing Element (MAC unit)
//
//  3-stage pipeline:
//    Stage 1: Operand load (register A/B inputs)
//    Stage 2: Multiply    (DSP-mapped multiplier)
//    Stage 3: Accumulate  (add to running accumulator with saturation)
//
//  Supports INT8 / INT4 / INT2 modes.  busy is asserted whenever a valid
//  result is being computed (Stages 2 and 3 occupied).
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module pe_core #(
    parameter integer DWIDTH    = 8,     // operand width (8/4/2)
    parameter integer ACC_WIDTH = 32     // accumulator width
)(
    input  wire                  clk,
    input  wire                  resetn,

    // Control
    input  wire [1:0]            mode_sel,    // 00=INT8, 01=INT4, 10=INT2, 11=AUTO
    input  wire                  gate_en,     // 1 = accept new operands this cycle
    input  wire                  acc_clear,   // 1 = clear accumulator before add
    input  wire                  slice_en,    // 1 = dual-nibble INT4 mode

    // Operands (8-bit containers; only lower bits used in INT4/2 mode)
    input  wire  [DWIDTH-1:0]    A_in,
    input  wire  [DWIDTH-1:0]    B_in,

    // Outputs
    output reg   [ACC_WIDTH-1:0] result,      // registered accumulator output
    output wire                  busy,        // 1 while pipeline stages 2-3 are occupied
    output reg                   valid_out    // result is valid
);

    // -------------------------------------------------------------------------
    // Sign-extend helpers (combinational functions)
    // -------------------------------------------------------------------------
    function automatic signed [15:0] sx8;
        input [7:0] v;
        sx8 = {{8{v[7]}}, v};
    endfunction

    function automatic signed [7:0] sx4;
        input [3:0] v;
        sx4 = {{4{v[3]}}, v};
    endfunction

    function automatic signed [3:0] sx2;
        input [1:0] v;
        sx2 = {{2{v[1]}}, v};
    endfunction

    // -------------------------------------------------------------------------
    // Stage 1: Operand Latch
    // -------------------------------------------------------------------------
    reg [DWIDTH-1:0] s1_A, s1_B;
    reg              s1_valid;

    always @(posedge clk) begin
        if (!resetn) begin
            s1_A     <= {DWIDTH{1'b0}};
            s1_B     <= {DWIDTH{1'b0}};
            s1_valid <= 1'b0;
        end else begin
            s1_A     <= A_in;
            s1_B     <= B_in;
            s1_valid <= gate_en;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply
    // -------------------------------------------------------------------------
    reg signed [ACC_WIDTH-1:0] s2_prod;
    reg                         s2_valid;
    reg                         s2_acc_clear;

    wire signed [7:0] A8 = s1_A[7:0];
    wire signed [7:0] B8 = s1_B[7:0];

    // INT8 product
    `DSP_ATTR
    wire signed [15:0] mul_int8 = sx8(A8) * sx8(B8);
    wire signed [ACC_WIDTH-1:0] prod_int8 = {{(ACC_WIDTH-16){mul_int8[15]}}, mul_int8};

    // INT4 product (low nibble × low nibble)
    `NO_DSP
    wire signed [7:0] mul_int4_lo = sx4(A8[3:0]) * sx4(B8[3:0]);
    `NO_DSP
    wire signed [7:0] mul_int4_hi = sx4(A8[7:4]) * sx4(B8[7:4]);
    wire signed [ACC_WIDTH-1:0] prod_int4_lo = {{(ACC_WIDTH-8){mul_int4_lo[7]}}, mul_int4_lo};
    wire signed [ACC_WIDTH-1:0] prod_int4_hi = {{(ACC_WIDTH-8){mul_int4_hi[7]}}, mul_int4_hi};

    // INT2 product
    `NO_DSP
    wire signed [3:0] mul_int2 = sx2(A8[1:0]) * sx2(B8[1:0]);
    wire signed [ACC_WIDTH-1:0] prod_int2 = {{(ACC_WIDTH-4){mul_int2[3]}}, mul_int2};

    // AUTO: use INT8 when any high bits set, else INT4
    wire auto_is_int8 = (|A8[7:4]) | (|B8[7:4]);

    reg signed [ACC_WIDTH-1:0] chosen_prod;
    always @(*) begin
        case (mode_sel)
            2'b00: chosen_prod = prod_int8;
            2'b01: chosen_prod = slice_en ?
                                    (prod_int4_lo + prod_int4_hi) : prod_int4_lo;
            2'b10: chosen_prod = prod_int2;
            2'b11: chosen_prod = auto_is_int8 ? prod_int8 :
                                  (slice_en ? prod_int4_lo + prod_int4_hi : prod_int4_lo);
            default: chosen_prod = {ACC_WIDTH{1'b0}};
        endcase
    end

    always @(posedge clk) begin
        if (!resetn) begin
            s2_prod      <= {ACC_WIDTH{1'b0}};
            s2_valid     <= 1'b0;
            s2_acc_clear <= 1'b0;
        end else begin
            s2_prod      <= chosen_prod;
            s2_valid     <= s1_valid;
            s2_acc_clear <= acc_clear;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Accumulate with saturation
    // -------------------------------------------------------------------------
    reg signed [ACC_WIDTH-1:0] acc;

    // Saturating add: acc ± prod, clamp to [-(2^(ACC_WIDTH-1)), 2^(ACC_WIDTH-1)-1]
    wire signed [ACC_WIDTH:0] sum_ext =
        {acc[ACC_WIDTH-1], acc} + {s2_prod[ACC_WIDTH-1], s2_prod};

    // Overflow detection
    wire overflow_pos = (~sum_ext[ACC_WIDTH] & sum_ext[ACC_WIDTH-1]);  // pos wrap
    wire overflow_neg = ( sum_ext[ACC_WIDTH] & ~sum_ext[ACC_WIDTH-1]); // neg wrap

    wire signed [ACC_WIDTH-1:0] sum_sat =
        overflow_pos ? {1'b0, {(ACC_WIDTH-1){1'b1}}} :
        overflow_neg ? {1'b1, {(ACC_WIDTH-1){1'b0}}} :
                        sum_ext[ACC_WIDTH-1:0];

    always @(posedge clk) begin
        if (!resetn) begin
            acc       <= {ACC_WIDTH{1'b0}};
            result    <= {ACC_WIDTH{1'b0}};
            valid_out <= 1'b0;
        end else begin
            if (s2_valid) begin
                acc <= s2_acc_clear ? s2_prod : sum_sat;
            end
            result    <= acc;
            valid_out <= s2_valid;
        end
    end

    // -------------------------------------------------------------------------
    // Busy: stages 2 or 3 have valid work
    // -------------------------------------------------------------------------
    assign busy = s1_valid | s2_valid;

endmodule
