`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module mac_tile_8x8 #(
    parameter integer WIDTH      = 8,
    parameter integer ACC_WIDTH  = 64,
    parameter integer ROWS       = 8,
    parameter integer COLS       = 8,
    parameter integer NUM_PES    = (ROWS * COLS),
    parameter integer PE_LATENCY = 1
)(
    input  wire                        clk,
    input  wire                        resetn,
    input  wire [1:0]                  mode_sel,
    input  wire                        slice_en_global,
    input  wire [3:0]                  msb_stat_thres,
    input  wire [NUM_PES-1:0]          gate_en,
    input  wire [NUM_PES*WIDTH-1:0]    A_flat,
    input  wire [NUM_PES*WIDTH-1:0]    B_flat,
    // Systolic mode: one A value broadcast per row, one B value per column
    input  wire [ROWS*WIDTH-1:0]       a_row_in,
    input  wire [COLS*WIDTH-1:0]       b_col_in,
    input  wire                        systolic_en,  // 1=systolic accumulation, 0=SIMD
    input  wire                        acc_clear,    // synchronous accumulator reset
    output reg  [NUM_PES-1:0]          pe_busy,
    output reg  signed [ACC_WIDTH-1:0] tile_sum,
    // Per-row partial sums for matrix-multiply output (improvement #3)
    output reg  signed [ROWS*ACC_WIDTH-1:0] tile_row_sums
);

    // --------------------------------------------------------------------
    //  NUM_PES MUST BE POWER OF TWO (for reduction tree)
    // --------------------------------------------------------------------
    initial begin
        if ((NUM_PES & (NUM_PES - 1)) != 0)
            $error("NUM_PES must be power of 2 for reduction tree to work.");
    end

    // --------------------------------------------------------------------
    // INTERNALS
    // --------------------------------------------------------------------
    // pe_acc: unified accumulator - in systolic mode, accumulates per K-slice;
    //         in SIMD mode, holds single registered product.
    reg signed [ACC_WIDTH-1:0] pe_acc   [0:NUM_PES-1];
    reg [3:0] msb_count [0:NUM_PES-1];
    reg [31:0] busy_cnt [0:NUM_PES-1];

    integer r, c, pi;

    // --------------------------------------------------------------------
    // NARROW SIGN-EXTEND FUNCTIONS
    // --------------------------------------------------------------------
    function signed [15:0] signext8;      // for INT8
        input [7:0] v;
        begin
            signext8 = {{8{v[7]}}, v};
        end
    endfunction

    function signed [7:0] signext4;       // for INT4
        input [3:0] v;
        begin
            signext4 = {{4{v[3]}}, v};
        end
    endfunction

    function signed [3:0] signext2;       // for INT2
        input [1:0] v;
        begin
            signext2 = {{2{v[1]}}, v};
        end
    endfunction

    // --------------------------------------------------------------------
    // PRODUCT WIRES (DSP HINT ON MULTIPLIERS, NOT ON ACC-WIDTH BUSES)
    // --------------------------------------------------------------------
    wire signed [ACC_WIDTH-1:0] prod_wire [0:NUM_PES-1];

    genvar gi_r, gi_c;
    generate
        for (gi_r = 0; gi_r < ROWS; gi_r = gi_r + 1) begin : gen_row
            for (gi_c = 0; gi_c < COLS; gi_c = gi_c + 1) begin : gen_col

                localparam integer idx = gi_r * COLS + gi_c;

                // SIMD mode: each PE has its own A/B from A_flat / B_flat
                wire signed [WIDTH-1:0] A_in = A_flat[(idx+1)*WIDTH-1 -: WIDTH];
                wire signed [WIDTH-1:0] B_in = B_flat[(idx+1)*WIDTH-1 -: WIDTH];

                // Systolic mode: entire row shares one A; entire column shares one B
                wire signed [WIDTH-1:0] A_sys = a_row_in[(gi_r+1)*WIDTH-1 -: WIDTH];
                wire signed [WIDTH-1:0] B_sys = b_col_in[(gi_c+1)*WIDTH-1 -: WIDTH];

                // Select source based on systolic_en
                wire signed [WIDTH-1:0] A_sel = systolic_en ? A_sys : A_in;
                wire signed [WIDTH-1:0] B_sel = systolic_en ? B_sys : B_in;

                // Normalize to 8-bit signed
                wire signed [7:0] A8 = (WIDTH==8) ? A_sel
                                       : {{(8-WIDTH){A_sel[WIDTH-1]}}, A_sel};
                wire signed [7:0] B8 = (WIDTH==8) ? B_sel
                                       : {{(8-WIDTH){B_sel[WIDTH-1]}}, B_sel};

                // Nibbles (for 4-bit mode)
                wire signed [3:0] A4_low  = A8[3:0];
                wire signed [3:0] A4_high = A8[7:4];
                wire signed [3:0] B4_low  = B8[3:0];
                wire signed [3:0] B4_high = B8[7:4];

                // INT2 mode: all 4 two-bit lanes packed into one byte (improvement #7)
                // Lane layout: [7:6]=lane3, [5:4]=lane2, [3:2]=lane1, [1:0]=lane0
                wire signed [1:0] A2_0 = A8[1:0];
                wire signed [1:0] A2_1 = A8[3:2];
                wire signed [1:0] A2_2 = A8[5:4];
                wire signed [1:0] A2_3 = A8[7:6];
                wire signed [1:0] B2_0 = B8[1:0];
                wire signed [1:0] B2_1 = B8[3:2];
                wire signed [1:0] B2_2 = B8[5:4];
                wire signed [1:0] B2_3 = B8[7:6];

                // Auto-detect helpers
                wire auto_is_int8        = (|A8[7:4]) | (|B8[7:4]);
                wire high_nibble_present = (|A4_high) && (|B4_high);

                // -------------------------
                // MULTIPLIERS
                // -------------------------

                // INT8: 8x8 -> 16-bit, then extend to ACC_WIDTH
                (* use_dsp = "yes" *)
                wire signed [15:0] mul8 = signext8(A8) * signext8(B8);
                wire signed [ACC_WIDTH-1:0] comb_int8_prod =
                    {{(ACC_WIDTH-16){mul8[15]}}, mul8};

                // INT4: 4x4 -> 8-bit, use DSP to free LUTs
                (* use_dsp = "yes" *)
                wire signed [7:0] mul4_low  = signext4(A4_low)  * signext4(B4_low);
                (* use_dsp = "yes" *)
                wire signed [7:0] mul4_high = signext4(A4_high) * signext4(B4_high);

                wire signed [ACC_WIDTH-1:0] comb_low4_prod =
                    {{(ACC_WIDTH-8){mul4_low[7]}},  mul4_low};
                wire signed [ACC_WIDTH-1:0] comb_high4_prod =
                    {{(ACC_WIDTH-8){mul4_high[7]}}, mul4_high};

                // INT2: 4 lanes × 2x2→4-bit products, prefer LUTs
                (* use_dsp = "no" *)
                wire signed [3:0] mul2_0 = signext2(A2_0) * signext2(B2_0);
                (* use_dsp = "no" *)
                wire signed [3:0] mul2_1 = signext2(A2_1) * signext2(B2_1);
                (* use_dsp = "no" *)
                wire signed [3:0] mul2_2 = signext2(A2_2) * signext2(B2_2);
                (* use_dsp = "no" *)
                wire signed [3:0] mul2_3 = signext2(A2_3) * signext2(B2_3);

                // Sum all four INT2 lane products for 4× throughput per data bus cycle
                wire signed [ACC_WIDTH-1:0] comb_int2_prod =
                    {{(ACC_WIDTH-4){mul2_0[3]}}, mul2_0} +
                    {{(ACC_WIDTH-4){mul2_1[3]}}, mul2_1} +
                    {{(ACC_WIDTH-4){mul2_2[3]}}, mul2_2} +
                    {{(ACC_WIDTH-4){mul2_3[3]}}, mul2_3};

                // Chosen product per mode
                reg signed [ACC_WIDTH-1:0] chosen_prod_comb;

                always @(*) begin
                    case (mode_sel)
                        // 00: pure INT8
                        2'b00: chosen_prod_comb = comb_int8_prod;

                        // 01: INT4 (with optional high nibble)
                        2'b01: begin
                            if (slice_en_global &&
                                high_nibble_present &&
                                (msb_count[idx] >= msb_stat_thres))
                                chosen_prod_comb = comb_low4_prod + comb_high4_prod;
                            else
                                chosen_prod_comb = comb_low4_prod;
                        end

                        // 10: INT2
                        2'b10: chosen_prod_comb = comb_int2_prod;

                        // 11: AUTO
                        2'b11: begin
                            if (auto_is_int8)
                                chosen_prod_comb = comb_int8_prod;
                            else begin
                                if (slice_en_global &&
                                    high_nibble_present &&
                                    (msb_count[idx] >= msb_stat_thres))
                                    chosen_prod_comb = comb_low4_prod + comb_high4_prod;
                                else
                                    chosen_prod_comb = comb_low4_prod;
                            end
                        end

                        default: chosen_prod_comb = {ACC_WIDTH{1'b0}};
                    endcase
                end

                assign prod_wire[idx] = chosen_prod_comb;

                // -------------------------
                // Sequential PE acceptance + busy tracking
                // Systolic mode:  pe_acc accumulates every time gate_en fires.
                // SIMD mode:      pe_acc registers the product once per dispatch.
                // -------------------------
                always @(posedge clk) begin
                    if (!resetn || acc_clear) begin
                        pe_acc[idx]    <= 0;
                        msb_count[idx] <= 0;
                        busy_cnt[idx]  <= 0;
                        pe_busy[idx]   <= 0;
                    end else begin
                        // adapt msb counter (simple hysteresis)
                        if (auto_is_int8) begin
                            if (msb_count[idx] != 4'hF)
                                msb_count[idx] <= msb_count[idx] + 1;
                        end else begin
                            if (msb_count[idx] != 4'h0)
                                msb_count[idx] <= msb_count[idx] - 1;
                        end

                        if (systolic_en) begin
                            // Systolic: accumulate over K-slices while gate fires
                            if (gate_en[idx])
                                pe_acc[idx] <= pe_acc[idx] + chosen_prod_comb;
                            pe_busy[idx] <= gate_en[idx];
                        end else begin
                            // SIMD: accept new product when PE is free and gate_en is high
                            if (gate_en[idx] && (busy_cnt[idx] == 0)) begin
                                pe_acc[idx]   <= chosen_prod_comb;
                                busy_cnt[idx] <= (PE_LATENCY == 0) ? 32'd1 : PE_LATENCY;
                            end else if (busy_cnt[idx] > 0) begin
                                busy_cnt[idx] <= busy_cnt[idx] - 1;
                            end
                            pe_busy[idx] <= (busy_cnt[idx] != 0);
                        end
                    end
                end

            end
        end
    endgenerate

    // ------------------------------
    // Reduction Tree  (improvement #8: red_s0 eliminated — saves one pipeline stage)
    // Directly pair pe_acc into red_s1 without the intermediate copy.
    // ------------------------------
    // Global sum tree (6 stages for 64 PEs: log2(64)=6)
    reg signed [ACC_WIDTH-1:0] red_s1 [0:(NUM_PES/2)-1];
    reg signed [ACC_WIDTH-1:0] red_s2 [0:(NUM_PES/4)-1];
    reg signed [ACC_WIDTH-1:0] red_s3 [0:(NUM_PES/8)-1];
    reg signed [ACC_WIDTH-1:0] red_s4 [0:(NUM_PES/16)-1];
    reg signed [ACC_WIDTH-1:0] red_s5 [0:(NUM_PES/32)-1];
    reg signed [ACC_WIDTH-1:0] red_s6 [0:0];

    // Per-row reduction (3 stages for COLS elements per row)
    // row_r1[r*COLS/2 +: COLS/2]: stage-1 pairs within row r
    // row_r2[r*COLS/4 +: COLS/4]: stage-2 pairs within row r
    // row_r3[r]                 : final row sum
    reg signed [ACC_WIDTH-1:0] row_r1 [0:ROWS*(COLS/2)-1];
    reg signed [ACC_WIDTH-1:0] row_r2 [0:ROWS*(COLS/4)-1];
    reg signed [ACC_WIDTH-1:0] row_r3 [0:ROWS-1];

    integer k, rr, cc;
    always @(posedge clk) begin
        if (!resetn) begin
            for (k = 0; k < (NUM_PES/2);   k = k + 1) red_s1[k] <= 0;
            for (k = 0; k < (NUM_PES/4);   k = k + 1) red_s2[k] <= 0;
            for (k = 0; k < (NUM_PES/8);   k = k + 1) red_s3[k] <= 0;
            for (k = 0; k < (NUM_PES/16);  k = k + 1) red_s4[k] <= 0;
            for (k = 0; k < (NUM_PES/32);  k = k + 1) red_s5[k] <= 0;
            red_s6[0] <= 0;
            tile_sum  <= 0;
            for (k = 0; k < ROWS*(COLS/2); k = k + 1) row_r1[k] <= 0;
            for (k = 0; k < ROWS*(COLS/4); k = k + 1) row_r2[k] <= 0;
            for (k = 0; k < ROWS;          k = k + 1) row_r3[k] <= 0;
            tile_row_sums <= {ROWS*ACC_WIDTH{1'b0}};
        end else begin
            // ---- Global sum: red_s0 removed; feed pe_acc directly into red_s1 ----
            for (k = 0; k < (NUM_PES/2);  k = k + 1)
                red_s1[k] <= pe_acc[2*k]   + pe_acc[2*k+1];
            for (k = 0; k < (NUM_PES/4);  k = k + 1)
                red_s2[k] <= red_s1[2*k]   + red_s1[2*k+1];
            for (k = 0; k < (NUM_PES/8);  k = k + 1)
                red_s3[k] <= red_s2[2*k]   + red_s2[2*k+1];
            for (k = 0; k < (NUM_PES/16); k = k + 1)
                red_s4[k] <= red_s3[2*k]   + red_s3[2*k+1];
            for (k = 0; k < (NUM_PES/32); k = k + 1)
                red_s5[k] <= red_s4[2*k]   + red_s4[2*k+1];
            red_s6[0] <= red_s5[0] + red_s5[1];
            tile_sum  <= red_s6[0];

            // ---- Per-row reduction: 3 stages for COLS elements per row ----
            for (rr = 0; rr < ROWS; rr = rr + 1) begin
                for (cc = 0; cc < COLS/2; cc = cc + 1)
                    row_r1[rr*(COLS/2)+cc] <=
                        pe_acc[rr*COLS + 2*cc] + pe_acc[rr*COLS + 2*cc+1];
                for (cc = 0; cc < COLS/4; cc = cc + 1)
                    row_r2[rr*(COLS/4)+cc] <=
                        row_r1[rr*(COLS/2) + 2*cc] + row_r1[rr*(COLS/2) + 2*cc+1];
                row_r3[rr] <= row_r2[rr*(COLS/4)] + row_r2[rr*(COLS/4)+1];
                tile_row_sums[(rr+1)*ACC_WIDTH-1 -: ACC_WIDTH] <= row_r3[rr];
            end
        end
    end

endmodule
