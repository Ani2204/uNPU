`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module mac_array #(
    parameter integer WIDTH     = 8,
    parameter integer ACC_WIDTH = 64,
    parameter integer ROWS      = 16,
    parameter integer COLS      = 16,
    parameter integer NUM_PES   = (ROWS * COLS),
    parameter integer PE_LATENCY = 1
)(
    input  wire                     clk,
    input  wire                     resetn,
    input  wire [1:0]               mode_sel,
    input  wire                     fuse_en,
    input  wire [NUM_PES*WIDTH-1:0] A_flat,
    input  wire [NUM_PES*WIDTH-1:0] B_flat,
    input  wire [NUM_PES-1:0]       gate_en,
    input  wire                     slice_en_global,
    input  wire [3:0]               msb_stat_thres,
    input  wire                     relu_en,
    input  wire signed [31:0]       bias,
    input  wire [3:0]               scale,
    input  wire [3:0]               shift,
    // Systolic mode inputs (improvement #4): one A per row, one B per column
    input  wire [ROWS*WIDTH-1:0]    a_row_in,
    input  wire [COLS*WIDTH-1:0]    b_col_in,
    input  wire                     systolic_en,
    input  wire                     acc_clear,
    // Legacy global-sum result (backward-compatible with testbenches)
    output reg  signed [31:0]       result,
    // Per-row results after post-processing (improvement #3)
    output reg  signed [ROWS*32-1:0] result_vec,
    output reg  [NUM_PES-1:0]       pe_busy
);

    // require even split into 4 tiles
    localparam integer TILE_ROWS = ROWS / 2;
    localparam integer TILE_COLS = COLS / 2;
    localparam integer T_PES = (TILE_ROWS * TILE_COLS); // NUM_PES / 4

    // sanity check
    initial begin
        if ((ROWS % 2) != 0 || (COLS % 2) != 0) begin
            $error("mac_array requires ROWS and COLS to be even so it can split into 4 tiles.");
        end
    end

    wire [T_PES*WIDTH-1:0] A_tile0, A_tile1, A_tile2, A_tile3;
    wire [T_PES*WIDTH-1:0] B_tile0, B_tile1, B_tile2, B_tile3;
    wire [T_PES-1:0] gate_tile0, gate_tile1, gate_tile2, gate_tile3;

    integer rr, cc, idx, t_idx;
    reg [T_PES*WIDTH-1:0] A_t0_r, A_t1_r, A_t2_r, A_t3_r;
    reg [T_PES*WIDTH-1:0] B_t0_r, B_t1_r, B_t2_r, B_t3_r;
    reg [T_PES-1:0] g_t0_r, g_t1_r, g_t2_r, g_t3_r;

    always @(*) begin
        // zero init
        A_t0_r = {T_PES*WIDTH{1'b0}}; A_t1_r = {T_PES*WIDTH{1'b0}};
        A_t2_r = {T_PES*WIDTH{1'b0}}; A_t3_r = {T_PES*WIDTH{1'b0}};
        B_t0_r = {T_PES*WIDTH{1'b0}}; B_t1_r = {T_PES*WIDTH{1'b0}};
        B_t2_r = {T_PES*WIDTH{1'b0}}; B_t3_r = {T_PES*WIDTH{1'b0}};
        g_t0_r = {T_PES{1'b0}}; g_t1_r = {T_PES{1'b0}};
        g_t2_r = {T_PES{1'b0}}; g_t3_r = {T_PES{1'b0}};

        // tile0: top-left (rows 0..TILE_ROWS-1, cols 0..TILE_COLS-1)
        t_idx = 0;
        for (rr = 0; rr < TILE_ROWS; rr = rr + 1) begin
            for (cc = 0; cc < TILE_COLS; cc = cc + 1) begin
                idx = rr * COLS + cc;
                A_t0_r[(t_idx+1)*WIDTH-1 -: WIDTH] = A_flat[(idx+1)*WIDTH-1 -: WIDTH];
                B_t0_r[(t_idx+1)*WIDTH-1 -: WIDTH] = B_flat[(idx+1)*WIDTH-1 -: WIDTH];
                g_t0_r[t_idx] = gate_en[idx];
                t_idx = t_idx + 1;
            end
        end

        // tile1: top-right (rows 0..TILE_ROWS-1, cols TILE_COLS..COLS-1)
        t_idx = 0;
        for (rr = 0; rr < TILE_ROWS; rr = rr + 1) begin
            for (cc = TILE_COLS; cc < COLS; cc = cc + 1) begin
                idx = rr * COLS + cc;
                A_t1_r[(t_idx+1)*WIDTH-1 -: WIDTH] = A_flat[(idx+1)*WIDTH-1 -: WIDTH];
                B_t1_r[(t_idx+1)*WIDTH-1 -: WIDTH] = B_flat[(idx+1)*WIDTH-1 -: WIDTH];
                g_t1_r[t_idx] = gate_en[idx];
                t_idx = t_idx + 1;
            end
        end

        // tile2: bottom-left (rows TILE_ROWS..ROWS-1, cols 0..TILE_COLS-1)
        t_idx = 0;
        for (rr = TILE_ROWS; rr < ROWS; rr = rr + 1) begin
            for (cc = 0; cc < TILE_COLS; cc = cc + 1) begin
                idx = rr * COLS + cc;
                A_t2_r[(t_idx+1)*WIDTH-1 -: WIDTH] = A_flat[(idx+1)*WIDTH-1 -: WIDTH];
                B_t2_r[(t_idx+1)*WIDTH-1 -: WIDTH] = B_flat[(idx+1)*WIDTH-1 -: WIDTH];
                g_t2_r[t_idx] = gate_en[idx];
                t_idx = t_idx + 1;
            end
        end

        // tile3: bottom-right (rows TILE_ROWS..ROWS-1, cols TILE_COLS..COLS-1)
        t_idx = 0;
        for (rr = TILE_ROWS; rr < ROWS; rr = rr + 1) begin
            for (cc = TILE_COLS; cc < COLS; cc = cc + 1) begin
                idx = rr * COLS + cc;
                A_t3_r[(t_idx+1)*WIDTH-1 -: WIDTH] = A_flat[(idx+1)*WIDTH-1 -: WIDTH];
                B_t3_r[(t_idx+1)*WIDTH-1 -: WIDTH] = B_flat[(idx+1)*WIDTH-1 -: WIDTH];
                g_t3_r[t_idx] = gate_en[idx];
                t_idx = t_idx + 1;
            end
        end
    end

    assign A_tile0 = A_t0_r; assign B_tile0 = B_t0_r; assign gate_tile0 = g_t0_r;
    assign A_tile1 = A_t1_r; assign B_tile1 = B_t1_r; assign gate_tile1 = g_t1_r;
    assign A_tile2 = A_t2_r; assign B_tile2 = B_t2_r; assign gate_tile2 = g_t2_r;
    assign A_tile3 = A_t3_r; assign B_tile3 = B_t3_r; assign gate_tile3 = g_t3_r;

    wire signed [ACC_WIDTH-1:0] tile_sum0, tile_sum1, tile_sum2, tile_sum3;
    wire [T_PES-1:0] tile_busy0, tile_busy1, tile_busy2, tile_busy3;
    // Per-row sums from each tile (TILE_ROWS sums per tile)
    wire signed [TILE_ROWS*ACC_WIDTH-1:0] tile_rs0, tile_rs1, tile_rs2, tile_rs3;

    // Systolic row/column slices per tile
    // top tiles (0,1): rows 0..TILE_ROWS-1  → a_row_in[0..TILE_ROWS-1]
    // bottom tiles (2,3): rows TILE_ROWS..ROWS-1 → a_row_in[TILE_ROWS..ROWS-1]
    // left tiles (0,2): cols 0..TILE_COLS-1  → b_col_in[0..TILE_COLS-1]
    // right tiles (1,3): cols TILE_COLS..COLS-1 → b_col_in[TILE_COLS..COLS-1]
    wire [TILE_ROWS*WIDTH-1:0] a_top   = a_row_in[TILE_ROWS*WIDTH-1:0];
    wire [TILE_ROWS*WIDTH-1:0] a_bot   = a_row_in[ROWS*WIDTH-1:TILE_ROWS*WIDTH];
    wire [TILE_COLS*WIDTH-1:0] b_left  = b_col_in[TILE_COLS*WIDTH-1:0];
    wire [TILE_COLS*WIDTH-1:0] b_right = b_col_in[COLS*WIDTH-1:TILE_COLS*WIDTH];

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile0 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile0),
            .A_flat(A_tile0), .B_flat(B_tile0),
            .a_row_in(a_top), .b_col_in(b_left),
            .systolic_en(systolic_en), .acc_clear(acc_clear),
            .pe_busy(tile_busy0),
            .tile_sum(tile_sum0),
            .tile_row_sums(tile_rs0)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile1 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile1),
            .A_flat(A_tile1), .B_flat(B_tile1),
            .a_row_in(a_top), .b_col_in(b_right),
            .systolic_en(systolic_en), .acc_clear(acc_clear),
            .pe_busy(tile_busy1),
            .tile_sum(tile_sum1),
            .tile_row_sums(tile_rs1)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile2 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile2),
            .A_flat(A_tile2), .B_flat(B_tile2),
            .a_row_in(a_bot), .b_col_in(b_left),
            .systolic_en(systolic_en), .acc_clear(acc_clear),
            .pe_busy(tile_busy2),
            .tile_sum(tile_sum2),
            .tile_row_sums(tile_rs2)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile3 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile3),
            .A_flat(A_tile3), .B_flat(B_tile3),
            .a_row_in(a_bot), .b_col_in(b_right),
            .systolic_en(systolic_en), .acc_clear(acc_clear),
            .pe_busy(tile_busy3),
            .tile_sum(tile_sum3),
            .tile_row_sums(tile_rs3)
        );

    integer gi;
    always @(posedge clk) begin
        if (!resetn) begin
            pe_busy <= {NUM_PES{1'b0}};
        end else begin
            // tile0 -> top-left
            gi = 0;
            for (rr = 0; rr < TILE_ROWS; rr = rr + 1) begin
                for (cc = 0; cc < TILE_COLS; cc = cc + 1) begin
                    idx = rr * COLS + cc;
                    pe_busy[idx] <= tile_busy0[gi];
                    gi = gi + 1;
                end
            end

            // tile1 -> top-right
            gi = 0;
            for (rr = 0; rr < TILE_ROWS; rr = rr + 1) begin
                for (cc = TILE_COLS; cc < COLS; cc = cc + 1) begin
                    idx = rr * COLS + cc;
                    pe_busy[idx] <= tile_busy1[gi];
                    gi = gi + 1;
                end
            end

            // tile2 -> bottom-left
            gi = 0;
            for (rr = TILE_ROWS; rr < ROWS; rr = rr + 1) begin
                for (cc = 0; cc < TILE_COLS; cc = cc + 1) begin
                    idx = rr * COLS + cc;
                    pe_busy[idx] <= tile_busy2[gi];
                    gi = gi + 1;
                end
            end

            // tile3 -> bottom-right
            gi = 0;
            for (rr = TILE_ROWS; rr < ROWS; rr = rr + 1) begin
                for (cc = TILE_COLS; cc < COLS; cc = cc + 1) begin
                    idx = rr * COLS + cc;
                    pe_busy[idx] <= tile_busy3[gi];
                    gi = gi + 1;
                end
            end
        end
    end

    // Combine tile sums for legacy global-sum output
    reg signed [ACC_WIDTH-1:0] sum01, sum23, total_sum;
    always @(posedge clk) begin
        if (!resetn) begin
            sum01 <= {ACC_WIDTH{1'b0}};
            sum23 <= {ACC_WIDTH{1'b0}};
            total_sum <= {ACC_WIDTH{1'b0}};
        end else begin
            sum01 <= tile_sum0 + tile_sum1;
            sum23 <= tile_sum2 + tile_sum3;
            total_sum <= sum01 + sum23;
        end
    end

    // Fuse accumulator for global result (legacy backward-compat)
    reg signed [ACC_WIDTH-1:0] acc;
    always @(posedge clk) begin
        if (!resetn) begin
            acc <= {ACC_WIDTH{1'b0}};
        end else begin
            if (fuse_en) acc <= acc + total_sum;
            else         acc <= total_sum;
        end
    end

    // ------------------------------------------------------------------
    // Post-processing pipeline (improvement #9): 3 stages
    // Stage 1: bias add
    // Stage 2: scale multiply + arithmetic shift
    // Stage 3: saturate to 32-bit, apply ReLU, register result
    // ------------------------------------------------------------------

    // ---- LEGACY global result ----
    reg signed [63:0] pp1_val;
    reg signed [63:0] pp2_val;
    always @(posedge clk) begin
        if (!resetn) begin
            pp1_val <= 64'sd0;
            pp2_val <= 64'sd0;
            result  <= 32'sd0;
        end else begin
            // Stage 1: bias add (sign-extend acc and bias to 64 bits)
            pp1_val <= {{(64-ACC_WIDTH){acc[ACC_WIDTH-1]}}, acc} +
                       {{32{bias[31]}}, bias};
            // Stage 2: scale × pp1 then arithmetic right-shift
            pp2_val <= (pp1_val * {{60{1'b0}}, scale}) >>> shift;
            // Stage 3: saturate and ReLU
            if      (pp2_val > 64'sh7FFF_FFFF) result <= 32'sh7FFF_FFFF;
            else if (pp2_val < -64'sh80000000) result <= -32'sh80000000;
            else                               result <= pp2_val[31:0];
            if (relu_en && pp2_val[63])        result <= 32'sd0;
        end
    end

    // ------------------------------------------------------------------
    // Per-row K-accumulators and per-row pipelined post-processing
    // (improvement #3 + #9)
    // Tile row sums: tile_rs0/1 hold TILE_ROWS sums each for top rows
    //               tile_rs2/3 hold TILE_ROWS sums each for bottom rows
    // Row r (0..ROWS-1) sum = left-tile + right-tile row-sum.
    // ------------------------------------------------------------------
    reg signed [ACC_WIDTH-1:0] row_sum  [0:ROWS-1];   // assembled across tiles
    reg signed [ACC_WIDTH-1:0] row_acc  [0:ROWS-1];   // K-accumulation per row
    // Pipeline registers per row
    reg signed [63:0] row_pp1 [0:ROWS-1]; // stage 1: bias add
    reg signed [63:0] row_pp2 [0:ROWS-1]; // stage 2: scale+shift
    reg signed [31:0] row_pp3 [0:ROWS-1]; // stage 3: saturate+ReLU

    integer row_i;
    always @(posedge clk) begin
        if (!resetn || acc_clear) begin
            for (row_i = 0; row_i < ROWS; row_i = row_i + 1) begin
                row_sum [row_i] <= {ACC_WIDTH{1'b0}};
                row_acc [row_i] <= {ACC_WIDTH{1'b0}};
                row_pp1 [row_i] <= 64'sd0;
                row_pp2 [row_i] <= 64'sd0;
                row_pp3 [row_i] <= 32'sd0;
            end
            result_vec <= {ROWS*32{1'b0}};
        end else begin
            // Step 1: assemble per-row sums across left and right tiles
            for (row_i = 0; row_i < TILE_ROWS; row_i = row_i + 1) begin
                // Top rows: tile0 (left) + tile1 (right)
                row_sum[row_i] <=
                    tile_rs0[(row_i+1)*ACC_WIDTH-1 -: ACC_WIDTH] +
                    tile_rs1[(row_i+1)*ACC_WIDTH-1 -: ACC_WIDTH];
                // Bottom rows: tile2 (left) + tile3 (right)
                row_sum[TILE_ROWS + row_i] <=
                    tile_rs2[(row_i+1)*ACC_WIDTH-1 -: ACC_WIDTH] +
                    tile_rs3[(row_i+1)*ACC_WIDTH-1 -: ACC_WIDTH];
            end

            // Step 2: K-accumulation per row
            for (row_i = 0; row_i < ROWS; row_i = row_i + 1) begin
                if (fuse_en) row_acc[row_i] <= row_acc[row_i] + row_sum[row_i];
                else         row_acc[row_i] <= row_sum[row_i];
            end

            // Step 3a (stage 1): bias add per row
            for (row_i = 0; row_i < ROWS; row_i = row_i + 1)
                row_pp1[row_i] <=
                    {{(64-ACC_WIDTH){row_acc[row_i][ACC_WIDTH-1]}}, row_acc[row_i]} +
                    {{32{bias[31]}}, bias};

            // Step 3b (stage 2): scale × pp1 + arithmetic shift per row
            for (row_i = 0; row_i < ROWS; row_i = row_i + 1)
                row_pp2[row_i] <= (row_pp1[row_i] * {{60{1'b0}}, scale}) >>> shift;

            // Step 3c (stage 3): saturate + ReLU, register into result_vec
            for (row_i = 0; row_i < ROWS; row_i = row_i + 1) begin
                if      (row_pp2[row_i] > 64'sh7FFF_FFFF) row_pp3[row_i] = 32'sh7FFF_FFFF;
                else if (row_pp2[row_i] < -64'sh80000000) row_pp3[row_i] = -32'sh80000000;
                else                                       row_pp3[row_i] = row_pp2[row_i][31:0];
                if (relu_en && row_pp2[row_i][63])         row_pp3[row_i] = 32'sd0;
                result_vec[(row_i+1)*32-1 -: 32] <= row_pp3[row_i];
            end
        end
    end

endmodule
