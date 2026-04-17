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
    output reg  signed [31:0]       result,
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

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile0 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile0),
            .A_flat(A_tile0), .B_flat(B_tile0),
            .pe_busy(tile_busy0),
            .tile_sum(tile_sum0)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile1 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile1),
            .A_flat(A_tile1), .B_flat(B_tile1),
            .pe_busy(tile_busy1),
            .tile_sum(tile_sum1)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile2 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile2),
            .A_flat(A_tile2), .B_flat(B_tile2),
            .pe_busy(tile_busy2),
            .tile_sum(tile_sum2)
        );

    (* keep_hierarchy = "yes", dont_touch = "yes" *)
    mac_tile_8x8 #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .ROWS(TILE_ROWS), .COLS(TILE_COLS), .NUM_PES(T_PES), .PE_LATENCY(PE_LATENCY))
        tile3 (
            .clk(clk), .resetn(resetn),
            .mode_sel(mode_sel), .slice_en_global(slice_en_global),
            .msb_stat_thres(msb_stat_thres),
            .gate_en(gate_tile3),
            .A_flat(A_tile3), .B_flat(B_tile3),
            .pe_busy(tile_busy3),
            .tile_sum(tile_sum3)
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

    // Combine tile sums
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

    // Fixed accumulator (fuse) - sequential, no combinational feedback
    reg signed [ACC_WIDTH-1:0] acc;
    always @(posedge clk) begin
        if (!resetn) begin
            acc <= {ACC_WIDTH{1'b0}};
        end else begin
            if (fuse_en) acc <= acc + total_sum;
            else         acc <= total_sum;
        end
    end

    // final combinational convert (safe to keep combinational)
    reg signed [63:0] tmp64;
    reg signed [31:0] tmp32;
    always @(*) begin
        // bias sign-extend to ACC_WIDTH then to 64-bit tmp64
        tmp64 = {{(64-ACC_WIDTH){acc[ACC_WIDTH-1]}}, acc[ACC_WIDTH-1:0]}; // extend acc to 64
        tmp64 = tmp64 + {{(64-32){bias[31]}}, bias};
        // multiply by scale (scale is small unsigned)
        tmp64 = tmp64 * {{(64-4){1'b0}}, scale};
        // arithmetic right shift by shift
        tmp64 = tmp64 >>> shift;
        // saturate to 32-bit signed
        if (tmp64 > 64'sh7FFF_FFFF) tmp32 = 32'sh7FFF_FFFF;
        else if (tmp64 < -64'sh80000000) tmp32 = -32'sh80000000;
        else tmp32 = tmp64[31:0];
        if (relu_en && tmp32[31]) tmp32 = 32'sd0; // if negative, zero-out
    end

    // register result
    always @(posedge clk) begin
        if (!resetn) result <= 32'sd0;
        else result <= tmp32;
    end

endmodule
