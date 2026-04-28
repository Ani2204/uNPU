// =============================================================================
//  tb_compute.sv – Compute Core Testbench
//
//  Tests pe_core, reduction_tree, pe_tile, and pe_cluster.
//  Runs parameterized with NUM_PES = 64 (8×8 tile) for fast simulation.
// =============================================================================
`timescale 1ns/1ps

module tb_compute;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer DWIDTH    = 8;
    localparam integer ACC_WIDTH = 32;
    localparam integer ROWS      = 8;
    localparam integer COLS      = 8;
    localparam integer NUM_PES   = ROWS * COLS;   // 64

    // =========================================================================
    // Clock / Reset
    // =========================================================================
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    reg resetn;

    // =========================================================================
    // DUT signals
    // =========================================================================
    // -- pe_core --
    reg  [1:0]              mode_sel;
    reg                     gate_en_c, acc_clear, slice_en;
    reg  [DWIDTH-1:0]       A_in, B_in;
    wire [ACC_WIDTH-1:0]    pe_result;
    wire                    pe_busy, pe_valid;

    pe_core #(.DWIDTH(DWIDTH), .ACC_WIDTH(ACC_WIDTH)) u_pe (
        .clk       (clk), .resetn    (resetn),
        .mode_sel  (mode_sel), .gate_en  (gate_en_c),
        .acc_clear (acc_clear), .slice_en (slice_en),
        .A_in      (A_in), .B_in       (B_in),
        .result    (pe_result), .busy     (pe_busy),
        .valid_out (pe_valid)
    );

    // -- pe_tile --
    reg  [NUM_PES*DWIDTH-1:0]  tile_A, tile_B;
    reg  [NUM_PES-1:0]          tile_gate;
    wire [ACC_WIDTH-1:0]        tile_sum;
    wire                        tile_valid_out;
    wire [NUM_PES-1:0]          tile_busy;

    pe_tile #(
        .DWIDTH    (DWIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .ROWS      (ROWS),
        .COLS      (COLS),
        .NUM_PES   (NUM_PES)
    ) u_tile (
        .clk       (clk), .resetn    (resetn),
        .tile_en   (1'b1),
        .mode_sel  (mode_sel), .acc_clear (acc_clear), .slice_en (slice_en),
        .A_flat    (tile_A), .B_flat   (tile_B),
        .gate_en   (tile_gate),
        .tile_sum  (tile_sum), .tile_valid (tile_valid_out),
        .pe_busy   (tile_busy)
    );

    // =========================================================================
    // Integer helper
    // =========================================================================
    integer i;

    // =========================================================================
    // Tasks
    // =========================================================================
    task reset_dut;
    begin
        resetn    = 0;
        gate_en_c = 0;
        acc_clear = 0;
        slice_en  = 0;
        mode_sel  = 2'b00;
        A_in      = 0;
        B_in      = 0;
        tile_A    = 0;
        tile_B    = 0;
        tile_gate = 0;
        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (5)  @(posedge clk);
    end
    endtask

    // =========================================================================
    // Test: pe_core – single INT8 multiply-accumulate
    // =========================================================================
    task test_pe_core_int8;
        integer timeout;
    begin
        $display("\n--- Test: pe_core INT8 ---");
        mode_sel  = 2'b00;
        slice_en  = 1'b0;
        acc_clear = 1'b1;
        A_in = 8'd10;
        B_in = 8'd5;
        gate_en_c = 1'b1;
        @(posedge clk);
        gate_en_c = 1'b0;
        acc_clear = 1'b0;

        // Wait for valid result (max 10 cycles)
        timeout = 0;
        while (!pe_valid && timeout < 10) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (pe_result == 32'd50)
            $display("PASS: pe_core INT8 10*5 = %0d", pe_result);
        else
            $display("FAIL: pe_core INT8 expected 50, got %0d", pe_result);
    end
    endtask

    // =========================================================================
    // Test: pe_tile – all PEs with A=3, B=4; expected sum = 64*12 = 768
    // =========================================================================
    task test_tile_all_same;
        integer timeout;
    begin
        $display("\n--- Test: pe_tile all-same (A=3, B=4, INT8) ---");
        mode_sel  = 2'b00;
        slice_en  = 1'b0;
        acc_clear = 1'b1;
        for (i = 0; i < NUM_PES; i = i + 1) begin
            tile_A[(i+1)*DWIDTH-1 -: DWIDTH] = 8'd3;
            tile_B[(i+1)*DWIDTH-1 -: DWIDTH] = 8'd4;
        end
        tile_gate = {NUM_PES{1'b1}};
        @(posedge clk);
        tile_gate = {NUM_PES{1'b0}};
        acc_clear = 1'b0;

        // Wait for tree output (up to 20 cycles)
        timeout = 0;
        while (!tile_valid_out && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        repeat (10) @(posedge clk); // Let tree drain

        if (tile_sum == 32'd768)
            $display("PASS: tile sum = %0d (expected 768)", tile_sum);
        else
            $display("FAIL: tile sum = %0d (expected 768)", tile_sum);
    end
    endtask

    // =========================================================================
    // Test: pe_core INT4 mode
    // =========================================================================
    task test_pe_core_int4;
        integer timeout;
    begin
        $display("\n--- Test: pe_core INT4 (low nibble A=3, B=2) ---");
        resetn = 0;
        @(posedge clk);
        resetn = 1;
        repeat (5) @(posedge clk);

        mode_sel  = 2'b01;
        slice_en  = 1'b0;
        acc_clear = 1'b1;
        // A = 0x?3 (low nibble = 3), B = 0x?2 (low nibble = 2)
        A_in = 8'h03;
        B_in = 8'h02;
        gate_en_c = 1'b1;
        @(posedge clk);
        gate_en_c = 1'b0;
        acc_clear = 1'b0;

        timeout = 0;
        while (!pe_valid && timeout < 10) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        // INT4 low nibble: 3 * 2 = 6
        if (pe_result == 32'd6)
            $display("PASS: pe_core INT4 low nibble 3*2 = %0d", pe_result);
        else
            $display("FAIL: pe_core INT4 expected 6, got %0d", pe_result);
    end
    endtask

    // =========================================================================
    // Main
    // =========================================================================
    initial begin
        reset_dut;
        test_pe_core_int8;
        test_pe_core_int4;
        test_tile_all_same;

        $display("\ntb_compute: all tests done.");
        $finish;
    end

endmodule
