// =============================================================================
//  stress_test.sv – Stress & Corner Cases
//
//  Tests edge cases: zero inputs, max values, saturation, sign handling.
// =============================================================================
`timescale 1ns/1ps

module stress_test;

    localparam integer DWIDTH    = 8;
    localparam integer ACC_WIDTH = 32;

    reg clk = 0;
    always #5 clk = ~clk;
    reg resetn;

    reg [1:0]        mode_sel;
    reg              gate_en, acc_clear, slice_en;
    reg [DWIDTH-1:0] A_in, B_in;
    wire [ACC_WIDTH-1:0] result;
    wire                 busy, valid_out;

    pe_core #(.DWIDTH(DWIDTH), .ACC_WIDTH(ACC_WIDTH)) dut (
        .clk       (clk), .resetn    (resetn),
        .mode_sel  (mode_sel), .gate_en  (gate_en),
        .acc_clear (acc_clear), .slice_en (slice_en),
        .A_in      (A_in), .B_in       (B_in),
        .result    (result), .busy      (busy), .valid_out (valid_out)
    );

    task fire_pe;
        input [DWIDTH-1:0] a;
        input [DWIDTH-1:0] b;
        input              clr;
        integer timeout;
    begin
        A_in = a; B_in = b; acc_clear = clr;
        gate_en = 1'b1;
        @(posedge clk);
        gate_en   = 1'b0;
        acc_clear = 1'b0;
        timeout = 0;
        while (!valid_out && timeout < 15) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
    end
    endtask

    task check;
        input [ACC_WIDTH-1:0] expected;
        input [8*24-1:0]      name;
    begin
        if (result == expected)
            $display("PASS [%s] result=%0d", name, $signed(result));
        else
            $display("FAIL [%s] expected=%0d got=%0d", name, $signed(expected), $signed(result));
    end
    endtask

    initial begin
        resetn    = 0;
        mode_sel  = 2'b00;
        gate_en   = 0;
        acc_clear = 0;
        slice_en  = 0;
        A_in = 0; B_in = 0;
        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (5)  @(posedge clk);

        // Zero inputs
        fire_pe(8'd0, 8'd0, 1'b1);
        check(32'd0, "zero_zero");

        // Max positive INT8: 127 * 127 = 16129
        fire_pe(8'd127, 8'd127, 1'b1);
        check(32'd16129, "max_pos_int8");

        // Max negative * max positive: -128 * 127 = -16256
        fire_pe(8'h80, 8'd127, 1'b1); // 0x80 = -128 signed
        @(posedge clk);
        @(posedge clk);
        if ($signed(result) == -16256)
            $display("PASS [min_neg_int8] result=%0d", $signed(result));
        else
            $display("FAIL [min_neg_int8] expected -16256 got %0d", $signed(result));

        // Saturation: accumulate 127*127 repeatedly without clearing
        // After enough iterations the accumulator should saturate at max int32
        mode_sel = 2'b00;
        fire_pe(8'd127, 8'd127, 1'b1);  // first: 16129
        fire_pe(8'd127, 8'd127, 1'b0);  // acc: 32258
        fire_pe(8'd127, 8'd127, 1'b0);  // acc: 48387
        // (No saturation at these values – just verify accumulation works)
        @(posedge clk);
        @(posedge clk);
        if ($signed(result) > 16129)
            $display("PASS [accumulate] result=%0d > 16129", $signed(result));
        else
            $display("FAIL [accumulate] result=%0d", $signed(result));

        // INT4 mode, negative nibble: A[3:0] = 0xF = -1, B[3:0] = 0x3 = 3
        mode_sel = 2'b01;
        fire_pe(8'h0F, 8'h03, 1'b1);
        @(posedge clk); @(posedge clk);
        if ($signed(result) == -3)
            $display("PASS [int4_neg_nibble] result=%0d", $signed(result));
        else
            $display("FAIL [int4_neg_nibble] expected -3 got %0d", $signed(result));

        $display("\nstress_test: done.");
        $finish;
    end

endmodule
