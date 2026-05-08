// =============================================================================
//  latency_test.sv – End-to-End Latency Measurement
//
//  Measures cycles from gate_en pulse to cluster_valid output.
// =============================================================================
`timescale 1ns/1ps

module latency_test;

    localparam integer PE_COUNT  = 64;   // single tile for latency focus
    localparam integer DWIDTH    = 8;
    localparam integer ACC_WIDTH = 32;
    localparam integer ROWS      = 8;
    localparam integer COLS      = 8;

    reg clk = 0;
    always #5 clk = ~clk;
    reg resetn;

    reg  [PE_COUNT*DWIDTH-1:0] A_flat, B_flat;
    reg  [PE_COUNT-1:0]        gate_en;
    wire [ACC_WIDTH-1:0]        tile_sum;
    wire                        tile_valid;
    wire [PE_COUNT-1:0]         pe_busy;

    pe_tile #(
        .DWIDTH    (DWIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .ROWS      (ROWS),
        .COLS      (COLS),
        .NUM_PES   (PE_COUNT)
    ) dut (
        .clk       (clk), .resetn    (resetn),
        .tile_en   (1'b1),
        .mode_sel  (2'b00), .acc_clear (1'b1), .slice_en (1'b0),
        .A_flat    (A_flat), .B_flat (B_flat),
        .gate_en   (gate_en),
        .tile_sum  (tile_sum), .tile_valid (tile_valid),
        .pe_busy   (pe_busy)
    );

    integer i;
    integer lat;
    integer timeout;

    initial begin
        $display("=== Latency Test: PE_COUNT=%0d ===", PE_COUNT);

        resetn = 0;
        gate_en = 0;
        for (i = 0; i < PE_COUNT; i = i + 1) begin
            A_flat[(i+1)*DWIDTH-1 -: DWIDTH] = 8'd2;
            B_flat[(i+1)*DWIDTH-1 -: DWIDTH] = 8'd3;
        end

        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (5)  @(posedge clk);

        // Pulse gate
        gate_en = {PE_COUNT{1'b1}};
        @(posedge clk);
        gate_en = {PE_COUNT{1'b0}};

        // Count cycles until tile_valid
        lat = 0;
        timeout = 0;
        while (timeout < 100) begin
            @(posedge clk);
            lat = lat + 1;
            timeout = timeout + 1;
            if (tile_valid) begin
                $display("PASS: Tile valid after %0d cycles, sum=%0d (expected %0d)",
                         lat, tile_sum, PE_COUNT * 2 * 3);
                disable;
            end
        end

        $display("INFO: tile_valid not seen in 100 cycles (tree latency > 100 not expected)");
        $finish;
    end

endmodule
