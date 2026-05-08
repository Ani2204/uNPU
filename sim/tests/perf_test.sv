// =============================================================================
//  perf_test.sv – Throughput Benchmark
//
//  Measures throughput (ops/cycle) by running back-to-back batches.
// =============================================================================
`timescale 1ns/1ps

module perf_test;

    localparam integer PE_COUNT  = 256;
    localparam integer DWIDTH    = 8;
    localparam integer ACC_WIDTH = 32;
    localparam integer TILE_ROWS = 8;
    localparam integer TILE_COLS = 8;
    localparam integer N_TILES   = PE_COUNT / (TILE_ROWS * TILE_COLS);

    reg clk = 0;
    always #5 clk = ~clk;

    reg resetn;
    reg [1:0] mode_sel = 2'b00;
    reg slice_en = 0;

    // Cluster signals
    reg  [PE_COUNT*DWIDTH-1:0] A_flat, B_flat;
    reg  [PE_COUNT-1:0]        gate_en;
    wire signed [ACC_WIDTH-1:0] cluster_sum;
    wire                        cluster_valid;
    wire [PE_COUNT-1:0]         pe_busy;

    pe_cluster #(
        .DWIDTH    (DWIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .TILE_ROWS (TILE_ROWS),
        .TILE_COLS (TILE_COLS),
        .N_TILES   (N_TILES)
    ) dut (
        .clk          (clk),
        .resetn       (resetn),
        .mode_sel     (mode_sel),
        .acc_clear    (1'b1),
        .slice_en     (slice_en),
        .tile_en      ({N_TILES{1'b1}}),
        .A_flat       (A_flat),
        .B_flat       (B_flat),
        .gate_en      (gate_en),
        .cluster_sum  (cluster_sum),
        .cluster_valid(cluster_valid),
        .pe_busy      (pe_busy)
    );

    integer i;
    integer batch_num;
    integer t_start, t_end;
    integer total_ops;
    integer NUM_BATCHES = 16;

    task run_batch;
        input [DWIDTH-1:0] a_val;
        input [DWIDTH-1:0] b_val;
    begin
        for (i = 0; i < PE_COUNT; i = i + 1) begin
            A_flat[(i+1)*DWIDTH-1 -: DWIDTH] = a_val;
            B_flat[(i+1)*DWIDTH-1 -: DWIDTH] = b_val;
        end
        gate_en = {PE_COUNT{1'b1}};
        @(posedge clk);
        gate_en = {PE_COUNT{1'b0}};
    end
    endtask

    initial begin
        $display("=== Throughput Benchmark: PE_COUNT=%0d ===", PE_COUNT);

        resetn = 0;
        A_flat = 0; B_flat = 0; gate_en = 0;
        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (5)  @(posedge clk);

        t_start   = $time;
        total_ops = 0;

        for (batch_num = 0; batch_num < NUM_BATCHES; batch_num = batch_num + 1) begin
            run_batch(8'd5, 8'd3);
            // Wait for result
            repeat (20) @(posedge clk);
            total_ops = total_ops + PE_COUNT;
        end

        t_end = $time;
        $display("Batches: %0d,  Total ops: %0d", NUM_BATCHES, total_ops);
        $display("Sim time: %0d ns,  Effective OPS/cycle ≈ %0d",
                 t_end - t_start, total_ops * 10 / (t_end - t_start));

        $display("Expected cluster_sum per batch: %0d", PE_COUNT * 5 * 3);
        $display("Last cluster_sum: %0d", cluster_sum);

        $finish;
    end

endmodule
