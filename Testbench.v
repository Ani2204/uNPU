`timescale 1ns/1ps
module tb_mac_array_mem;

    // Match your MAC params
    localparam integer NUM_PES   = 256;
    localparam integer ELEM_BITS = 8;
    localparam integer WIDTH     = NUM_PES * ELEM_BITS; // 2048
    localparam integer ACC_WIDTH = 64;

    reg clk = 0;
    always #5 clk = ~clk;    // 100 MHz
    real FCLK_MHZ = 100.0;

    reg resetn = 0;

    // MAC inputs
    reg  [1:0]          mode_sel;
    reg                 fuse_en;
    reg  [WIDTH-1:0]    A_flat;
    reg  [WIDTH-1:0]    B_flat;
    reg  [NUM_PES-1:0]  gate_en;
    reg                 slice_en_global;
    reg  [3:0]          msb_stat_thres;
    reg                 relu_en;
    reg  signed [31:0]  bias;
    reg  [3:0]          scale;
    reg  [3:0]          shift;

    wire signed [31:0]  result;
    wire [16*32-1:0]    result_vec;  // per-row results (ROWS=16; new; not checked here)
    wire [NUM_PES-1:0]  pe_busy;

    // simple perf counters
    reg [63:0] perf_cycles;
    reg [63:0] perf_ops_acc;

    // CSV file handles + metrics
    real    macs_per_cycle;
    real    macs_per_sec;
    localparam [1023:0] CSV_SUMMARY_PATH  = "D:\\project_1\\project_1.sim\\sim_1\\behav\\xsim\\perf_mac_array.csv";
    localparam [1023:0] CSV_TIMELINE_PATH = "D:\\project_1\\project_1.sim\\sim_1\\behav\\xsim\\perf_mac_array_timeline.csv";
    integer fd_summary;
    integer fd_timeline;

    // for monitor
    reg [31:0] last_res;

    // ----------------------------------------------------------------
    // DUT
    // ----------------------------------------------------------------
    mac_array #(
        .WIDTH(ELEM_BITS),
        .ACC_WIDTH(ACC_WIDTH),
        .ROWS(16),
        .COLS(16),
        .NUM_PES(NUM_PES),
        .PE_LATENCY(1)
    ) dut_mac (
        .clk(clk),
        .resetn(resetn),
        .mode_sel(mode_sel),
        .fuse_en(fuse_en),
        .A_flat(A_flat),
        .B_flat(B_flat),
        .gate_en(gate_en),
        .slice_en_global(slice_en_global),
        .msb_stat_thres(msb_stat_thres),
        .relu_en(relu_en),
        .bias(bias),
        .scale(scale),
        .shift(shift),
        // New systolic ports: tied off for SIMD testbench
        .a_row_in({16*ELEM_BITS{1'b0}}),
        .b_col_in({16*ELEM_BITS{1'b0}}),
        .systolic_en(1'b0),
        .acc_clear(1'b0),
        .result(result),
        .result_vec(result_vec),
        .pe_busy(pe_busy)
    );

    // ----------------------------------------------------------------
    // Load A/B from mem files
    // ----------------------------------------------------------------
    reg [7:0] A_hex [0:255];
    reg [7:0] B_hex [0:255];

    integer i;
    integer signed expected_sum;

    initial begin
        // change filenames if needed
        $readmemh("A_8bit.mem", A_hex);
        $readmemh("B_8bit.mem", B_hex);

        $display("First 8 A_hex:");
        for (i = 0; i < 8; i = i + 1)
            $display("  A[%0d] = %0d (0x%0h)", i, $signed(A_hex[i]), A_hex[i]);

        $display("First 8 B_hex:");
        for (i = 0; i < 8; i = i + 1)
            $display("  B[%0d] = %0d (0x%0h)", i, $signed(B_hex[i]), B_hex[i]);

        expected_sum = 0;
        for (i = 0; i < 256; i = i + 1)
            expected_sum = expected_sum + $signed(A_hex[i]) * $signed(B_hex[i]);

        $display("INFO: Expected DOT-PROD from mem = %0d", expected_sum);
    end

    // Pack A_hex/B_hex into A_flat/B_flat in the same order MAC expects
    task pack_AB_from_hex;
    begin
        A_flat = {WIDTH{1'b0}};
        B_flat = {WIDTH{1'b0}};
        for (i = 0; i < NUM_PES; i = i + 1) begin
            A_flat[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = A_hex[i];
            B_flat[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = B_hex[i];
        end
    end
    endtask

    // ----------------------------------------------------------------
    // Simple monitor + op count
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            last_res     <= 32'hDEAD_BEEF;
            perf_ops_acc <= 0;
        end else begin
            if (result !== last_res) begin
                $display("%0t : MAC RESULT = %0d (0x%0h)", $time, $signed(result), result);
                last_res     <= result;
                perf_ops_acc <= perf_ops_acc + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // cycle counter + per-cycle CSV logging
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            perf_cycles <= 0;
        end else begin
            perf_cycles <= perf_cycles + 1;

            // write one row per cycle into timeline CSV
            if (fd_timeline != 0) begin
                $fdisplay(fd_timeline,
                  "%0d,%0d,%0d",
                  perf_cycles, perf_ops_acc, $signed(result));
            end
        end
    end

    // ----------------------------------------------------------------
    // main sequence + SUMMARY CSV logging (INT8 only)
    // ----------------------------------------------------------------
    initial begin
        // open summary CSV
        fd_summary = $fopen(CSV_SUMMARY_PATH, "w");
        if (fd_summary == 0) begin
            $display("ERROR: could not open %s", CSV_SUMMARY_PATH);
            $finish;
        end
        $display("INFO: logging summary to %s", CSV_SUMMARY_PATH);
        $fdisplay(fd_summary,
          "mode_sel,fclk_MHz,perf_cycles,perf_ops_acc,mac_result,expected,macs_per_cycle,macs_per_sec");

        // open per-cycle CSV
        fd_timeline = $fopen(CSV_TIMELINE_PATH, "w");
        if (fd_timeline == 0) begin
            $display("ERROR: could not open %s", CSV_TIMELINE_PATH);
            $finish;
        end
        $display("INFO: logging timeline to %s", CSV_TIMELINE_PATH);
        $fdisplay(fd_timeline,
          "cycle,perf_ops_acc,mac_result");

        // -------- INT8 baseline config --------
        mode_sel         = 2'b00;   // INT8 only
        fuse_en          = 1'b0;
        gate_en          = {NUM_PES{1'b1}}; // enable all PEs
        slice_en_global  = 1'b0;
        msb_stat_thres   = 4'd0;
        relu_en          = 1'b0;
        bias             = 32'sd0;
        scale            = 4'd1;
        shift            = 4'd0;
        last_res         = 32'hDEAD_BEEF;

        // reset + apply inputs
        resetn = 0;
        pack_AB_from_hex();
        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (20) @(posedge clk);

        $display("=== DIRECT MAC DOT-PRODUCT TEST (INT8 FROM MEM) ===");

        // wait some cycles for reduction tree to settle
        repeat (200) @(posedge clk);

        $display("FINAL MAC RESULT = %0d (0x%0h)", $signed(result), result);

        if ($signed(result) === expected_sum)
            $display("TEST PASS: MAC equals expected dot-product from mem (%0d)", expected_sum);
        else
            $display("TEST FAIL: DUT=%0d, EXPECTED=%0d, DIFF=%0d",
                     $signed(result), expected_sum, $signed(result) - expected_sum);

        // DEBUG: show counters
        $display("DEBUG: perf_cycles=%0d perf_ops_acc=%0d", perf_cycles, perf_ops_acc);

        // compute and log SUMMARY to CSV
        if (perf_cycles != 0)
            macs_per_cycle = perf_ops_acc * 1.0 / perf_cycles;
        else
            macs_per_cycle = 0.0;

        macs_per_sec = macs_per_cycle * FCLK_MHZ * 1.0e6;

        $display("LOG: mode=%0d cycles=%0d ops=%0d result=%0d exp=%0d",
                 mode_sel, perf_cycles, perf_ops_acc, $signed(result), expected_sum);

        $fdisplay(fd_summary,
          "%0d,%0f,%0d,%0d,%0d,%0d,%0f,%0e",
          mode_sel, FCLK_MHZ, perf_cycles, perf_ops_acc,
          $signed(result), expected_sum, macs_per_cycle, macs_per_sec);

        // close files and finish
        $fclose(fd_summary);
        $fclose(fd_timeline);

        repeat (10) @(posedge clk);
        $finish;
    end

endmodule
