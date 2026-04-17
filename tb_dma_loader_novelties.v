`timescale 1ns/1ps
module tb_dma_loader_novelties;

    localparam integer NUM_PES   = 256;
    localparam integer ELEM_BITS = 8;
    localparam integer WIDTH     = NUM_PES * ELEM_BITS;
    localparam integer ACC_WIDTH = 64;

    // Clock / reset
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    reg resetn = 0;

    // Loader inputs/outputs
    reg  signed [WIDTH-1:0] A_flat_in;
    reg  signed [WIDTH-1:0] B_flat_in;
    reg  [ELEM_BITS-1:0]    threshold;
    reg                     dyn_sched_en;
    reg                     csr_gate_en;
    reg  [1:0]              mode_sel;

    wire signed [NUM_PES*ELEM_BITS-1:0] A_out;
    wire signed [NUM_PES*ELEM_BITS-1:0] B_out;
    wire [NUM_PES-1:0]                  loader_gate_en;
    wire [NUM_PES-1:0]                  pe_busy;
    wire                                batch_done;

    // MAC array control
    reg                     slice_en_global;
    reg  [3:0]              msb_stat_thres;
    reg                     relu_en;
    reg                     fuse_en;
    reg  signed [31:0]      bias;
    reg  [3:0]              scale;
    reg  [3:0]              shift;

    wire signed [31:0]      mac_result;
    wire [16*32-1:0]        mac_result_vec;  // per-row results (new; not checked here)

    // ---------------- DUTs ----------------

    // Use your updated loader (with dyn_sched_en)
    loader #(
        .WIDTH(WIDTH),
        .ELEM_BITS(ELEM_BITS),
        .NUM_PES(NUM_PES)
    ) u_loader (
        .clk(clk),
        .resetn(resetn),
        .A_flat_in(A_flat_in),
        .B_flat_in(B_flat_in),
        .threshold(threshold),
        .dyn_sched_en(dyn_sched_en),
        .csr_gate_en(csr_gate_en),
        .pe_busy(pe_busy),
        .mode_sel(mode_sel),
        .A_out(A_out),
        .B_out(B_out),
        .loader_gate_en(loader_gate_en),
        .batch_done(batch_done)
    );

    mac_array #(
        .WIDTH(ELEM_BITS),
        .ACC_WIDTH(ACC_WIDTH),
        .ROWS(16),
        .COLS(16),
        .NUM_PES(NUM_PES),
        .PE_LATENCY(1)
    ) u_mac (
        .clk(clk),
        .resetn(resetn),
        .mode_sel(mode_sel),
        .fuse_en(fuse_en),
        .A_flat(A_out),
        .B_flat(B_out),
        .gate_en(loader_gate_en),
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
        .result(mac_result),
        .result_vec(mac_result_vec),
        .pe_busy(pe_busy)
    );

    // ---------------- PATTERN GENERATORS FOR A/B ----------------
    integer i;

    task set_all_AB_zero;
    begin
        A_flat_in = {WIDTH{1'b0}};
        B_flat_in = {WIDTH{1'b0}};
    end
    endtask

    // N1: dense INT8 baseline (A=5, B=3 everywhere) → expected 3840
    task set_dense_AB_5x3;
    begin
        set_all_AB_zero();
        for (i = 0; i < NUM_PES; i = i + 1) begin
            A_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd5;
            B_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd3;
        end
    end
    endtask

    // N1_AUTO_MIX: small hand-crafted example for per-activation precision
    // idx0: A=0x0F (INT4 → -1), B=1
    // idx1: A=0x70 (INT8 → 112), B=1
    // idx2: A=0x03 (INT4 → 3),  B=1
    // expected sum = -1 + 112 + 3 = 114
    task set_auto_mixed_pattern;
    begin
        set_all_AB_zero();
        A_flat_in[1*ELEM_BITS-1 -: ELEM_BITS] = 8'h0F;  // idx0
        B_flat_in[1*ELEM_BITS-1 -: ELEM_BITS] = 8'sd1;
        A_flat_in[2*ELEM_BITS-1 -: ELEM_BITS] = 8'h70;  // idx1
        B_flat_in[2*ELEM_BITS-1 -: ELEM_BITS] = 8'sd1;
        A_flat_in[3*ELEM_BITS-1 -: ELEM_BITS] = 8'h03;  // idx2
        B_flat_in[3*ELEM_BITS-1 -: ELEM_BITS] = 8'sd1;
    end
    endtask

    // N2/N3: high nibble heavy pattern (exercise INT4 packing/gating)
    task set_high_nibble_pattern;
    begin
        set_all_AB_zero();
        for (i = 0; i < NUM_PES; i = i + 1) begin
            A_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd127; // 0x7F
            B_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd63;  // 0x3F
        end
    end
    endtask

    // Sparse-ish pattern for N4/N5 thresh sweeps:
    //   0..63:  A=1, B=2   → prod=2 (small; pruned at thr>=4)
    //   64..127:A=5, B=3   → prod=15
    //   128..255:A=4,B=1   → prod=4
    task set_sparse_pattern;
    begin
        set_all_AB_zero();
        // first 64 small
        for (i = 0; i < 64; i = i + 1) begin
            A_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd1;
            B_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd2;
        end
        // next 64 larger
        for (i = 64; i < 128; i = i + 1) begin
            A_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd5;
            B_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd3;
        end
        // remaining 128 medium
        for (i = 128; i < NUM_PES; i = i + 1) begin
            A_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd4;
            B_flat_in[(i+1)*ELEM_BITS-1 -: ELEM_BITS] = 8'sd1;
        end
    end
    endtask

    // ---------------- Simple monitor ----------------
    reg [31:0] last_res;

    always @(posedge clk) begin
        if (!resetn) begin
            last_res <= 32'hDEAD_BEEF;
        end else if (mac_result !== last_res) begin
            $display("%0t : MAC RESULT = %0d (0x%0h)",
                     $time, $signed(mac_result), mac_result);
            last_res <= mac_result;
        end
    end

    // ---------------- Phase runner ----------------
    task run_one_phase(
        input [8*24-1:0] phase_name,
        input [1:0]      t_mode_sel,
        input [7:0]      t_threshold,
        input            t_dyn_sched_en,
        input            t_slice_en_global,
        input [3:0]      t_msb_stat_thres,
        input integer    pattern_sel   // 0=dense,1=auto_mix,2=high_nib,3=sparse
    );
    begin
        $display("\n=== %0s : mode_sel=%0d thr=%0d dyn_sched_en=%0d slice_en_global=%0d msb_thres=%0d ===",
                 phase_name, t_mode_sel, t_threshold,
                 t_dyn_sched_en, t_slice_en_global, t_msb_stat_thres);

        // Select A/B pattern
        case (pattern_sel)
            0: set_dense_AB_5x3();
            1: set_auto_mixed_pattern();
            2: set_high_nibble_pattern();
            3: set_sparse_pattern();
            default: set_dense_AB_5x3();
        endcase

        // Program controls
        mode_sel        = t_mode_sel;
        threshold       = t_threshold;
        dyn_sched_en    = t_dyn_sched_en;
        slice_en_global = t_slice_en_global;
        msb_stat_thres  = t_msb_stat_thres;

        // Reset
        resetn   = 0;  repeat (5) @(posedge clk);
        resetn   = 1;  repeat (5) @(posedge clk);
        last_res = 32'hDEAD_BEEF;
        csr_gate_en = 1'b0;

        // Kick loader
        csr_gate_en = 1'b1;
        @(posedge clk);
        // Wait while batch in progress
        wait (batch_done == 1'b0);
        wait (batch_done == 1'b1);
        csr_gate_en = 1'b0;

        // Let MAC pipeline flush
        repeat (30) @(posedge clk);

        $display("%0s : FINAL MAC RESULT = %0d (0x%0h)\n",
                 phase_name, $signed(mac_result), mac_result);
    end
    endtask

    // ---------------- MAIN TEST SEQUENCE ----------------
    initial begin
        // Common defaults
        slice_en_global = 1'b0;
        msb_stat_thres  = 4'd0;
        relu_en         = 1'b0;
        fuse_en         = 1'b0;  // no cross-batch accumulation
        bias            = 32'sd0;
        scale           = 4'd1;
        shift           = 4'd0;
        last_res        = 32'hDEAD_BEEF;
        csr_gate_en     = 1'b0;
        set_all_AB_zero();

        // Global reset
        resetn = 0;
        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (10) @(posedge clk);

        // ============================
        // N1: INT8 vs AUTO (per-activation precision)
        // ============================
        // Dense INT8 baseline: A=5, B=3 everywhere → expected 3840
        run_one_phase("N1_INT8_BASE", 2'b00, 8'd0, 1'b1, 1'b0, 4'd0, 0);
        // AUTO per-activation example → expected 114
        run_one_phase("N1_AUTO_MIX",  2'b11, 8'd0, 1'b1, 1'b1, 4'd1, 1);

        // ============================
        // N2: INT4 base vs packed (bit-partitioning)
        // ============================
        // INT4 base: only low-4 bits contribute
        run_one_phase("N2_INT4_BASE", 2'b01, 8'd0, 1'b1, 1'b0, 4'd1, 2);
        // INT4 packed: low+high nibbles when stats allow
        run_one_phase("N2_INT4_PACK", 2'b01, 8'd0, 1'b1, 1'b1, 4'd1, 2);

        // ============================
        // N3: Precision gating via msb_stat_thres
        // (here using INT4 mode to make gating visible)
        // ============================
        run_one_phase("N3_GATE_LO", 2'b01, 8'd0, 1'b1, 1'b1, 4'd1, 2);
        run_one_phase("N3_GATE_HI", 2'b01, 8'd0, 1'b1, 1'b1, 4'd8, 2);

        // ============================
        // N4: Dynamic scheduler OFF vs ON
        // Same sparse pattern, same threshold; ONLY dyn_sched_en changes.
        // With 256 elems & 256 PEs, sums will match; this still proves that
        // enabling scheduler does not break correctness.
        // ============================
        run_one_phase("N4_SCHED_OFF", 2'b00, 8'd4, 1'b0, 1'b0, 4'd0, 3);
        run_one_phase("N4_SCHED_ON",  2'b00, 8'd4, 1'b1, 1'b0, 4'd0, 3);

        // ============================
        // N5: Sparsity + quantization fusion (threshold sweep)
        // ============================
        // INT8 sweep
        run_one_phase("N5_INT8_T0", 2'b00, 8'd0, 1'b1, 1'b0, 4'd0, 3);
        run_one_phase("N5_INT8_T2", 2'b00, 8'd2, 1'b1, 1'b0, 4'd0, 3);
        run_one_phase("N5_INT8_T4", 2'b00, 8'd4, 1'b1, 1'b0, 4'd0, 3);
        run_one_phase("N5_INT8_T6", 2'b00, 8'd6, 1'b1, 1'b0, 4'd0, 3);
        run_one_phase("N5_INT8_T8", 2'b00, 8'd8, 1'b1, 1'b0, 4'd0, 3);

        // AUTO sweep on same sparse pattern
        run_one_phase("N5_AUTO_T0", 2'b11, 8'd0, 1'b1, 1'b1, 4'd1, 3);
        run_one_phase("N5_AUTO_T4", 2'b11, 8'd4, 1'b1, 1'b1, 4'd1, 3);
        run_one_phase("N5_AUTO_T8", 2'b11, 8'd8, 1'b1, 1'b1, 4'd1, 3);

        $display("\nAll novelty tests done. $finish.\n");
        $finish;
    end

endmodule
