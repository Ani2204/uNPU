`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
//
// top_phase3.v -- uNPU top-level integration (all improvements #1-#10)
//
// Key changes vs prior version:
//   - CSR flat A/B data path removed; DMA is the sole operand source (#5/#6).
//   - Dual BRAM ports (A and B simultaneously loaded) via dma_bram (#2).
//   - Ping-pong batch buffers (refill hidden behind compute) (#6).
//   - Parallel loader dispatch (single-cycle static, priority-encoder dynamic) (#1).
//   - Systolic mode inputs (a_row_in / b_col_in) threaded through (#4).
//   - Per-row K-accumulation and result_vec from mac_array (#3/#9).
//   - AXI4-Stream output that bursts all ROWS per-row results after each batch (#10).
//
module top_phase3 #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter WIDTH      = 2048,
    parameter ACC_WIDTH  = 64,
    parameter NUM_PES    = 256,
    parameter ROWS       = 16,
    parameter COLS       = 16
)(
    input  wire                  clk,
    input  wire                  resetn,

    // AXI4-Lite control interface
    input  wire [ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire                  s_axi_awvalid,
    output wire                  s_axi_awready,
    input  wire [DATA_WIDTH-1:0] s_axi_wdata,
    input  wire [3:0]            s_axi_wstrb,
    input  wire                  s_axi_wvalid,
    output wire                  s_axi_wready,
    output wire [1:0]            s_axi_bresp,
    output wire                  s_axi_bvalid,
    input  wire                  s_axi_bready,
    input  wire [ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire                  s_axi_arvalid,
    output wire                  s_axi_arready,
    output wire [DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0]            s_axi_rresp,
    output wire                  s_axi_rvalid,
    input  wire                  s_axi_rready,

    // AXI4-Stream result output — bursts ROWS 32-bit words after each batch (#10)
    output wire                  m_axis_tvalid,
    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                  m_axis_tlast,
    input  wire                  m_axis_tready
);

    // ----------------------------------------------------------------
    // CSR control signals
    // ----------------------------------------------------------------
    wire [1:0]  mode_sel;
    wire signed [31:0] bias;
    wire [3:0]  scale, shift;
    wire        fuse_en;
    wire [3:0]  act_sel;
    wire        gate_en;
    wire [7:0]  threshold;
    wire        dyn_sched_en;
    wire        slice_en;
    wire [3:0]  msb_stat_thres;
    wire        relu_en;
    wire        systolic_en;
    wire        acc_clear;

    wire [31:0] dma_ctrl, dma_addr, dma_len, dma_status;
    wire        perf_start;
    wire        perf_done_to_csr;

    reg  [63:0] perf_cycles;
    reg  [63:0] perf_ops_acc;
    reg  [31:0] perf_active_pe_sum;
    reg  [31:0] perf_last_latency;

    // BRAM write ports (CSR → dma_bram)
    wire [7:0]  csr_bram_wdata_a, csr_bram_wdata_b;
    wire        csr_bram_we_a,    csr_bram_we_b;
    wire [15:0] csr_bram_waddr_a, csr_bram_waddr_b;

    // DMA streaming
    wire dma_a_valid, dma_a_ready;
    wire [7:0] dma_a_data;
    wire dma_b_valid, dma_b_ready;
    wire [7:0] dma_b_data;

    // FIFO outputs
    wire a_fifo_valid, a_fifo_ready;
    wire [7:0] a_fifo_data;
    wire b_fifo_valid, b_fifo_ready;
    wire [7:0] b_fifo_data;

    // Batch buffer outputs
    wire signed [WIDTH-1:0] A_flat_dma, B_flat_dma;
    wire A_batch_ready, B_batch_ready;
    reg  A_batch_consume, B_batch_consume;

    // Loader / MAC interconnect
    wire signed [NUM_PES*8-1:0] loader_A, loader_B;
    wire [NUM_PES-1:0]          loader_gate_en;
    wire [NUM_PES-1:0]          pe_busy;
    wire signed [31:0]          mac_result;
    wire [ROWS*32-1:0]          result_vec;
    wire                        loader_batch_done;

    // Synchroniser registers
    reg loader_batch_done_r, loader_batch_done_rr;
    wire done_pulse;
    reg [NUM_PES-1:0] pe_busy_sync;

    // Latched DMA flat buses for loader
    reg signed [WIDTH-1:0] A_flat_src, B_flat_src;

    // Start-load control FSM
    reg start_load, start_load_ack;

    // ----------------------------------------------------------------
    // CSR instantiation
    // ----------------------------------------------------------------
    axi_csr_if #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .ROWS(ROWS)
    ) u_csr (
        .clk(clk), .resetn(resetn),
        .awaddr(s_axi_awaddr),  .awvalid(s_axi_awvalid), .awready(s_axi_awready),
        .wdata(s_axi_wdata),    .wstrb(s_axi_wstrb),    .wvalid(s_axi_wvalid),
        .wready(s_axi_wready),  .bresp(s_axi_bresp),    .bvalid(s_axi_bvalid),
        .bready(s_axi_bready),
        .araddr(s_axi_araddr),  .arvalid(s_axi_arvalid),.arready(s_axi_arready),
        .rdata(s_axi_rdata),    .rresp(s_axi_rresp),    .rvalid(s_axi_rvalid),
        .rready(s_axi_rready),

        .mode_sel(mode_sel), .bias(bias), .scale(scale), .shift(shift),
        .fuse_en(fuse_en),   .act_sel(act_sel),
        .gate_en(gate_en),   .relu_en(relu_en),
        .systolic_en(systolic_en), .acc_clear(acc_clear),
        .threshold(threshold), .dyn_sched_en(dyn_sched_en),
        .slice_en(slice_en),   .msb_stat_thres(msb_stat_thres),

        .perf_start(perf_start),        .perf_done(perf_done_to_csr),
        .perf_cycles(perf_cycles),      .perf_ops_acc(perf_ops_acc),
        .perf_active_pe_sum(perf_active_pe_sum),
        .perf_last_latency(perf_last_latency),

        .dma_ctrl(dma_ctrl), .dma_addr(dma_addr),
        .dma_len(dma_len),   .dma_status(dma_status),

        .mac_result(mac_result),
        .result_vec(result_vec),

        .bram_wdata_a(csr_bram_wdata_a), .bram_we_a(csr_bram_we_a),
        .bram_waddr_a(csr_bram_waddr_a),
        .bram_wdata_b(csr_bram_wdata_b), .bram_we_b(csr_bram_we_b),
        .bram_waddr_b(csr_bram_waddr_b)
    );

    // ----------------------------------------------------------------
    // dma_bram: dual A+B block RAM, concurrent streaming
    // ----------------------------------------------------------------
    dma_bram u_dma (
        .clk(clk), .resetn(resetn),
        .dma_ctrl(dma_ctrl), .dma_addr(dma_addr),
        .dma_len(dma_len),   .dma_status(dma_status),

        .bram_wdata_a(csr_bram_wdata_a), .bram_we_a(csr_bram_we_a),
        .bram_waddr_a(csr_bram_waddr_a),
        .bram_wdata_b(csr_bram_wdata_b), .bram_we_b(csr_bram_we_b),
        .bram_waddr_b(csr_bram_waddr_b),

        .a_valid(dma_a_valid), .a_data(dma_a_data), .a_ready(dma_a_ready),
        .b_valid(dma_b_valid), .b_data(dma_b_data), .b_ready(dma_b_ready)
    );

    // ----------------------------------------------------------------
    // FIFOs (elastic buffering between DMA and batch buffers)
    // ----------------------------------------------------------------
    byte_fifo #(.DEPTH(512)) u_fifo_a (
        .clk(clk), .resetn(resetn),
        .in_valid(dma_a_valid), .in_data(dma_a_data), .in_ready(dma_a_ready),
        .out_valid(a_fifo_valid), .out_data(a_fifo_data), .out_ready(a_fifo_ready)
    );

    byte_fifo #(.DEPTH(512)) u_fifo_b (
        .clk(clk), .resetn(resetn),
        .in_valid(dma_b_valid), .in_data(dma_b_data), .in_ready(dma_b_ready),
        .out_valid(b_fifo_valid), .out_data(b_fifo_data), .out_ready(b_fifo_ready)
    );

    // ----------------------------------------------------------------
    // Ping-pong batch buffers (improvement #6)
    // ----------------------------------------------------------------
    batch_buffer #(.NUM_ELEMS(NUM_PES)) u_batch_A (
        .clk(clk), .resetn(resetn),
        .in_valid(a_fifo_valid), .in_byte(a_fifo_data),
        .mode_sel(mode_sel),     .in_ready(a_fifo_ready),
        .consume(A_batch_consume),
        .flat_out(A_flat_dma),   .batch_ready(A_batch_ready)
    );

    batch_buffer #(.NUM_ELEMS(NUM_PES)) u_batch_B (
        .clk(clk), .resetn(resetn),
        .in_valid(b_fifo_valid), .in_byte(b_fifo_data),
        .mode_sel(mode_sel),     .in_ready(b_fifo_ready),
        .consume(B_batch_consume),
        .flat_out(B_flat_dma),   .batch_ready(B_batch_ready)
    );

    // ----------------------------------------------------------------
    // Loader — parallel dispatch (improvement #1)
    // ----------------------------------------------------------------
    loader #(.WIDTH(WIDTH), .ELEM_BITS(8), .NUM_PES(NUM_PES)) u_loader (
        .clk(clk), .resetn(resetn),
        .A_flat_in(A_flat_src), .B_flat_in(B_flat_src),
        .threshold(threshold),  .dyn_sched_en(dyn_sched_en),
        .csr_gate_en(start_load),
        .pe_busy(pe_busy_sync), .mode_sel(mode_sel),
        .A_out(loader_A),       .B_out(loader_B),
        .loader_gate_en(loader_gate_en),
        .batch_done(loader_batch_done)
    );

    // two-flop synchroniser + rising-edge detector for batch_done
    always @(posedge clk) begin
        if (!resetn) begin
            loader_batch_done_r  <= 1'b0;
            loader_batch_done_rr <= 1'b0;
        end else begin
            loader_batch_done_r  <= loader_batch_done;
            loader_batch_done_rr <= loader_batch_done_r;
        end
    end
    assign done_pulse       = loader_batch_done_r & ~loader_batch_done_rr;
    assign perf_done_to_csr = done_pulse;

    always @(posedge clk) begin
        if (!resetn) pe_busy_sync <= {NUM_PES{1'b0}};
        else         pe_busy_sync <= pe_busy;
    end

    // ----------------------------------------------------------------
    // DMA → loader start / consume control
    // CSR flat path removed; DMA is the sole data source.
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            A_flat_src      <= {WIDTH{1'b0}};
            B_flat_src      <= {WIDTH{1'b0}};
            A_batch_consume <= 1'b0;
            B_batch_consume <= 1'b0;
            start_load      <= 1'b0;
            start_load_ack  <= 1'b0;
        end else begin
            A_batch_consume <= 1'b0;
            B_batch_consume <= 1'b0;

            // Arm load when both buffers have a complete batch
            if (!start_load && A_batch_ready && B_batch_ready)
                start_load <= 1'b1;

            // Latch batch data once before triggering loader
            if (start_load && !start_load_ack) begin
                A_flat_src     <= A_flat_dma;
                B_flat_src     <= B_flat_dma;
                start_load_ack <= 1'b1;
            end

            // Release batch buffers once loader is done
            if (done_pulse) begin
                start_load      <= 1'b0;
                start_load_ack  <= 1'b0;
                A_batch_consume <= 1'b1;
                B_batch_consume <= 1'b1;
            end
        end
    end

    // ----------------------------------------------------------------
    // MAC array — systolic + per-row accumulators (improvements #3/#4/#9)
    // ----------------------------------------------------------------
    // In systolic mode, a_row_in / b_col_in come from the loader output.
    // The loader's A_out row-0 value = A_out[7:0] (first COLS values are
    // the K-slice for rows 0..ROWS-1; each row gets WIDTH bits of its slice).
    // For SIMD mode (systolic_en=0) a_row_in / b_col_in are ignored by the tiles.
    wire [ROWS*8-1:0] a_row_slice;
    wire [COLS*8-1:0] b_col_slice;

    // Extract row slices: row r = loader_A[r*(COLS*8) +: 8] (first PE of each row)
    genvar rg;
    generate
        for (rg = 0; rg < ROWS; rg = rg + 1) begin : gen_row_slice
            assign a_row_slice[(rg+1)*8-1 -: 8] =
                loader_A[(rg*COLS + 0 + 1)*8-1 -: 8];
        end
    endgenerate

    genvar cg;
    generate
        for (cg = 0; cg < COLS; cg = cg + 1) begin : gen_col_slice
            assign b_col_slice[(cg+1)*8-1 -: 8] =
                loader_B[(0*COLS + cg + 1)*8-1 -: 8];
        end
    endgenerate

    mac_array #(
        .WIDTH(8), .ACC_WIDTH(ACC_WIDTH),
        .ROWS(ROWS), .COLS(COLS),
        .NUM_PES(NUM_PES), .PE_LATENCY(1)
    ) u_mac (
        .clk(clk), .resetn(resetn),
        .mode_sel(mode_sel),  .fuse_en(fuse_en),
        .A_flat(loader_A),    .B_flat(loader_B),
        .gate_en(loader_gate_en & {NUM_PES{gate_en}}),
        .slice_en_global(slice_en),
        .msb_stat_thres(msb_stat_thres),
        .relu_en(relu_en),
        .bias(bias), .scale(scale), .shift(shift),
        .a_row_in(a_row_slice), .b_col_in(b_col_slice),
        .systolic_en(systolic_en), .acc_clear(acc_clear),
        .result(mac_result),
        .result_vec(result_vec),
        .pe_busy(pe_busy)
    );

    // ----------------------------------------------------------------
    // AXI4-Stream result burst — serialises result_vec after each batch
    // (improvement #10)
    // ----------------------------------------------------------------
    reg [$clog2(ROWS)-1:0] burst_idx;
    reg                    burst_active;

    assign m_axis_tvalid = burst_active;
    assign m_axis_tdata  = result_vec[burst_idx*32 +: 32];
    assign m_axis_tlast  = (burst_idx == ROWS - 1);

    always @(posedge clk) begin
        if (!resetn) begin
            burst_active <= 1'b0;
            burst_idx    <= {$clog2(ROWS){1'b0}};
        end else begin
            if (done_pulse) begin
                burst_active <= 1'b1;
                burst_idx    <= {$clog2(ROWS){1'b0}};
            end else if (burst_active && m_axis_tready) begin
                if (burst_idx == ROWS - 1)
                    burst_active <= 1'b0;
                else
                    burst_idx <= burst_idx + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Performance counters
    // ----------------------------------------------------------------
    function [15:0] popcount_bus;
        input [NUM_PES-1:0] v;
        integer ii;
        reg [15:0] c;
        begin
            c = 0;
            for (ii = 0; ii < NUM_PES; ii = ii + 1)
                c = c + v[ii];
            popcount_bus = c;
        end
    endfunction

    reg perf_armed;
    always @(posedge clk) begin
        if (!resetn) begin
            perf_cycles        <= 64'd0;
            perf_ops_acc       <= 64'd0;
            perf_active_pe_sum <= 32'd0;
            perf_last_latency  <= 32'd0;
            perf_armed         <= 1'b0;
        end else begin
            if (perf_start) begin
                perf_armed         <= 1'b1;
                perf_cycles        <= 64'd0;
                perf_ops_acc       <= 64'd0;
                perf_active_pe_sum <= 32'd0;
            end
            if (perf_armed) begin
                perf_cycles        <= perf_cycles + 1;
                perf_ops_acc       <= perf_ops_acc + popcount_bus(loader_gate_en);
                perf_active_pe_sum <= perf_active_pe_sum + popcount_bus(pe_busy);
            end
            if (done_pulse && perf_armed) begin
                perf_armed        <= 1'b0;
                perf_last_latency <= perf_cycles[31:0];
            end
        end
    end

endmodule
