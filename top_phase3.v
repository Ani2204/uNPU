`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module top_phase3 #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter WIDTH      = 2048,
    parameter ACC_WIDTH  = 64,
    parameter NUM_PES    = 256
)(
    input  wire                  clk,
    input  wire                  resetn,

    // AXI4-lite (connect your master to these)
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
    input  wire                  s_axi_rready
);

    // === CSR wires (read from CSR) ===
    wire [1:0] mode_sel;
    wire signed [31:0] bias;
    wire [3:0] scale, shift;
    wire fuse_en;
    wire [3:0] act_sel;
    wire signed [WIDTH-1:0] A_flat_csr, B_flat_csr;
    wire gate_en;
    wire [7:0] threshold;
    wire dyn_sched_en;
    wire slice_en;
    wire [3:0] msb_stat_thres;
    wire relu_en;
    wire data_sel;

    wire [31:0] dma_ctrl, dma_addr, dma_len, dma_status;
    wire perf_start;
    wire perf_done_to_csr;

    reg  [63:0] perf_cycles;
    reg  [63:0] perf_ops_acc;
    reg  [31:0] perf_active_pe_sum;
    reg  [31:0] perf_last_latency;

    // --- BRAM preload outputs from CSR (CPU writes) ---
    wire [31:0] csr_bram_wdata;
    wire        csr_bram_we;
    wire [15:0] csr_bram_waddr;

    // DMA streaming outputs
    wire dma_a_valid, dma_a_ready;
    wire [7:0] dma_a_data;
    wire dma_b_valid, dma_b_ready;
    wire [7:0] dma_b_data;

    // FIFO inputs/outputs
    wire a_fifo_valid, a_fifo_ready;
    wire [7:0] a_fifo_data;
    wire b_fifo_valid, b_fifo_ready;
    wire [7:0] b_fifo_data;

    // Batch buffer outputs
    wire signed [WIDTH-1:0] A_flat_dma;
    wire A_batch_ready;
    reg  A_batch_consume;
    wire signed [WIDTH-1:0] B_flat_dma;
    wire B_batch_ready;
    reg  B_batch_consume;

    // Loader / MAC nets
    wire signed [NUM_PES*8-1:0] loader_A, loader_B;
    wire [NUM_PES-1:0] loader_gate_en;
    wire [NUM_PES-1:0] pe_busy;
    wire signed [31:0] mac_result;
    wire loader_batch_done;

    // sync regs
    reg loader_batch_done_r, loader_batch_done_rr;
    wire done_pulse;
    reg [NUM_PES-1:0] pe_busy_sync;

    // source selection
    reg signed [WIDTH-1:0] A_flat_src, B_flat_src;

    // --- START/LOAD control
    reg start_load;         // latched start request for loader using DMA data
    reg start_load_ack;     // latched acknowledgement that flats were captured
    wire csr_gate_start;    // drives loader.csr_gate_en

    // ------------------------------------------------------------------
    // CSR instance (now exports csr_bram_* signals)
    // ------------------------------------------------------------------
    axi_csr_if #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .WIDTH(WIDTH)) u_csr (
        .clk(clk), .resetn(resetn),
        .awaddr(s_axi_awaddr), .awvalid(s_axi_awvalid), .awready(s_axi_awready),
        .wdata(s_axi_wdata), .wstrb(s_axi_wstrb), .wvalid(s_axi_wvalid), .wready(s_axi_wready),
        .bresp(s_axi_bresp), .bvalid(s_axi_bvalid), .bready(s_axi_bready),
        .araddr(s_axi_araddr), .arvalid(s_axi_arvalid), .arready(s_axi_arready),
        .rdata(s_axi_rdata), .rresp(s_axi_rresp), .rvalid(s_axi_rvalid), .rready(s_axi_rready),

        .mode_sel(mode_sel), .bias(bias), .scale(scale), .shift(shift),
        .fuse_en(fuse_en), .act_sel(act_sel),
        .A_flat(A_flat_csr), .B_flat(B_flat_csr),
        .gate_en(gate_en), .threshold(threshold), .dyn_sched_en(dyn_sched_en),
        .slice_en(slice_en), .msb_stat_thres(msb_stat_thres), .relu_en(relu_en),
        .perf_start(perf_start), .perf_done(perf_done_to_csr),
        .perf_cycles(perf_cycles), .perf_ops_acc(perf_ops_acc),
        .perf_active_pe_sum(perf_active_pe_sum), .perf_last_latency(perf_last_latency),
        .dma_ctrl(dma_ctrl), .dma_addr(dma_addr), .dma_len(dma_len), .dma_status(dma_status),
        .mac_result(mac_result), .data_sel(data_sel),

        // BRAM preload bridge (CPU -> dma_bram)
        .bram_wdata(csr_bram_wdata),
        .bram_we(csr_bram_we),
        .bram_waddr(csr_bram_waddr)
    );

    // ------------------------------------------------------------------
    // dma_bram instance (preload via CSR write signals)
    // ------------------------------------------------------------------
    dma_bram u_dma (
        .clk(clk), .resetn(resetn),
        .dma_ctrl(dma_ctrl), .dma_addr(dma_addr), .dma_len(dma_len), .dma_status(dma_status),

        // CPU preload path (CSR -> dma_bram)
        .bram_wdata(csr_bram_wdata),
        .bram_we(csr_bram_we),
        .bram_waddr(csr_bram_waddr),

        // streaming outputs -> FIFO
        .a_valid(dma_a_valid), .a_data(dma_a_data), .a_ready(dma_a_ready),
        .b_valid(dma_b_valid), .b_data(dma_b_data), .b_ready(dma_b_ready)
    );

    // ------------------------------------------------------------------
    // FIFOs: DMA streaming -> byte_fifo -> batch_buffer -> loader
    // ------------------------------------------------------------------
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

    batch_buffer #(.NUM_ELEMS(NUM_PES)) u_batch_A (
        .clk(clk), .resetn(resetn),
        .in_valid(a_fifo_valid), .in_byte(a_fifo_data), .mode_sel(mode_sel), .in_ready(a_fifo_ready),
        .consume(A_batch_consume),
        .flat_out(A_flat_dma), .batch_ready(A_batch_ready)
    );

    batch_buffer #(.NUM_ELEMS(NUM_PES)) u_batch_B (
        .clk(clk), .resetn(resetn),
        .in_valid(b_fifo_valid), .in_byte(b_fifo_data), .mode_sel(mode_sel), .in_ready(b_fifo_ready),
        .consume(B_batch_consume),
        .flat_out(B_flat_dma), .batch_ready(B_batch_ready)
    );

    // ------------------------------------------------------------------
    // Loader (DMA/CSR source selection)
    // ------------------------------------------------------------------
    loader #(.WIDTH(WIDTH), .ELEM_BITS(8), .NUM_PES(NUM_PES)) u_loader (
        .clk(clk), .resetn(resetn),
        .A_flat_in(A_flat_src), .B_flat_in(B_flat_src),
        .threshold(threshold), .dyn_sched_en(dyn_sched_en), .csr_gate_en(csr_gate_start),
        .pe_busy(pe_busy_sync),
        .mode_sel(mode_sel),
        .A_out(loader_A), .B_out(loader_B),
        .loader_gate_en(loader_gate_en),
        .batch_done(loader_batch_done)
    );

    // two-flop sync + single-cycle pulse
    always @(posedge clk) begin
        if (!resetn) begin
            loader_batch_done_r  <= 1'b0;
            loader_batch_done_rr <= 1'b0;
        end else begin
            loader_batch_done_r  <= loader_batch_done;
            loader_batch_done_rr <= loader_batch_done_r;
        end
    end

    assign done_pulse        = loader_batch_done_r & ~loader_batch_done_rr;
    assign perf_done_to_csr  = done_pulse;

    always @(posedge clk) begin
        if (!resetn)
            pe_busy_sync <= {NUM_PES{1'b0}};
        else
            pe_busy_sync <= pe_busy;
    end

    // data source muxing + start/consume control
    always @(posedge clk) begin
        if (!resetn) begin
            A_flat_src       <= {WIDTH{1'b0}};
            B_flat_src       <= {WIDTH{1'b0}};
            A_batch_consume  <= 1'b0;
            B_batch_consume  <= 1'b0;
            start_load       <= 1'b0;
            start_load_ack   <= 1'b0;
        end else begin
            // default: no consume unless loader finished
            A_batch_consume <= 1'b0;
            B_batch_consume <= 1'b0;

            // REQUEST: capture start when BOTH batches ready (DMA mode)
            if (!start_load && A_batch_ready && B_batch_ready && (data_sel == 1'b0)) begin
                start_load <= 1'b1;
            end

            // LATCH DMA flats once
            if (start_load && !start_load_ack) begin
                A_flat_src     <= A_flat_dma;
                B_flat_src     <= B_flat_dma;
                start_load_ack <= 1'b1;
            end

            // DONE: clear start and pulse consume
            if (done_pulse) begin
                start_load      <= 1'b0;
                start_load_ack  <= 1'b0;
                A_batch_consume <= 1'b1;
                B_batch_consume <= 1'b1;
            end

            // If not using DMA start, route CSR flats
            if (!start_load) begin
                if (data_sel == 1'b1) begin
                    A_flat_src <= A_flat_csr;
                    B_flat_src <= B_flat_csr;
                end else begin
                    // If DMA not ready, fall back to CSR; otherwise keep latched DMA until done
                    if (!(A_batch_ready && B_batch_ready)) begin
                        A_flat_src <= A_flat_csr;
                        B_flat_src <= B_flat_csr;
                    end
                end
            end
        end
    end

    // csr_gate_start:
    //  - CSR mode: directly use gate_en (loader runs but ignored by MAC in CSR tests)
    //  - DMA mode: use start_load FSM
    assign csr_gate_start =
        (data_sel == 1'b1) ? gate_en :
                             start_load;

    // ------------------------------------------------------------------
    // MAC input mux: CSR vs DMA/loader
    // ------------------------------------------------------------------
    wire signed [NUM_PES*8-1:0] mac_A_flat;
    wire signed [NUM_PES*8-1:0] mac_B_flat;
    wire [NUM_PES-1:0]          mac_gate_en;

    // CSR mode: direct A_flat_csr/B_flat_csr → MAC
    // DMA mode: loader_A/loader_B → MAC
    assign mac_A_flat =
        (data_sel == 1'b1) ? A_flat_csr[NUM_PES*8-1:0] : loader_A;

    assign mac_B_flat =
        (data_sel == 1'b1) ? B_flat_csr[NUM_PES*8-1:0] : loader_B;

    // gate_en:
    //  - CSR mode: broadcast global gate_en to all PEs
    //  - DMA mode: per-PE loader gating masked by gate_en
    assign mac_gate_en =
        (data_sel == 1'b1) ? {NUM_PES{gate_en}}
                           : (loader_gate_en & {NUM_PES{gate_en}});

    // ------------------------------------------------------------------
    // MAC array instance
    // ------------------------------------------------------------------
    mac_array #(
        .WIDTH(8),
        .ACC_WIDTH(ACC_WIDTH),
        .ROWS(16),
        .COLS(16),
        .NUM_PES(NUM_PES),
        .PE_LATENCY(1)
    ) u_mac (
        .clk(clk), .resetn(resetn),

        .mode_sel(mode_sel),
        .fuse_en(fuse_en),

        .A_flat(mac_A_flat),
        .B_flat(mac_B_flat),

        .gate_en(mac_gate_en),

        .slice_en_global(slice_en),

        .bias(bias),
        .scale(scale),
        .shift(shift),
        .msb_stat_thres(msb_stat_thres),
        .relu_en(relu_en),

        .result(mac_result),

        .pe_busy(pe_busy)
    );

    // perf counters (simple sampling)
    function [15:0] popcount_bus;
        input [NUM_PES-1:0] v;
        integer ii;
        reg [15:0] c;
        begin
            c = 0;
            for (ii=0; ii<NUM_PES; ii=ii+1)
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
                perf_armed        <= 1'b1;
                perf_cycles       <= 64'd0;
                perf_ops_acc      <= 64'd0;
                perf_active_pe_sum<= 32'd0;
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
