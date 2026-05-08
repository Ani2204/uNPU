// =============================================================================
//  npu_top.v – Parametric NPU Top-Level Integration
//
//  Supported PE counts: 256 / 512 / 1024 / 2048 (set PE_COUNT at elaboration)
//
//  Architecture:
//    Stage 0: BRAM → DMA Controller → stream_fifo → input_formatter
//    Stage 1: load_balancer → pe_cluster (compute)
//    Stage 2: reduction_tree (embedded in pe_cluster)
//    Stage 3: output_formatter → quantizer → output_fifo → CSR/AXI
//
//  Control: AXI4-Full slave → axi_slave_if → csr_register_file
//           instr_sequencer, interrupt_controller, perf_monitor
//  Memory:  memory_controller (4 banks, ECC, BIST)
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module npu_top #(
    parameter integer PE_COUNT   = 256,    // 256 | 512 | 1024 | 2048
    parameter integer DWIDTH     = 8,
    parameter integer ACC_WIDTH  = 32,
    parameter integer BRAM_DEPTH = 4096,
    parameter integer BUS_WIDTH  = 64,
    parameter integer AXI_AW     = 16,
    parameter integer AXI_DW     = 32,
    parameter integer AXI_IDW    = 4
)(
    input  wire                  clk,
    input  wire                  resetn_async,  // async active-low reset

    // -------------------------------------------------------------------------
    // AXI4-Full slave
    // -------------------------------------------------------------------------
    input  wire [AXI_IDW-1:0]   s_awid,
    input  wire [AXI_AW-1:0]    s_awaddr,
    input  wire [7:0]            s_awlen,
    input  wire [2:0]            s_awsize,
    input  wire [1:0]            s_awburst,
    input  wire                  s_awvalid,
    output wire                  s_awready,

    input  wire [AXI_DW-1:0]    s_wdata,
    input  wire [AXI_DW/8-1:0]  s_wstrb,
    input  wire                  s_wlast,
    input  wire                  s_wvalid,
    output wire                  s_wready,

    output wire [AXI_IDW-1:0]   s_bid,
    output wire [1:0]            s_bresp,
    output wire                  s_bvalid,
    input  wire                  s_bready,

    input  wire [AXI_IDW-1:0]   s_arid,
    input  wire [AXI_AW-1:0]    s_araddr,
    input  wire [7:0]            s_arlen,
    input  wire [2:0]            s_arsize,
    input  wire [1:0]            s_arburst,
    input  wire                  s_arvalid,
    output wire                  s_arready,

    output wire [AXI_IDW-1:0]   s_rid,
    output wire [AXI_DW-1:0]    s_rdata,
    output wire [1:0]            s_rresp,
    output wire                  s_rlast,
    output wire                  s_rvalid,
    input  wire                  s_rready,

    // -------------------------------------------------------------------------
    // Interrupt
    // -------------------------------------------------------------------------
    output wire                  irq_out
);

    // =========================================================================
    // Derived parameters
    // =========================================================================
    localparam integer TILE_ROWS = 8;
    localparam integer TILE_COLS = 8;
    localparam integer TILE_PES  = TILE_ROWS * TILE_COLS;  // 64
    localparam integer N_TILES   = PE_COUNT / TILE_PES;    // 4 for 256 PEs
    localparam integer FLAT_WORDS = (PE_COUNT * DWIDTH) / AXI_DW;

    // =========================================================================
    // Reset synchroniser
    // =========================================================================
    wire resetn;
    reset_sync u_rstsync (
        .clk          (clk),
        .resetn_async (resetn_async),
        .resetn_sync  (resetn)
    );

    // =========================================================================
    // AXI Slave → CSR register file
    // =========================================================================
    wire [15:0]        csr_wr_addr;
    wire [AXI_DW-1:0]  csr_wr_data;
    wire [AXI_DW/8-1:0] csr_wr_strb;
    wire               csr_wr_en;
    wire [15:0]        csr_rd_addr;
    wire [AXI_DW-1:0]  csr_rd_data;

    axi_slave_if #(
        .ADDR_WIDTH (AXI_AW),
        .DATA_WIDTH (AXI_DW),
        .ID_WIDTH   (AXI_IDW)
    ) u_axi (
        .clk         (clk),
        .resetn      (resetn),
        .s_awid      (s_awid),    .s_awaddr  (s_awaddr),
        .s_awlen     (s_awlen),   .s_awsize  (s_awsize),
        .s_awburst   (s_awburst), .s_awvalid (s_awvalid),  .s_awready (s_awready),
        .s_wdata     (s_wdata),   .s_wstrb   (s_wstrb),
        .s_wlast     (s_wlast),   .s_wvalid  (s_wvalid),   .s_wready  (s_wready),
        .s_bid       (s_bid),     .s_bresp   (s_bresp),
        .s_bvalid    (s_bvalid),  .s_bready  (s_bready),
        .s_arid      (s_arid),    .s_araddr  (s_araddr),
        .s_arlen     (s_arlen),   .s_arsize  (s_arsize),
        .s_arburst   (s_arburst), .s_arvalid (s_arvalid),  .s_arready (s_arready),
        .s_rid       (s_rid),     .s_rdata   (s_rdata),
        .s_rresp     (s_rresp),   .s_rlast   (s_rlast),
        .s_rvalid    (s_rvalid),  .s_rready  (s_rready),
        .csr_wr_addr (csr_wr_addr), .csr_wr_data (csr_wr_data),
        .csr_wr_strb (csr_wr_strb), .csr_wr_en   (csr_wr_en),
        .csr_rd_addr (csr_rd_addr), .csr_rd_data (csr_rd_data)
    );

    // =========================================================================
    // CSR Register File
    // =========================================================================
    wire [1:0]   mode_sel;
    wire         fuse_en, relu_en, gate_en, dyn_sched_en, slice_en, data_sel;
    wire [3:0]   act_sel, scale, shift, msb_stat_thres;
    wire [7:0]   threshold;
    wire signed [31:0] bias;
    wire [31:0]  dma_ctrl, dma_src_addr, dma_dst_addr, dma_len;
    wire         pm_start, pm_clear, bist_start;
    wire [6:0]   irq_enable, irq_clear_csr;
    wire [N_TILES*TILE_PES*DWIDTH-1:0] A_flat_csr, B_flat_csr;
    wire [31:0]  bram_wdata_csr;
    wire         bram_we_csr;
    wire [15:0]  bram_waddr_csr;

    // Feedback wires from datapath
    wire [63:0]  pm_cyc_count, pm_op_count;
    wire [31:0]  pm_active_pe, pm_stall_cyc, pm_latency;
    wire [6:0]   irq_status_wire;
    wire         ecc_single, ecc_double;
    wire         bist_done_wire, bist_pass_wire, bist_fail_wire;
    wire [31:0]  dma_status_wire;
    wire signed [31:0] mac_result_wire;

    wire [31:0] npu_status_wire;

    csr_register_file #(
        .DATA_WIDTH (AXI_DW),
        .FLAT_WORDS (FLAT_WORDS)
    ) u_csr (
        .clk           (clk),         .resetn        (resetn),
        .wr_addr       (csr_wr_addr), .wr_data       (csr_wr_data),
        .wr_strb       (csr_wr_strb), .wr_en         (csr_wr_en),
        .rd_addr       (csr_rd_addr), .rd_data       (csr_rd_data),
        .mode_sel      (mode_sel),    .fuse_en       (fuse_en),
        .act_sel       (act_sel),     .bias          (bias),
        .scale         (scale),       .shift         (shift),
        .gate_en       (gate_en),     .threshold     (threshold),
        .dyn_sched_en  (dyn_sched_en),.slice_en      (slice_en),
        .msb_stat_thres(msb_stat_thres), .relu_en   (relu_en),
        .data_sel      (data_sel),
        .dma_ctrl      (dma_ctrl),    .dma_src_addr  (dma_src_addr),
        .dma_dst_addr  (dma_dst_addr),.dma_len       (dma_len),
        .pm_start      (pm_start),    .pm_clear      (pm_clear),
        .pm_cyc_count  (pm_cyc_count),.pm_op_count   (pm_op_count),
        .pm_active_pe  (pm_active_pe),.pm_stall_cyc  (pm_stall_cyc),
        .pm_latency    (pm_latency),
        .npu_status    (npu_status_wire),
        .irq_status    (irq_status_wire), .irq_enable (irq_enable),
        .irq_clear     (irq_clear_csr),
        .ecc_single    (ecc_single),  .ecc_double    (ecc_double),
        .bist_pass     (bist_pass_wire), .bist_fail  (bist_fail_wire),
        .bist_start    (bist_start),
        .dma_status    (dma_status_wire),
        .mac_result    (mac_result_wire),
        .A_flat        (A_flat_csr),  .B_flat        (B_flat_csr),
        .bram_wdata    (bram_wdata_csr), .bram_we    (bram_we_csr),
        .bram_waddr    (bram_waddr_csr)
    );

    // =========================================================================
    // Memory Controller
    // =========================================================================
    localparam integer MEM_ADDR_W = $clog2(BRAM_DEPTH);
    localparam integer NUM_MEM_PORTS = 4;

    wire [NUM_MEM_PORTS*MEM_ADDR_W-1:0] mc_req_addr;
    wire [NUM_MEM_PORTS*BUS_WIDTH-1:0]  mc_req_wdata;
    wire [NUM_MEM_PORTS*(BUS_WIDTH/8)-1:0] mc_req_wstrb;
    wire [NUM_MEM_PORTS*2-1:0]          mc_req_bank;
    wire [NUM_MEM_PORTS-1:0]            mc_req_wen, mc_req_ren, mc_req_valid;
    wire [NUM_MEM_PORTS-1:0]            mc_req_ready;
    wire [NUM_MEM_PORTS*BUS_WIDTH-1:0]  mc_rsp_rdata;
    wire [NUM_MEM_PORTS-1:0]            mc_rsp_valid;

    memory_controller #(
        .DEPTH     (BRAM_DEPTH),
        .BUS_WIDTH (BUS_WIDTH),
        .NUM_BANKS (4),
        .NUM_PORTS (NUM_MEM_PORTS),
        .ECC_EN    (1)
    ) u_mem (
        .clk        (clk),          .resetn     (resetn),
        .req_addr   (mc_req_addr),  .req_wdata  (mc_req_wdata),
        .req_wstrb  (mc_req_wstrb), .req_bank   (mc_req_bank),
        .req_wen    (mc_req_wen),   .req_ren    (mc_req_ren),
        .req_valid  (mc_req_valid), .req_ready  (mc_req_ready),
        .rsp_rdata  (mc_rsp_rdata), .rsp_valid  (mc_rsp_valid),
        .ecc_single (ecc_single),   .ecc_double (ecc_double),
        .bist_start (bist_start),   .bist_done  (bist_done_wire),
        .bist_pass  (bist_pass_wire), .bist_fail (bist_fail_wire)
    );

    // =========================================================================
    // DMA Controller + Scheduler + Prefetch
    // =========================================================================
    wire         dma_done_w, dma_error_w;
    wire [7:0]   dma_out_data;
    wire         dma_out_valid, dma_out_ready;
    wire [MEM_ADDR_W-1:0] dma_rd_addr_w;
    wire                  dma_rd_en_w;
    wire [BUS_WIDTH-1:0]  dma_rd_data_w;
    wire                  dma_rd_valid_w;

    // DMA port 0 connections
    assign mc_req_addr [1*MEM_ADDR_W-1 -: MEM_ADDR_W] = dma_rd_addr_w;
    assign mc_req_wdata[1*BUS_WIDTH-1   -: BUS_WIDTH]  = {BUS_WIDTH{1'b0}};
    assign mc_req_wstrb[1*(BUS_WIDTH/8)-1 -: BUS_WIDTH/8] = {(BUS_WIDTH/8){1'b0}};
    assign mc_req_bank [1*2-1 -: 2]                    = 2'b01; // activation bank
    assign mc_req_wen  [0]                              = 1'b0;
    assign mc_req_ren  [0]                              = dma_rd_en_w;
    assign mc_req_valid[0]                              = dma_rd_en_w;
    assign dma_rd_data_w  = mc_rsp_rdata[1*BUS_WIDTH-1 -: BUS_WIDTH];
    assign dma_rd_valid_w = mc_rsp_valid[0];

    // Tie off unused ports 1-3 for now (compute/output ports)
    assign mc_req_addr [(2)*MEM_ADDR_W-1 -: MEM_ADDR_W] = {MEM_ADDR_W{1'b0}};
    assign mc_req_addr [(3)*MEM_ADDR_W-1 -: MEM_ADDR_W] = {MEM_ADDR_W{1'b0}};
    assign mc_req_addr [(4)*MEM_ADDR_W-1 -: MEM_ADDR_W] = {MEM_ADDR_W{1'b0}};
    assign mc_req_wdata[(2)*BUS_WIDTH-1 -: BUS_WIDTH]    = {BUS_WIDTH{1'b0}};
    assign mc_req_wdata[(3)*BUS_WIDTH-1 -: BUS_WIDTH]    = {BUS_WIDTH{1'b0}};
    assign mc_req_wdata[(4)*BUS_WIDTH-1 -: BUS_WIDTH]    = {BUS_WIDTH{1'b0}};
    assign mc_req_wstrb[(2)*(BUS_WIDTH/8)-1 -: BUS_WIDTH/8] = {(BUS_WIDTH/8){1'b0}};
    assign mc_req_wstrb[(3)*(BUS_WIDTH/8)-1 -: BUS_WIDTH/8] = {(BUS_WIDTH/8){1'b0}};
    assign mc_req_wstrb[(4)*(BUS_WIDTH/8)-1 -: BUS_WIDTH/8] = {(BUS_WIDTH/8){1'b0}};
    assign mc_req_bank [(2)*2-1 -: 2] = 2'b00;
    assign mc_req_bank [(3)*2-1 -: 2] = 2'b10;
    assign mc_req_bank [(4)*2-1 -: 2] = 2'b11;
    assign mc_req_wen  [3:1]           = 3'b000;
    assign mc_req_ren  [3:1]           = 3'b000;
    assign mc_req_valid[3:1]           = 3'b000;

    dma_controller #(
        .BRAM_DEPTH (BRAM_DEPTH),
        .BUS_WIDTH  (BUS_WIDTH),
        .DWIDTH     (DWIDTH)
    ) u_dma (
        .clk             (clk),
        .resetn          (resetn),
        .src_addr        (dma_src_addr[15:0]),
        .byte_len        (dma_len[15:0]),
        .mode            (dma_ctrl[1:0]),
        .dma_start       (dma_ctrl[0]),
        .dma_done        (dma_done_w),
        .dma_error       (dma_error_w),
        .mem_rd_addr     (dma_rd_addr_w),
        .mem_rd_en       (dma_rd_en_w),
        .mem_rd_data     (dma_rd_data_w),
        .mem_rd_valid    (dma_rd_valid_w),
        .pf_rd_ptr       (),
        .pf_active       (),
        .pf_data         ({BUS_WIDTH{1'b0}}),
        .pf_valid        (1'b0),
        .pf_consume      (),
        .out_data        (dma_out_data),
        .out_valid       (dma_out_valid),
        .out_ready       (dma_out_ready),
        .bytes_transferred (dma_status_wire[15:0])
    );

    assign dma_status_wire[31:16] = {14'd0, dma_error_w, dma_done_w};

    // =========================================================================
    // Input Stage: stream_fifo → input_formatter
    // =========================================================================
    wire [DWIDTH-1:0] fifo_out_data;
    wire              fifo_out_valid, fifo_out_ready;
    wire [DWIDTH-1:0] fmt_out_data;
    wire              fmt_out_valid, fmt_out_ready;

    stream_fifo #(.DWIDTH(DWIDTH), .DEPTH(512)) u_in_fifo_a (
        .clk        (clk),        .resetn    (resetn),
        .in_data    (dma_out_data), .in_valid (dma_out_valid), .in_ready (dma_out_ready),
        .out_data   (fifo_out_data), .out_valid (fifo_out_valid), .out_ready (fifo_out_ready),
        .fill_level (), .almost_full (), .almost_empty ()
    );

    input_formatter #(.DWIDTH(DWIDTH)) u_fmt (
        .clk       (clk),     .resetn    (resetn),
        .mode_sel  (mode_sel),
        .raw_data  (fifo_out_data),  .raw_valid (fifo_out_valid), .raw_ready (fifo_out_ready),
        .fmt_data  (fmt_out_data),   .fmt_valid (fmt_out_valid),  .fmt_ready (fmt_out_ready)
    );

    // =========================================================================
    // Load Balancer
    // =========================================================================
    wire [PE_COUNT*DWIDTH-1:0] lb_A_flat, lb_B_flat;
    wire [PE_COUNT-1:0]        lb_gate_en;
    wire                       lb_batch_done;
    wire [PE_COUNT-1:0]        pe_busy_wire;

    // For simplicity, A and B share the same input stream in this integration
    // (B is from A_flat_csr / B_flat_csr in CSR mode)
    load_balancer #(
        .DWIDTH  (DWIDTH),
        .NUM_PES (PE_COUNT)
    ) u_lb (
        .clk          (clk),      .resetn       (resetn),
        .threshold    (threshold),.dyn_sched_en (dyn_sched_en),
        .start        (gate_en),  .mode_sel     (mode_sel),
        .a_data       (data_sel ? A_flat_csr[DWIDTH-1:0] : fmt_out_data),
        .a_valid      (data_sel ? 1'b1 : fmt_out_valid),
        .a_ready      (fmt_out_ready),
        .b_data       (data_sel ? B_flat_csr[DWIDTH-1:0] : fmt_out_data),
        .b_valid      (data_sel ? 1'b1 : fmt_out_valid),
        .b_ready      (),
        .pe_busy      (pe_busy_wire),
        .A_flat       (lb_A_flat), .B_flat     (lb_B_flat),
        .gate_en      (lb_gate_en),.batch_done (lb_batch_done)
    );

    // Mux: CSR mode uses A/B flat directly
    wire [PE_COUNT*DWIDTH-1:0] mac_A_flat =
        data_sel ? A_flat_csr[PE_COUNT*DWIDTH-1:0] : lb_A_flat;
    wire [PE_COUNT*DWIDTH-1:0] mac_B_flat =
        data_sel ? B_flat_csr[PE_COUNT*DWIDTH-1:0] : lb_B_flat;
    wire [PE_COUNT-1:0] mac_gate_en =
        data_sel ? {PE_COUNT{gate_en}} : lb_gate_en;

    // =========================================================================
    // PE Cluster (Compute Stage)
    // =========================================================================
    wire signed [ACC_WIDTH-1:0] cluster_sum;
    wire                        cluster_valid;

    pe_cluster #(
        .DWIDTH    (DWIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .TILE_ROWS (TILE_ROWS),
        .TILE_COLS (TILE_COLS),
        .N_TILES   (N_TILES)
    ) u_cluster (
        .clk          (clk),
        .resetn       (resetn),
        .mode_sel     (mode_sel),
        .acc_clear    (1'b0),
        .slice_en     (slice_en),
        .tile_en      ({N_TILES{1'b1}}),
        .A_flat       (mac_A_flat),
        .B_flat       (mac_B_flat),
        .gate_en      (mac_gate_en),
        .cluster_sum  (cluster_sum),
        .cluster_valid(cluster_valid),
        .pe_busy      (pe_busy_wire)
    );

    // =========================================================================
    // Output Stage: output_formatter → quantizer → output_fifo
    // =========================================================================
    wire signed [ACC_WIDTH-1:0] fmt_result;
    wire                        fmt_result_valid;
    wire signed [ACC_WIDTH-1:0] q_result;
    wire                        q_valid;

    output_formatter #(.ACC_WIDTH(ACC_WIDTH)) u_out_fmt (
        .clk        (clk),      .resetn    (resetn),
        .bias       (bias),     .scale     (scale),
        .shift      (shift),    .relu_en   (relu_en),
        .act_sel    (act_sel),  .fuse_en   (fuse_en),
        .sum_in     (cluster_sum),  .sum_valid (cluster_valid),
        .fmt_out    (fmt_result),   .fmt_valid (fmt_result_valid)
    );

    quantizer #(.ACC_WIDTH(ACC_WIDTH)) u_quant (
        .clk       (clk),     .resetn   (resetn),
        .out_mode  (mode_sel), .scale   (scale),   .shift  (shift),
        .acc_in    (fmt_result),  .acc_valid (fmt_result_valid),
        .q_out     (q_result),    .q_valid   (q_valid)
    );

    wire [ACC_WIDTH-1:0] out_fifo_data;
    wire                 out_fifo_valid, out_fifo_ready;

    output_fifo #(.DWIDTH(ACC_WIDTH), .DEPTH(64)) u_out_fifo (
        .clk       (clk),    .resetn    (resetn),
        .in_data   (q_result),   .in_valid  (q_valid),    .in_ready  (),
        .out_data  (out_fifo_data), .out_valid (out_fifo_valid), .out_ready (1'b1),
        .fill_level (), .overflow  ()
    );

    assign mac_result_wire = out_fifo_data;

    // =========================================================================
    // Performance Monitor
    // =========================================================================
    wire [3:0] pipe_stall_w = 4'b0; // connect to pipeline_control when used

    perf_monitor #(.NUM_PES(PE_COUNT)) u_perf (
        .clk          (clk),        .resetn       (resetn),
        .pm_start     (pm_start),   .pm_clear     (pm_clear),
        .pm_stop      (lb_batch_done),
        .gate_en      (mac_gate_en), .pe_busy     (pe_busy_wire),
        .pipe_stall   (pipe_stall_w),
        .cyc_count    (pm_cyc_count), .op_count   (pm_op_count),
        .active_pe_sum(pm_active_pe), .stall_cycles (pm_stall_cyc),
        .last_latency (pm_latency)
    );

    // =========================================================================
    // Interrupt Controller
    // =========================================================================
    wire [6:0] irq_src_w = {
        bist_fail_wire,       // [6]
        bist_done_wire,       // [5]
        1'b0,                 // [4] watchdog – not implemented
        ecc_double,           // [3]
        ecc_single,           // [2]
        lb_batch_done,        // [1] compute done
        dma_done_w            // [0] DMA done
    };

    interrupt_controller u_irq (
        .clk        (clk),      .resetn     (resetn),
        .irq_src    (irq_src_w),
        .irq_enable (irq_enable), .irq_clear (irq_clear_csr),
        .irq_status (irq_status_wire),
        .irq_out    (irq_out)
    );

    // =========================================================================
    // NPU Status word
    // =========================================================================
    assign npu_status_wire = {
        24'd0,
        ecc_double,            // [7]
        ecc_single,            // [6]
        bist_fail_wire,        // [5]
        bist_pass_wire,        // [4]
        dma_error_w,           // [3]
        dma_done_w,            // [2]
        lb_batch_done,         // [1] compute done
        |pe_busy_wire          // [0] busy
    };

endmodule
