// =============================================================================
//  memory_controller.v – Multi-Port Arbitrated Memory Controller
//
//  4 BRAM banks (weights / activations / intermediate / outputs).
//  Round-robin arbitration with priority override for DMA (highest priority).
//
//  Port mapping:
//    Port 0 – DMA engine  (highest priority)
//    Port 1 – Compute cluster (read operands)
//    Port 2 – Output formatter (write results)
//    Port 3 – CPU/CSR (background, lowest priority)
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module memory_controller #(
    parameter integer DEPTH     = 4096,
    parameter integer BUS_WIDTH = 64,
    parameter integer NUM_BANKS = 4,
    parameter integer NUM_PORTS = 4,
    parameter integer ECC_EN    = 1
)(
    input  wire                                   clk,
    input  wire                                   resetn,

    // Requester ports (all share the same address space; bank_sel picks bank)
    // Flat packed: [port_N] at bits [(N+1)*W-1 : N*W]

    input  wire [NUM_PORTS*$clog2(DEPTH)-1:0]    req_addr,
    input  wire [NUM_PORTS*BUS_WIDTH-1:0]        req_wdata,
    input  wire [NUM_PORTS*(BUS_WIDTH/8)-1:0]    req_wstrb,
    input  wire [NUM_PORTS*2-1:0]                req_bank,   // which bank
    input  wire [NUM_PORTS-1:0]                  req_wen,
    input  wire [NUM_PORTS-1:0]                  req_ren,
    input  wire [NUM_PORTS-1:0]                  req_valid,
    output reg  [NUM_PORTS-1:0]                  req_ready,  // granted this cycle

    // Read response (1-cycle latency per granted port)
    output reg  [NUM_PORTS*BUS_WIDTH-1:0]        rsp_rdata,
    output reg  [NUM_PORTS-1:0]                  rsp_valid,

    // ECC alerts (OR across all banks)
    output wire                                  ecc_single,
    output wire                                  ecc_double,

    // BIST control
    input  wire                                  bist_start,
    output wire                                  bist_done,
    output wire                                  bist_pass,
    output wire                                  bist_fail
);

    localparam integer ADDR_W = $clog2(DEPTH);

    // -------------------------------------------------------------------------
    // Arbiter: 4-port round-robin with port-0 priority override
    // -------------------------------------------------------------------------
    reg [1:0] rr_ptr;   // round-robin counter

    // Grant logic: port 0 always wins if valid; others rotate
    wire [NUM_PORTS-1:0] grant;
    reg  [NUM_PORTS-1:0] grant_r;

    // Simple round-robin grant (port 0 = highest priority)
    reg [NUM_PORTS-1:0] arb_grant;
    always @(*) begin
        arb_grant = {NUM_PORTS{1'b0}};
        if (req_valid[0])
            arb_grant[0] = 1'b1;
        else begin
            // round-robin among ports 1..3
            if (req_valid[rr_ptr])
                arb_grant[rr_ptr] = 1'b1;
            else begin
                // find next valid
                if (req_valid[1]) arb_grant[1] = 1'b1;
                else if (req_valid[2]) arb_grant[2] = 1'b1;
                else if (req_valid[3]) arb_grant[3] = 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            rr_ptr <= 2'd1;
        end else begin
            if (|arb_grant[3:1]) begin
                // advance past the just-served port
                if      (arb_grant[1]) rr_ptr <= 2'd2;
                else if (arb_grant[2]) rr_ptr <= 2'd3;
                else                   rr_ptr <= 2'd1;
            end
        end
    end

    assign grant = arb_grant;

    // -------------------------------------------------------------------------
    // Issue granted request to the appropriate bank
    // -------------------------------------------------------------------------
    // Decode which bank and extract request fields of granted port
    wire [1:0]        sel_bank;
    wire [ADDR_W-1:0] sel_addr;
    wire [BUS_WIDTH-1:0]     sel_wdata;
    wire [BUS_WIDTH/8-1:0]   sel_wstrb;
    wire              sel_wen;
    wire              sel_ren;

    // One-hot to index
    wire [1:0] grant_idx = grant[1] ? 2'd1 :
                           grant[2] ? 2'd2 :
                           grant[3] ? 2'd3 : 2'd0;

    assign sel_bank  = req_bank  [(grant_idx+1)*2-1 -: 2];
    assign sel_addr  = req_addr  [(grant_idx+1)*ADDR_W-1 -: ADDR_W];
    assign sel_wdata = req_wdata [(grant_idx+1)*BUS_WIDTH-1 -: BUS_WIDTH];
    assign sel_wstrb = req_wstrb [(grant_idx+1)*(BUS_WIDTH/8)-1 -: (BUS_WIDTH/8)];
    assign sel_wen   = req_wen   [grant_idx] & |grant;
    assign sel_ren   = req_ren   [grant_idx] & |grant;

    // -------------------------------------------------------------------------
    // Bank instances
    // -------------------------------------------------------------------------
    wire [NUM_BANKS-1:0]   bank_wr_en_vec;
    wire [NUM_BANKS-1:0]   bank_rd_en_vec;
    wire [BUS_WIDTH-1:0]   bank_rd_data [0:NUM_BANKS-1];
    wire [NUM_BANKS-1:0]   bank_rd_valid;
    wire [NUM_BANKS-1:0]   bank_ecc_s;
    wire [NUM_BANKS-1:0]   bank_ecc_d;

    // BIST signals (connect to bank 0 for simplicity)
    wire [ADDR_W-1:0]     bist_wr_addr_w, bist_rd_addr_w;
    wire [BUS_WIDTH-1:0]  bist_wr_data_w;
    wire [BUS_WIDTH/8-1:0] bist_wr_strb_w;
    wire                   bist_wr_en_w, bist_rd_en_w;
    wire [BUS_WIDTH-1:0]   bist_rd_data_w;
    wire                   bist_rd_valid_w;
    wire                   bist_active_w;

    mem_bist #(.DEPTH(DEPTH), .BUS_WIDTH(BUS_WIDTH)) u_bist (
        .clk          (clk),
        .resetn       (resetn),
        .bist_start   (bist_start),
        .bist_done    (bist_done),
        .bist_pass    (bist_pass),
        .bist_fail    (bist_fail),
        .bist_wr_addr (bist_wr_addr_w),
        .bist_wr_data (bist_wr_data_w),
        .bist_wr_strb (bist_wr_strb_w),
        .bist_wr_en   (bist_wr_en_w),
        .bist_rd_addr (bist_rd_addr_w),
        .bist_rd_en   (bist_rd_en_w),
        .bist_rd_data (bist_rd_data_w),
        .bist_rd_valid(bist_rd_valid_w),
        .bist_active  (bist_active_w)
    );

    genvar b;
    generate
        for (b = 0; b < NUM_BANKS; b = b + 1) begin : gen_bank
            // Bank select
            wire bank_sel_match = (sel_bank == b[1:0]) && |grant;
            assign bank_wr_en_vec[b] = bist_active_w ? (b==0 ? bist_wr_en_w  : 1'b0)
                                                      : (sel_wen && bank_sel_match);
            assign bank_rd_en_vec[b] = bist_active_w ? (b==0 ? bist_rd_en_w  : 1'b0)
                                                      : (sel_ren && bank_sel_match);

            mem_bank #(
                .DEPTH     (DEPTH),
                .BUS_WIDTH (BUS_WIDTH),
                .ECC_EN    (ECC_EN)
            ) u_bank (
                .clk        (clk),
                .resetn     (resetn),
                .wr_addr    (bist_active_w && b==0 ? bist_wr_addr_w : sel_addr),
                .wr_data    (bist_active_w && b==0 ? bist_wr_data_w : sel_wdata),
                .wr_strb    (bist_active_w && b==0 ? bist_wr_strb_w : sel_wstrb),
                .wr_en      (bank_wr_en_vec[b]),
                .rd_addr    (bist_active_w && b==0 ? bist_rd_addr_w : sel_addr),
                .rd_en      (bank_rd_en_vec[b]),
                .rd_data    (bank_rd_data[b]),
                .rd_valid   (bank_rd_valid[b]),
                .ecc_single (bank_ecc_s[b]),
                .ecc_double (bank_ecc_d[b])
            );
        end
    endgenerate

    // Wire BIST read data from bank 0
    assign bist_rd_data_w  = bank_rd_data[0];
    assign bist_rd_valid_w = bank_rd_valid[0];

    // -------------------------------------------------------------------------
    // Route read response back to requesting port
    // -------------------------------------------------------------------------
    wire [BUS_WIDTH-1:0] rd_mux;
    assign rd_mux = bank_rd_data[0] | bank_rd_data[1] |
                    bank_rd_data[2] | bank_rd_data[3];

    reg [NUM_PORTS-1:0] grant_pipe; // latch grant to align with read latency
    reg [1:0]           gidx_pipe;

    always @(posedge clk) begin
        if (!resetn) begin
            grant_pipe <= {NUM_PORTS{1'b0}};
            gidx_pipe  <= 2'd0;
            req_ready  <= {NUM_PORTS{1'b0}};
            rsp_valid  <= {NUM_PORTS{1'b0}};
            rsp_rdata  <= {NUM_PORTS*BUS_WIDTH{1'b0}};
        end else begin
            req_ready  <= grant;
            grant_pipe <= grant;
            gidx_pipe  <= grant_idx;

            rsp_valid  <= {NUM_PORTS{1'b0}};
            if (|grant_pipe && |bank_rd_valid) begin
                rsp_rdata[(gidx_pipe+1)*BUS_WIDTH-1 -: BUS_WIDTH] <= rd_mux;
                rsp_valid[gidx_pipe] <= 1'b1;
            end
        end
    end

    // ECC aggregation
    assign ecc_single = |bank_ecc_s;
    assign ecc_double = |bank_ecc_d;

endmodule
