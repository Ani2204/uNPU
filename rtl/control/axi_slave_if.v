// =============================================================================
//  axi_slave_if.v – AXI4-Full Slave Interface with CSR mapping
//
//  Supports:
//    - AXI4-Full: burst transactions (AWLEN, AWSIZE, AWBURST)
//    - Write strobes (WSTRB)
//    - Proper AW/W/B and AR/R channel handshaking
//    - ID passthrough (AWID/ARID → BID/RID)
//
//  The decoded address and data are forwarded to csr_register_file via a
//  simple single-cycle write/read port.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module axi_slave_if #(
    parameter integer ADDR_WIDTH = 16,
    parameter integer DATA_WIDTH = 32,
    parameter integer ID_WIDTH   = 4
)(
    input  wire                    clk,
    input  wire                    resetn,

    // -------------------------------------------------------------------------
    // AXI4 Write Address Channel
    // -------------------------------------------------------------------------
    input  wire [ID_WIDTH-1:0]     s_awid,
    input  wire [ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [7:0]              s_awlen,     // burst length - 1
    input  wire [2:0]              s_awsize,    // log2(bytes per beat)
    input  wire [1:0]              s_awburst,   // FIXED/INCR/WRAP
    input  wire                    s_awvalid,
    output reg                     s_awready,

    // AXI4 Write Data Channel
    input  wire [DATA_WIDTH-1:0]   s_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                    s_wlast,
    input  wire                    s_wvalid,
    output reg                     s_wready,

    // AXI4 Write Response Channel
    output reg  [ID_WIDTH-1:0]     s_bid,
    output reg  [1:0]              s_bresp,
    output reg                     s_bvalid,
    input  wire                    s_bready,

    // -------------------------------------------------------------------------
    // AXI4 Read Address Channel
    // -------------------------------------------------------------------------
    input  wire [ID_WIDTH-1:0]     s_arid,
    input  wire [ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]              s_arlen,
    input  wire [2:0]              s_arsize,
    input  wire [1:0]              s_arburst,
    input  wire                    s_arvalid,
    output reg                     s_arready,

    // AXI4 Read Data Channel
    output reg  [ID_WIDTH-1:0]     s_rid,
    output reg  [DATA_WIDTH-1:0]   s_rdata,
    output reg  [1:0]              s_rresp,
    output reg                     s_rlast,
    output reg                     s_rvalid,
    input  wire                    s_rready,

    // -------------------------------------------------------------------------
    // CSR register file interface
    // -------------------------------------------------------------------------
    output reg  [15:0]             csr_wr_addr,
    output reg  [DATA_WIDTH-1:0]   csr_wr_data,
    output reg  [DATA_WIDTH/8-1:0] csr_wr_strb,
    output reg                     csr_wr_en,

    output reg  [15:0]             csr_rd_addr,
    input  wire [DATA_WIDTH-1:0]   csr_rd_data
);

    // -------------------------------------------------------------------------
    // Write address channel: latch AW beat
    // -------------------------------------------------------------------------
    reg                  aw_pending;
    reg [ID_WIDTH-1:0]   aw_id_r;
    reg [ADDR_WIDTH-1:0] aw_addr_r;
    reg [7:0]            aw_len_r;
    reg [2:0]            aw_size_r;
    reg [1:0]            aw_burst_r;
    reg [7:0]            aw_beat;   // current beat counter (0..awlen)

    // Current beat address (incremented for INCR burst)
    reg [ADDR_WIDTH-1:0] cur_wr_addr;

    always @(posedge clk) begin
        if (!resetn) begin
            aw_pending <= 1'b0;
            s_awready  <= 1'b0;
        end else begin
            s_awready <= 1'b0;
            if (!aw_pending && s_awvalid) begin
                aw_id_r    <= s_awid;
                aw_addr_r  <= s_awaddr;
                aw_len_r   <= s_awlen;
                aw_size_r  <= s_awsize;
                aw_burst_r <= s_awburst;
                aw_beat    <= 8'd0;
                cur_wr_addr<= s_awaddr;
                aw_pending <= 1'b1;
                s_awready  <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Write data channel + CSR write dispatch
    // -------------------------------------------------------------------------
    reg resp_pending;

    always @(posedge clk) begin
        if (!resetn) begin
            s_wready     <= 1'b0;
            csr_wr_en    <= 1'b0;
            csr_wr_addr  <= 16'd0;
            csr_wr_data  <= {DATA_WIDTH{1'b0}};
            csr_wr_strb  <= {DATA_WIDTH/8{1'b0}};
            resp_pending <= 1'b0;
            aw_beat      <= 8'd0;
        end else begin
            s_wready  <= aw_pending; // ready whenever we have a latched address
            csr_wr_en <= 1'b0;

            if (aw_pending && s_wvalid && s_wready) begin
                // Dispatch CSR write
                csr_wr_addr  <= cur_wr_addr[15:0];
                csr_wr_data  <= s_wdata;
                csr_wr_strb  <= s_wstrb;
                csr_wr_en    <= 1'b1;

                // Increment address for INCR burst
                if (aw_burst_r == 2'b01)
                    cur_wr_addr <= cur_wr_addr + (1 << aw_size_r);

                if (s_wlast || (aw_beat == aw_len_r)) begin
                    aw_pending   <= 1'b0;
                    resp_pending <= 1'b1;
                    aw_beat      <= 8'd0;
                end else
                    aw_beat <= aw_beat + 1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Write response channel
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            s_bvalid <= 1'b0;
            s_bresp  <= 2'b00;
            s_bid    <= {ID_WIDTH{1'b0}};
        end else begin
            if (resp_pending && !s_bvalid) begin
                s_bid    <= aw_id_r;
                s_bresp  <= `AXI_OKAY;
                s_bvalid <= 1'b1;
                resp_pending <= 1'b0;
            end else if (s_bvalid && s_bready) begin
                s_bvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read address channel + CSR read dispatch
    // -------------------------------------------------------------------------
    reg                  ar_pending;
    reg [ID_WIDTH-1:0]   ar_id_r;
    reg [ADDR_WIDTH-1:0] ar_addr_r;
    reg [7:0]            ar_len_r;
    reg [2:0]            ar_size_r;
    reg [1:0]            ar_burst_r;
    reg [7:0]            ar_beat;
    reg [ADDR_WIDTH-1:0] cur_rd_addr;

    always @(posedge clk) begin
        if (!resetn) begin
            ar_pending <= 1'b0;
            s_arready  <= 1'b0;
        end else begin
            s_arready <= 1'b0;
            if (!ar_pending && s_arvalid) begin
                ar_id_r    <= s_arid;
                ar_addr_r  <= s_araddr;
                ar_len_r   <= s_arlen;
                ar_size_r  <= s_arsize;
                ar_burst_r <= s_arburst;
                ar_beat    <= 8'd0;
                cur_rd_addr<= s_araddr;
                ar_pending <= 1'b1;
                s_arready  <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read data channel: issue one beat per cycle while ar_pending
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            s_rvalid    <= 1'b0;
            s_rdata     <= {DATA_WIDTH{1'b0}};
            s_rresp     <= 2'b00;
            s_rlast     <= 1'b0;
            s_rid       <= {ID_WIDTH{1'b0}};
            csr_rd_addr <= 16'd0;
        end else begin
            if (ar_pending && (!s_rvalid || s_rready)) begin
                csr_rd_addr <= cur_rd_addr[15:0];
                s_rid       <= ar_id_r;
                s_rdata     <= csr_rd_data;
                s_rresp     <= `AXI_OKAY;
                s_rlast     <= (ar_beat == ar_len_r);
                s_rvalid    <= 1'b1;

                // Increment address for INCR burst
                if (ar_burst_r == 2'b01)
                    cur_rd_addr <= cur_rd_addr + (1 << ar_size_r);

                if (ar_beat == ar_len_r) begin
                    ar_pending <= 1'b0;
                    ar_beat    <= 8'd0;
                end else
                    ar_beat <= ar_beat + 1;
            end else if (s_rvalid && s_rready) begin
                s_rvalid <= 1'b0;
            end
        end
    end

endmodule
