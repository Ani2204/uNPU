// =============================================================================
//  csr_register_file.v – Systematic CSR Bank
//
//  Address map:
//    0x0000-0x00FF  Configuration registers
//    0x0100-0x01FF  Performance monitor readout
//    0x0200-0x02FF  Status & interrupt registers
//    0x0300-0x7FFF  Data windows (A/B flat operands)
//
//  Designed to be instantiated inside axi_slave_if, which drives wr_* / rd_*
//  directly from the AXI channel state machines.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module csr_register_file #(
    parameter integer DATA_WIDTH = 32,
    parameter integer FLAT_WORDS = 64   // A/B flat width / 32
)(
    input  wire                   clk,
    input  wire                   resetn,

    // Internal write port (from AXI slave)
    input  wire [15:0]            wr_addr,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    input  wire [DATA_WIDTH/8-1:0] wr_strb,
    input  wire                   wr_en,

    // Internal read port
    input  wire [15:0]            rd_addr,
    output reg  [DATA_WIDTH-1:0]  rd_data,

    // -------------------------------------------------------------------------
    // Config registers (0x0000 region) – outputs
    // -------------------------------------------------------------------------
    output reg  [1:0]             mode_sel,
    output reg                    fuse_en,
    output reg  [3:0]             act_sel,
    output reg  signed [31:0]     bias,
    output reg  [3:0]             scale,
    output reg  [3:0]             shift,
    output reg                    gate_en,
    output reg  [7:0]             threshold,
    output reg                    dyn_sched_en,
    output reg                    slice_en,
    output reg  [3:0]             msb_stat_thres,
    output reg                    relu_en,
    output reg                    data_sel,

    // DMA descriptor registers
    output reg  [31:0]            dma_ctrl,
    output reg  [31:0]            dma_src_addr,
    output reg  [31:0]            dma_dst_addr,
    output reg  [31:0]            dma_len,

    // -------------------------------------------------------------------------
    // Perf monitor CSR outputs
    // -------------------------------------------------------------------------
    output reg                    pm_start,     // 1-cycle pulse
    output reg                    pm_clear,     // 1-cycle pulse

    // Perf monitor inputs (from perf_monitor)
    input  wire [63:0]            pm_cyc_count,
    input  wire [63:0]            pm_op_count,
    input  wire [31:0]            pm_active_pe,
    input  wire [31:0]            pm_stall_cyc,
    input  wire [31:0]            pm_latency,

    // -------------------------------------------------------------------------
    // Status / IRQ
    // -------------------------------------------------------------------------
    input  wire [31:0]            npu_status,   // live status word
    input  wire [6:0]             irq_status,
    output reg  [6:0]             irq_enable,
    output reg  [6:0]             irq_clear,    // 1-cycle pulse
    input  wire                   ecc_single,
    input  wire                   ecc_double,
    input  wire                   bist_pass,
    input  wire                   bist_fail,
    output reg                    bist_start,   // 1-cycle pulse

    // DMA status input
    input  wire [31:0]            dma_status,

    // MAC result (read-only)
    input  wire signed [31:0]     mac_result,

    // -------------------------------------------------------------------------
    // A/B flat data windows
    // -------------------------------------------------------------------------
    output reg  [FLAT_WORDS*32-1:0] A_flat,
    output reg  [FLAT_WORDS*32-1:0] B_flat,

    // BRAM preload output (byte-granular write to dma_bram)
    output reg  [31:0]            bram_wdata,
    output reg                    bram_we,
    output reg  [15:0]            bram_waddr
);

    localparam integer AW_BASE = 12'h100;   // A_flat base
    localparam integer BW_BASE = 12'h200;   // B_flat base (inside data window)

    integer bi;

    // -------------------------------------------------------------------------
    // Write decoder
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            mode_sel      <= 2'b00;
            fuse_en       <= 1'b0;
            act_sel       <= 4'd0;
            bias          <= 32'd0;
            scale         <= 4'd1;
            shift         <= 4'd0;
            gate_en       <= 1'b0;
            threshold     <= 8'd0;
            dyn_sched_en  <= 1'b0;
            slice_en      <= 1'b1;
            msb_stat_thres<= 4'd1;
            relu_en       <= 1'b0;
            data_sel      <= 1'b0;
            dma_ctrl      <= 32'd0;
            dma_src_addr  <= 32'd0;
            dma_dst_addr  <= 32'd0;
            dma_len       <= 32'd0;
            irq_enable    <= 7'd0;
            irq_clear     <= 7'd0;
            bist_start    <= 1'b0;
            pm_start      <= 1'b0;
            pm_clear      <= 1'b0;
            bram_we       <= 1'b0;
            bram_wdata    <= 32'd0;
            bram_waddr    <= 16'd0;
            A_flat        <= {FLAT_WORDS*32{1'b0}};
            B_flat        <= {FLAT_WORDS*32{1'b0}};
        end else begin
            // Pulse defaults
            pm_start   <= 1'b0;
            pm_clear   <= 1'b0;
            irq_clear  <= 7'd0;
            bist_start <= 1'b0;
            bram_we    <= 1'b0;

            if (wr_en) begin
                casez (wr_addr)
                    // ---- Config (0x0000 - 0x00FF) ----
                    16'h0000: if (wr_strb[0]) mode_sel      <= wr_data[1:0];
                    16'h0004: if (wr_strb[0]) fuse_en       <= wr_data[0];
                    16'h0008: if (wr_strb[0]) act_sel       <= wr_data[3:0];
                    16'h000C: for (bi=0;bi<4;bi=bi+1)
                                if (wr_strb[bi]) bias[8*bi+:8]  <= wr_data[8*bi+:8];
                    16'h0010: if (wr_strb[0]) scale         <= wr_data[3:0];
                    16'h0014: if (wr_strb[0]) shift         <= wr_data[3:0];
                    16'h0018: if (wr_strb[0]) gate_en       <= wr_data[0];
                    16'h001C: if (wr_strb[0]) threshold     <= wr_data[7:0];
                    16'h0020: if (wr_strb[0]) dyn_sched_en  <= wr_data[0];
                    16'h0024: if (wr_strb[0]) slice_en      <= wr_data[0];
                    16'h0028: if (wr_strb[0]) msb_stat_thres<= wr_data[3:0];
                    16'h002C: if (wr_strb[0]) relu_en       <= wr_data[0];
                    16'h0030: if (wr_strb[0]) data_sel      <= wr_data[0];

                    // ---- Perf monitor (0x0100) ----
                    16'h0100: begin
                        if (wr_strb[0] && wr_data[0]) pm_start <= 1'b1;
                        if (wr_strb[0] && wr_data[1]) pm_clear <= 1'b1;
                    end

                    // ---- Status / IRQ (0x0200 - 0x02FF) ----
                    16'h0208: if (wr_strb[0]) irq_enable <= wr_data[6:0];
                    16'h020C: if (wr_strb[0]) irq_clear  <= wr_data[6:0];
                    16'h0214: if (wr_strb[0] && wr_data[0]) bist_start <= 1'b1;

                    // ---- DMA (0x0240 - 0x0250) ----
                    16'h0240: dma_ctrl     <= wr_data;
                    16'h0244: dma_src_addr <= wr_data;
                    16'h0248: dma_dst_addr <= wr_data;
                    16'h024C: dma_len      <= wr_data;

                    // ---- A_flat / B_flat data window ----
                    default: begin
                        if (wr_addr >= 16'h0300 && wr_addr < (16'h0300 + FLAT_WORDS*4)) begin
                            // A_flat
                            for (bi=0;bi<4;bi=bi+1)
                                if (wr_strb[bi])
                                    A_flat[((wr_addr - 16'h0300)/4)*32 + 8*bi +: 8] <= wr_data[8*bi+:8];
                            // Also emit BRAM byte writes
                            bram_we    <= 1'b1;
                            bram_waddr <= (wr_addr - 16'h0300);
                            bram_wdata <= wr_data;
                        end else if (wr_addr >= 16'h1000 && wr_addr < (16'h1000 + FLAT_WORDS*4)) begin
                            // B_flat
                            for (bi=0;bi<4;bi=bi+1)
                                if (wr_strb[bi])
                                    B_flat[((wr_addr - 16'h1000)/4)*32 + 8*bi +: 8] <= wr_data[8*bi+:8];
                            bram_we    <= 1'b1;
                            bram_waddr <= (wr_addr - 16'h1000);
                            bram_wdata <= wr_data;
                        end
                    end
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read decoder
    // -------------------------------------------------------------------------
    always @(*) begin
        rd_data = 32'hDEAD_BEEF;
        casez (rd_addr)
            16'h0000: rd_data = {30'd0, mode_sel};
            16'h0004: rd_data = {31'd0, fuse_en};
            16'h0008: rd_data = {28'd0, act_sel};
            16'h000C: rd_data = bias;
            16'h0010: rd_data = {28'd0, scale};
            16'h0014: rd_data = {28'd0, shift};
            16'h0018: rd_data = {31'd0, gate_en};
            16'h001C: rd_data = {24'd0, threshold};
            16'h0020: rd_data = {31'd0, dyn_sched_en};
            16'h0024: rd_data = {31'd0, slice_en};
            16'h0028: rd_data = {28'd0, msb_stat_thres};
            16'h002C: rd_data = {31'd0, relu_en};
            16'h0030: rd_data = {31'd0, data_sel};
            16'h0038: rd_data = mac_result;

            // Perf (0x0100)
            16'h0104: rd_data = pm_cyc_count[31:0];
            16'h0108: rd_data = pm_cyc_count[63:32];
            16'h010C: rd_data = pm_op_count[31:0];
            16'h0110: rd_data = pm_op_count[63:32];
            16'h0114: rd_data = pm_active_pe;
            16'h0118: rd_data = pm_latency;
            16'h011C: rd_data = pm_stall_cyc;

            // Status (0x0200)
            16'h0200: rd_data = npu_status;
            16'h0204: rd_data = {25'd0, irq_status};
            16'h0208: rd_data = {25'd0, irq_enable};
            16'h0210: rd_data = {30'd0, ecc_double, ecc_single};
            16'h0214: rd_data = {29'd0, bist_fail, bist_pass, 1'b0};

            // DMA
            16'h0240: rd_data = dma_ctrl;
            16'h0244: rd_data = dma_src_addr;
            16'h0248: rd_data = dma_dst_addr;
            16'h024C: rd_data = dma_len;
            16'h0250: rd_data = dma_status;

            default: rd_data = 32'hDEAD_BEEF;
        endcase
    end

endmodule
