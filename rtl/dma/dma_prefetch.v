// =============================================================================
//  dma_prefetch.v – Look-Ahead Prefetch Engine
//
//  Monitors the current DMA address and prefetches the next cache line
//  (BUS_WIDTH-byte aligned) into a local prefetch buffer.
//  When the main DMA engine requests the next block, it is served from the
//  buffer without incurring memory latency.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module dma_prefetch #(
    parameter integer BRAM_DEPTH   = 4096,
    parameter integer BUS_WIDTH    = 64,
    parameter integer PREFETCH_LINES = 2    // number of lines to prefetch ahead
)(
    input  wire                     clk,
    input  wire                     resetn,

    // Current DMA read pointer (byte address)
    input  wire [31:0]              dma_rd_ptr,
    input  wire                     dma_active,

    // Memory read port (to memory controller)
    output reg  [$clog2(BRAM_DEPTH)-1:0] pf_rd_addr,
    output reg                      pf_rd_en,
    input  wire [BUS_WIDTH-1:0]     pf_rd_data,
    input  wire                     pf_rd_valid,

    // Prefetch buffer output (to DMA controller)
    output reg  [BUS_WIDTH-1:0]     pf_data,
    output reg                      pf_valid,   // data in buffer is valid
    input  wire                     pf_consume  // DMA consumed the buffer
);

    localparam integer ADDR_W = $clog2(BRAM_DEPTH);
    localparam integer LINE_BYTES = BUS_WIDTH / 8;

    // Prefetch address: current ptr + PREFETCH_LINES lines ahead
    wire [31:0] pf_target_addr = dma_rd_ptr + (PREFETCH_LINES * LINE_BYTES);
    wire [ADDR_W-1:0] pf_mem_addr = pf_target_addr[ADDR_W + $clog2(LINE_BYTES) - 1 :
                                                    $clog2(LINE_BYTES)];

    localparam [1:0] PF_IDLE    = 2'd0;
    localparam [1:0] PF_FETCH   = 2'd1;
    localparam [1:0] PF_VALID   = 2'd2;

    reg [1:0] state;
    reg [ADDR_W-1:0] fetched_addr;

    always @(posedge clk) begin
        if (!resetn) begin
            state        <= PF_IDLE;
            pf_rd_en     <= 1'b0;
            pf_valid     <= 1'b0;
            pf_data      <= {BUS_WIDTH{1'b0}};
            fetched_addr <= {ADDR_W{1'b0}};
        end else begin
            pf_rd_en <= 1'b0;

            case (state)
                PF_IDLE: begin
                    if (dma_active) begin
                        // Issue prefetch read
                        pf_rd_addr   <= pf_mem_addr;
                        pf_rd_en     <= 1'b1;
                        fetched_addr <= pf_mem_addr;
                        state        <= PF_FETCH;
                    end
                end

                PF_FETCH: begin
                    if (pf_rd_valid) begin
                        pf_data  <= pf_rd_data;
                        pf_valid <= 1'b1;
                        state    <= PF_VALID;
                    end
                end

                PF_VALID: begin
                    if (pf_consume) begin
                        pf_valid <= 1'b0;
                        state    <= PF_IDLE;
                    end else if (!dma_active) begin
                        pf_valid <= 1'b0;
                        state    <= PF_IDLE;
                    end
                end

                default: state <= PF_IDLE;
            endcase
        end
    end

endmodule
