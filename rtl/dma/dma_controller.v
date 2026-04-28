// =============================================================================
//  dma_controller.v – Multi-Burst DMA Engine
//
//  Reads from on-chip BRAM (via memory_controller) and streams data to the
//  input formatter stage through a valid/ready handshake.
//
//  Features:
//    - Supports INT8 and INT4 modes (nibble extraction)
//    - Integrates with dma_prefetch for look-ahead loading
//    - Reports done/error to interrupt controller
//    - Accepts jobs from dma_scheduler
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module dma_controller #(
    parameter integer BRAM_DEPTH = 4096,
    parameter integer BUS_WIDTH  = 64,
    parameter integer DWIDTH     = 8
)(
    input  wire               clk,
    input  wire               resetn,

    // DMA descriptor (from scheduler)
    input  wire [31:0]        src_addr,    // byte address in BRAM
    input  wire [15:0]        byte_len,    // total bytes to transfer
    input  wire [1:0]         mode,        // 00=INT8, 01=INT4
    input  wire               dma_start,   // pulse to begin transfer
    output reg                dma_done,    // pulse when complete
    output reg                dma_error,   // pulse on error

    // Memory read port (to memory_controller)
    output reg  [$clog2(BRAM_DEPTH)-1:0] mem_rd_addr,
    output reg                           mem_rd_en,
    input  wire [BUS_WIDTH-1:0]          mem_rd_data,
    input  wire                          mem_rd_valid,

    // Prefetch interface
    output wire [31:0]        pf_rd_ptr,
    output wire               pf_active,
    input  wire [BUS_WIDTH-1:0] pf_data,
    input  wire               pf_valid,
    output wire               pf_consume,

    // Output stream (to input_formatter / stream_fifo)
    output reg  [DWIDTH-1:0]  out_data,
    output reg                out_valid,
    input  wire               out_ready,

    // Status
    output reg  [31:0]        bytes_transferred
);

    localparam integer ADDR_W    = $clog2(BRAM_DEPTH);
    localparam integer BYTES_PER_WORD = BUS_WIDTH / 8;

    localparam [1:0] DMA_IDLE = 2'd0;
    localparam [1:0] DMA_LOAD = 2'd1;
    localparam [1:0] DMA_EMIT = 2'd2;
    localparam [1:0] DMA_DONE = 2'd3;

    reg [1:0]  state;
    reg [31:0] ptr;           // current byte pointer
    reg [31:0] remaining;     // bytes remaining
    reg        nib_toggle;    // INT4 nibble toggle

    // Loaded word holding register
    reg [BUS_WIDTH-1:0] word_buf;
    reg [$clog2(BYTES_PER_WORD):0] byte_ptr; // byte offset within word_buf

    // Prefetch wires
    assign pf_rd_ptr = ptr;
    assign pf_active = (state == DMA_LOAD || state == DMA_EMIT);
    assign pf_consume = 1'b0; // simplified: prefetch serves next word

    // Nibble sign-extend
    function [7:0] sx4;
        input [3:0] v;
        begin sx4 = {{4{v[3]}}, v}; end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            state             <= DMA_IDLE;
            ptr               <= 32'd0;
            remaining         <= 32'd0;
            nib_toggle        <= 1'b0;
            dma_done          <= 1'b0;
            dma_error         <= 1'b0;
            out_valid         <= 1'b0;
            out_data          <= {DWIDTH{1'b0}};
            mem_rd_en         <= 1'b0;
            bytes_transferred <= 32'd0;
            byte_ptr          <= 0;
            word_buf          <= {BUS_WIDTH{1'b0}};
        end else begin
            dma_done  <= 1'b0;
            dma_error <= 1'b0;
            mem_rd_en <= 1'b0;

            case (state)

                DMA_IDLE: begin
                    out_valid <= 1'b0;
                    if (dma_start) begin
                        // Validate address
                        if (src_addr >= BRAM_DEPTH) begin
                            dma_error <= 1'b1;
                            state     <= DMA_IDLE;
                        end else begin
                            ptr              <= src_addr;
                            remaining        <= {16'd0, byte_len};
                            nib_toggle       <= 1'b0;
                            bytes_transferred <= 32'd0;
                            byte_ptr         <= 0;
                            state            <= DMA_LOAD;
                        end
                    end
                end

                DMA_LOAD: begin
                    if (remaining == 0) begin
                        state <= DMA_DONE;
                    end else begin
                        // Issue read for aligned word
                        mem_rd_addr <= ptr[ADDR_W-1+$clog2(BYTES_PER_WORD) :
                                           $clog2(BYTES_PER_WORD)];
                        mem_rd_en   <= 1'b1;
                        state       <= DMA_EMIT;
                    end
                end

                DMA_EMIT: begin
                    if (mem_rd_valid && !out_valid) begin
                        word_buf <= mem_rd_data;
                        byte_ptr <= 0;
                        state    <= DMA_EMIT;
                    end

                    if (!out_valid || out_ready) begin
                        if (remaining == 0) begin
                            out_valid <= 1'b0;
                            state     <= DMA_DONE;
                        end else if (mem_rd_valid || byte_ptr < BYTES_PER_WORD) begin
                            if (mode == 2'b01) begin
                                // INT4: emit two nibbles per byte
                                if (!nib_toggle) begin
                                    out_data      <= sx4(word_buf[byte_ptr*8 +: 4]);
                                    out_valid     <= 1'b1;
                                    nib_toggle    <= 1'b1;
                                    remaining     <= remaining - 1;
                                    bytes_transferred <= bytes_transferred + 1;
                                end else begin
                                    out_data      <= sx4(word_buf[byte_ptr*8+4 +: 4]);
                                    out_valid     <= 1'b1;
                                    nib_toggle    <= 1'b0;
                                    byte_ptr      <= byte_ptr + 1;
                                    remaining     <= remaining - 1;
                                    bytes_transferred <= bytes_transferred + 1;
                                    if (byte_ptr + 1 >= BYTES_PER_WORD) begin
                                        ptr   <= ptr + BYTES_PER_WORD;
                                        state <= DMA_LOAD;
                                    end
                                end
                            end else begin
                                // INT8: one byte per cycle
                                out_data          <= word_buf[byte_ptr*8 +: 8];
                                out_valid         <= 1'b1;
                                byte_ptr          <= byte_ptr + 1;
                                remaining         <= remaining - 1;
                                bytes_transferred <= bytes_transferred + 1;
                                if (byte_ptr + 1 >= BYTES_PER_WORD) begin
                                    ptr   <= ptr + BYTES_PER_WORD;
                                    state <= DMA_LOAD;
                                end
                            end
                        end
                    end
                end

                DMA_DONE: begin
                    out_valid <= 1'b0;
                    dma_done  <= 1'b1;
                    state     <= DMA_IDLE;
                end

                default: state <= DMA_IDLE;
            endcase
        end
    end

endmodule
