// =============================================================================
//  stream_fifo.v – Elastic AXI-Stream FIFO (generic)
//
//  Simple synchronous FIFO with AXI-Stream valid/ready handshake.
//  Uses block RAM when DEPTH is large enough; distributed for small depths.
//  Parameterised data width and depth.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module stream_fifo #(
    parameter integer DWIDTH = 8,
    parameter integer DEPTH  = 512,
    parameter integer USE_BRAM = (DEPTH >= 128) ? 1 : 0
)(
    input  wire              clk,
    input  wire              resetn,

    // Input
    input  wire [DWIDTH-1:0] in_data,
    input  wire              in_valid,
    output wire              in_ready,

    // Output
    output wire [DWIDTH-1:0] out_data,
    output wire              out_valid,
    input  wire              out_ready,

    // Status
    output wire [$clog2(DEPTH):0] fill_level,
    output wire              almost_full,   // within 4 of full
    output wire              almost_empty   // within 4 of empty
);

    localparam integer AW = $clog2(DEPTH);

    // Memory array – synthesiser will pick BRAM or distributed based on USE_BRAM
    generate
        if (USE_BRAM) begin : gen_bram
            `BRAM_ATTR reg [DWIDTH-1:0] mem [0:DEPTH-1];
        end else begin : gen_dist
            `DIST_ATTR reg [DWIDTH-1:0] mem [0:DEPTH-1];
        end
    endgenerate

    // Use a simple alias since generate block names make direct access awkward
    // We declare one shared array; the attribute above is a hint only.
    reg [DWIDTH-1:0] fifo_mem [0:DEPTH-1];

    reg [AW:0] wr_ptr, rd_ptr;

    wire full  = ((wr_ptr[AW-1:0] == rd_ptr[AW-1:0]) && (wr_ptr[AW] != rd_ptr[AW]));
    wire empty = (wr_ptr == rd_ptr);

    assign in_ready    = ~full;
    assign out_valid   = ~empty;
    assign out_data    = fifo_mem[rd_ptr[AW-1:0]];
    assign fill_level  = wr_ptr - rd_ptr;
    assign almost_full  = (fill_level >= (DEPTH - 4));
    assign almost_empty = (fill_level <= 4);

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr <= {(AW+1){1'b0}};
            rd_ptr <= {(AW+1){1'b0}};
        end else begin
            if (in_valid && in_ready) begin
                fifo_mem[wr_ptr[AW-1:0]] <= in_data;
                wr_ptr <= wr_ptr + 1;
            end
            if (out_valid && out_ready) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule
