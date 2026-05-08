// =============================================================================
//  output_fifo.v – Result Buffering FIFO
//
//  Buffers quantized output results before they are read back by the CPU
//  via the CSR/AXI interface.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module output_fifo #(
    parameter integer DWIDTH = 32,
    parameter integer DEPTH  = 64
)(
    input  wire              clk,
    input  wire              resetn,

    // Input from quantizer
    input  wire [DWIDTH-1:0] in_data,
    input  wire              in_valid,
    output wire              in_ready,

    // Output to CPU/AXI
    output wire [DWIDTH-1:0] out_data,
    output wire              out_valid,
    input  wire              out_ready,

    // Status
    output wire [$clog2(DEPTH):0] fill_level,
    output wire              overflow   // wrote to full FIFO (error)
);

    localparam integer AW = $clog2(DEPTH);

    `BRAM_ATTR reg [DWIDTH-1:0] mem [0:DEPTH-1];

    reg [AW:0] wr_ptr, rd_ptr;

    wire full  = ((wr_ptr[AW-1:0] == rd_ptr[AW-1:0]) && (wr_ptr[AW] != rd_ptr[AW]));
    wire empty = (wr_ptr == rd_ptr);

    assign in_ready   = ~full;
    assign out_valid  = ~empty;
    assign out_data   = mem[rd_ptr[AW-1:0]];
    assign fill_level = wr_ptr - rd_ptr;
    assign overflow   = in_valid && full;

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr <= {(AW+1){1'b0}};
            rd_ptr <= {(AW+1){1'b0}};
        end else begin
            if (in_valid && in_ready) begin
                mem[wr_ptr[AW-1:0]] <= in_data;
                wr_ptr <= wr_ptr + 1;
            end
            if (out_valid && out_ready) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule
