`timescale 1ns / 1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module byte_fifo #(
    parameter DEPTH = 512
)(
    input  wire       clk,
    input  wire       resetn,
    input  wire       in_valid,
    input  wire [7:0] in_data,
    output wire       in_ready,
    output wire       out_valid,
    output wire [7:0] out_data,
    input  wire       out_ready
);
    localparam AW = $clog2(DEPTH);
    reg [7:0] mem [0:DEPTH-1];
    reg [AW:0] wr_ptr, rd_ptr;
    wire empty = (wr_ptr == rd_ptr);
    wire full  = ((wr_ptr[AW-1:0] == rd_ptr[AW-1:0]) && (wr_ptr[AW] != rd_ptr[AW]));
    assign in_ready = ~full;
    assign out_valid = ~empty;
    assign out_data = mem[rd_ptr[AW-1:0]];

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr <= { (AW+1){1'b0} };
            rd_ptr <= { (AW+1){1'b0} };
        end else begin
            if (in_valid && in_ready) begin
                mem[wr_ptr[AW-1:0]] <= in_data;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (out_valid && out_ready) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end
endmodule
