// =============================================================================
//  input_formatter.v – Format Decoder (INT8 / INT4 / INT2)
//
//  Accepts a byte stream from the DMA FIFO and unpacks it into signed
//  operands at the configured precision.  Outputs are always 8-bit signed.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module input_formatter #(
    parameter integer DWIDTH = 8
)(
    input  wire              clk,
    input  wire              resetn,

    // Precision mode
    input  wire [1:0]        mode_sel,    // 00=INT8, 01=INT4, 10=INT2

    // Input: raw byte stream from DMA
    input  wire [7:0]        raw_data,
    input  wire              raw_valid,
    output wire              raw_ready,

    // Output: formatted 8-bit signed operand
    output reg  [DWIDTH-1:0] fmt_data,
    output reg               fmt_valid,
    input  wire              fmt_ready
);

    // -------------------------------------------------------------------------
    // Internal state for sub-byte modes
    // -------------------------------------------------------------------------
    reg [1:0] sub_idx;   // which sub-element within the byte (INT4: 0-1, INT2: 0-3)
    reg [7:0] byte_hold; // holds the current raw byte

    function [7:0] sx4;
        input [3:0] v;
        begin sx4 = {{4{v[3]}}, v}; end
    endfunction

    function [7:0] sx2;
        input [1:0] v;
        begin sx2 = {{6{v[1]}}, v}; end
    endfunction

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam [1:0] S_FETCH = 2'd0;
    localparam [1:0] S_EMIT  = 2'd1;

    reg [1:0] state;
    reg [1:0] elems_per_byte;

    always @(*) begin
        case (mode_sel)
            2'b01:   elems_per_byte = 2'd2;  // INT4: 2 nibbles per byte
            2'b10:   elems_per_byte = 2'd4;  // INT2: 4 pairs per byte (max 2-bit value)
            default: elems_per_byte = 2'd1;  // INT8: 1 byte per element
        endcase
    end

    assign raw_ready = (state == S_FETCH) && fmt_ready;

    always @(posedge clk) begin
        if (!resetn) begin
            state     <= S_FETCH;
            sub_idx   <= 2'd0;
            byte_hold <= 8'd0;
            fmt_data  <= {DWIDTH{1'b0}};
            fmt_valid <= 1'b0;
        end else begin
            // De-assert valid unless we hold it this cycle
            if (fmt_valid && fmt_ready)
                fmt_valid <= 1'b0;

            case (state)
                S_FETCH: begin
                    if (raw_valid && fmt_ready) begin
                        byte_hold <= raw_data;
                        sub_idx   <= 2'd0;
                        if (mode_sel == 2'b00) begin
                            // INT8: direct emit
                            fmt_data  <= raw_data;
                            fmt_valid <= 1'b1;
                            // stay in S_FETCH for next byte
                        end else begin
                            state <= S_EMIT;
                        end
                    end
                end

                S_EMIT: begin
                    if (!fmt_valid || fmt_ready) begin
                        case (mode_sel)
                            2'b01: begin // INT4
                                fmt_data  <= (sub_idx == 0) ? sx4(byte_hold[3:0])
                                                            : sx4(byte_hold[7:4]);
                                fmt_valid <= 1'b1;
                                if (sub_idx == 1) begin
                                    sub_idx <= 2'd0;
                                    state   <= S_FETCH;
                                end else
                                    sub_idx <= sub_idx + 1;
                            end
                            2'b10: begin // INT2
                                case (sub_idx)
                                    2'd0: fmt_data <= sx2(byte_hold[1:0]);
                                    2'd1: fmt_data <= sx2(byte_hold[3:2]);
                                    2'd2: fmt_data <= sx2(byte_hold[5:4]);
                                    2'd3: fmt_data <= sx2(byte_hold[7:6]);
                                    default: fmt_data <= 8'd0;
                                endcase
                                fmt_valid <= 1'b1;
                                if (sub_idx == 3) begin
                                    sub_idx <= 2'd0;
                                    state   <= S_FETCH;
                                end else
                                    sub_idx <= sub_idx + 1;
                            end
                            default: begin
                                state <= S_FETCH;
                            end
                        endcase
                    end
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
