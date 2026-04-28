// =============================================================================
//  output_formatter.v – Post-Processing Stage
//
//  Applies bias, scale, shift, and activation function to the cluster sum.
//  Outputs go to the quantizer and then to the output FIFO.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module output_formatter #(
    parameter integer ACC_WIDTH = 32
)(
    input  wire                      clk,
    input  wire                      resetn,

    // Control
    input  wire signed [ACC_WIDTH-1:0] bias,
    input  wire [3:0]                scale,
    input  wire [3:0]                shift,
    input  wire                      relu_en,
    input  wire [3:0]                act_sel,    // 0=none, 1=relu, 2=sigmoid_approx
    input  wire                      fuse_en,    // accumulate across batches

    // Input from reduction tree
    input  wire signed [ACC_WIDTH-1:0] sum_in,
    input  wire                      sum_valid,

    // Output to quantizer
    output reg  signed [ACC_WIDTH-1:0] fmt_out,
    output reg                       fmt_valid
);

    reg signed [ACC_WIDTH-1:0] fuse_acc;

    // Activation function
    function automatic signed [ACC_WIDTH-1:0] apply_act;
        input signed [ACC_WIDTH-1:0] v;
        input [3:0] sel;
        input relu_en_in;
        begin
            if (relu_en_in && v[ACC_WIDTH-1])
                apply_act = {ACC_WIDTH{1'b0}};   // ReLU
            else begin
                case (sel)
                    4'd0: apply_act = v;           // identity
                    4'd1: apply_act = v[ACC_WIDTH-1] ? {ACC_WIDTH{1'b0}} : v; // ReLU
                    4'd2: begin                    // Leaky ReLU approx (v/4 if negative)
                        if (v[ACC_WIDTH-1])
                            apply_act = v >>> 2;
                        else
                            apply_act = v;
                    end
                    default: apply_act = v;
                endcase
            end
        end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            fmt_out   <= {ACC_WIDTH{1'b0}};
            fmt_valid <= 1'b0;
            fuse_acc  <= {ACC_WIDTH{1'b0}};
        end else begin
            fmt_valid <= 1'b0;

            if (sum_valid) begin
                // Bias add (with fuse accumulation)
                reg signed [ACC_WIDTH-1:0] biased;
                biased = fuse_en ? (fuse_acc + sum_in + bias) : (sum_in + bias);

                // Activation
                reg signed [ACC_WIDTH-1:0] activated;
                activated = apply_act(biased, act_sel, relu_en);

                // Update fuse accumulator
                fuse_acc <= fuse_en ? activated : {ACC_WIDTH{1'b0}};

                // Output
                fmt_out   <= activated;
                fmt_valid <= 1'b1;
            end
        end
    end

endmodule
