// =============================================================================
//  quantizer.v – Post-Processing Quantization (INT8 / INT4 / INT2)
//
//  Input: 32-bit signed accumulator value
//  Output: quantized value (saturated to target precision)
// =============================================================================
`timescale 1ns/1ps

module quantizer #(
    parameter integer ACC_WIDTH = 32
)(
    input  wire              clk,
    input  wire              resetn,

    // Control
    input  wire [1:0]        out_mode,    // 00=INT32 passthru, 01=INT8, 10=INT4, 11=INT2
    input  wire [3:0]        scale,       // multiply before quantize
    input  wire [3:0]        shift,       // arithmetic right shift

    // Input
    input  wire signed [ACC_WIDTH-1:0] acc_in,
    input  wire              acc_valid,

    // Output
    output reg  [ACC_WIDTH-1:0] q_out,
    output reg               q_valid
);

    reg signed [63:0] tmp;
    reg signed [ACC_WIDTH-1:0] sat_val;

    always @(*) begin
        // Scale and shift
        tmp = {{(64-ACC_WIDTH){acc_in[ACC_WIDTH-1]}}, acc_in};
        tmp = tmp * {{60{1'b0}}, scale};
        tmp = tmp >>> shift;

        // Saturate based on out_mode
        case (out_mode)
            2'b01: begin // INT8: clamp to [-128, 127]
                if (tmp > 64'sh7F)        sat_val = 32'sh0000007F;
                else if (tmp < -64'sh80)  sat_val = -32'sh00000080;
                else                      sat_val = {24'd0, tmp[7:0]};
            end
            2'b10: begin // INT4: clamp to [-8, 7]
                if (tmp > 64'sh7)         sat_val = 32'sh00000007;
                else if (tmp < -64'sh8)   sat_val = -32'sh00000008;
                else                      sat_val = {28'd0, tmp[3:0]};
            end
            2'b11: begin // INT2: clamp to [-2, 1]
                if (tmp > 64'sh1)         sat_val = 32'sh00000001;
                else if (tmp < -64'sh2)   sat_val = -32'sh00000002;
                else                      sat_val = {30'd0, tmp[1:0]};
            end
            default: // INT32 passthru with saturation
                if (tmp > 64'sh7FFFFFFF)        sat_val = 32'sh7FFFFFFF;
                else if (tmp < -64'sh80000000)  sat_val = -32'sh80000000;
                else                             sat_val = tmp[31:0];
        endcase
    end

    always @(posedge clk) begin
        if (!resetn) begin
            q_out   <= {ACC_WIDTH{1'b0}};
            q_valid <= 1'b0;
        end else begin
            q_out   <= sat_val;
            q_valid <= acc_valid;
        end
    end

endmodule
