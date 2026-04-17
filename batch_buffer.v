`timescale 1ns / 1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module batch_buffer #(
    parameter NUM_ELEMS = 256   // logical elements (bytes for INT8, nibbles for INT4)
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     in_valid,
    input  wire [7:0]               in_byte,     // full byte (INT8) or nibble (INT4: use [3:0])
    input  wire [1:0]               mode_sel,    // 00 = INT8, 01 = INT4

    output wire                     in_ready,
    input  wire                     consume,

    // IMPORTANT: loader expects NUM_ELEMS bytes aligned in flat_out
    output reg  [NUM_ELEMS*8-1:0]   flat_out,
    output reg                      batch_ready
);

    // Logical element counter
    localparam CNTW = $clog2(NUM_ELEMS + 1);
    reg [CNTW-1:0] cnt;

    // INT4 packing registers
    reg            half_toggle;    // 0 = waiting low nibble, 1 = waiting high nibble
    reg [3:0]      low_nib;        // holds previous low nibble when packing

    integer byte_idx;

    // Can accept new data while we still have space for logical elements
    assign in_ready = (cnt < NUM_ELEMS);

    always @(posedge clk) begin
        if (!resetn) begin
            flat_out    <= {NUM_ELEMS*8{1'b0}};
            cnt         <= 0;
            batch_ready <= 0;
            half_toggle <= 0;
            low_nib     <= 4'd0;
        end else begin

            // If a full batch is ready, wait for consume
            if (batch_ready) begin
                if (consume) begin
                    // Reset for new batch
                    cnt         <= 0;
                    batch_ready <= 0;
                    half_toggle <= 0;
                    low_nib     <= 4'd0;
                    // No need to clear flat_out (loader overwrites everything)
                end
            end

            // Normal operation
            else if (in_valid && in_ready) begin

                //---------------------------
                // INT4 MODE (mode_sel == 01)
                //---------------------------
                if (mode_sel == 2'b01) begin
                    byte_idx = (cnt >> 1);  // 2 logical elems per output byte

                    if (!half_toggle) begin
                        // Capture low nibble, wait for next nibble
                        low_nib     <= in_byte[3:0];
                        half_toggle <= 1'b1;
                        cnt         <= cnt + 1;

                        // If NUM_ELEMS is odd and this was the very last nibble
                        if (cnt + 1 == NUM_ELEMS) begin
                            // Store {high=0, low=low_nib}
                            flat_out[(byte_idx+1)*8-1 -: 8] <= {4'd0, in_byte[3:0]};
                            batch_ready <= 1'b1;
                        end

                    end else begin
                        // We have low_nib from before; now place high nibble
                        flat_out[(byte_idx+1)*8-1 -: 8] <= {in_byte[3:0], low_nib};

                        half_toggle <= 1'b0;
                        cnt         <= cnt + 1;

                        if (cnt + 1 == NUM_ELEMS)
                            batch_ready <= 1'b1;
                    end
                end

                //---------------------------
                // INT8 MODE (mode_sel != 01)
                //---------------------------
                else begin
                    byte_idx = cnt;

                    // Store 1 byte per logical element
                    flat_out[(byte_idx+1)*8-1 -: 8] <= in_byte;

                    cnt <= cnt + 1;

                    if (cnt + 1 == NUM_ELEMS)
                        batch_ready <= 1'b1;
                end
            end
        end
    end

endmodule
