// =============================================================================
//  reset_sync.v – Asynchronous Assert / Synchronous De-assert Reset
//
//  The output resetn_sync is the de-asserted (high) version of resetn,
//  synchronised to clk to prevent metastability on exit from reset.
// =============================================================================
`timescale 1ns/1ps

module reset_sync (
    input  wire clk,
    input  wire resetn_async,   // active-low asynchronous reset input
    output wire resetn_sync     // active-low synchronous reset output
);

    reg [1:0] sync_ff;

    always @(posedge clk or negedge resetn_async) begin
        if (!resetn_async)
            sync_ff <= 2'b00;
        else
            sync_ff <= {sync_ff[0], 1'b1};
    end

    assign resetn_sync = sync_ff[1];

endmodule
