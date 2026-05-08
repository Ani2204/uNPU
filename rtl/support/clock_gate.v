// =============================================================================
//  clock_gate.v – Integrated Clock Gating Cell
//
//  RTL model of an ICG (Integrated Clock Gate).  The latch-based enable
//  prevents glitches on the output clock.
//
//  Synthesis note: Vivado/Synopsys will recognise this pattern and map it to
//  the device-specific ICG primitive.
// =============================================================================
`timescale 1ns/1ps

(* dont_touch = "yes" *)
module clock_gate (
    input  wire clk_in,
    input  wire enable,
    input  wire test_en,    // test/scan bypass: forces clock on
    output wire clk_out
);

    // Latch enables on the low phase to avoid glitches
    reg en_latch;

    always @(*) begin
        if (!clk_in)                        // transparent when clock low
            en_latch = enable | test_en;
    end

    assign clk_out = clk_in & en_latch;

endmodule
