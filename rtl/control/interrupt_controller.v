// =============================================================================
//  interrupt_controller.v – IRQ Management
//
//  Collects interrupt sources, applies enable masking, and drives a single
//  irq_out line to the system interrupt controller.
//
//  IRQ sources (bit positions):
//    [0] DMA done
//    [1] Compute done (batch_done)
//    [2] ECC single-bit error (correctable)
//    [3] ECC double-bit error (uncorrectable)
//    [4] Watchdog timeout
//    [5] BIST done
//    [6] BIST fail
// =============================================================================
`timescale 1ns/1ps

module interrupt_controller (
    input  wire        clk,
    input  wire        resetn,

    // IRQ sources (level or pulse; this block edge-detects pulses)
    input  wire [6:0]  irq_src,       // raw IRQ sources

    // CSR interface
    input  wire [6:0]  irq_enable,    // write: enable mask
    input  wire [6:0]  irq_clear,     // write: write-1-to-clear pending
    output reg  [6:0]  irq_status,    // read: pending IRQs (sticky)

    // Output to CPU
    output wire        irq_out        // asserted when any unmasked IRQ pending
);

    reg [6:0] src_prev;

    // Edge detect (rising edge of each source)
    wire [6:0] src_edge = irq_src & ~src_prev;

    always @(posedge clk) begin
        if (!resetn) begin
            irq_status <= 7'd0;
            src_prev   <= 7'd0;
        end else begin
            src_prev <= irq_src;

            // Set on rising edge of source
            irq_status <= (irq_status | src_edge) & ~irq_clear;
        end
    end

    assign irq_out = |(irq_status & irq_enable);

endmodule
