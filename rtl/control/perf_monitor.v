// =============================================================================
//  perf_monitor.v – Performance Counters
//
//  Tracks: cycle count, operation count, active PE sum, pipeline stalls,
//          and last-batch latency.  Controlled via start/clear from CSR.
// =============================================================================
`timescale 1ns/1ps

module perf_monitor #(
    parameter integer NUM_PES = 256
)(
    input  wire               clk,
    input  wire               resetn,

    // Control
    input  wire               pm_start,    // 1-cycle pulse to start counting
    input  wire               pm_clear,    // 1-cycle pulse to clear all counters
    input  wire               pm_stop,     // batch done → latch latency

    // Inputs from datapath
    input  wire [NUM_PES-1:0] gate_en,     // active operations this cycle
    input  wire [NUM_PES-1:0] pe_busy,     // busy PEs this cycle
    input  wire [3:0]         pipe_stall,  // pipeline stall bits

    // Readable counters (to CSR)
    output reg  [63:0]        cyc_count,
    output reg  [63:0]        op_count,
    output reg  [31:0]        active_pe_sum,
    output reg  [31:0]        stall_cycles,
    output reg  [31:0]        last_latency
);

    // Popcount helper
    function [15:0] popcount;
        input [NUM_PES-1:0] v;
        integer k;
        reg [15:0] c;
        begin
            c = 0;
            for (k = 0; k < NUM_PES; k = k + 1)
                c = c + v[k];
            popcount = c;
        end
    endfunction

    reg armed;

    always @(posedge clk) begin
        if (!resetn) begin
            armed          <= 1'b0;
            cyc_count      <= 64'd0;
            op_count       <= 64'd0;
            active_pe_sum  <= 32'd0;
            stall_cycles   <= 32'd0;
            last_latency   <= 32'd0;
        end else begin
            if (pm_clear) begin
                armed         <= 1'b0;
                cyc_count     <= 64'd0;
                op_count      <= 64'd0;
                active_pe_sum <= 32'd0;
                stall_cycles  <= 32'd0;
            end

            if (pm_start)
                armed <= 1'b1;

            if (armed) begin
                cyc_count     <= cyc_count + 1;
                op_count      <= op_count + {48'd0, popcount(gate_en)};
                active_pe_sum <= active_pe_sum + {16'd0, popcount(pe_busy)};
                stall_cycles  <= stall_cycles + {{28{1'b0}}, |pipe_stall};
            end

            if (pm_stop && armed) begin
                last_latency <= cyc_count[31:0];
                armed        <= 1'b0;
            end
        end
    end

endmodule
