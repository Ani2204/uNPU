// =============================================================================
//  pipeline_control.v – 4-Stage Pipeline Hazard Detection & Flow Control
//
//  Manages valid/ready handshakes between:
//    Stage 0: DMA + Input Formatter
//    Stage 1: Compute Clusters
//    Stage 2: Reduction Tree
//    Stage 3: Output Formatter
//
//  Provides stall signals when a downstream stage is not ready.
//  Also provides a flush mechanism for error recovery.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module pipeline_control #(
    parameter integer NUM_STAGES = 4
)(
    input  wire                   clk,
    input  wire                   resetn,

    // Per-stage valid/ready from datapaths
    input  wire [NUM_STAGES-1:0]  stage_valid_in,   // stage produced data
    output reg  [NUM_STAGES-1:0]  stage_ready_out,  // pipeline accepts data

    // Stall outputs (to each stage)
    output reg  [NUM_STAGES-1:0]  stall,

    // Flush (from error / watchdog)
    input  wire                   flush,

    // Pipeline occupancy (for perf counters)
    output reg  [NUM_STAGES-1:0]  pipe_occupied
);

    // -------------------------------------------------------------------------
    // Stage occupancy tracking
    // -------------------------------------------------------------------------
    reg [NUM_STAGES-1:0] occupied;

    // A stage is ready to accept if it is empty OR its downstream is ready
    // Simple elastic pipeline: stall if valid and downstream not ready
    // Stage N-1 (last) always ready (sinks to output FIFO)

    integer s;

    always @(posedge clk) begin
        if (!resetn || flush) begin
            occupied        <= {NUM_STAGES{1'b0}};
            stage_ready_out <= {NUM_STAGES{1'b1}};
            stall           <= {NUM_STAGES{1'b0}};
            pipe_occupied   <= {NUM_STAGES{1'b0}};
        end else begin
            // Last stage always accepts (output FIFO handles backpressure externally)
            stage_ready_out[NUM_STAGES-1] <= 1'b1;

            for (s = 0; s < NUM_STAGES-1; s = s + 1) begin
                // Stage s can accept if stage s+1 is ready
                stage_ready_out[s] <= stage_ready_out[s+1] | ~occupied[s+1];
            end

            // Update occupancy
            for (s = 0; s < NUM_STAGES; s = s + 1) begin
                if (stage_valid_in[s] && stage_ready_out[s])
                    occupied[s] <= 1'b1;
                else if (s < NUM_STAGES-1 && occupied[s] && stage_ready_out[s+1])
                    occupied[s] <= 1'b0;
                else if (s == NUM_STAGES-1 && occupied[s])
                    occupied[s] <= 1'b0; // last stage drains each cycle
            end

            // Stall = valid input but stage not ready
            stall <= stage_valid_in & ~stage_ready_out;

            pipe_occupied <= occupied;
        end
    end

endmodule
