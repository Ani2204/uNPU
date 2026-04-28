// =============================================================================
//  load_balancer.v – PE Allocation & Scheduling
//
//  Receives formatted operand pairs from two stream FIFOs (A and B) and
//  distributes them to PE lanes.  Supports:
//    - Static mode: element i → PE i
//    - Dynamic mode: round-robin search for a free PE
//    - Sparsity pruning: skip if |A| < threshold AND |B| < threshold
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module load_balancer #(
    parameter integer DWIDTH  = 8,
    parameter integer NUM_PES = 256
)(
    input  wire                         clk,
    input  wire                         resetn,

    // Configuration
    input  wire [DWIDTH-1:0]            threshold,     // sparsity pruning threshold
    input  wire                         dyn_sched_en,  // 1 = dynamic round-robin
    input  wire                         start,         // pulse to begin batch
    input  wire [1:0]                   mode_sel,

    // Input streams (A and B operands)
    input  wire [DWIDTH-1:0]            a_data,
    input  wire                         a_valid,
    output wire                         a_ready,

    input  wire [DWIDTH-1:0]            b_data,
    input  wire                         b_valid,
    output wire                         b_ready,

    // PE feedback
    input  wire [NUM_PES-1:0]           pe_busy,

    // Outputs to PE cluster
    output reg  [NUM_PES*DWIDTH-1:0]    A_flat,
    output reg  [NUM_PES*DWIDTH-1:0]    B_flat,
    output reg  [NUM_PES-1:0]           gate_en,
    output reg                          batch_done
);

    localparam integer PTRW = $clog2(NUM_PES);

    // -------------------------------------------------------------------------
    // Absolute value
    // -------------------------------------------------------------------------
    function [DWIDTH-1:0] absd;
        input signed [DWIDTH-1:0] v;
        begin absd = v[DWIDTH-1] ? -v : v; end
    endfunction

    // -------------------------------------------------------------------------
    // Candidate check
    // -------------------------------------------------------------------------
    wire candidate = (absd(a_data) >= threshold) || (absd(b_data) >= threshold);

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_CHECK  = 3'd1;
    localparam [2:0] S_FINDPE = 3'd2;
    localparam [2:0] S_ASSIGN = 3'd3;
    localparam [2:0] S_FIRE   = 3'd4;
    localparam [2:0] S_DRAIN  = 3'd5;

    reg [2:0]    state;
    reg [PTRW-1:0] pe_ptr;
    reg [PTRW-1:0] pe_fire_idx;
    reg [DWIDTH-1:0] a_hold, b_hold;

    // Track how many elements have been consumed
    reg [PTRW:0] elem_count;
    reg          consuming; // 1 = we have accepted a pair from input

    assign a_ready = (state == S_CHECK) && a_valid && b_valid;
    assign b_ready = (state == S_CHECK) && a_valid && b_valid;

    always @(posedge clk) begin
        if (!resetn) begin
            state      <= S_IDLE;
            pe_ptr     <= {PTRW{1'b0}};
            pe_fire_idx<= {PTRW{1'b0}};
            gate_en    <= {NUM_PES{1'b0}};
            A_flat     <= {NUM_PES*DWIDTH{1'b0}};
            B_flat     <= {NUM_PES*DWIDTH{1'b0}};
            batch_done <= 1'b0;
            elem_count <= {(PTRW+1){1'b0}};
            a_hold     <= {DWIDTH{1'b0}};
            b_hold     <= {DWIDTH{1'b0}};
            consuming  <= 1'b0;
        end else begin
            gate_en    <= {NUM_PES{1'b0}};
            batch_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        pe_ptr     <= {PTRW{1'b0}};
                        elem_count <= {(PTRW+1){1'b0}};
                        state      <= S_CHECK;
                    end
                end

                S_CHECK: begin
                    // Wait for a valid pair on both channels
                    if (a_valid && b_valid) begin
                        a_hold <= a_data;
                        b_hold <= b_data;
                        if (candidate) begin
                            if (dyn_sched_en)
                                state <= S_FINDPE;
                            else begin
                                pe_ptr <= elem_count[PTRW-1:0];
                                state  <= S_ASSIGN;
                            end
                        end else begin
                            // prune: skip this element
                            elem_count <= elem_count + 1;
                            if (elem_count + 1 == NUM_PES)
                                state <= S_DRAIN;
                        end
                    end
                end

                S_FINDPE: begin
                    if (!pe_busy[pe_ptr])
                        state <= S_ASSIGN;
                    else
                        pe_ptr <= pe_ptr + 1;
                end

                S_ASSIGN: begin
                    A_flat[(pe_ptr+1)*DWIDTH-1 -: DWIDTH] <= a_hold;
                    B_flat[(pe_ptr+1)*DWIDTH-1 -: DWIDTH] <= b_hold;
                    pe_fire_idx <= pe_ptr;
                    pe_ptr      <= pe_ptr + 1;
                    elem_count  <= elem_count + 1;
                    state       <= S_FIRE;
                end

                S_FIRE: begin
                    gate_en[pe_fire_idx] <= 1'b1;
                    if (elem_count >= NUM_PES)
                        state <= S_DRAIN;
                    else
                        state <= S_CHECK;
                end

                S_DRAIN: begin
                    // Wait for all PEs to finish
                    if (!|pe_busy) begin
                        batch_done <= 1'b1;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
