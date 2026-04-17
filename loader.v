`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
//
// loader.v  -- parallel dispatch engine (improvement #1)
//
// Static mode  : all above-threshold elements dispatched to their direct-mapped
//                PEs simultaneously in ONE clock (2 cycles total incl. S_READY).
//                Replaces the original 768-cycle serial FSM.
//
// Dynamic mode : combinatorial priority encoder selects the next pending element
//                and the first free PE (round-robin).  One assignment per clock,
//                no separate S_FINDPE search phase.
//
// INT2 threshold (improvement #7): abs computed from the 2-bit signed value
//                 stored in A_arr[gi][1:0], not the full 8-bit byte.
//
module loader #(
    parameter WIDTH       = 2048,     // NUM_PES * 8
    parameter ELEM_BITS   = 8,
    parameter NUM_PES     = 256
)(
    input  wire                              clk,
    input  wire                              resetn,

    input  wire signed [WIDTH-1:0]           A_flat_in,
    input  wire signed [WIDTH-1:0]           B_flat_in,
    input  wire [ELEM_BITS-1:0]              threshold,
    input  wire                              dyn_sched_en,   // 1=dynamic, 0=static
    input  wire                              csr_gate_en,
    input  wire [NUM_PES-1:0]                pe_busy,
    input  wire [1:0]                        mode_sel,

    output reg  signed [NUM_PES*ELEM_BITS-1:0] A_out,
    output reg  signed [NUM_PES*ELEM_BITS-1:0] B_out,
    output reg  [NUM_PES-1:0]                  loader_gate_en,
    output wire                                batch_done
);

    localparam NUM_ELEMS = WIDTH / ELEM_BITS;
    localparam PTRW      = $clog2(NUM_PES);

    // ---------------------------------------------------------------
    // Unpacked element registers (refreshed every clock from flat in)
    // ---------------------------------------------------------------
    reg signed [ELEM_BITS-1:0] A_arr [0:NUM_ELEMS-1];
    reg signed [ELEM_BITS-1:0] B_arr [0:NUM_ELEMS-1];

    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < NUM_ELEMS; i = i + 1) begin
                A_arr[i] <= 0;
                B_arr[i] <= 0;
            end
        end else if (mode_sel == 2'b01) begin
            // INT4: sign-extend each nibble into an 8-bit slot
            for (i = 0; i < NUM_ELEMS/2; i = i + 1) begin
                A_arr[2*i]     <= {{4{A_flat_in[(i*8)+3]}}, A_flat_in[(i*8)+:4]};
                B_arr[2*i]     <= {{4{B_flat_in[(i*8)+3]}}, B_flat_in[(i*8)+:4]};
                A_arr[2*i + 1] <= {{4{A_flat_in[(i*8)+7]}}, A_flat_in[(i*8+4)+:4]};
                B_arr[2*i + 1] <= {{4{B_flat_in[(i*8)+7]}}, B_flat_in[(i*8+4)+:4]};
            end
        end else begin
            // INT8 (and INT2 — raw bytes; threshold logic handles the 2-bit case)
            for (i = 0; i < NUM_ELEMS; i = i + 1) begin
                A_arr[i] <= A_flat_in[(i+1)*8-1 -: 8];
                B_arr[i] <= B_flat_in[(i+1)*8-1 -: 8];
            end
        end
    end

    // ---------------------------------------------------------------
    // Parallel threshold comparator for all elements (combinatorial).
    // Improvement #7: INT2 uses the 2-bit signed value in [1:0], giving
    // the correct abs rather than treating the byte as INT8.
    // ---------------------------------------------------------------
    wire [NUM_ELEMS-1:0] above_thresh;
    genvar gi;
    generate
        for (gi = 0; gi < NUM_ELEMS; gi = gi + 1) begin : gen_thresh
            wire signed [7:0] a_int2_se = {{6{A_arr[gi][1]}}, A_arr[gi][1:0]};
            wire signed [7:0] b_int2_se = {{6{B_arr[gi][1]}}, B_arr[gi][1:0]};

            wire [ELEM_BITS-1:0] a_abs =
                (mode_sel == 2'b10)
                    ? (a_int2_se[7] ? -a_int2_se : a_int2_se)   // INT2
                    : (A_arr[gi][ELEM_BITS-1] ? -A_arr[gi] : A_arr[gi]); // INT4/INT8

            wire [ELEM_BITS-1:0] b_abs =
                (mode_sel == 2'b10)
                    ? (b_int2_se[7] ? -b_int2_se : b_int2_se)
                    : (B_arr[gi][ELEM_BITS-1] ? -B_arr[gi] : B_arr[gi]);

            assign above_thresh[gi] = (a_abs >= threshold) | (b_abs >= threshold);
        end
    endgenerate

    // ---------------------------------------------------------------
    // Priority functions (synthesise as priority-encoder chains)
    // ---------------------------------------------------------------
    // Lowest-indexed set bit in mask
    function [PTRW-1:0] lowest_set;
        input [NUM_PES-1:0] mask;
        integer ls_k;
        reg [PTRW-1:0] ls_r;
        reg ls_found;
        begin
            ls_r     = {PTRW{1'b0}};
            ls_found = 1'b0;
            for (ls_k = 0; ls_k < NUM_PES; ls_k = ls_k + 1)
                if (!ls_found && mask[ls_k]) begin
                    ls_r     = ls_k[PTRW-1:0];
                    ls_found = 1'b1;
                end
            lowest_set = ls_r;
        end
    endfunction

    // First free PE starting from round-robin pointer (power-of-2 wrap via truncation)
    function [PTRW-1:0] find_free;
        input [NUM_PES-1:0] busy;
        input [PTRW-1:0]    start;
        integer ff_k;
        reg [PTRW-1:0] ff_r;
        reg ff_found;
        begin
            ff_r     = start;
            ff_found = 1'b0;
            for (ff_k = 0; ff_k < NUM_PES; ff_k = ff_k + 1)
                if (!ff_found && !busy[start + ff_k[PTRW-1:0]]) begin
                    ff_r     = start + ff_k[PTRW-1:0];
                    ff_found = 1'b1;
                end
            find_free = ff_r;
        end
    endfunction

    // ---------------------------------------------------------------
    // FSM states
    // ---------------------------------------------------------------
    localparam S_IDLE     = 2'd0;
    localparam S_READY    = 2'd1;   // 1-cycle wait: A_arr/B_arr settle → above_thresh valid
    localparam S_STATIC   = 2'd2;   // 1-cycle parallel dispatch (static mode)
    localparam S_DYN_ITER = 2'd3;   // dynamic: 1 assignment per clock

    reg [1:0]           st;
    reg [NUM_ELEMS-1:0] dyn_pending;
    reg [PTRW-1:0]      pe_rr_ptr;      // round-robin PE start

    // Combinational dynamic-mode helpers
    wire [PTRW-1:0] dyn_elem_idx = lowest_set(dyn_pending);
    wire [PTRW-1:0] dyn_free_pe  = find_free(pe_busy, pe_rr_ptr);
    wire            dyn_any_free = (pe_busy != {NUM_PES{1'b1}});

    assign batch_done = (st == S_IDLE);

    always @(posedge clk) begin
        if (!resetn) begin
            st             <= S_IDLE;
            dyn_pending    <= {NUM_ELEMS{1'b0}};
            pe_rr_ptr      <= {PTRW{1'b0}};
            A_out          <= {NUM_PES*ELEM_BITS{1'b0}};
            B_out          <= {NUM_PES*ELEM_BITS{1'b0}};
            loader_gate_en <= {NUM_PES{1'b0}};
        end else begin
            // gate_en is a single-cycle pulse; deassert by default
            loader_gate_en <= {NUM_PES{1'b0}};

            case (st)
            // ----------------------------------------------------------
            // IDLE: wait for start trigger
            S_IDLE: begin
                if (csr_gate_en)
                    st <= S_READY;
            end

            // ----------------------------------------------------------
            // READY: A_arr / B_arr were registered last cycle; above_thresh
            //        is now valid.  Decide which dispatch path to follow.
            S_READY: begin
                if (!dyn_sched_en)
                    st <= S_STATIC;
                else begin
                    dyn_pending <= above_thresh;
                    pe_rr_ptr   <= {PTRW{1'b0}};
                    st          <= S_DYN_ITER;
                end
            end

            // ----------------------------------------------------------
            // STATIC: dispatch ALL above-threshold elements to their
            //         direct-mapped PEs in ONE single clock cycle.
            //         (elem i → PE i)
            S_STATIC: begin
                for (i = 0; i < NUM_PES; i = i + 1) begin
                    if (above_thresh[i]) begin
                        A_out[(i+1)*ELEM_BITS-1 -: ELEM_BITS] <= A_arr[i];
                        B_out[(i+1)*ELEM_BITS-1 -: ELEM_BITS] <= B_arr[i];
                        loader_gate_en[i] <= 1'b1;
                    end
                end
                st <= S_IDLE;
            end

            // ----------------------------------------------------------
            // DYN_ITER: one element assigned to one free PE per clock.
            //   Combinatorial priority encoder selects:
            //     - lowest-indexed pending candidate (dyn_elem_idx)
            //     - first free PE from round-robin pointer (dyn_free_pe)
            //   If all PEs are busy, stall until one frees.
            S_DYN_ITER: begin
                if (dyn_pending == {NUM_ELEMS{1'b0}}) begin
                    st <= S_IDLE;
                end else if (dyn_any_free) begin
                    A_out[(dyn_free_pe+1)*ELEM_BITS-1 -: ELEM_BITS] <= A_arr[dyn_elem_idx];
                    B_out[(dyn_free_pe+1)*ELEM_BITS-1 -: ELEM_BITS] <= B_arr[dyn_elem_idx];
                    loader_gate_en[dyn_free_pe]              <= 1'b1;
                    dyn_pending[dyn_elem_idx]                <= 1'b0;
                    pe_rr_ptr                                <= dyn_free_pe + 1'b1;
                end
                // else: all PEs busy — remain in S_DYN_ITER until a PE frees
            end

            endcase
        end
    end

endmodule
