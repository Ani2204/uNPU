`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module loader #(
    parameter WIDTH       = 2048,     // NUM_PES * 8
    parameter ELEM_BITS   = 8,
    parameter NUM_PES     = 256
)(
    input  wire                         clk,
    input  wire                         resetn,

    input  wire signed [WIDTH-1:0]      A_flat_in,
    input  wire signed [WIDTH-1:0]      B_flat_in,
    input  wire [ELEM_BITS-1:0]         threshold,
    input  wire                         dyn_sched_en,   // 1: dynamic, 0: static
    input  wire                         csr_gate_en,
    input  wire [NUM_PES-1:0]           pe_busy,
    input  wire [1:0]                   mode_sel,

    output reg  signed [NUM_PES*ELEM_BITS-1:0] A_out,
    output reg  signed [NUM_PES*ELEM_BITS-1:0] B_out,
    output reg  [NUM_PES-1:0]                  loader_gate_en,
    output wire                                batch_done
);

    localparam NUM_ELEMS = WIDTH / ELEM_BITS;
    localparam PTRW  = $clog2(NUM_PES);
    localparam EIDXW = $clog2(NUM_ELEMS);

    //------------------------------------------------------------
    // Internal storage
    //------------------------------------------------------------
    reg signed [ELEM_BITS-1:0] A_arr [0:NUM_ELEMS-1];
    reg signed [ELEM_BITS-1:0] B_arr [0:NUM_ELEMS-1];

    reg [NUM_ELEMS-1:0] pending_mask;
    reg [EIDXW-1:0]     elem_idx;
    reg [PTRW-1:0]      pe_ptr;

    //------------------------------------------------------------
    // Unpack INT8 or INT4
    //------------------------------------------------------------
    integer i;

    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < NUM_ELEMS; i = i + 1) begin
                A_arr[i] <= 0;
                B_arr[i] <= 0;
            end
        end else begin
            if (mode_sel == 2'b01) begin
                // INT4 mode (2 elems per 8-bit)
                for (i = 0; i < NUM_ELEMS/2; i = i + 1) begin
                    // low nibble
                    A_arr[2*i]     <= {{4{A_flat_in[(i*8)+3]}}, A_flat_in[(i*8)+:4]};
                    B_arr[2*i]     <= {{4{B_flat_in[(i*8)+3]}}, B_flat_in[(i*8)+:4]};
                    // high nibble
                    A_arr[2*i + 1] <= {{4{A_flat_in[(i*8)+7]}}, A_flat_in[(i*8+4)+:4]};
                    B_arr[2*i + 1] <= {{4{B_flat_in[(i*8)+7]}}, B_flat_in[(i*8+4)+:4]};
                end
            end else begin
                // INT8 mode
                for (i = 0; i < NUM_ELEMS; i = i + 1) begin
                    A_arr[i] <= A_flat_in[(i+1)*8-1 -: 8];
                    B_arr[i] <= B_flat_in[(i+1)*8-1 -: 8];
                end
            end
        end
    end

    //------------------------------------------------------------
    // abs helpers
    //------------------------------------------------------------
    function [7:0] abs8;
        input signed [7:0] v;
        begin abs8 = (v[7] ? -v : v); end
    endfunction

    function [7:0] abs4;
        input signed [7:0] v4;
        begin abs4 = (v4[7] ? -v4 : v4); end
    endfunction

    //------------------------------------------------------------
    // FSM
    //------------------------------------------------------------
    localparam S_IDLE   = 3'd0;
    localparam S_CHECK  = 3'd1;
    localparam S_FINDPE = 3'd2;
    localparam S_ASSIGN = 3'd3;
    localparam S_FIRE   = 3'd4;   // pulse gate one cycle after data
    localparam S_NEXT   = 3'd5;

    reg [2:0] st;

    // Remember which PE we just assigned data to
    reg [PTRW-1:0] pe_fire_idx;

    //------------------------------------------------------------
    // Combinational candidate check
    //------------------------------------------------------------
    wire candidate_now =
        (mode_sel == 2'b01)
            ? (abs4(A_arr[elem_idx]) >= threshold ||
               abs4(B_arr[elem_idx]) >= threshold)
            : (abs8(A_arr[elem_idx]) >= threshold ||
               abs8(B_arr[elem_idx]) >= threshold);

    //------------------------------------------------------------
    // batch_done output
    //------------------------------------------------------------
    assign batch_done = (pending_mask == 0);

    //------------------------------------------------------------
    // Main FSM
    //------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            st             <= S_IDLE;
            pending_mask   <= {NUM_ELEMS{1'b0}};
            elem_idx       <= {EIDXW{1'b0}};
            pe_ptr         <= {PTRW{1'b0}};
            pe_fire_idx    <= {PTRW{1'b0}};

            A_out          <= {NUM_PES*ELEM_BITS{1'b0}};
            B_out          <= {NUM_PES*ELEM_BITS{1'b0}};
            loader_gate_en <= {NUM_PES{1'b0}};
        end else begin

            // default: no gates unless we explicitly fire
            loader_gate_en <= {NUM_PES{1'b0}};

            case (st)
            //------------------------------------------------------
            S_IDLE: begin
                if (csr_gate_en) begin
                    pending_mask <= {NUM_ELEMS{1'b1}};
                    elem_idx     <= {EIDXW{1'b0}};
                    pe_ptr       <= {PTRW{1'b0}};
                    st           <= S_CHECK;
                end
            end

            //------------------------------------------------------
            S_CHECK: begin
                // If element already handled, skip
                if (!pending_mask[elem_idx]) begin
                    st <= S_NEXT;
                end else begin
                    // Decide if this element is worth sending
                    if (candidate_now) begin
                        if (dyn_sched_en) begin
                            // DYNAMIC: search for a free PE
                            st <= S_FINDPE;
                        end else begin
                            // STATIC: direct mapping elem_idx -> PE index
                            pe_ptr <= elem_idx[PTRW-1:0];
                            st     <= S_ASSIGN;
                        end
                    end else begin
                        // prune as "too small"
                        pending_mask[elem_idx] <= 1'b0;
                        st <= S_NEXT;
                    end
                end
            end

            //------------------------------------------------------
            // S_FINDPE: only when dyn_sched_en = 1
            //------------------------------------------------------
            S_FINDPE: begin
                // simple round-robin search
                if (!pe_busy[pe_ptr]) begin
                    st <= S_ASSIGN;
                end else begin
                    pe_ptr <= pe_ptr + 1'b1;
                    st     <= S_FINDPE;
                end
            end

            //------------------------------------------------------
            // ASSIGN only writes data, no gate yet
            //------------------------------------------------------
            S_ASSIGN: begin
                A_out[(pe_ptr+1)*ELEM_BITS-1 -: ELEM_BITS] <= A_arr[elem_idx];
                B_out[(pe_ptr+1)*ELEM_BITS-1 -: ELEM_BITS] <= B_arr[elem_idx];

                pending_mask[elem_idx] <= 1'b0;
                pe_fire_idx            <= pe_ptr;        // remember which PE to fire
                pe_ptr                 <= pe_ptr + 1'b1;

                st <= S_FIRE;
            end

            //------------------------------------------------------
            // FIRE gate one cycle later (data now stable)
            //------------------------------------------------------
            S_FIRE: begin
                loader_gate_en[pe_fire_idx] <= 1'b1;
                st <= S_NEXT;
            end

            //------------------------------------------------------
            S_NEXT: begin
                if (elem_idx == NUM_ELEMS - 1) begin
                    if (pending_mask == 0)
                        st <= S_IDLE;
                    else begin
                        elem_idx <= {EIDXW{1'b0}};
                        st       <= S_CHECK;
                    end
                end else begin
                    elem_idx <= elem_idx + 1'b1;
                    st       <= S_CHECK;
                end
            end

            endcase
        end
    end

endmodule
