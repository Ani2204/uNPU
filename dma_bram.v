`timescale 1ns / 1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
//
// dma_bram.v  -- dual-port BRAM, concurrent A+B streaming (improvements #2 + #5)
//
// Improvements:
//   - Two independent block-RAM memories: mem_a and mem_b (improvement #2).
//   - A and B streams start simultaneously when triggered; each has its own
//     pointer and counter so they advance at their own pace (improvement #2).
//   - ram_style = "block" for area-efficient large RAMs (improvement #2).
//   - Separate CPU write ports for A and B (bram_we_a/b, wdata_a/b, waddr_a/b).
//   - dma_ctrl [0]=start, [1]=clear, [2]=INT4-mode  (stream_B bit removed).
//
module dma_bram #(
    parameter ELEM_BITS = 8,
    parameter DEPTH     = 4096
)(
    input  wire         clk,
    input  wire         resetn,

    input  wire [31:0]  dma_ctrl,    // [0]=start, [1]=clear, [2]=INT4-mode
    input  wire [31:0]  dma_addr,
    input  wire [31:0]  dma_len,
    output reg  [31:0]  dma_status,

    // CPU preload port for A memory
    input  wire [7:0]   bram_wdata_a,
    input  wire         bram_we_a,
    input  wire [15:0]  bram_waddr_a,

    // CPU preload port for B memory
    input  wire [7:0]   bram_wdata_b,
    input  wire         bram_we_b,
    input  wire [15:0]  bram_waddr_b,

    // A output stream
    output reg          a_valid,
    output reg [7:0]    a_data,
    input  wire         a_ready,

    // B output stream (independent from A)
    output reg          b_valid,
    output reg [7:0]    b_data,
    input  wire         b_ready
);

    // Block RAM for A and B operands
    (* ram_style = "block", keep = "true", dont_touch = "true" *)
    reg [7:0] mem_a [0:DEPTH-1];
    (* ram_style = "block", keep = "true", dont_touch = "true" *)
    reg [7:0] mem_b [0:DEPTH-1];

    // DMA FSM
    localparam IDLE = 2'b00, BUSY = 2'b01, DONE = 2'b10;
    // dma_status encoding: bit[31]=error, bits[1:0]=state code (1=BUSY, 2=DONE)
    localparam STATUS_IDLE  = 32'd0;
    localparam STATUS_BUSY  = 32'd1;
    localparam STATUS_DONE  = 32'd2;
    localparam STATUS_ERR   = 32'h8000_0000;
    reg [1:0] st;

    // Separate A/B stream state
    reg [31:0] ptr_a, ptr_b;           // read pointer per stream
    reg [31:0] remaining_a, remaining_b;
    reg        a_nib_toggle, b_nib_toggle;
    reg        a_done,       b_done;
    reg        mode_int4;
    reg        bram_err_latch;  // sticky error bit; cleared by dma_ctrl[1] or new DMA start

    integer clamp_space;

    function [7:0] sx4;
        input [3:0] v;
        begin sx4 = {{4{v[3]}}, v}; end
    endfunction

    // ---------------------------------------------------------------
    // Main FSM
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            st           <= IDLE;
            dma_status   <= STATUS_IDLE;
            ptr_a        <= 32'd0;
            ptr_b        <= 32'd0;
            remaining_a  <= 32'd0;
            remaining_b  <= 32'd0;
            a_valid      <= 1'b0;
            b_valid      <= 1'b0;
            a_data       <= 8'd0;
            b_data       <= 8'd0;
            a_nib_toggle <= 1'b0;
            b_nib_toggle <= 1'b0;
            a_done       <= 1'b0;
            b_done       <= 1'b0;
            mode_int4    <= 1'b0;
        end else begin

            if (dma_ctrl[1]) begin
                // synchronous clear
                st           <= IDLE;
                dma_status   <= STATUS_IDLE;
                a_valid      <= 1'b0;
                b_valid      <= 1'b0;
                a_nib_toggle <= 1'b0;
                b_nib_toggle <= 1'b0;
                a_done       <= 1'b0;
                b_done       <= 1'b0;
                mode_int4    <= 1'b0;
                remaining_a  <= 32'd0;
                remaining_b  <= 32'd0;
            end else begin
                case (st)
                // -----------------------------------------------
                IDLE: begin
                    a_valid <= 1'b0;
                    b_valid <= 1'b0;
                    a_done  <= 1'b0;
                    b_done  <= 1'b0;
                    if (dma_ctrl[0]) begin
                        mode_int4 <= dma_ctrl[2];
                        if (dma_addr >= DEPTH) begin
                            dma_status <= STATUS_ERR;
                            st         <= DONE;
                        end else begin
                            clamp_space = DEPTH - dma_addr;
                            if (!dma_ctrl[2]) begin
                                remaining_a <= (dma_len > clamp_space) ?
                                               clamp_space : dma_len;
                                remaining_b <= (dma_len > clamp_space) ?
                                               clamp_space : dma_len;
                            end else begin
                                // INT4: two elements per byte → double element count
                                remaining_a <= (dma_len > (clamp_space << 1)) ?
                                               (clamp_space << 1) : dma_len;
                                remaining_b <= (dma_len > (clamp_space << 1)) ?
                                               (clamp_space << 1) : dma_len;
                            end
                            ptr_a        <= dma_addr;
                            ptr_b        <= dma_addr;
                            a_nib_toggle <= 1'b0;
                            b_nib_toggle <= 1'b0;
                            dma_status   <= STATUS_BUSY;
                            st           <= BUSY;
                        end
                    end
                end

                // -----------------------------------------------
                // BUSY: A and B streams run concurrently and independently.
                // Each advances based on its own remaining counter and ready
                // signal, so they can finish at different times.
                BUSY: begin
                    dma_status <= {bram_err_latch, 31'd0} | STATUS_BUSY;

                    // ---- A stream ----
                    if (!a_done) begin
                        if (!a_valid || a_ready) begin
                            if (remaining_a != 0) begin
                                if (mode_int4) begin
                                    if (!a_nib_toggle) begin
                                        a_data       <= sx4(mem_a[ptr_a][3:0]);
                                        a_valid      <= 1'b1;
                                        a_nib_toggle <= 1'b1;
                                        remaining_a  <= remaining_a - 1;
                                    end else begin
                                        a_data       <= sx4(mem_a[ptr_a][7:4]);
                                        a_valid      <= 1'b1;
                                        a_nib_toggle <= 1'b0;
                                        ptr_a        <= ptr_a + 1;
                                        remaining_a  <= remaining_a - 1;
                                    end
                                end else begin
                                    a_data      <= mem_a[ptr_a];
                                    a_valid     <= 1'b1;
                                    ptr_a       <= ptr_a + 1;
                                    remaining_a <= remaining_a - 1;
                                end
                            end else begin
                                a_valid <= 1'b0;
                                a_done  <= 1'b1;
                            end
                        end
                    end

                    // ---- B stream (identical logic, uses mem_b and ptr_b) ----
                    if (!b_done) begin
                        if (!b_valid || b_ready) begin
                            if (remaining_b != 0) begin
                                if (mode_int4) begin
                                    if (!b_nib_toggle) begin
                                        b_data       <= sx4(mem_b[ptr_b][3:0]);
                                        b_valid      <= 1'b1;
                                        b_nib_toggle <= 1'b1;
                                        remaining_b  <= remaining_b - 1;
                                    end else begin
                                        b_data       <= sx4(mem_b[ptr_b][7:4]);
                                        b_valid      <= 1'b1;
                                        b_nib_toggle <= 1'b0;
                                        ptr_b        <= ptr_b + 1;
                                        remaining_b  <= remaining_b - 1;
                                    end
                                end else begin
                                    b_data      <= mem_b[ptr_b];
                                    b_valid     <= 1'b1;
                                    ptr_b       <= ptr_b + 1;
                                    remaining_b <= remaining_b - 1;
                                end
                            end else begin
                                b_valid <= 1'b0;
                                b_done  <= 1'b1;
                            end
                        end
                    end

                    // Done when both streams have finished
                    if (a_done && b_done) begin
                        dma_status <= {bram_err_latch, 31'd0} | STATUS_DONE;
                        st         <= DONE;
                    end
                end

                // -----------------------------------------------
                DONE: begin
                    if (!dma_ctrl[0]) begin
                        st         <= IDLE;
                        dma_status <= STATUS_IDLE;
                    end
                end
                endcase
            end
        end
    end

    // ---------------------------------------------------------------
    // BRAM error latch — single driver.
    // Set when a CPU write is attempted during DMA or to an out-of-range
    // address.  Cleared on reset, dma_ctrl[1] (software clear), or when
    // a new DMA transfer starts (so each run begins with a clean slate).
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            bram_err_latch <= 1'b0;
        end else if (dma_ctrl[1] || (st == IDLE && dma_ctrl[0])) begin
            bram_err_latch <= 1'b0;
        end else if ((bram_we_a && (st == BUSY || bram_waddr_a >= DEPTH)) ||
                     (bram_we_b && (st == BUSY || bram_waddr_b >= DEPTH))) begin
            bram_err_latch <= 1'b1;
        end
    end

    // ---------------------------------------------------------------
    // CPU BRAM preload — A memory (write only; errors tracked above)
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (bram_we_a && st != BUSY && bram_waddr_a < DEPTH)
            mem_a[bram_waddr_a] <= bram_wdata_a;
    end

    // ---------------------------------------------------------------
    // CPU BRAM preload — B memory
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (bram_we_b && st != BUSY && bram_waddr_b < DEPTH)
            mem_b[bram_waddr_b] <= bram_wdata_b;
    end

endmodule
