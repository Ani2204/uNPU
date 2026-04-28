// =============================================================================
//  instr_sequencer.v – Micro-Instruction ISA Decoder
//
//  ISA opcodes (3-bit):
//    000 NOP     – no operation
//    001 LOAD    – issue DMA load descriptor
//    010 COMPUTE – fire PE cluster for N cycles
//    011 REDUCE  – trigger reduction tree output
//    100 STORE   – write result to memory bank
//    101 SYNC    – stall until all pipeline stages empty
//
//  Instructions are written to a program FIFO by the CPU.
//  The sequencer pops and dispatches them to subsystem control signals.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module instr_sequencer #(
    parameter integer PROG_DEPTH = 64   // instruction FIFO depth
)(
    input  wire        clk,
    input  wire        resetn,

    // Instruction FIFO push (from CSR)
    input  wire [31:0] instr_push,      // {opcode[2:0], flags[4:0], operand[23:0]}
    input  wire        instr_push_valid,
    output wire        instr_push_ready,

    // Datapath busy signals (for SYNC)
    input  wire        dma_busy,
    input  wire        compute_busy,
    input  wire        reduce_busy,

    // Dispatch outputs
    output reg         seq_dma_start,
    output reg [23:0]  seq_dma_operand,
    output reg         seq_compute_start,
    output reg [23:0]  seq_compute_operand,
    output reg         seq_reduce_start,
    output reg         seq_store_start,
    output reg [23:0]  seq_store_operand,
    output reg         seq_sync,

    // Status
    output wire        seq_idle
);

    localparam integer AW = $clog2(PROG_DEPTH);

    // -------------------------------------------------------------------------
    // Instruction FIFO
    // -------------------------------------------------------------------------
    reg [31:0] prog_mem [0:PROG_DEPTH-1];
    reg [AW:0] wr_ptr, rd_ptr;

    wire fifo_empty = (wr_ptr == rd_ptr);
    wire fifo_full  = ((wr_ptr[AW-1:0] == rd_ptr[AW-1:0]) &&
                       (wr_ptr[AW] != rd_ptr[AW]));

    assign instr_push_ready = ~fifo_full;

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr <= {(AW+1){1'b0}};
        end else begin
            if (instr_push_valid && instr_push_ready) begin
                prog_mem[wr_ptr[AW-1:0]] <= instr_push;
                wr_ptr <= wr_ptr + 1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Dispatch FSM
    // -------------------------------------------------------------------------
    localparam [1:0] S_FETCH    = 2'd0;
    localparam [1:0] S_DISPATCH = 2'd1;
    localparam [1:0] S_WAIT     = 2'd2;

    reg [1:0]  state;
    reg [31:0] cur_instr;

    wire [2:0]  opcode  = cur_instr[31:29];
    wire [4:0]  flags   = cur_instr[28:24];
    wire [23:0] operand = cur_instr[23:0];

    assign seq_idle = fifo_empty && (state == S_FETCH);

    always @(posedge clk) begin
        if (!resetn) begin
            state              <= S_FETCH;
            rd_ptr             <= {(AW+1){1'b0}};
            cur_instr          <= 32'd0;
            seq_dma_start      <= 1'b0;
            seq_compute_start  <= 1'b0;
            seq_reduce_start   <= 1'b0;
            seq_store_start    <= 1'b0;
            seq_sync           <= 1'b0;
        end else begin
            // Default: deassert all pulses
            seq_dma_start     <= 1'b0;
            seq_compute_start <= 1'b0;
            seq_reduce_start  <= 1'b0;
            seq_store_start   <= 1'b0;
            seq_sync          <= 1'b0;

            case (state)

                S_FETCH: begin
                    if (!fifo_empty) begin
                        cur_instr <= prog_mem[rd_ptr[AW-1:0]];
                        rd_ptr    <= rd_ptr + 1;
                        state     <= S_DISPATCH;
                    end
                end

                S_DISPATCH: begin
                    case (opcode)
                        3'b000: state <= S_FETCH;   // NOP
                        3'b001: begin               // LOAD
                            seq_dma_start     <= 1'b1;
                            seq_dma_operand   <= operand;
                            state             <= S_WAIT;
                        end
                        3'b010: begin               // COMPUTE
                            seq_compute_start   <= 1'b1;
                            seq_compute_operand <= operand;
                            state               <= S_WAIT;
                        end
                        3'b011: begin               // REDUCE
                            seq_reduce_start <= 1'b1;
                            state            <= S_WAIT;
                        end
                        3'b100: begin               // STORE
                            seq_store_start   <= 1'b1;
                            seq_store_operand <= operand;
                            state             <= S_WAIT;
                        end
                        3'b101: begin               // SYNC
                            seq_sync <= 1'b1;
                            state    <= S_WAIT;
                        end
                        default: state <= S_FETCH;
                    endcase
                end

                S_WAIT: begin
                    // Wait for the dispatched operation to complete
                    case (opcode)
                        3'b001: if (!dma_busy)     state <= S_FETCH;
                        3'b010: if (!compute_busy) state <= S_FETCH;
                        3'b011: if (!reduce_busy)  state <= S_FETCH;
                        3'b100: if (!compute_busy) state <= S_FETCH;
                        3'b101: begin
                            if (!dma_busy && !compute_busy && !reduce_busy) begin
                                seq_sync <= 1'b0;
                                state    <= S_FETCH;
                            end
                        end
                        default: state <= S_FETCH;
                    endcase
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
