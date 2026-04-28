// =============================================================================
//  mem_bist.v – Built-In Self Test for Block RAM
//
//  Implements a March C- algorithm:
//    (Up){W0}; (Up){R0,W1}; (Up){R1,W0}; (Down){R0,W1}; (Down){R1,W0}; (Up){R0}
//
//  Activated by bist_start.  bist_done pulses when complete.
//  bist_pass=1 means no errors detected; bist_fail=1 means one or more errors.
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

module mem_bist #(
    parameter integer DEPTH     = 4096,
    parameter integer BUS_WIDTH = 64
)(
    input  wire                    clk,
    input  wire                    resetn,

    // Control
    input  wire                    bist_start,  // pulse to begin
    output reg                     bist_done,
    output reg                     bist_pass,
    output reg                     bist_fail,

    // Memory interface (takes over from normal user during BIST)
    output reg  [$clog2(DEPTH)-1:0] bist_wr_addr,
    output reg  [BUS_WIDTH-1:0]     bist_wr_data,
    output reg  [BUS_WIDTH/8-1:0]   bist_wr_strb,
    output reg                      bist_wr_en,

    output reg  [$clog2(DEPTH)-1:0] bist_rd_addr,
    output reg                      bist_rd_en,

    input  wire [BUS_WIDTH-1:0]     bist_rd_data,
    input  wire                     bist_rd_valid,

    // BIST active flag (so controller can mux memory ports)
    output reg                      bist_active
);

    localparam integer ADDR_W = $clog2(DEPTH);

    // March C- phases
    localparam [2:0] MARCH_IDLE  = 3'd0;
    localparam [2:0] MARCH_W0    = 3'd1;   // (Up){W0}
    localparam [2:0] MARCH_R0W1  = 3'd2;   // (Up){R0,W1}
    localparam [2:0] MARCH_R1W0  = 3'd3;   // (Up){R1,W0}
    localparam [2:0] MARCH_DR0W1 = 3'd4;   // (Down){R0,W1}  -- skipped in March C-
    localparam [2:0] MARCH_DR1W0 = 3'd5;   // (Down){R1,W0}
    localparam [2:0] MARCH_R0    = 3'd6;   // (Up){R0}
    localparam [2:0] MARCH_DONE  = 3'd7;

    reg [2:0]       phase;
    reg [ADDR_W:0]  addr;    // one extra bit for boundary check
    reg             rd_wait; // waiting for read result

    always @(posedge clk) begin
        if (!resetn) begin
            phase      <= MARCH_IDLE;
            addr       <= {(ADDR_W+1){1'b0}};
            rd_wait    <= 1'b0;
            bist_done  <= 1'b0;
            bist_pass  <= 1'b0;
            bist_fail  <= 1'b0;
            bist_active<= 1'b0;
            bist_wr_en <= 1'b0;
            bist_rd_en <= 1'b0;
            bist_wr_strb <= {(BUS_WIDTH/8){1'b1}};
        end else begin
            // default: deassert pulse signals
            bist_wr_en <= 1'b0;
            bist_rd_en <= 1'b0;
            bist_done  <= 1'b0;

            case (phase)

                MARCH_IDLE: begin
                    bist_pass <= 1'b0;
                    bist_fail <= 1'b0;
                    if (bist_start) begin
                        phase       <= MARCH_W0;
                        addr        <= {(ADDR_W+1){1'b0}};
                        bist_active <= 1'b1;
                    end
                end

                // ---- Phase 1: Write all-zeros upward ----
                MARCH_W0: begin
                    bist_wr_addr <= addr[ADDR_W-1:0];
                    bist_wr_data <= {BUS_WIDTH{1'b0}};
                    bist_wr_strb <= {(BUS_WIDTH/8){1'b1}};
                    bist_wr_en   <= 1'b1;
                    if (addr == DEPTH-1) begin
                        addr  <= {(ADDR_W+1){1'b0}};
                        phase <= MARCH_R0W1;
                    end else
                        addr <= addr + 1;
                end

                // ---- Phase 2: Read 0, Write 1 (upward) ----
                MARCH_R0W1: begin
                    if (!rd_wait) begin
                        bist_rd_addr <= addr[ADDR_W-1:0];
                        bist_rd_en   <= 1'b1;
                        rd_wait      <= 1'b1;
                    end else if (bist_rd_valid) begin
                        rd_wait <= 1'b0;
                        if (bist_rd_data != {BUS_WIDTH{1'b0}})
                            bist_fail <= 1'b1;
                        // Write ones
                        bist_wr_addr <= addr[ADDR_W-1:0];
                        bist_wr_data <= {BUS_WIDTH{1'b1}};
                        bist_wr_en   <= 1'b1;
                        if (addr == DEPTH-1) begin
                            addr  <= {(ADDR_W+1){1'b0}};
                            phase <= MARCH_R1W0;
                        end else
                            addr <= addr + 1;
                    end
                end

                // ---- Phase 3: Read 1, Write 0 (upward) ----
                MARCH_R1W0: begin
                    if (!rd_wait) begin
                        bist_rd_addr <= addr[ADDR_W-1:0];
                        bist_rd_en   <= 1'b1;
                        rd_wait      <= 1'b1;
                    end else if (bist_rd_valid) begin
                        rd_wait <= 1'b0;
                        if (bist_rd_data != {BUS_WIDTH{1'b1}})
                            bist_fail <= 1'b1;
                        bist_wr_addr <= addr[ADDR_W-1:0];
                        bist_wr_data <= {BUS_WIDTH{1'b0}};
                        bist_wr_en   <= 1'b1;
                        if (addr == DEPTH-1) begin
                            addr  <= DEPTH-1;
                            phase <= MARCH_DR1W0;
                        end else
                            addr <= addr + 1;
                    end
                end

                // ---- Phase 4: Read 0, Write 1 (downward) – skipped → DR1W0 ----
                MARCH_DR1W0: begin
                    if (!rd_wait) begin
                        bist_rd_addr <= addr[ADDR_W-1:0];
                        bist_rd_en   <= 1'b1;
                        rd_wait      <= 1'b1;
                    end else if (bist_rd_valid) begin
                        rd_wait <= 1'b0;
                        if (bist_rd_data != {BUS_WIDTH{1'b0}})
                            bist_fail <= 1'b1;
                        bist_wr_addr <= addr[ADDR_W-1:0];
                        bist_wr_data <= {BUS_WIDTH{1'b1}};
                        bist_wr_en   <= 1'b1;
                        if (addr == 0) begin
                            addr  <= {(ADDR_W+1){1'b0}};
                            phase <= MARCH_R0;
                        end else
                            addr <= addr - 1;
                    end
                end

                // ---- Phase 5: Final Read (upward) ----
                MARCH_R0: begin
                    if (!rd_wait) begin
                        bist_rd_addr <= addr[ADDR_W-1:0];
                        bist_rd_en   <= 1'b1;
                        rd_wait      <= 1'b1;
                    end else if (bist_rd_valid) begin
                        rd_wait <= 1'b0;
                        // After DR1W0, memory should contain all-ones
                        if (bist_rd_data != {BUS_WIDTH{1'b1}})
                            bist_fail <= 1'b1;
                        if (addr == DEPTH-1) begin
                            phase <= MARCH_DONE;
                        end else
                            addr <= addr + 1;
                    end
                end

                MARCH_DONE: begin
                    bist_done   <= 1'b1;
                    bist_active <= 1'b0;
                    bist_pass   <= ~bist_fail;
                    phase       <= MARCH_IDLE;
                end

                default: phase <= MARCH_IDLE;
            endcase
        end
    end

endmodule
