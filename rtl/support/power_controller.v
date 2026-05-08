// =============================================================================
//  power_controller.v – Power Domain Management
//
//  Tracks power state per cluster and drives isolation/retention signals.
//  FSM per domain:
//    ACTIVE → ISOLATE → POWER_OFF
//    POWER_OFF → RESTORE → ACTIVE
// =============================================================================
`timescale 1ns/1ps

module power_controller #(
    parameter integer N_DOMAINS = 4   // one per tile cluster
)(
    input  wire              clk,
    input  wire              resetn,

    // Software control: request power down/up per domain
    input  wire [N_DOMAINS-1:0] pw_down_req,   // 1 = request power-off
    input  wire [N_DOMAINS-1:0] pw_up_req,     // 1 = request power-on

    // Outputs to domains
    output reg  [N_DOMAINS-1:0] iso_en,        // isolation enable
    output reg  [N_DOMAINS-1:0] ret_en,        // retention enable
    output reg  [N_DOMAINS-1:0] pw_good,       // domain powered-on flag
    output reg  [N_DOMAINS-1:0] clk_en         // clock enable to domain
);

    localparam [1:0] PW_ACTIVE    = 2'd0;
    localparam [1:0] PW_ISOLATE   = 2'd1;
    localparam [1:0] PW_OFF       = 2'd2;
    localparam [1:0] PW_RESTORE   = 2'd3;

    // Power-down delay counter (simple)
    localparam integer DELAY_CYC = 4;

    reg [1:0]                   pw_state   [0:N_DOMAINS-1];
    reg [$clog2(DELAY_CYC)-1:0] pw_cnt     [0:N_DOMAINS-1];

    integer d;

    always @(posedge clk) begin
        if (!resetn) begin
            for (d = 0; d < N_DOMAINS; d = d + 1) begin
                pw_state[d] <= PW_ACTIVE;
                pw_cnt[d]   <= 0;
                iso_en[d]   <= 1'b0;
                ret_en[d]   <= 1'b0;
                pw_good[d]  <= 1'b1;
                clk_en[d]   <= 1'b1;
            end
        end else begin
            for (d = 0; d < N_DOMAINS; d = d + 1) begin
                case (pw_state[d])

                    PW_ACTIVE: begin
                        pw_good[d] <= 1'b1;
                        clk_en[d]  <= 1'b1;
                        iso_en[d]  <= 1'b0;
                        ret_en[d]  <= 1'b0;
                        if (pw_down_req[d]) begin
                            iso_en[d]   <= 1'b1;
                            pw_state[d] <= PW_ISOLATE;
                            pw_cnt[d]   <= 0;
                        end
                    end

                    PW_ISOLATE: begin
                        if (pw_cnt[d] == DELAY_CYC-1) begin
                            clk_en[d]   <= 1'b0;
                            ret_en[d]   <= 1'b1;
                            pw_good[d]  <= 1'b0;
                            pw_state[d] <= PW_OFF;
                            pw_cnt[d]   <= 0;
                        end else
                            pw_cnt[d] <= pw_cnt[d] + 1;
                    end

                    PW_OFF: begin
                        if (pw_up_req[d]) begin
                            pw_state[d] <= PW_RESTORE;
                            pw_cnt[d]   <= 0;
                        end
                    end

                    PW_RESTORE: begin
                        if (pw_cnt[d] == DELAY_CYC-1) begin
                            ret_en[d]   <= 1'b0;
                            iso_en[d]   <= 1'b0;
                            clk_en[d]   <= 1'b1;
                            pw_good[d]  <= 1'b1;
                            pw_state[d] <= PW_ACTIVE;
                        end else
                            pw_cnt[d] <= pw_cnt[d] + 1;
                    end

                    default: pw_state[d] <= PW_ACTIVE;
                endcase
            end
        end
    end

endmodule
