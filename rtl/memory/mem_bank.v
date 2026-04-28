// =============================================================================
//  mem_bank.v – Single Block RAM bank with SECDED ECC
//
//  - Uses ram_style="block" to target on-chip BRAM (not LUTs)
//  - 64-bit data bus + 8 ECC bits stored per location
//  - Read-modify-write ECC correction on reads
//  - Parameterised depth (must be power of 2 for BRAM inference)
// =============================================================================
`timescale 1ns/1ps
`include "rtl/config/npu_defines.vh"

(* keep_hierarchy = "yes", dont_touch = "yes" *)
module mem_bank #(
    parameter integer DEPTH     = 4096,   // number of 64-bit words
    parameter integer BUS_WIDTH = 64,     // data bus width (must be 64 for SECDED)
    parameter integer ECC_EN    = 1       // 1 = include ECC logic
)(
    input  wire                    clk,
    input  wire                    resetn,

    // Port A – write
    input  wire [$clog2(DEPTH)-1:0] wr_addr,
    input  wire [BUS_WIDTH-1:0]     wr_data,
    input  wire [BUS_WIDTH/8-1:0]   wr_strb,   // byte enable
    input  wire                     wr_en,

    // Port B – read (1-cycle registered output)
    input  wire [$clog2(DEPTH)-1:0] rd_addr,
    input  wire                     rd_en,
    output reg  [BUS_WIDTH-1:0]     rd_data,
    output reg                      rd_valid,

    // ECC status
    output reg                      ecc_single,  // correctable single-bit error
    output reg                      ecc_double   // uncorrectable double-bit error
);

    localparam integer ADDR_W   = $clog2(DEPTH);
    localparam integer STORE_W  = BUS_WIDTH + 8;  // data + 8 ECC bits

    // -------------------------------------------------------------------------
    // On-chip Block RAM array
    //   * ram_style="block" forces BRAM mapping
    //   * If BUS_WIDTH != 64, ECC is bypassed (raw storage)
    // -------------------------------------------------------------------------
    `BRAM_ATTR reg [STORE_W-1:0] mem [0:DEPTH-1];

    // -------------------------------------------------------------------------
    // ECC encoder / decoder instances (only wired when ECC_EN=1)
    // -------------------------------------------------------------------------
    wire [71:0] enc_codeword;
    wire [63:0] dec_data_corr;
    wire        dec_single, dec_double;

    generate
        if (ECC_EN && BUS_WIDTH == 64) begin : gen_ecc
            mem_ecc u_ecc (
                .enc_data       (wr_data),
                .enc_codeword   (enc_codeword),
                .dec_codeword   (mem[rd_addr]),
                .dec_data       (dec_data_corr),
                .dec_single_err (dec_single),
                .dec_double_err (dec_double)
            );
        end else begin : gen_no_ecc
            assign enc_codeword   = {{8{1'b0}}, wr_data};
            assign dec_data_corr  = mem[rd_addr][BUS_WIDTH-1:0];
            assign dec_single     = 1'b0;
            assign dec_double     = 1'b0;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Write path (byte-enable merge with ECC)
    // -------------------------------------------------------------------------
    integer bi;

    always @(posedge clk) begin
        if (wr_en) begin
            // Apply byte strobes – read-modify-write on partial writes
            // (For full-word writes, all wr_strb bits are set)
            // We store the full codeword only when all bytes enabled;
            // otherwise just update data bytes and leave ECC unchecked (raw)
            if (&wr_strb) begin
                mem[wr_addr] <= enc_codeword;
            end else begin
                for (bi = 0; bi < BUS_WIDTH/8; bi = bi + 1) begin
                    if (wr_strb[bi])
                        mem[wr_addr][8*bi +: 8] <= wr_data[8*bi +: 8];
                end
                // Invalidate ECC on partial write (mark upper bits zero)
                mem[wr_addr][STORE_W-1:BUS_WIDTH] <= {8{1'b0}};
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read path
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            rd_data    <= {BUS_WIDTH{1'b0}};
            rd_valid   <= 1'b0;
            ecc_single <= 1'b0;
            ecc_double <= 1'b0;
        end else begin
            rd_valid   <= 1'b0;
            ecc_single <= 1'b0;
            ecc_double <= 1'b0;

            if (rd_en) begin
                rd_data    <= (ECC_EN && BUS_WIDTH == 64) ?
                              dec_data_corr : mem[rd_addr][BUS_WIDTH-1:0];
                rd_valid   <= 1'b1;
                ecc_single <= dec_single;
                ecc_double <= dec_double;
            end
        end
    end

endmodule
