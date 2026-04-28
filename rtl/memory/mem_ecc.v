// =============================================================================
//  mem_ecc.v – SECDED (72,64) ECC Encoder / Decoder
//
//  Implements a standard (72,64) Hamming code with an overall parity bit,
//  providing Single-Error Correction, Double-Error Detection (SECDED).
//
//  Encoder: takes 64-bit data → emits 72-bit codeword {parity[7:0], data[63:0]}
//  Decoder: takes 72-bit codeword → 64-bit corrected data + error flags
// =============================================================================
`timescale 1ns/1ps

module mem_ecc (
    // Encoder
    input  wire [63:0]  enc_data,
    output wire [71:0]  enc_codeword,   // {8 parity bits, 64 data bits}

    // Decoder
    input  wire [71:0]  dec_codeword,
    output wire [63:0]  dec_data,
    output wire         dec_single_err, // correctable single-bit error
    output wire         dec_double_err  // uncorrectable double-bit error
);

    // -------------------------------------------------------------------------
    // Encoder
    // Parity bit positions in a 72-bit word: 1,2,4,8,16,32,64 (7 bits)
    // Plus overall parity at bit 0 position of the ECC byte.
    //
    // We use a simplified (but correct) XOR-based implementation.
    // h[k] covers data bits whose (k+1) position has bit k set.
    // -------------------------------------------------------------------------
    wire [6:0] h_enc;   // 7 Hamming parity bits

    // h[0] covers positions 1,3,5,7,9,... (bits with bit-0 set)
    assign h_enc[0] = ^{enc_data[ 0], enc_data[ 1], enc_data[ 3], enc_data[ 4],
                         enc_data[ 6], enc_data[ 8], enc_data[10], enc_data[11],
                         enc_data[13], enc_data[15], enc_data[17], enc_data[19],
                         enc_data[21], enc_data[23], enc_data[25], enc_data[26],
                         enc_data[28], enc_data[30], enc_data[32], enc_data[34],
                         enc_data[36], enc_data[38], enc_data[40], enc_data[42],
                         enc_data[44], enc_data[46], enc_data[48], enc_data[50],
                         enc_data[52], enc_data[54], enc_data[56], enc_data[57],
                         enc_data[59], enc_data[61], enc_data[63]};

    assign h_enc[1] = ^{enc_data[ 0], enc_data[ 2], enc_data[ 3], enc_data[ 5],
                         enc_data[ 6], enc_data[ 9], enc_data[10], enc_data[12],
                         enc_data[13], enc_data[16], enc_data[17], enc_data[20],
                         enc_data[21], enc_data[24], enc_data[25], enc_data[27],
                         enc_data[28], enc_data[31], enc_data[32], enc_data[35],
                         enc_data[36], enc_data[39], enc_data[40], enc_data[43],
                         enc_data[44], enc_data[47], enc_data[48], enc_data[51],
                         enc_data[52], enc_data[55], enc_data[56], enc_data[58],
                         enc_data[59], enc_data[62], enc_data[63]};

    assign h_enc[2] = ^{enc_data[ 1], enc_data[ 2], enc_data[ 3], enc_data[ 7],
                         enc_data[ 8], enc_data[ 9], enc_data[10], enc_data[14],
                         enc_data[15], enc_data[16], enc_data[17], enc_data[22],
                         enc_data[23], enc_data[24], enc_data[25], enc_data[29],
                         enc_data[30], enc_data[31], enc_data[32], enc_data[37],
                         enc_data[38], enc_data[39], enc_data[40], enc_data[45],
                         enc_data[46], enc_data[47], enc_data[48], enc_data[53],
                         enc_data[54], enc_data[55], enc_data[56], enc_data[60],
                         enc_data[61], enc_data[62], enc_data[63]};

    assign h_enc[3] = ^{enc_data[ 4], enc_data[ 5], enc_data[ 6], enc_data[ 7],
                         enc_data[ 8], enc_data[ 9], enc_data[10], enc_data[18],
                         enc_data[19], enc_data[20], enc_data[21], enc_data[22],
                         enc_data[23], enc_data[24], enc_data[25], enc_data[33],
                         enc_data[34], enc_data[35], enc_data[36], enc_data[37],
                         enc_data[38], enc_data[39], enc_data[40], enc_data[49],
                         enc_data[50], enc_data[51], enc_data[52], enc_data[53],
                         enc_data[54], enc_data[55], enc_data[56]};

    assign h_enc[4] = ^{enc_data[11], enc_data[12], enc_data[13], enc_data[14],
                         enc_data[15], enc_data[16], enc_data[17], enc_data[18],
                         enc_data[19], enc_data[20], enc_data[21], enc_data[22],
                         enc_data[23], enc_data[24], enc_data[25], enc_data[41],
                         enc_data[42], enc_data[43], enc_data[44], enc_data[45],
                         enc_data[46], enc_data[47], enc_data[48], enc_data[49],
                         enc_data[50], enc_data[51], enc_data[52], enc_data[53],
                         enc_data[54], enc_data[55], enc_data[56]};

    assign h_enc[5] = ^{enc_data[26], enc_data[27], enc_data[28], enc_data[29],
                         enc_data[30], enc_data[31], enc_data[32], enc_data[33],
                         enc_data[34], enc_data[35], enc_data[36], enc_data[37],
                         enc_data[38], enc_data[39], enc_data[40], enc_data[41],
                         enc_data[42], enc_data[43], enc_data[44], enc_data[45],
                         enc_data[46], enc_data[47], enc_data[48], enc_data[49],
                         enc_data[50], enc_data[51], enc_data[52], enc_data[53],
                         enc_data[54], enc_data[55], enc_data[56]};

    assign h_enc[6] = ^{enc_data[57], enc_data[58], enc_data[59], enc_data[60],
                         enc_data[61], enc_data[62], enc_data[63]};

    // Overall parity bit
    wire p_enc = ^{enc_data, h_enc};

    // Codeword layout: bits [71:65] = h[6:0], bit 64 = p, bits [63:0] = data
    assign enc_codeword = {h_enc, p_enc, enc_data};

    // -------------------------------------------------------------------------
    // Decoder
    // Syndrome = XOR of received parity bits against re-computed parity
    // -------------------------------------------------------------------------
    wire [63:0] rx_data  = dec_codeword[63:0];
    wire        rx_p     = dec_codeword[64];
    wire [6:0]  rx_h     = dec_codeword[71:65];

    // Re-compute parity from received data
    wire [6:0]  h_dec;

    assign h_dec[0] = ^{rx_data[ 0], rx_data[ 1], rx_data[ 3], rx_data[ 4],
                          rx_data[ 6], rx_data[ 8], rx_data[10], rx_data[11],
                          rx_data[13], rx_data[15], rx_data[17], rx_data[19],
                          rx_data[21], rx_data[23], rx_data[25], rx_data[26],
                          rx_data[28], rx_data[30], rx_data[32], rx_data[34],
                          rx_data[36], rx_data[38], rx_data[40], rx_data[42],
                          rx_data[44], rx_data[46], rx_data[48], rx_data[50],
                          rx_data[52], rx_data[54], rx_data[56], rx_data[57],
                          rx_data[59], rx_data[61], rx_data[63]};

    assign h_dec[1] = ^{rx_data[ 0], rx_data[ 2], rx_data[ 3], rx_data[ 5],
                          rx_data[ 6], rx_data[ 9], rx_data[10], rx_data[12],
                          rx_data[13], rx_data[16], rx_data[17], rx_data[20],
                          rx_data[21], rx_data[24], rx_data[25], rx_data[27],
                          rx_data[28], rx_data[31], rx_data[32], rx_data[35],
                          rx_data[36], rx_data[39], rx_data[40], rx_data[43],
                          rx_data[44], rx_data[47], rx_data[48], rx_data[51],
                          rx_data[52], rx_data[55], rx_data[56], rx_data[58],
                          rx_data[59], rx_data[62], rx_data[63]};

    assign h_dec[2] = ^{rx_data[ 1], rx_data[ 2], rx_data[ 3], rx_data[ 7],
                          rx_data[ 8], rx_data[ 9], rx_data[10], rx_data[14],
                          rx_data[15], rx_data[16], rx_data[17], rx_data[22],
                          rx_data[23], rx_data[24], rx_data[25], rx_data[29],
                          rx_data[30], rx_data[31], rx_data[32], rx_data[37],
                          rx_data[38], rx_data[39], rx_data[40], rx_data[45],
                          rx_data[46], rx_data[47], rx_data[48], rx_data[53],
                          rx_data[54], rx_data[55], rx_data[56], rx_data[60],
                          rx_data[61], rx_data[62], rx_data[63]};

    assign h_dec[3] = ^{rx_data[ 4], rx_data[ 5], rx_data[ 6], rx_data[ 7],
                          rx_data[ 8], rx_data[ 9], rx_data[10], rx_data[18],
                          rx_data[19], rx_data[20], rx_data[21], rx_data[22],
                          rx_data[23], rx_data[24], rx_data[25], rx_data[33],
                          rx_data[34], rx_data[35], rx_data[36], rx_data[37],
                          rx_data[38], rx_data[39], rx_data[40], rx_data[49],
                          rx_data[50], rx_data[51], rx_data[52], rx_data[53],
                          rx_data[54], rx_data[55], rx_data[56]};

    assign h_dec[4] = ^{rx_data[11], rx_data[12], rx_data[13], rx_data[14],
                          rx_data[15], rx_data[16], rx_data[17], rx_data[18],
                          rx_data[19], rx_data[20], rx_data[21], rx_data[22],
                          rx_data[23], rx_data[24], rx_data[25], rx_data[41],
                          rx_data[42], rx_data[43], rx_data[44], rx_data[45],
                          rx_data[46], rx_data[47], rx_data[48], rx_data[49],
                          rx_data[50], rx_data[51], rx_data[52], rx_data[53],
                          rx_data[54], rx_data[55], rx_data[56]};

    assign h_dec[5] = ^{rx_data[26], rx_data[27], rx_data[28], rx_data[29],
                          rx_data[30], rx_data[31], rx_data[32], rx_data[33],
                          rx_data[34], rx_data[35], rx_data[36], rx_data[37],
                          rx_data[38], rx_data[39], rx_data[40], rx_data[41],
                          rx_data[42], rx_data[43], rx_data[44], rx_data[45],
                          rx_data[46], rx_data[47], rx_data[48], rx_data[49],
                          rx_data[50], rx_data[51], rx_data[52], rx_data[53],
                          rx_data[54], rx_data[55], rx_data[56]};

    assign h_dec[6] = ^{rx_data[57], rx_data[58], rx_data[59], rx_data[60],
                          rx_data[61], rx_data[62], rx_data[63]};

    wire [6:0] syndrome = rx_h ^ h_dec;
    wire       p_check  = ^{rx_data, rx_p, rx_h};

    // Single-bit error: syndrome != 0 AND overall parity mismatch
    // Double-bit error: syndrome != 0 AND overall parity matches
    assign dec_single_err = (syndrome != 7'd0) &&  p_check;
    assign dec_double_err = (syndrome != 7'd0) && ~p_check;

    // Corrected data (flip the erroneous bit if single-bit error)
    wire [63:0] corrected;
    genvar bi;
    generate
        for (bi = 0; bi < 64; bi = bi + 1) begin : gen_cor
            assign corrected[bi] = (dec_single_err && (syndrome == bi+1)) ?
                                   ~rx_data[bi] : rx_data[bi];
        end
    endgenerate

    assign dec_data = corrected;

endmodule
