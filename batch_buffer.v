`timescale 1ns / 1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
//
// batch_buffer.v  -- ping-pong double buffer (improvement #6)
//
// One bank fills from DMA while the other is consumed by the loader.
// The refill latency is completely hidden behind compute time.
//
// Protocol (unchanged from original single-buffer interface):
//   in_valid / in_byte / mode_sel  → producer push path
//   in_ready                       → backpressure to producer
//   consume                        → consumer signals "done reading"
//   flat_out / batch_ready         → consumer pull path
//
module batch_buffer #(
    parameter NUM_ELEMS = 256
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     in_valid,
    input  wire [7:0]               in_byte,
    input  wire [1:0]               mode_sel,

    output wire                     in_ready,
    input  wire                     consume,

    output reg  [NUM_ELEMS*8-1:0]   flat_out,
    output reg                      batch_ready
);

    localparam CNTW = $clog2(NUM_ELEMS + 1);

    // Two buffer banks
    reg [NUM_ELEMS*8-1:0] bank_data [0:1];
    reg [CNTW-1:0]        bank_cnt  [0:1];   // fill count
    reg                   bank_full [0:1];   // complete batch flag
    reg                   half_tog  [0:1];   // INT4 nibble packing state
    reg [3:0]             low_nib   [0:1];   // saved low nibble (INT4)

    reg fill_sel;   // which bank the producer is currently filling

    integer byte_idx;

    // Producer may push when the fill bank still has room
    assign in_ready = (bank_cnt[fill_sel] < NUM_ELEMS);

    always @(posedge clk) begin
        if (!resetn) begin
            fill_sel        <= 1'b0;
            bank_cnt[0]     <= 0;       bank_cnt[1]     <= 0;
            bank_full[0]    <= 1'b0;    bank_full[1]    <= 1'b0;
            half_tog[0]     <= 1'b0;    half_tog[1]     <= 1'b0;
            low_nib[0]      <= 4'd0;    low_nib[1]      <= 4'd0;
            bank_data[0]    <= {NUM_ELEMS*8{1'b0}};
            bank_data[1]    <= {NUM_ELEMS*8{1'b0}};
            flat_out        <= {NUM_ELEMS*8{1'b0}};
            batch_ready     <= 1'b0;
        end else begin

            // --- Consumer: release read bank on consume ---
            // Read bank is always bank[!fill_sel]
            if (consume && bank_full[!fill_sel]) begin
                bank_full[!fill_sel] <= 1'b0;
                bank_cnt[!fill_sel]  <= 0;
                half_tog[!fill_sel]  <= 1'b0;
                low_nib[!fill_sel]   <= 4'd0;
            end

            // --- Producer: write one byte into fill bank ---
            if (in_valid && (bank_cnt[fill_sel] < NUM_ELEMS)) begin
                if (mode_sel == 2'b01) begin
                    // INT4: pack two nibbles per byte slot
                    byte_idx = (bank_cnt[fill_sel] >> 1);
                    if (!half_tog[fill_sel]) begin
                        low_nib[fill_sel]  <= in_byte[3:0];
                        half_tog[fill_sel] <= 1'b1;
                        bank_cnt[fill_sel] <= bank_cnt[fill_sel] + 1;
                        if (bank_cnt[fill_sel] + 1 == NUM_ELEMS) begin
                            // odd final element: store low nibble only
                            bank_data[fill_sel][(byte_idx+1)*8-1 -: 8] <=
                                {4'd0, in_byte[3:0]};
                            bank_full[fill_sel] <= 1'b1;
                        end
                    end else begin
                        bank_data[fill_sel][(byte_idx+1)*8-1 -: 8] <=
                            {in_byte[3:0], low_nib[fill_sel]};
                        half_tog[fill_sel] <= 1'b0;
                        bank_cnt[fill_sel] <= bank_cnt[fill_sel] + 1;
                        if (bank_cnt[fill_sel] + 1 == NUM_ELEMS)
                            bank_full[fill_sel] <= 1'b1;
                    end
                end else begin
                    // INT8 (also INT2: raw bytes)
                    byte_idx = bank_cnt[fill_sel];
                    bank_data[fill_sel][(byte_idx+1)*8-1 -: 8] <= in_byte;
                    bank_cnt[fill_sel] <= bank_cnt[fill_sel] + 1;
                    if (bank_cnt[fill_sel] + 1 == NUM_ELEMS)
                        bank_full[fill_sel] <= 1'b1;
                end
            end

            // --- Switch fill bank when full and read bank is free ---
            // (read bank freed on the consume cycle or already empty)
            if (bank_full[fill_sel] && !bank_full[!fill_sel])
                fill_sel <= !fill_sel;

            // --- Update outputs: expose the current read bank ---
            flat_out    <= bank_data[!fill_sel];
            batch_ready <= bank_full[!fill_sel];
        end
    end

endmodule
