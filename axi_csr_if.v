`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module axi_csr_if #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter WIDTH      = 2048
)(
    input  wire                      clk,
    input  wire                      resetn,

    // AXI-lite write
    input  wire [ADDR_WIDTH-1:0]     awaddr,
    input  wire                      awvalid,
    output reg                       awready,
    input  wire [DATA_WIDTH-1:0]     wdata,
    input  wire [DATA_WIDTH/8-1:0]   wstrb,
    input  wire                      wvalid,
    output reg                       wready,
    output reg [1:0]                 bresp,
    output reg                       bvalid,
    input  wire                      bready,

    // AXI-lite read
    input  wire [ADDR_WIDTH-1:0]     araddr,
    input  wire                      arvalid,
    output reg                       arready,
    output reg [DATA_WIDTH-1:0]      rdata,
    output reg [1:0]                 rresp,
    output reg                       rvalid,
    input  wire                      rready,

    // CSR outputs
    output reg  [1:0]                mode_sel,
    output reg  signed [31:0]        bias,
    output reg  [3:0]                scale,
    output reg  [3:0]                shift,
    output reg                       fuse_en,
    output reg  [3:0]                act_sel,
    output reg  [WIDTH-1:0]          A_flat,
    output reg  [WIDTH-1:0]          B_flat,
    output reg                       gate_en,
    output reg                       relu_en,
    output reg                       data_sel,

    // perf
    output reg                       perf_start, // 1-cycle pulse
    input  wire                      perf_done,
    input  wire [63:0]               perf_cycles,
    input  wire [63:0]               perf_ops_acc,
    input  wire [31:0]               perf_active_pe_sum,
    input  wire [31:0]               perf_last_latency,

    // DMA
    output reg  [31:0]               dma_ctrl,
    output reg  [31:0]               dma_addr,
    output reg  [31:0]               dma_len,
    input  wire [31:0]               dma_status,

    output reg  [7:0]                threshold,
    output reg                       dyn_sched_en,
    output reg                       slice_en,
    output reg  [3:0]                msb_stat_thres,

    // read-only MAC result
    input  wire signed [31:0]        mac_result,

    // --- BRAM preload interface (CPU -> dma_bram) ---
    // Each asserted bram_we is a single-cycle pulse carrying one byte in bram_wdata[7:0]
    output reg [7:0]                 bram_wdata,
    output reg                       bram_we,
    output reg [15:0]                bram_waddr
);

    // Derived constants
    localparam FLAT_WORDS = WIDTH / 32;
    localparam integer A_BASE = 12'h100;
    localparam integer B_BASE = 12'h200;

    // Internal storage for A_flat and B_flat (word array)
    reg [31:0] A_words [0:FLAT_WORDS-1];
    reg [31:0] B_words [0:FLAT_WORDS-1];

    integer i, byte_i;

    // ------------------------------------------------------------
    // AXI-Lite write handshake/capture
    // ------------------------------------------------------------
    reg                     aw_pending;
    reg [ADDR_WIDTH-1:0]    awaddr_reg;
    reg                     w_pending;
    reg [DATA_WIDTH-1:0]    wdata_reg;
    reg [DATA_WIDTH/8-1:0]  wstrb_reg;
    reg write_fire_r;

    // AW channel
    always @(posedge clk) begin
        if (!resetn) begin
            awready <= 1'b0; aw_pending <= 1'b0; awaddr_reg <= {ADDR_WIDTH{1'b0}};
        end else begin
            awready <= 1'b0;
            if (!aw_pending && awvalid) begin
                awaddr_reg <= awaddr;
                aw_pending <= 1'b1;
                awready <= 1'b1;
            end
        end
    end

    // W channel
    always @(posedge clk) begin
        if (!resetn) begin
            wready <= 1'b0; w_pending <= 1'b0; wdata_reg <= {DATA_WIDTH{1'b0}}; wstrb_reg <= {DATA_WIDTH/8{1'b0}};
        end else begin
            wready <= 1'b0;
            if (!w_pending && wvalid) begin
                wdata_reg <= wdata;
                wstrb_reg <= wstrb;
                w_pending <= 1'b1;
                wready <= 1'b1;
            end
        end
    end

    // Fire when both captured
    always @(posedge clk) begin
        if (!resetn) write_fire_r <= 1'b0;
        else write_fire_r <= (aw_pending && w_pending);
    end
    wire write_fire = write_fire_r;

    // Response channel (unchanged behavior)
    reg resp_pending;
    always @(posedge clk) begin
        if (!resetn) begin
            bvalid <= 1'b0; bresp <= 2'b00; resp_pending <= 1'b0;
        end else begin
            if (write_fire && !resp_pending) begin
                bvalid <= 1'b1; bresp <= 2'b00; resp_pending <= 1'b1;
                aw_pending <= 1'b0; w_pending <= 1'b0;
            end else if (resp_pending) begin
                if (bvalid && bready) begin bvalid <= 1'b0; resp_pending <= 1'b0; end
            end else bvalid <= 1'b0;
        end
    end

    // ------------------------------------------------------------
    // perf_start pulse
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) perf_start <= 1'b0;
        else begin
            perf_start <= 1'b0;
            if (write_fire && (awaddr_reg == 12'h070) && (wstrb_reg[0]) && (wdata_reg[0]))
                perf_start <= 1'b1;
        end
    end

    // ------------------------------------------------------------
    // BRAM byte-emission machinery
    // - When a write_fire hits A_BASE/B_BASE region, we start a
    //   short emission sequence that produces 0..4 single-cycle
    //   bram_we pulses (one per enabled byte lane) across next cycles.
    // - bram_waddr is a byte address; bram_wdata[7:0] carries the byte.
    // ------------------------------------------------------------
    reg bram_emit_active;
    reg [1:0] bram_emit_ptr;            // 0..3 index into bytes of the word
    reg [15:0] bram_emit_base_byteaddr; // base byte address (word_index*4)
    reg [3:0]  bram_emit_mask;          // which byte lanes to emit (from wstrb)
    reg [31:0] bram_emit_data;          // saved wdata

    // prepare emission on write_fire inside the register write block below

    // ------------------------------------------------------------
    // Register writes + start BRAM emission when applicable
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            // defaults
            mode_sel <= 2'b00; fuse_en <= 1'b0; act_sel <= 4'd0;
            bias <= 32'd0; scale <= 4'd1; shift <= 4'd0;
            gate_en <= 1'b0; threshold <= 8'd0; dyn_sched_en <= 1'b0;
            slice_en <= 1'b1; msb_stat_thres <= 4'd1; relu_en <= 1'b0;
            data_sel <= 1'b0;
            dma_ctrl <= 32'd0; dma_addr <= 32'd0; dma_len <= 32'd0;
            for (i=0; i<FLAT_WORDS; i=i+1) begin A_words[i] <= 32'd0; B_words[i] <= 32'd0; end

            // bram emission defaults
            bram_emit_active <= 1'b0;
            bram_emit_ptr <= 2'd0;
            bram_emit_base_byteaddr <= 16'd0;
            bram_emit_mask <= 4'd0;
            bram_emit_data <= 32'd0;
            bram_we <= 1'b0; bram_wdata <= 32'd0; bram_waddr <= 16'd0;
        end else begin
            // default: no bram write unless emission requests it this cycle
            bram_we <= 1'b0;
            bram_wdata <= 32'd0;
            bram_waddr <= 16'd0;

            // If emission is active, produce the next per-byte pulse
            if (bram_emit_active) begin
                if (bram_emit_ptr < 2'd4) begin
                    if (bram_emit_mask[bram_emit_ptr]) begin
                        // produce single-cycle byte write
                        bram_we <= 1'b1;
                        bram_waddr <= bram_emit_base_byteaddr + bram_emit_ptr;
                        bram_wdata <= bram_emit_data[8*bram_emit_ptr +: 8];
                    end
                    // advance pointer every cycle
                    bram_emit_ptr <= bram_emit_ptr + 1;
                    if (bram_emit_ptr == 2'd3) begin
                        // finished after this cycle
                        bram_emit_active <= 1'b0;
                    end
                end else begin
                    bram_emit_active <= 1'b0;
                end
            end

            // handle normal CPU-written registers ONCE when write_fire occurs
            if (write_fire) begin
                bresp <= 2'b00;
                case (awaddr_reg)
                    12'h000: if (wstrb_reg[0]) mode_sel <= wdata_reg[1:0];
                    12'h004: if (wstrb_reg[0]) fuse_en <= wdata_reg[0];
                    12'h008: if (wstrb_reg[0]) act_sel <= wdata_reg[3:0];
                    12'h00C: for (byte_i=0; byte_i<4; byte_i=byte_i+1)
                               if (wstrb_reg[byte_i]) bias[8*byte_i +: 8] <= wdata_reg[8*byte_i +: 8];
                    12'h010: if (wstrb_reg[0]) scale <= wdata_reg[3:0];
                    12'h014: if (wstrb_reg[0]) shift <= wdata_reg[3:0];
                    12'h028: if (wstrb_reg[0]) gate_en <= wdata_reg[0];
                    12'h02C: if (wstrb_reg[0]) threshold <= wdata_reg[7:0];
                    12'h030: if (wstrb_reg[0]) dyn_sched_en <= wdata_reg[0];
                    12'h034: if (wstrb_reg[0]) slice_en <= wdata_reg[0];
                    12'h038: if (wstrb_reg[0]) msb_stat_thres <= wdata_reg[3:0];
                    12'h03C: if (wstrb_reg[0]) relu_en <= wdata_reg[0];
                    12'h060: begin
                        dma_ctrl <= wdata_reg;
                        dma_ctrl[3] <= (mode_sel == 2'b01);
                    end
                    12'h064: dma_addr <= wdata_reg;
                    12'h068: dma_len  <= wdata_reg;
                    12'h070: begin /* perf_start handled above */ end
                    12'h080: if (wstrb_reg[0]) data_sel <= wdata_reg[0];
                    default: begin
                        // A/B flat region -> update word array AND start byte-wise BRAM emission
                        if (awaddr_reg >= A_BASE && awaddr_reg < (A_BASE + FLAT_WORDS*4)) begin
                            A_words[(awaddr_reg - A_BASE) >> 2] <= wdata_reg;

                            // prepare emission: base byte address = word_index * 4
                            bram_emit_active <= 1'b1;
                            bram_emit_ptr <= 2'd0;
                            bram_emit_base_byteaddr <= ((awaddr_reg - A_BASE) >> 2) * 4;
                            bram_emit_mask <= wstrb_reg;
                            bram_emit_data <= wdata_reg;
                        end else if (awaddr_reg >= B_BASE && awaddr_reg < (B_BASE + FLAT_WORDS*4)) begin
                            B_words[(awaddr_reg - B_BASE) >> 2] <= wdata_reg;

                            bram_emit_active <= 1'b1;
                            bram_emit_ptr <= 2'd0;
                            bram_emit_base_byteaddr <= ((awaddr_reg - B_BASE) >> 2) * 4;
                            bram_emit_mask <= wstrb_reg;
                            bram_emit_data <= wdata_reg;
                        end else begin
                            bresp <= 2'b10; // SLVERR
                        end
                    end
                endcase
            end
        end
    end

    // Compose A_flat/B_flat combinatorially from word arrays
    always @(*) begin
        A_flat = {WIDTH{1'b0}};
        B_flat = {WIDTH{1'b0}};
        for (i=0; i<FLAT_WORDS; i=i+1) begin
            A_flat[(i+1)*32-1 -: 32] = A_words[i];
            B_flat[(i+1)*32-1 -: 32] = B_words[i];
        end
    end

    // ------------------------------------------------------------
    // AXI-Lite read side (use araddr_reg for safety)
    // ------------------------------------------------------------
    reg ar_pending;
    reg [ADDR_WIDTH-1:0] araddr_reg;

    always @(posedge clk) begin
        if (!resetn) begin
            arready <= 1'b0; rvalid <= 1'b0; rdata <= {DATA_WIDTH{1'b0}}; rresp <= 2'b00; ar_pending <= 1'b0; araddr_reg <= {ADDR_WIDTH{1'b0}};
        end else begin
            arready <= 1'b0;
            if (!ar_pending && arvalid) begin
                araddr_reg <= araddr;
                ar_pending <= 1'b1;
                arready <= 1'b1;
                rvalid <= 1'b1;
                rresp <= 2'b00;
                case (araddr)
                    12'h000: rdata <= {30'd0, mode_sel};
                    12'h004: rdata <= {31'd0, fuse_en};
                    12'h008: rdata <= {28'd0, act_sel};
                    12'h00C: rdata <= bias;
                    12'h010: rdata <= {28'd0, scale};
                    12'h014: rdata <= {28'd0, shift};
                    12'h020: rdata <= mac_result;
                    12'h024: rdata <= {31'd0, gate_en};
                    12'h02C: rdata <= {24'd0, threshold};
                    12'h030: rdata <= {31'd0, dyn_sched_en};
                    12'h034: rdata <= {31'd0, slice_en};
                    12'h038: rdata <= {28'd0, msb_stat_thres};
                    12'h03C: rdata <= {31'd0, relu_en};
                    12'h044: rdata <= perf_cycles[31:0];
                    12'h048: rdata <= perf_cycles[63:32];
                    12'h04C: rdata <= perf_ops_acc[31:0];
                    12'h050: rdata <= perf_ops_acc[63:32];
                    12'h054: rdata <= perf_active_pe_sum;
                    12'h058: rdata <= perf_last_latency;
                    12'h060: rdata <= dma_ctrl;
                    12'h064: rdata <= dma_addr;
                    12'h068: rdata <= dma_len;
                    12'h06C: rdata <= dma_status;
                    12'h080: rdata <= {31'd0, data_sel};
                    default: begin
                        if (araddr >= A_BASE && araddr < (A_BASE + FLAT_WORDS*4))
                            rdata <= A_words[(araddr - A_BASE) >> 2];
                        else if (araddr >= B_BASE && araddr < (B_BASE + FLAT_WORDS*4))
                            rdata <= B_words[(araddr - B_BASE) >> 2];
                        else
                            rdata <= 32'hDEAD_BEEF;
                    end
                endcase
            end else begin
                arready <= 1'b0;
            end

            if (rvalid && rready) begin
                rvalid <= 1'b0;
                ar_pending <= 1'b0;
            end
        end
    end

endmodule
