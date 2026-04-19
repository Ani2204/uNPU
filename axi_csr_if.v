`timescale 1ns/1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
//
// axi_csr_if.v -- AXI-Lite control/status register interface (improvement #5/#6/#10)
//
// Improvements vs prior version:
//   - REMOVED the 2048-bit A_words/B_words flat arrays and A_flat/B_flat outputs.
//     DMA (dma_bram) is now the sole operand data path; the CPU no longer needs
//     128 AXI transactions just to fill both matrices.
//   - SPLIT bram_we/wdata/waddr into independent A and B ports
//     (bram_we_a/b, bram_wdata_a/b, bram_waddr_a/b), matching dma_bram.v.
//   - dma_ctrl bit [2] = INT4-mode (was bit [3]; bit [2] was stream_B which is
//     now removed since both A and B always stream concurrently).
//   - Added result_vec input and CSR read addresses 0x300..0x3FC for per-row
//     results (improvement #10 read-back).
//   - Added systolic_en / acc_clear CSR outputs (improvement #4 control).
//
module axi_csr_if #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter ROWS       = 16     // number of per-row results to expose
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
    output reg  [1:0]                bresp,
    output reg                       bvalid,
    input  wire                      bready,

    // AXI-lite read
    input  wire [ADDR_WIDTH-1:0]     araddr,
    input  wire                      arvalid,
    output reg                       arready,
    output reg  [DATA_WIDTH-1:0]     rdata,
    output reg  [1:0]                rresp,
    output reg                       rvalid,
    input  wire                      rready,

    // CSR outputs
    output reg  [1:0]                mode_sel,
    output reg  signed [31:0]        bias,
    output reg  [3:0]                scale,
    output reg  [3:0]                shift,
    output reg                       fuse_en,
    output reg  [3:0]                act_sel,
    output reg                       gate_en,
    output reg                       relu_en,
    output reg                       systolic_en,   // new: systolic mode
    output reg                       acc_clear,     // new: synchronous accumulator reset

    // perf
    output reg                       perf_start,
    input  wire                      perf_done,
    input  wire [63:0]               perf_cycles,
    input  wire [63:0]               perf_ops_acc,
    input  wire [31:0]               perf_active_pe_sum,
    input  wire [31:0]               perf_last_latency,

    // DMA control
    output reg  [31:0]               dma_ctrl,   // [0]=start, [1]=clear, [2]=INT4
    output reg  [31:0]               dma_addr,
    output reg  [31:0]               dma_len,
    input  wire [31:0]               dma_status,

    output reg  [7:0]                threshold,
    output reg                       dyn_sched_en,
    output reg                       slice_en,
    output reg  [3:0]                msb_stat_thres,

    // read-only MAC results
    input  wire signed [31:0]        mac_result,        // global sum (backward compat)
    input  wire [ROWS*32-1:0]        result_vec,        // per-row results (new)

    // BRAM write port for A memory (CPU → dma_bram.mem_a)
    output reg [7:0]                 bram_wdata_a,
    output reg                       bram_we_a,
    output reg [15:0]                bram_waddr_a,

    // BRAM write port for B memory (CPU → dma_bram.mem_b)
    output reg [7:0]                 bram_wdata_b,
    output reg                       bram_we_b,
    output reg [15:0]                bram_waddr_b
);

    localparam integer A_BASE = 12'h100;  // byte address of A BRAM region
    localparam integer B_BASE = 12'h200;  // byte address of B BRAM region

    integer i, byte_i;

    // ----------------------------------------------------------------
    // AXI-Lite write handshake
    // ----------------------------------------------------------------
    reg                     aw_pending;
    reg [ADDR_WIDTH-1:0]    awaddr_reg;
    reg                     w_pending;
    reg [DATA_WIDTH-1:0]    wdata_reg;
    reg [DATA_WIDTH/8-1:0]  wstrb_reg;
    reg                     write_fire_r;

    always @(posedge clk) begin
        if (!resetn) begin
            awready <= 1'b0; aw_pending <= 1'b0;
            awaddr_reg <= {ADDR_WIDTH{1'b0}};
        end else begin
            awready <= 1'b0;
            if (!aw_pending && awvalid) begin
                awaddr_reg <= awaddr;
                aw_pending <= 1'b1;
                awready    <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            wready <= 1'b0; w_pending <= 1'b0;
            wdata_reg <= {DATA_WIDTH{1'b0}};
            wstrb_reg <= {DATA_WIDTH/8{1'b0}};
        end else begin
            wready <= 1'b0;
            if (!w_pending && wvalid) begin
                wdata_reg <= wdata;
                wstrb_reg <= wstrb;
                w_pending <= 1'b1;
                wready    <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn) write_fire_r <= 1'b0;
        else         write_fire_r <= (aw_pending && w_pending);
    end
    wire write_fire = write_fire_r;

    reg resp_pending;
    always @(posedge clk) begin
        if (!resetn) begin
            bvalid <= 1'b0; bresp <= 2'b00; resp_pending <= 1'b0;
        end else begin
            if (write_fire && !resp_pending) begin
                bvalid       <= 1'b1;
                bresp        <= 2'b00;
                resp_pending <= 1'b1;
                aw_pending   <= 1'b0;
                w_pending    <= 1'b0;
            end else if (resp_pending) begin
                if (bvalid && bready) begin
                    bvalid       <= 1'b0;
                    resp_pending <= 1'b0;
                end
            end else bvalid <= 1'b0;
        end
    end

    // ----------------------------------------------------------------
    // perf_start one-cycle pulse
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) perf_start <= 1'b0;
        else begin
            perf_start <= 1'b0;
            if (write_fire && (awaddr_reg == 12'h070) &&
                wstrb_reg[0] && wdata_reg[0])
                perf_start <= 1'b1;
        end
    end

    // ----------------------------------------------------------------
    // BRAM byte-emission machinery
    // When a write_fire targets A_BASE or B_BASE, we serialise the
    // 32-bit wdata into individual byte writes (one per active wstrb
    // lane).  A and B each have independent emission channels.
    // ----------------------------------------------------------------
    reg        a_emit_active;
    reg [1:0]  a_emit_ptr;
    reg [15:0] a_emit_base;
    reg [3:0]  a_emit_mask;
    reg [31:0] a_emit_data;

    reg        b_emit_active;
    reg [1:0]  b_emit_ptr;
    reg [15:0] b_emit_base;
    reg [3:0]  b_emit_mask;
    reg [31:0] b_emit_data;

    // ----------------------------------------------------------------
    // Register writes
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            mode_sel       <= 2'b00;   fuse_en       <= 1'b0;
            act_sel        <= 4'd0;    bias          <= 32'd0;
            scale          <= 4'd1;    shift         <= 4'd0;
            gate_en        <= 1'b0;    relu_en       <= 1'b0;
            threshold      <= 8'd0;    dyn_sched_en  <= 1'b0;
            slice_en       <= 1'b1;    msb_stat_thres<= 4'd1;
            systolic_en    <= 1'b0;    acc_clear     <= 1'b0;
            dma_ctrl       <= 32'd0;   dma_addr      <= 32'd0;
            dma_len        <= 32'd0;

            a_emit_active  <= 1'b0;    b_emit_active  <= 1'b0;
            a_emit_ptr     <= 2'd0;    b_emit_ptr     <= 2'd0;
            a_emit_base    <= 16'd0;   b_emit_base    <= 16'd0;
            a_emit_mask    <= 4'd0;    b_emit_mask    <= 4'd0;
            a_emit_data    <= 32'd0;   b_emit_data    <= 32'd0;
            bram_we_a      <= 1'b0;    bram_we_b      <= 1'b0;
            bram_wdata_a   <= 8'd0;    bram_wdata_b   <= 8'd0;
            bram_waddr_a   <= 16'd0;   bram_waddr_b   <= 16'd0;
        end else begin
            // Default: no BRAM write pulse
            bram_we_a    <= 1'b0;
            bram_we_b    <= 1'b0;
            bram_wdata_a <= 8'd0;
            bram_wdata_b <= 8'd0;
            bram_waddr_a <= 16'd0;
            bram_waddr_b <= 16'd0;

            // acc_clear is a single-cycle strobe
            acc_clear <= 1'b0;

            // --- A BRAM emission ---
            if (a_emit_active) begin
                if (a_emit_mask[a_emit_ptr]) begin
                    bram_we_a    <= 1'b1;
                    bram_waddr_a <= a_emit_base + {14'd0, a_emit_ptr};
                    bram_wdata_a <= a_emit_data[8*a_emit_ptr +: 8];
                end
                a_emit_ptr <= a_emit_ptr + 1;
                if (a_emit_ptr == 2'd3)
                    a_emit_active <= 1'b0;
            end

            // --- B BRAM emission ---
            if (b_emit_active) begin
                if (b_emit_mask[b_emit_ptr]) begin
                    bram_we_b    <= 1'b1;
                    bram_waddr_b <= b_emit_base + {14'd0, b_emit_ptr};
                    bram_wdata_b <= b_emit_data[8*b_emit_ptr +: 8];
                end
                b_emit_ptr <= b_emit_ptr + 1;
                if (b_emit_ptr == 2'd3)
                    b_emit_active <= 1'b0;
            end

            // --- Process register writes ---
            if (write_fire) begin
                bresp <= 2'b00;
                case (awaddr_reg)
                    12'h000: if (wstrb_reg[0]) mode_sel     <= wdata_reg[1:0];
                    12'h004: if (wstrb_reg[0]) fuse_en      <= wdata_reg[0];
                    12'h008: if (wstrb_reg[0]) act_sel      <= wdata_reg[3:0];
                    12'h00C: for (byte_i=0;byte_i<4;byte_i=byte_i+1)
                                 if (wstrb_reg[byte_i])
                                     bias[8*byte_i +: 8] <= wdata_reg[8*byte_i +: 8];
                    12'h010: if (wstrb_reg[0]) scale        <= wdata_reg[3:0];
                    12'h014: if (wstrb_reg[0]) shift        <= wdata_reg[3:0];
                    12'h028: if (wstrb_reg[0]) gate_en      <= wdata_reg[0];
                    12'h02C: if (wstrb_reg[0]) threshold    <= wdata_reg[7:0];
                    12'h030: if (wstrb_reg[0]) dyn_sched_en <= wdata_reg[0];
                    12'h034: if (wstrb_reg[0]) slice_en     <= wdata_reg[0];
                    12'h038: if (wstrb_reg[0]) msb_stat_thres <= wdata_reg[3:0];
                    12'h03C: if (wstrb_reg[0]) relu_en      <= wdata_reg[0];
                    12'h040: if (wstrb_reg[0]) systolic_en  <= wdata_reg[0];
                    12'h044: if (wstrb_reg[0]) acc_clear    <= wdata_reg[0]; // one-cycle pulse
                    12'h060: begin
                        // dma_ctrl: pass start/clear bits; inject INT4 from mode_sel
                        dma_ctrl     <= wdata_reg;
                        dma_ctrl[2]  <= (mode_sel == 2'b01);  // INT4 on bit [2]
                    end
                    12'h064: dma_addr <= wdata_reg;
                    12'h068: dma_len  <= wdata_reg;
                    12'h070: begin /* perf_start handled separately */ end
                    default: begin
                        // A BRAM region: 0x100..0x100+FLAT_WORDS*4-1
                        if (awaddr_reg >= A_BASE &&
                            awaddr_reg < (A_BASE + 12'd256)) begin
                            a_emit_active <= 1'b1;
                            a_emit_ptr    <= 2'd0;
                            a_emit_base   <= {4'd0, awaddr_reg[11:0]} - 16'd256;
                            a_emit_mask   <= wstrb_reg;
                            a_emit_data   <= wdata_reg;
                        // B BRAM region: 0x200..0x200+FLAT_WORDS*4-1
                        end else if (awaddr_reg >= B_BASE &&
                                     awaddr_reg < (B_BASE + 12'd256)) begin
                            b_emit_active <= 1'b1;
                            b_emit_ptr    <= 2'd0;
                            b_emit_base   <= {4'd0, awaddr_reg[11:0]} - 16'd512;
                            b_emit_mask   <= wstrb_reg;
                            b_emit_data   <= wdata_reg;
                        end else begin
                            bresp <= 2'b10; // SLVERR for unknown address
                        end
                    end
                endcase
            end
        end
    end

    // ----------------------------------------------------------------
    // AXI-Lite read side
    // ----------------------------------------------------------------
    reg ar_pending;
    reg [ADDR_WIDTH-1:0] araddr_reg;

    always @(posedge clk) begin
        if (!resetn) begin
            arready    <= 1'b0;  rvalid <= 1'b0;
            rdata      <= {DATA_WIDTH{1'b0}};
            rresp      <= 2'b00; ar_pending <= 1'b0;
            araddr_reg <= {ADDR_WIDTH{1'b0}};
        end else begin
            arready <= 1'b0;
            if (!ar_pending && arvalid) begin
                araddr_reg <= araddr;
                ar_pending <= 1'b1;
                arready    <= 1'b1;
                rvalid     <= 1'b1;
                rresp      <= 2'b00;
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
                    12'h040: rdata <= {31'd0, systolic_en};
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
                    default: begin
                        // Per-row result readback at 0x300..0x300+(ROWS-1)*4
                        if (araddr >= 12'h300 &&
                            araddr < (12'h300 + ROWS*4)) begin
                            rdata <= result_vec[((araddr - 12'h300) >> 2)*32 +: 32];
                        end else begin
                            rdata <= 32'hDEAD_BEEF;
                        end
                    end
                endcase
            end else begin
                arready <= 1'b0;
            end

            if (rvalid && rready) begin
                rvalid     <= 1'b0;
                ar_pending <= 1'b0;
            end
        end
    end

endmodule
