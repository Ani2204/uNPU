`timescale 1ns / 1ps
(* keep_hierarchy = "yes", dont_touch = "yes" *)
module dma_bram #(
  parameter ELEM_BITS = 8,
  parameter DEPTH = 4096
)(
  input  wire         clk,
  input  wire         resetn,

  input  wire [31:0]  dma_ctrl,   // [0]=start, [1]=clear, [2]=B-stream, [3]=INT4-mode
  input  wire [31:0]  dma_addr,   // byte address
  input  wire [31:0]  dma_len,    // bytes (INT8) or num nibbles (INT4)

  output reg  [31:0]  dma_status, // [1]=busy, [0]=done, [31]=error

  // CPU preload (from CSR) - byte writes or nibble writes depending on mode
  input  wire [7:0]  bram_wdata,
  input  wire         bram_we,
  input  wire [15:0]  bram_waddr,

  // stream (A or B)
  output reg          a_valid,
  output reg [7:0]    a_data,
  input  wire         a_ready,

  output reg          b_valid,
  output reg [7:0]    b_data,
  input  wire         b_ready
);

  // ============================================================
  // Internal BRAM
  // ============================================================
  (* ram_style="block" *)
  reg [7:0] mem [0:DEPTH-1];

  // ============================================================
  // DMA State
  // ============================================================
  reg [31:0] ptr, remaining;
  reg a_nib_toggle, b_nib_toggle;

  localparam IDLE = 2'b00, BUSY = 2'b01, DONE = 2'b10;
  reg [1:0] st;

  // latched control
  reg mode_int4;      
  reg stream_B;      
  reg [31:0] dma_len_latched;
  reg [31:0] dma_addr_latched;

  // nibble sign-extend
  function [7:0] sx4;
    input [3:0] v;
    begin sx4 = {{4{v[3]}}, v}; end
  endfunction

  integer clamp_space;

  // ============================================================
  // MAIN FSM
  // ============================================================
  always @(posedge clk) begin
    if (!resetn) begin
      st <= IDLE;
      dma_status <= 32'd0;
      ptr <= 32'd0;
      remaining <= 32'd0;

      a_valid <= 1'b0;
      b_valid <= 1'b0;

      a_data <= 8'd0;
      b_data <= 8'd0;

      a_nib_toggle <= 1'b0;
      b_nib_toggle <= 1'b0;

      mode_int4 <= 1'b0;
      stream_B  <= 1'b0;

      dma_len_latched <= 0;
      dma_addr_latched <= 0;
    end

    else begin
      // ================================
      // CLEAR
      // ================================
      if (dma_ctrl[1]) begin
        st <= IDLE;
        dma_status <= 32'd0;
        a_valid <= 1'b0;
        b_valid <= 1'b0;
        a_nib_toggle <= 1'b0;
        b_nib_toggle <= 1'b0;
        mode_int4 <= 1'b0;
        stream_B <= 1'b0;
        dma_len_latched <= 0;
        dma_addr_latched <= 0;
      end

      case (st)

        // ==========================================
        // IDLE
        // ==========================================
        IDLE: begin
          a_valid <= 1'b0;
          b_valid <= 1'b0;

          if (dma_ctrl[0]) begin  // start
            mode_int4 <= dma_ctrl[3];
            stream_B  <= dma_ctrl[2];

            dma_len_latched  <= dma_len;
            dma_addr_latched <= dma_addr;

            // bounds checking
            if (dma_addr >= DEPTH) begin
              remaining <= 0;
              dma_status <= 32'h80000000;  // error
              ptr <= dma_addr;
              st <= DONE;
            end
            else begin
              clamp_space = DEPTH - dma_addr;

              if (!dma_ctrl[3]) begin         // INT8
                remaining <= (dma_len > clamp_space) ? clamp_space : dma_len;
              end else begin                  // INT4
                remaining <= (dma_len > (clamp_space<<1)) ? (clamp_space<<1) : dma_len;
              end

              ptr <= dma_addr;
              st <= BUSY;
              dma_status <= 32'd1; // busy

              a_nib_toggle <= 1'b0;
              b_nib_toggle <= 1'b0;
            end
          end
        end

        // ==========================================
        // BUSY
        // ==========================================
        BUSY: begin
          dma_status <= 32'd1;  // keep busy

          if (!stream_B) begin
            // ---------------- A stream ----------------
            if (!a_valid || (a_valid && a_ready)) begin
              if (remaining != 0) begin
                if (mode_int4) begin
                  // nibble by nibble
                  if (!a_nib_toggle) begin
                    a_data <= sx4(mem[ptr][3:0]);
                    a_valid <= 1'b1;
                    a_nib_toggle <= 1'b1;
                    remaining <= remaining - 1;
                  end else begin
                    a_data <= sx4(mem[ptr][7:4]);
                    a_valid <= 1'b1;
                    a_nib_toggle <= 1'b0;
                    ptr <= ptr + 1;
                    remaining <= remaining - 1;
                  end
                end
                else begin
                  // INT8
                  a_data <= mem[ptr];
                  a_valid <= 1'b1;
                  ptr <= ptr + 1;
                  remaining <= remaining - 1;
                end
              end else begin
                a_valid <= 1'b0;
                dma_status <= 32'd2; // done
                st <= DONE;
              end
            end

          end else begin
            // ---------------- B stream ----------------
            if (!b_valid || (b_valid && b_ready)) begin
              if (remaining != 0) begin
                if (mode_int4) begin
                  if (!b_nib_toggle) begin
                    b_data <= sx4(mem[ptr][3:0]);
                    b_valid <= 1'b1;
                    b_nib_toggle <= 1'b1;
                    remaining <= remaining - 1;
                  end else begin
                    b_data <= sx4(mem[ptr][7:4]);
                    b_valid <= 1'b1;
                    b_nib_toggle <= 1'b0;
                    ptr <= ptr + 1;
                    remaining <= remaining - 1;
                  end
                end
                else begin
                  b_data <= mem[ptr];
                  b_valid <= 1'b1;
                  ptr <= ptr + 1;
                  remaining <= remaining - 1;
                end
              end else begin
                b_valid <= 1'b0;
                dma_status <= 32'd2; // done
                st <= DONE;
              end
            end
          end
        end

        // ==========================================
        // DONE
        // ==========================================
        DONE: begin
          if (!dma_ctrl[0]) begin
            st <= IDLE;
            dma_status <= 32'd0;
          end
        end
      endcase
    end
  end

  // ============================================================
  // CPU BRAM preloads (byte writes + nibble writes)
  // ============================================================
  reg [15:0] tgt;

  always @(posedge clk) begin
    if (!resetn) begin
      // nothing to reset inside mem
    end else begin
      if (bram_we) begin

        // If DMA is streaming, any CPU write is an ERROR
        if (st == BUSY) begin
          dma_status <= dma_status | 32'h80000000;
        end

        else begin
          if (mode_int4) begin
            // nibble-write: bram_waddr[0] selects nibble
            tgt = bram_waddr >> 1;

            if (tgt < DEPTH) begin
              if (bram_waddr[0] == 1'b0)
                mem[tgt] <= { mem[tgt][7:4], bram_wdata[3:0] };
              else
                mem[tgt] <= { bram_wdata[3:0], mem[tgt][3:0] };
            end else begin
              dma_status <= dma_status | 32'h80000000;
            end

          end else begin
            // BYTE write
            if (bram_waddr < DEPTH)
              mem[bram_waddr] <= bram_wdata[7:0];
            else
              dma_status <= dma_status | 32'h80000000;
          end
        end

      end
    end
  end

endmodule
