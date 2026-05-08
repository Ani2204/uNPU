// =============================================================================
//  tb_dma.sv – DMA Engine Testbench
//
//  Tests dma_bram (legacy, now with block RAM), dma_controller, and
//  dma_scheduler.
// =============================================================================
`timescale 1ns/1ps

module tb_dma;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer BRAM_DEPTH = 256;   // small for fast test
    localparam integer BUS_WIDTH  = 64;
    localparam integer DWIDTH     = 8;

    // =========================================================================
    // Clock / Reset
    // =========================================================================
    reg clk = 0;
    always #5 clk = ~clk;

    reg resetn;

    // =========================================================================
    // dma_bram DUT (legacy module with fixed BRAM pragma)
    // =========================================================================
    reg  [31:0]  dma_ctrl_lb;
    reg  [31:0]  dma_addr_lb;
    reg  [31:0]  dma_len_lb;
    wire [31:0]  dma_status_lb;

    reg  [7:0]   bram_wdata_lb;
    reg          bram_we_lb;
    reg  [15:0]  bram_waddr_lb;

    wire         a_valid_lb, b_valid_lb;
    wire [7:0]   a_data_lb, b_data_lb;
    reg          a_ready_lb = 1, b_ready_lb = 1;

    dma_bram #(.ELEM_BITS(8), .DEPTH(BRAM_DEPTH)) u_dma_bram (
        .clk        (clk), .resetn     (resetn),
        .dma_ctrl   (dma_ctrl_lb), .dma_addr (dma_addr_lb), .dma_len  (dma_len_lb),
        .dma_status (dma_status_lb),
        .bram_wdata (bram_wdata_lb), .bram_we (bram_we_lb), .bram_waddr (bram_waddr_lb),
        .a_valid    (a_valid_lb), .a_data (a_data_lb), .a_ready (a_ready_lb),
        .b_valid    (b_valid_lb), .b_data (b_data_lb), .b_ready (b_ready_lb)
    );

    // =========================================================================
    // dma_controller DUT
    // =========================================================================
    localparam integer ADDR_W = $clog2(BRAM_DEPTH);

    reg  [15:0]           dc_src_addr;
    reg  [15:0]           dc_byte_len;
    reg  [1:0]            dc_mode;
    reg                   dc_start;
    wire                  dc_done, dc_error;
    wire [ADDR_W-1:0]     dc_mem_rd_addr;
    wire                  dc_mem_rd_en;
    reg  [BUS_WIDTH-1:0]  dc_mem_rd_data;
    reg                   dc_mem_rd_valid;
    wire [DWIDTH-1:0]     dc_out_data;
    wire                  dc_out_valid;
    reg                   dc_out_ready = 1;
    wire [31:0]           dc_bytes_xfr;

    dma_controller #(
        .BRAM_DEPTH (BRAM_DEPTH),
        .BUS_WIDTH  (BUS_WIDTH),
        .DWIDTH     (DWIDTH)
    ) u_dma_ctrl (
        .clk          (clk), .resetn    (resetn),
        .src_addr     (dc_src_addr), .byte_len (dc_byte_len), .mode (dc_mode),
        .dma_start    (dc_start), .dma_done (dc_done), .dma_error (dc_error),
        .mem_rd_addr  (dc_mem_rd_addr), .mem_rd_en (dc_mem_rd_en),
        .mem_rd_data  (dc_mem_rd_data), .mem_rd_valid (dc_mem_rd_valid),
        .pf_rd_ptr    (), .pf_active (), .pf_data ({BUS_WIDTH{1'b0}}),
        .pf_valid     (1'b0), .pf_consume (),
        .out_data     (dc_out_data), .out_valid (dc_out_valid), .out_ready (dc_out_ready),
        .bytes_transferred (dc_bytes_xfr)
    );

    // =========================================================================
    // Simple memory model for dma_controller
    // =========================================================================
    reg [BUS_WIDTH-1:0] model_mem [0:BRAM_DEPTH/8-1];
    integer mi;

    always @(posedge clk) begin
        dc_mem_rd_valid <= 1'b0;
        if (dc_mem_rd_en) begin
            dc_mem_rd_data  <= model_mem[dc_mem_rd_addr];
            dc_mem_rd_valid <= 1'b1;
        end
    end

    // =========================================================================
    // Tests
    // =========================================================================
    integer byte_count;
    reg [7:0] received [0:15];
    integer t, timeout;

    task preload_bram;
        input integer base_addr;
        input integer count;
        integer k;
    begin
        for (k = 0; k < count; k = k + 1) begin
            bram_waddr_lb = base_addr + k;
            bram_wdata_lb = k + 1;   // pattern: 1,2,3,...
            bram_we_lb    = 1'b1;
            @(posedge clk);
        end
        bram_we_lb = 1'b0;
        @(posedge clk);
    end
    endtask

    initial begin
        resetn        = 0;
        dma_ctrl_lb   = 32'd0;
        dma_addr_lb   = 32'd0;
        dma_len_lb    = 32'd0;
        bram_we_lb    = 1'b0;
        bram_waddr_lb = 16'd0;
        bram_wdata_lb = 8'd0;
        dc_start      = 1'b0;
        dc_src_addr   = 16'd0;
        dc_byte_len   = 16'd0;
        dc_mode       = 2'b00;
        dc_mem_rd_valid = 1'b0;
        dc_mem_rd_data  = {BUS_WIDTH{1'b0}};

        // Initialise model memory
        for (mi = 0; mi < BRAM_DEPTH/8; mi = mi + 1)
            model_mem[mi] = {8{8'(mi)}}; // each word = byte_index repeated

        repeat (10) @(posedge clk);
        resetn = 1;
        repeat (5)  @(posedge clk);

        // ----- Test 1: dma_bram preload + stream INT8 -----
        $display("--- Test: dma_bram preload + INT8 stream ---");
        preload_bram(0, 8);

        // Start DMA (A stream, INT8, 8 bytes from addr 0)
        dma_addr_lb = 32'd0;
        dma_len_lb  = 32'd8;
        dma_ctrl_lb = 32'h1;  // start, INT8
        @(posedge clk);
        dma_ctrl_lb = 32'd0;

        byte_count = 0;
        timeout    = 0;
        while (byte_count < 8 && timeout < 200) begin
            @(posedge clk);
            if (a_valid_lb && a_ready_lb) begin
                received[byte_count] = a_data_lb;
                byte_count = byte_count + 1;
            end
            timeout = timeout + 1;
        end

        if (byte_count == 8 && received[0] == 8'd1 && received[7] == 8'd8)
            $display("PASS: dma_bram INT8 stream, %0d bytes received, first=%0d last=%0d",
                     byte_count, received[0], received[7]);
        else
            $display("FAIL: dma_bram INT8 count=%0d first=%0d last=%0d",
                     byte_count, received[0], received[7]);

        // Wait for done
        timeout = 0;
        while (dma_status_lb[1:0] != 2'b10 && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        if (dma_status_lb[1:0] == 2'b10)
            $display("PASS: dma_bram done flag set");
        else
            $display("FAIL: dma_bram done not set in time");

        // ----- Test 2: dma_controller basic transfer -----
        $display("--- Test: dma_controller basic transfer ---");
        dc_src_addr = 16'd0;
        dc_byte_len = 16'd8;
        dc_mode     = 2'b00;
        dc_start    = 1'b1;
        @(posedge clk);
        dc_start    = 1'b0;

        byte_count = 0;
        timeout    = 0;
        while (!dc_done && timeout < 500) begin
            @(posedge clk);
            if (dc_out_valid && dc_out_ready)
                byte_count = byte_count + 1;
            timeout = timeout + 1;
        end

        if (dc_done)
            $display("PASS: dma_controller done in %0d cycles, bytes=%0d", timeout, byte_count);
        else
            $display("FAIL: dma_controller did not assert done");

        $display("\ntb_dma: all tests done.");
        $finish;
    end

endmodule
