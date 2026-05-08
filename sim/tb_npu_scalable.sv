// =============================================================================
//  tb_npu_scalable.sv – Main Parameterized Testbench for npu_top
//
//  Drives the AXI4-Full slave interface to configure the NPU, fire a batch,
//  and verify the output.  Parameterized: change PE_COUNT to test 512/1024.
// =============================================================================
`timescale 1ns/1ps

module tb_npu_scalable;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer PE_COUNT  = 256;   // change for 512/1024/2048
    localparam integer AXI_AW    = 16;
    localparam integer AXI_DW    = 32;
    localparam integer AXI_IDW   = 4;
    localparam integer DWIDTH    = 8;
    localparam integer ACC_WIDTH = 32;

    // =========================================================================
    // Clock / Reset
    // =========================================================================
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz (sim)

    reg resetn_async;

    // =========================================================================
    // AXI signals
    // =========================================================================
    reg  [AXI_IDW-1:0]  m_awid;
    reg  [AXI_AW-1:0]   m_awaddr;
    reg  [7:0]           m_awlen;
    reg  [2:0]           m_awsize;
    reg  [1:0]           m_awburst;
    reg                  m_awvalid;
    wire                 m_awready;

    reg  [AXI_DW-1:0]   m_wdata;
    reg  [AXI_DW/8-1:0] m_wstrb;
    reg                  m_wlast;
    reg                  m_wvalid;
    wire                 m_wready;

    wire [AXI_IDW-1:0]  m_bid;
    wire [1:0]           m_bresp;
    wire                 m_bvalid;
    reg                  m_bready;

    reg  [AXI_IDW-1:0]  m_arid;
    reg  [AXI_AW-1:0]   m_araddr;
    reg  [7:0]           m_arlen;
    reg  [2:0]           m_arsize;
    reg  [1:0]           m_arburst;
    reg                  m_arvalid;
    wire                 m_arready;

    wire [AXI_IDW-1:0]  m_rid;
    wire [AXI_DW-1:0]   m_rdata;
    wire [1:0]           m_rresp;
    wire                 m_rlast;
    wire                 m_rvalid;
    reg                  m_rready;

    wire                 irq_out;

    // =========================================================================
    // DUT
    // =========================================================================
    npu_top #(
        .PE_COUNT   (PE_COUNT),
        .DWIDTH     (DWIDTH),
        .ACC_WIDTH  (ACC_WIDTH),
        .BRAM_DEPTH (4096),
        .BUS_WIDTH  (64),
        .AXI_AW     (AXI_AW),
        .AXI_DW     (AXI_DW),
        .AXI_IDW    (AXI_IDW)
    ) dut (
        .clk         (clk),
        .resetn_async(resetn_async),
        .s_awid      (m_awid),    .s_awaddr  (m_awaddr),
        .s_awlen     (m_awlen),   .s_awsize  (m_awsize),
        .s_awburst   (m_awburst), .s_awvalid (m_awvalid), .s_awready (m_awready),
        .s_wdata     (m_wdata),   .s_wstrb   (m_wstrb),
        .s_wlast     (m_wlast),   .s_wvalid  (m_wvalid),  .s_wready  (m_wready),
        .s_bid       (m_bid),     .s_bresp   (m_bresp),
        .s_bvalid    (m_bvalid),  .s_bready  (m_bready),
        .s_arid      (m_arid),    .s_araddr  (m_araddr),
        .s_arlen     (m_arlen),   .s_arsize  (m_arsize),
        .s_arburst   (m_arburst), .s_arvalid (m_arvalid), .s_arready (m_arready),
        .s_rid       (m_rid),     .s_rdata   (m_rdata),
        .s_rresp     (m_rresp),   .s_rlast   (m_rlast),
        .s_rvalid    (m_rvalid),  .s_rready  (m_rready),
        .irq_out     (irq_out)
    );

    // =========================================================================
    // AXI Master Tasks
    // =========================================================================
    task axi_write;
        input [AXI_AW-1:0] addr;
        input [AXI_DW-1:0] data;
        integer timeout;
    begin
        // AW channel
        m_awid    = 4'd0;
        m_awaddr  = addr;
        m_awlen   = 8'd0;
        m_awsize  = 3'd2;    // 4 bytes
        m_awburst = 2'b01;   // INCR
        m_awvalid = 1'b1;
        timeout   = 0;
        @(posedge clk);
        while (!m_awready && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        m_awvalid = 1'b0;

        // W channel
        m_wdata  = data;
        m_wstrb  = 4'hF;
        m_wlast  = 1'b1;
        m_wvalid = 1'b1;
        timeout  = 0;
        @(posedge clk);
        while (!m_wready && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        m_wvalid = 1'b0;
        m_wlast  = 1'b0;

        // B channel
        m_bready = 1'b1;
        timeout  = 0;
        @(posedge clk);
        while (!m_bvalid && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        m_bready = 1'b0;
        @(posedge clk);
    end
    endtask

    task axi_read;
        input  [AXI_AW-1:0] addr;
        output [AXI_DW-1:0] data;
        integer timeout;
    begin
        m_arid    = 4'd0;
        m_araddr  = addr;
        m_arlen   = 8'd0;
        m_arsize  = 3'd2;
        m_arburst = 2'b01;
        m_arvalid = 1'b1;
        timeout   = 0;
        @(posedge clk);
        while (!m_arready && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        m_arvalid = 1'b0;

        m_rready = 1'b1;
        timeout  = 0;
        @(posedge clk);
        while (!m_rvalid && timeout < 20) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        data     = m_rdata;
        m_rready = 1'b0;
        @(posedge clk);
    end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    reg [AXI_DW-1:0] rd_val;
    integer i;
    integer timeout_main;

    initial begin
        $display("=== tb_npu_scalable: PE_COUNT=%0d ===", PE_COUNT);

        // Init AXI signals
        m_awid = 0; m_awaddr = 0; m_awlen = 0; m_awsize = 0;
        m_awburst = 0; m_awvalid = 0;
        m_wdata = 0; m_wstrb = 0; m_wlast = 0; m_wvalid = 0;
        m_bready = 0;
        m_arid = 0; m_araddr = 0; m_arlen = 0; m_arsize = 0;
        m_arburst = 0; m_arvalid = 0;
        m_rready = 0;

        // Reset
        resetn_async = 0;
        repeat (20) @(posedge clk);
        resetn_async = 1;
        repeat (10) @(posedge clk);

        // ===========================
        // Test 1: Read version / ID
        // ===========================
        $display("--- Test 1: Config register read/write ---");
        // Write mode_sel = INT8
        axi_write(16'h0000, 32'h0);   // mode_sel = 00 (INT8)
        axi_read (16'h0000, rd_val);
        if (rd_val[1:0] == 2'b00)
            $display("PASS: mode_sel read back as INT8 (%0h)", rd_val);
        else
            $display("FAIL: mode_sel expected 0, got %0h", rd_val);

        // Write scale = 1, shift = 0
        axi_write(16'h0010, 32'h1);   // scale = 1
        axi_write(16'h0014, 32'h0);   // shift = 0

        // ===========================
        // Test 2: CSR mode – write A/B flat, trigger gate_en, read result
        // ===========================
        $display("--- Test 2: CSR mode direct compute (A=5, B=3, all PEs) ---");
        // Enable data_sel (CSR mode)
        axi_write(16'h0030, 32'h1);   // data_sel = 1
        axi_write(16'h0018, 32'h0);   // gate_en = 0 initially

        // Write A/B flat: all elements = 5 and 3 respectively
        for (i = 0; i < (PE_COUNT * 8 / 32); i = i + 1) begin
            // A_flat at 0x0300 + i*4
            axi_write(16'h0300 + i*4, 32'h05050505);
            // B_flat at 0x1000 + i*4
            axi_write(16'h1000 + i*4, 32'h03030303);
        end

        // Enable slice_en off, gate_en
        axi_write(16'h0024, 32'h0);   // slice_en = 0
        axi_write(16'h0018, 32'h1);   // gate_en = 1

        // Let pipeline drain
        repeat (50) @(posedge clk);
        axi_write(16'h0018, 32'h0);   // gate_en = 0

        // Read MAC result
        axi_read(16'h0038, rd_val);
        $display("MAC result after CSR-mode compute: %0d (0x%0h)", $signed(rd_val), rd_val);

        // ===========================
        // Test 3: Performance counter
        // ===========================
        $display("--- Test 3: Performance counters ---");
        axi_write(16'h0100, 32'h2);  // pm_clear
        axi_write(16'h0100, 32'h1);  // pm_start
        repeat (100) @(posedge clk);
        axi_read (16'h0104, rd_val);
        if (rd_val > 0)
            $display("PASS: Perf cycle counter = %0d", rd_val);
        else
            $display("FAIL: Perf cycle counter = 0");

        // ===========================
        // Test 4: Status register
        // ===========================
        $display("--- Test 4: Status register ---");
        axi_read(16'h0200, rd_val);
        $display("NPU status = 0x%0h", rd_val);

        // ===========================
        // Test 5: IRQ enable / status
        // ===========================
        $display("--- Test 5: IRQ configuration ---");
        axi_write(16'h0208, 32'h7F);  // enable all IRQs
        axi_read (16'h0208, rd_val);
        if (rd_val[6:0] == 7'h7F)
            $display("PASS: IRQ enable mask = 0x%0h", rd_val[6:0]);
        else
            $display("FAIL: IRQ enable mask expected 0x7F, got 0x%0h", rd_val[6:0]);

        $display("\n=== tb_npu_scalable: all tests complete ===\n");
        $finish;
    end

endmodule
