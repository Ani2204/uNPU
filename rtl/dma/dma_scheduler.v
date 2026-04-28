// =============================================================================
//  dma_scheduler.v – DMA Job Queue (8-entry FIFO)
//
//  Software writes DMA descriptors via the CSR interface.
//  The DMA controller pops jobs from this queue.
// =============================================================================
`timescale 1ns/1ps

module dma_scheduler #(
    parameter integer FIFO_DEPTH = 8   // number of outstanding DMA jobs
)(
    input  wire        clk,
    input  wire        resetn,

    // Push interface (from CSR)
    input  wire [95:0] push_job,    // {dst_addr[31:0], src_addr[31:0], len[15:0], mode[1:0], 14'b0}
    input  wire        push_valid,
    output wire        push_ready,  // 0 = queue full

    // Pop interface (to DMA controller)
    output wire [95:0] pop_job,
    output wire        pop_valid,
    input  wire        pop_ready,

    // Status
    output wire [$clog2(FIFO_DEPTH):0] occupancy
);

    localparam integer AW = $clog2(FIFO_DEPTH);

    reg [95:0] fifo [0:FIFO_DEPTH-1];
    reg [AW:0] wr_ptr, rd_ptr;

    wire empty = (wr_ptr == rd_ptr);
    wire full  = ((wr_ptr[AW-1:0] == rd_ptr[AW-1:0]) &&
                  (wr_ptr[AW]     != rd_ptr[AW]));

    assign push_ready = ~full;
    assign pop_valid  = ~empty;
    assign pop_job    = fifo[rd_ptr[AW-1:0]];
    assign occupancy  = wr_ptr - rd_ptr;

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr <= {(AW+1){1'b0}};
            rd_ptr <= {(AW+1){1'b0}};
        end else begin
            if (push_valid && push_ready) begin
                fifo[wr_ptr[AW-1:0]] <= push_job;
                wr_ptr <= wr_ptr + 1;
            end
            if (pop_valid && pop_ready) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule
