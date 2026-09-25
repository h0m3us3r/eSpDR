// Dual-clock FIFO with Gray-coded pointers.
//
// A write while full is dropped and counted in `overflow`; the FIFO never
// back-pressures its writer. Each payload word is written once before the
// synchronized write pointer makes it visible to the reader.
//
// SHOW_AHEAD = 0: rd_enable requests a word; rd_valid pulses with rd_data
//                 on the following clock.
// SHOW_AHEAD = 1: rd_valid/rd_data present the head word and hold it until
//                 rd_enable pops it. With RAM_STYLE "block" the block RAM's
//                 own output register holds the head.
module async_fifo #(
    parameter WIDTH = 32,
    parameter ADDR_BITS = 4,
    parameter RAM_STYLE = "distributed",
    parameter SHOW_AHEAD = 0
) (
    input  wire                 wr_clk,
    input  wire                 wr_reset,
    input  wire                 wr_valid,
    input  wire [WIDTH-1:0]     wr_data,
    output wire                 full,
    output reg  [31:0]          overflow = 0,
    output wire [ADDR_BITS:0]   wr_level,

    input  wire                 rd_clk,
    input  wire                 rd_reset,
    input  wire                 rd_enable,
    output reg                  rd_valid = 0,
    output reg  [WIDTH-1:0]     rd_data = 0,
    output wire                 empty,
    output wire [ADDR_BITS:0]   rd_level
);
    (* ram_style = RAM_STYLE *) reg [WIDTH-1:0] memory [0:(1 << ADDR_BITS) - 1];

    // Binary and Gray pointers, each owned by its clock domain, plus
    // two-stage synchronizers of the other domain's Gray pointer.
    reg [ADDR_BITS:0] wr_bin = 0, wr_gray = 0;
    reg [ADDR_BITS:0] rd_bin = 0, rd_gray = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] rd_gray_sync1 = 0, rd_gray_sync2 = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] wr_gray_sync1 = 0, wr_gray_sync2 = 0;

    wire [ADDR_BITS:0] wr_bin_next = wr_bin + 1'b1;
    wire [ADDR_BITS:0] rd_bin_next = rd_bin + 1'b1;

    wire [ADDR_BITS:0] rd_bin_in_wr_domain;
    wire [ADDR_BITS:0] wr_bin_in_rd_domain;
    genvar i;
    generate
        for (i = 0; i <= ADDR_BITS; i = i + 1) begin : gray_to_binary
            assign rd_bin_in_wr_domain[i] = ^rd_gray_sync2[ADDR_BITS:i];
            assign wr_bin_in_rd_domain[i] = ^wr_gray_sync2[ADDR_BITS:i];
        end
    endgenerate

    assign wr_level = wr_bin - rd_bin_in_wr_domain;
    assign rd_level = wr_bin_in_rd_domain - rd_bin + ((SHOW_AHEAD && rd_valid) ? 1'b1 : 1'b0);
    assign full = wr_gray == {~rd_gray_sync2[ADDR_BITS:ADDR_BITS-1], rd_gray_sync2[ADDR_BITS-2:0]};
    assign empty = rd_gray == wr_gray_sync2;

    always @(posedge wr_clk) begin
        if (wr_reset) begin
            wr_bin <= 0;
            wr_gray <= 0;
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
            overflow <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
            if (wr_valid) begin
                if (full) begin
                    overflow <= overflow + 1;
                end else begin
                    memory[wr_bin[ADDR_BITS-1:0]] <= wr_data;
                    wr_bin <= wr_bin_next;
                    wr_gray <= (wr_bin_next >> 1) ^ wr_bin_next;
                end
            end
        end
    end

    wire read_advance = SHOW_AHEAD ? (!rd_valid || rd_enable) : rd_enable;

    always @(posedge rd_clk) begin
        if (rd_reset) begin
            rd_bin <= 0;
            rd_gray <= 0;
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
            rd_valid <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
            if (!SHOW_AHEAD || read_advance)
                rd_valid <= read_advance && !empty;
            if (read_advance && !empty) begin
                rd_data <= memory[rd_bin[ADDR_BITS-1:0]];
                rd_bin <= rd_bin_next;
                rd_gray <= (rd_bin_next >> 1) ^ rd_bin_next;
            end
        end
    end
endmodule
