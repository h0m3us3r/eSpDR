// 8N1 UART. DIVIDER = clock cycles per bit. A missing stop bit reports `error`.
module uart_rx #(
    parameter DIVIDER = 120
) (
    input  wire       clk,
    input  wire       reset,
    input  wire       rx,
    output reg  [7:0] data = 0,
    output reg        valid = 0,
    output reg        error = 0
);
    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_sync = 2'b11;
    reg [15:0] count = 0;
    reg [2:0]  bit_index = 0;
    reg [1:0]  state = IDLE;
    wire line = rx_sync[1];

    always @(posedge clk) begin
        rx_sync <= {rx_sync[0], rx};
        valid <= 0;
        error <= 0;
        if (reset) begin
            state <= IDLE;
            count <= 0;
        end else case (state)
            IDLE: if (!line) begin
                count <= DIVIDER / 2 - 1;
                state <= START;
            end
            START: if (count != 0) begin
                count <= count - 1'b1;
            end else if (!line) begin
                count <= DIVIDER - 1;
                bit_index <= 0;
                state <= DATA;
            end else begin
                state <= IDLE;              // glitch, not a start bit
            end
            DATA: if (count != 0) begin
                count <= count - 1'b1;
            end else begin
                data[bit_index] <= line;
                count <= DIVIDER - 1;
                if (bit_index == 7) state <= STOP;
                else bit_index <= bit_index + 1'b1;
            end
            STOP: if (count != 0) begin
                count <= count - 1'b1;
            end else begin
                valid <= line;
                error <= !line;
                state <= IDLE;
            end
        endcase
    end
endmodule

module uart_tx #(
    parameter DIVIDER = 120
) (
    input  wire       clk,
    input  wire       reset,
    input  wire       valid,
    input  wire [7:0] data,
    output wire       ready,
    output wire       tx
);
    reg [9:0]  shift = 10'h3FF;
    reg [15:0] count = 0;
    reg [3:0]  bits_left = 0;
    assign ready = bits_left == 0;
    assign tx = shift[0];

    always @(posedge clk) begin
        if (reset) begin
            bits_left <= 0;
            shift <= 10'h3FF;
            count <= 0;
        end else if (ready) begin
            if (valid) begin
                shift <= {1'b1, data, 1'b0};
                bits_left <= 10;
                count <= DIVIDER - 1;
            end
        end else if (count != 0) begin
            count <= count - 1'b1;
        end else begin
            shift <= {1'b1, shift[9:1]};
            bits_left <= bits_left - 1'b1;
            count <= DIVIDER - 1;
        end
    end
endmodule
