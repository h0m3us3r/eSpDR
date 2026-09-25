`include "protocol.vh"
// Control protocol endpoint (protocol/control.h) over a UART.
//
// Reception runs independently of responding and holds one complete request
// until the previous response has been sent. Each valid request is presented
// for one clock on request_valid with its op and arg. The owner acts on it
// and holds `status` and `value` for the reply; they are sampled SETTLE
// clocks later, then the response is sent. Requests
// with a bad CRC get no response. A request whose bytes stop arriving for
// BYTE_TIMEOUT clocks is abandoned, so a truncated request cannot swallow the
// start of the next one.
module control_port #(
    parameter DIVIDER = 120,            // clocks per UART bit
    parameter BYTE_TIMEOUT = 120000,    // clocks
    parameter SETTLE = 400,             // clocks from request to sampling status/value
    parameter NODE = `CTL_NODE_FPGA
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        rx,
    output wire        tx,

    output reg         request_valid = 0,
    output reg  [7:0]  op = 0,
    output reg  [15:0] arg = 0,
    input  wire [7:0]  status,
    input  wire [31:0] value
);
    function automatic [31:0] crc_byte(input [31:0] c, input [7:0] b);
        reg [31:0] v;
        integer j;
        begin
            v = c ^ b;
            for (j = 0; j < 8; j = j + 1)
                v = v[0] ? (v >> 1) ^ 32'hEDB88320 : v >> 1;
            crc_byte = v;
        end
    endfunction

    wire [7:0] rx_data;
    wire       rx_valid, rx_error, tx_ready;
    reg        tx_valid = 0;
    reg  [7:0] tx_data = 0;
    uart_rx #(.DIVIDER(DIVIDER)) receiver (
        .clk(clk), .reset(reset), .rx(rx), .data(rx_data), .valid(rx_valid), .error(rx_error));
    uart_tx #(.DIVIDER(DIVIDER)) transmitter (
        .clk(clk), .reset(reset), .valid(tx_valid), .data(tx_data), .ready(tx_ready), .tx(tx));

    // ---- reception: always running, holds one complete request ----
    reg [79:0] request = 0;
    reg [3:0]  received = 0;
    reg [31:0] request_crc = 0;
    reg [16:0] idle = 0;                // clocks since the last request byte
    reg        complete = 0;            // all bytes in, CRC not yet checked
    reg        pending = 0;             // a checked request awaits the responder
    reg [7:0]  pending_op = 0;
    reg [15:0] pending_arg = 0, pending_sequence = 0;
    wire       take;

    always @(posedge clk) begin
        complete <= 0;
        if (reset) begin
            received <= 0;
            idle <= 0;
            pending <= 0;
        end else begin
            if (rx_valid || received == 0) idle <= 0;
            else if (idle != BYTE_TIMEOUT) idle <= idle + 1'b1;

            if (rx_error) begin
                received <= 0;
            end else if (rx_valid && (received != 0 || rx_data == `CTL_REQUEST_MAGIC)) begin
                request[received * 8 +: 8] <= rx_data;
                if (received < 6)
                    request_crc <= crc_byte(received == 0 ? 32'hFFFFFFFF : request_crc, rx_data);
                if (received == `CTL_REQUEST_BYTES - 1) begin
                    received <= 0;
                    complete <= 1;
                end else begin
                    received <= received + 1'b1;
                end
            end else if (idle == BYTE_TIMEOUT) begin
                received <= 0;          // the rest of this request never came
            end

            if (take) pending <= 0;
            if (complete && request[79:48] == ~request_crc) begin   // bad CRC: ignored
                pending <= 1;
                pending_op <= request[15:8];
                pending_arg <= request[31:16];
                pending_sequence <= request[47:32];
            end
        end
    end

    // ---- execution and response ----
    localparam IDLE = 0, SETTLING = 1, RESPOND = 2, SEND = 3;
    reg [1:0]  state = IDLE;
    reg [15:0] sequence_number = 0;
    reg [15:0] settle = 0;
    reg [95:0] response = 0;            // bytes 0..11; the CRC follows
    reg [31:0] response_crc = 0;
    reg [3:0]  sent = 0;
    assign take = state == IDLE && pending;

    always @(posedge clk) begin
        tx_valid <= 0;
        request_valid <= 0;
        if (reset) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE: if (pending) begin
                    op <= pending_op;
                    arg <= pending_arg;
                    sequence_number <= pending_sequence;
                    request_valid <= 1;
                    settle <= 0;
                    state <= SETTLING;
                end

                SETTLING: if (settle == SETTLE) state <= RESPOND;
                          else settle <= settle + 1'b1;

                RESPOND: begin
                    response <= {value, 16'b0, sequence_number, status, op, 8'(NODE),
                                 8'(`CTL_RESPONSE_MAGIC)};
                    response_crc <= 32'hFFFFFFFF;
                    sent <= 0;
                    state <= SEND;
                end

                SEND: if (tx_ready && !tx_valid) begin
                    tx_valid <= 1;
                    if (sent < 12) begin
                        tx_data <= response[sent * 8 +: 8];
                        response_crc <= crc_byte(response_crc, response[sent * 8 +: 8]);
                    end else begin
                        tx_data <= 8'(~response_crc >> ((sent - 12) * 8));
                    end
                    if (sent == `CTL_RESPONSE_BYTES - 1) state <= IDLE;
                    else sent <= sent + 1'b1;
                end
            endcase
        end
    end
endmodule
