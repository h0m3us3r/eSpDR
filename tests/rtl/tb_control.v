`timescale 1ns/1ps
`include "protocol.vh"
// Control port over a real UART waveform: valid requests, a truncated
// request followed by a valid one, a corrupt request and back-to-back requests.
module tb;
    localparam DIVIDER = 8, TIMEOUT = 400;
    reg clk = 0;
    always #4 clk = ~clk;

    reg reset = 1, rx = 1;
    wire tx, request_valid;
    wire [7:0] op;
    wire [15:0] arg;
    // The owner answers with fields derived from the request.
    wire [7:0] status = op ^ 8'h5A;
    wire [31:0] value = {arg, 8'hC3, op};
    control_port #(.DIVIDER(DIVIDER), .BYTE_TIMEOUT(TIMEOUT), .SETTLE(20)) dut (
        .clk(clk), .reset(reset), .rx(rx), .tx(tx), .request_valid(request_valid), .op(op),
        .arg(arg), .status(status), .value(value));

    function automatic [31:0] crc_byte(input [31:0] c, input [7:0] b);
        reg [31:0] v;
        integer j;
        begin
            v = c ^ b;
            for (j = 0; j < 8; j = j + 1) v = v[0] ? (v >> 1) ^ 32'hEDB88320 : v >> 1;
            crc_byte = v;
        end
    endfunction

    task send_byte(input [7:0] b);
        integer i;
        begin
            rx = 0; repeat (DIVIDER) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin rx = b[i]; repeat (DIVIDER) @(posedge clk); end
            rx = 1; repeat (DIVIDER) @(posedge clk);
        end
    endtask

    reg [79:0] request;
    task send_request(input [7:0] request_op, input [15:0] request_arg, input [15:0] number,
                      input integer bytes, input corrupt);
        reg [31:0] c;
        integer i;
        begin
            request = {32'b0, number, request_arg, request_op, 8'(`CTL_REQUEST_MAGIC)};
            c = 32'hFFFFFFFF;
            for (i = 0; i < 6; i = i + 1) c = crc_byte(c, request[i * 8 +: 8]);
            request[79:48] = ~c ^ (corrupt ? 32'h1 : 32'h0);
            for (i = 0; i < bytes; i = i + 1) send_byte(request[i * 8 +: 8]);
        end
    endtask

    // Response receiver.
    reg [7:0] response [0:15];
    integer received = 0, i;
    task receive_byte(output [7:0] b);
        integer k;
        begin
            @(negedge tx);
            repeat (DIVIDER + DIVIDER / 2) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin b[k] = tx; repeat (DIVIDER) @(posedge clk); end
        end
    endtask
    task expect_response(input [7:0] request_op, input [15:0] request_arg, input [15:0] number);
        reg [31:0] c;
        begin
            for (i = 0; i < 16; i = i + 1) receive_byte(response[i]);
            c = 32'hFFFFFFFF;
            for (i = 0; i < 12; i = i + 1) c = crc_byte(c, response[i]);
            if (response[0] != `CTL_RESPONSE_MAGIC || response[1] != `CTL_NODE_FPGA ||
                response[2] != request_op || response[3] != (request_op ^ 8'h5A) ||
                {response[5], response[4]} != number ||
                {response[11], response[10], response[9], response[8]} != {request_arg, 8'hC3, request_op} ||
                {response[15], response[14], response[13], response[12]} != ~c)
                $fatal(1, "bad response to op %0d sequence %0d", request_op, number);
            received = received + 1;
        end
    endtask

    integer requests = 0;
    always @(posedge clk) if (request_valid) requests = requests + 1;

    initial begin
        repeat (10) @(posedge clk);
        reset = 0;
        repeat (10) @(posedge clk);

        fork send_request(1, 16'h1234, 1, 10, 0); expect_response(1, 16'h1234, 1); join

        // Truncated request, then silence past the timeout, then a good one.
        send_request(3, 16'hAAAA, 2, 4, 0);
        repeat (TIMEOUT + 50) @(posedge clk);
        fork send_request(3, 16'h0005, 3, 10, 0); expect_response(3, 16'h0005, 3); join

        // Corrupt request: no response and no action; the next one works.
        send_request(17, 0, 4, 10, 1);
        fork send_request(18, 7, 5, 10, 0); expect_response(18, 7, 5); join

        // Back to back.
        fork
            begin send_request(2, 0, 6, 10, 0); send_request(16, 1, 7, 10, 0); end
            begin expect_response(2, 0, 6); expect_response(16, 1, 7); end
        join

        if (requests != 5 || received != 5) $fatal(1, "requests=%0d responses=%0d", requests, received);
        $display("PASS control port: %0d requests answered, truncated and corrupt requests handled", received);
        $finish;
    end

    initial begin #20000000; $fatal(1, "timeout"); end
endmodule
