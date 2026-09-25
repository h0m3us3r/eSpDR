`timescale 1ns/1ps
// Continuous, consistent copies of a multi-bit value in another clock domain.
//
// A toggle handshake: the destination asks for a copy, the source loads
// `held` and answers, and the destination takes `held` only after the answer
// has crossed back, then asks again. `held` therefore never changes while it
// is being taken, so every copy is a value the source really had, never a
// mixture of two. A new copy arrives every few clocks of the slower domain.
module snapshot #(
    parameter WIDTH = 32
) (
    input  wire             source_clk,
    input  wire [WIDTH-1:0] source_value,
    input  wire             clk,
    output reg  [WIDTH-1:0] value = 0
);
    reg [WIDTH-1:0] held = 0;           // source domain
    reg request = 0;                    // destination domain
    reg acknowledge = 0;                // source domain: equals request once held is loaded
    (* ASYNC_REG = "TRUE" *) reg request_meta = 0, request_stable = 0;
    (* ASYNC_REG = "TRUE" *) reg acknowledge_meta = 0, acknowledge_stable = 0;

    always @(posedge source_clk) begin
        request_meta <= request;
        request_stable <= request_meta;
        if (request_stable != acknowledge) begin
            held <= source_value;
            acknowledge <= request_stable;
        end
    end

    always @(posedge clk) begin
        acknowledge_meta <= acknowledge;
        acknowledge_stable <= acknowledge_meta;
        if (acknowledge_stable == request) begin
            value <= held;
            request <= !request;
        end
    end
endmodule
