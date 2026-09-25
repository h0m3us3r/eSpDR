// 40 MHz reference for the ESP32-S3's crystal input. High impedance until
// enabled; start and stop it only while the ESP is held in reset.
module reference_output (
    input  wire clock40,
    input  wire locked,
    input  wire enable,                 // any clock domain
    output wire running,                // clock40 domain
    inout  wire pad
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] enable_sync = 0;
    reg output_enable = 0;
    always @(posedge clock40 or negedge locked)
        if (!locked) begin
            enable_sync <= 0;
            output_enable <= 0;
        end else begin
            enable_sync <= {enable_sync[0], enable};
            output_enable <= enable_sync[1];
        end

    wire forwarded;
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) forward (
        .C(clock40), .CE(1'b1), .D1(1'b0), .D2(1'b1), .R(1'b0), .S(1'b0), .Q(forwarded));
    OBUFT output_buffer (.I(forwarded), .T(!output_enable), .O(pad));
    assign running = output_enable;
endmodule
