`timescale 1ps/1ps
// Snapshot handshake between unrelated clocks: the source counts every
// clock; each copy must be a count the source really had, copies never go
// backwards, and a fresh copy keeps arriving within a bounded delay.
module tb;
    parameter SOURCE_PS = 4167;         // source clock period
    parameter DESTINATION_PS = 8333;    // destination clock period
    localparam CLOCKS = 100000;         // destination clocks observed
    // A round trip is two synchronizer stages each way plus the updates.
    localparam MAX_GAP = 2 * ((SOURCE_PS + DESTINATION_PS - 1) / DESTINATION_PS) + 6;

    reg source_clk = 0, clk = 0;
    always #(SOURCE_PS / 2) source_clk = ~source_clk;
    // A slightly irregular destination clock sweeps every phase relationship.
    integer wobble = 0;
    always begin
        #(DESTINATION_PS / 2 + (wobble % 7)) clk = ~clk;
        wobble = wobble + 1;
    end

    reg [31:0] count = 0;
    always @(posedge source_clk) count <= count + 1;

    wire [31:0] value;
    snapshot dut (.source_clk(source_clk), .source_value(count), .clk(clk), .value(value));

    reg [31:0] previous = 0;
    integer copies = 0, since_change = 0, worst_gap = 0, worst_lag = 0;
    initial begin
        repeat (CLOCKS) begin
            @(posedge clk);
            #1;
            if (value > count) $fatal(1, "copy %0d is ahead of the source count %0d", value, count);
            if (value < previous) $fatal(1, "copy went backwards from %0d to %0d", previous, value);
            if (value != previous) begin
                copies = copies + 1;
                if (since_change > worst_gap) worst_gap = since_change;
                since_change = 0;
            end else begin
                since_change = since_change + 1;
            end
            if (count - value > worst_lag) worst_lag = count - value;
            previous = value;
        end
        if (worst_gap + 1 > MAX_GAP)
            $fatal(1, "copies up to %0d destination clocks apart (limit %0d)", worst_gap + 1, MAX_GAP);
        $display("PASS snapshot: %0d copies, at most %0d clocks apart, lag at most %0d source counts",
                 copies, worst_gap + 1, worst_lag);
        $finish;
    end
endmodule
