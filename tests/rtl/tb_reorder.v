`timescale 1ns/1ps
// Reorder: two lanes of interleaved units in, one chronological stream out.
// Unit ring positions are contiguous, as the ESP produces them. LOSE_EVERY
// drops every n-th unit entirely (header and pairs); the output must carry
// a skip of exactly that unit's length in its place. LOSE_RUN loses lane 0's
// units 20..59 (a lane going silent for a while). LOSE_BURST loses units
// 30..32 and 34: the next unit's lane stays silent while the other lane is
// several units ahead, and one skip covers three units. GARBLE announces some
// units with a misread sequence number (far ahead, a few ahead right after a
// delivered unit, or already passed): their pairs arrive but must be
// discarded, leaving a gap. GARBLE=2 additionally
// misreads lane 0's unit 48 far ahead while lane 1 is silent (units 41..55),
// so that no other lane can contradict it. OVERRUN=1 stops the
// consumer and checks that overflows are counted.
module tb #(parameter FIXED_LENGTH = 0, OVERRUN = 0, LOSE_EVERY = 0, LOSE_RUN = 0, LOSE_BURST = 0,
            GARBLE = 0);
    localparam UNITS = 96;
    reg clk = 0;
    always #4.166 clk = ~clk;

    reg reset = 1, out_ready = 0, skip_ready = 0;
    reg [1:0] valid = 0, last = 0, unit_valid = 0;
    reg [19:0] pair0 = 0, pair1 = 0;
    reg [31:0] sequence0 = 0, sequence1 = 1;
    reg [13:0] first0 = 0, first1 = 0, count0 = 0, count1 = 0;
    wire out_valid, skip_valid;
    wire [19:0] out_pair;
    wire [31:0] skip_pairs, overflow0, overflow1, lost_units, discarded_units;
    wire [14:0] peak0, peak1;
    reorder dut (
        .clk(clk), .reset(reset), .valid(valid), .last(last), .pair0(pair0), .pair1(pair1),
        .unit_valid(unit_valid), .unit_sequence0(sequence0), .unit_sequence1(sequence1),
        .unit_first0(first0), .unit_first1(first1), .unit_count0(count0), .unit_count1(count1),
        .out_valid(out_valid), .out_ready(out_ready), .out_pair(out_pair),
        .skip_valid(skip_valid), .skip_ready(skip_ready), .skip_pairs(skip_pairs),
        .overflow0(overflow0), .overflow1(overflow1), .lost_units(lost_units),
        .discarded_units(discarded_units), .peak0(peak0), .peak1(peak1));

    function integer unit_length(input integer unit);
        unit_length = FIXED_LENGTH ? FIXED_LENGTH : 15360 + (unit * 37) % 929;
    endfunction
    function integer unit_first(input integer unit);
        integer u;
        begin
            unit_first = 0;
            for (u = 0; u < unit; u = u + 1) unit_first = (unit_first + unit_length(u)) % 16384;
        end
    endfunction
    // Misread sequence numbers keep the lane's parity, as the lane checks it.
    function integer announced(input integer unit);
        announced = !GARBLE ? unit : unit % 22 == 5 ? unit + 2000 : (unit % 22 == 16 && unit >= 20) ? unit - 20
                  : (GARBLE == 1 && unit % 22 == 10) ? unit + 2 : (GARBLE == 1 && unit % 22 == 13) ? unit + 4
                  : (GARBLE == 2 && unit == 48) ? unit + 2000 : unit;
    endfunction
    function garbled(input integer unit);
        garbled = announced(unit) != unit;
    endfunction
    function lost(input integer unit);
        if (GARBLE == 2 && unit % 2 == 1 && unit >= 41 && unit <= 55) lost = 1; else
        lost = (LOSE_EVERY != 0 && unit % LOSE_EVERY == LOSE_EVERY - 1) ||
               (LOSE_RUN != 0 && unit % 2 == 0 && unit >= 20 && unit < 60) ||
               (LOSE_BURST != 0 && ((unit >= 30 && unit <= 32) || unit == 34));
    endfunction
    function [19:0] pair_value(input integer unit, input integer at);
        pair_value = 20'((unit * 53871) ^ (at * 273));
    endfunction

    integer unit0 = 0, unit1 = 1, at0 = 0, at1 = 0, cycle = 0;
    integer expected_unit = 0, expected_at = 0, pairs = 0, skips = 0, garbles = 0, u;
    integer skipped;
    initial begin
        for (u = 0; u < UNITS - 2; u = u + 1) if (garbled(u) && !lost(u)) garbles = garbles + 1;
        repeat (10) @(negedge clk);
        reset = 0;
        wait (expected_unit >= UNITS - 2);
        repeat (20) @(negedge clk);
        if (overflow0 || overflow1) $fatal(1, "overflows %0d %0d", overflow0, overflow1);
        if ((LOSE_EVERY || LOSE_RUN || LOSE_BURST || GARBLE) && lost_units != skips)
            $fatal(1, "lost_units %0d, skips %0d", lost_units, skips);
        if (GARBLE && discarded_units != garbles) $fatal(1, "discarded %0d of %0d", discarded_units, garbles);
        $display("PASS reorder: %0d units, %0d pairs, %0d lost units skipped, peak %0d/%0d",
                 expected_unit, pairs, skips, peak0, peak1);
        $finish;
    end

    // Lanes deliver 32 of every 91 cycles' worth of pairs, like the link; a
    // lane announces a unit on its first pair's cycle.
    always @(negedge clk) if (!reset) begin
        cycle = cycle + 1;
        if (OVERRUN && cycle == 110000) begin
            if (overflow0 == 0 || overflow1 == 0) $fatal(1, "overflow not counted");
            $display("PASS reorder overflow counted: %0d/%0d", overflow0, overflow1);
            $finish;
        end
        out_ready = !OVERRUN && cycle % 211 < 207 && cycle % 19000 < 18900;
        skip_ready = cycle % 5 == 0;
        valid[0] = unit0 < UNITS && (cycle * 2) % 91 < 32;
        valid[1] = unit1 < UNITS && ((cycle + 7) * 2) % 91 < 32;
        unit_valid[0] = valid[0] && at0 == 0 && !lost(unit0);
        unit_valid[1] = valid[1] && at1 == 0 && !lost(unit1);
        sequence0 = announced(unit0); first0 = unit_first(unit0); count0 = unit_length(unit0);
        sequence1 = announced(unit1); first1 = unit_first(unit1); count1 = unit_length(unit1);
        pair0 = pair_value(unit0, at0);
        pair1 = pair_value(unit1, at1);
        last[0] = at0 + 1 == unit_length(unit0);
        last[1] = at1 + 1 == unit_length(unit1);
        if (lost(unit0)) valid[0] = 0;  // the lane never received it...
        if (lost(unit1)) valid[1] = 0;
    end

    always @(posedge clk) if (!reset) begin
        // ...but its time passes all the same.
        if ((cycle * 2) % 91 < 32 && unit0 < UNITS) begin
            if (at0 + 1 == unit_length(unit0)) begin unit0 = unit0 + 2; at0 = 0; end else at0 = at0 + 1;
        end
        if (((cycle + 7) * 2) % 91 < 32 && unit1 < UNITS) begin
            if (at1 + 1 == unit_length(unit1)) begin unit1 = unit1 + 2; at1 = 0; end else at1 = at1 + 1;
        end
        // A skip covers one or more whole units that never arrived intact.
        if (skip_valid && skip_ready) begin
            if (expected_at != 0) $fatal(1, "skip inside unit %0d", expected_unit);
            skipped = 0;
            while (skipped < skip_pairs) begin
                if (!(lost(expected_unit) || garbled(expected_unit)))
                    $fatal(1, "skip of %0d over unit %0d", skip_pairs, expected_unit);
                skipped = skipped + unit_length(expected_unit);
                skips = skips + 1;
                expected_unit = expected_unit + 1;
            end
            if (skipped != skip_pairs) $fatal(1, "skip of %0d, units hold %0d", skip_pairs, skipped);
        end
        if (out_valid && out_ready) begin
            if (lost(expected_unit) || garbled(expected_unit) ||
                out_pair !== pair_value(expected_unit, expected_at))
                $fatal(1, "order at unit %0d pair %0d", expected_unit, expected_at);
            pairs = pairs + 1;
            if (expected_at + 1 == unit_length(expected_unit)) begin
                expected_unit = expected_unit + 1;
                expected_at = 0;
            end else begin
                expected_at = expected_at + 1;
            end
        end
    end

    initial begin #100000000; $fatal(1, "timeout at unit %0d", expected_unit); end
endmodule
