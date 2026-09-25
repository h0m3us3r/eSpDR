`timescale 1ns/1ps
// Link receiver: replays a lane waveform (a rising- and a falling-edge sample
// per 240 MHz cycle, four per 120 MHz clock) through link_lane and link_unpack
// and checks every recovered pair, unit boundary and error counter. +units and +framing give
// the numbers of units to accept and headers to reject.
module tb;
    reg clk = 0;
    always #4.166 clk = ~clk;

    reg reset = 1, pair_valid = 0;
    reg [31:0] samples = 0;
    wire byte_valid, unpacked_valid, unpacked_last, unit_valid;
    wire [13:0] unit_first, unit_count;
    wire [7:0] byte_value;
    wire [15:0] byte_index;
    wire [31:0] sequence_word, units, framing_errors, checksum_errors, end_marker_errors, unpack_errors;
    wire [3:0] head_pairs, tail_pairs;
    wire [10:0] total_groups;
    wire [19:0] unpacked;
    link_lane #(.LANE(0)) lane (
        .clk(clk), .reset(reset), .run(1'b1), .pair_valid(pair_valid), .samples(samples),
        .byte_valid(byte_valid), .byte_value(byte_value), .byte_index(byte_index),
        .sequence_word(sequence_word), .unit_valid(unit_valid), .unit_first(unit_first),
        .unit_count(unit_count), .head_pairs(head_pairs), .tail_pairs(tail_pairs),
        .total_groups(total_groups), .units(units), .framing_errors(framing_errors),
        .checksum_errors(checksum_errors), .end_marker_errors(end_marker_errors));
    link_unpack unpack (
        .clk(clk), .reset(reset), .byte_valid(byte_valid), .byte_value(byte_value),
        .byte_index(byte_index), .head_pairs(head_pairs), .tail_pairs(tail_pairs),
        .total_groups(total_groups), .pair_valid(unpacked_valid), .pair(unpacked),
        .last(unpacked_last), .errors(unpack_errors));

    integer wave_file, pairs_file, early, early_fall, late, late_fall, n, received = 0, lasts = 0, announced = 0;
    integer UNITS, FRAMING;
    reg [31:0] word;
    reg [1023:0] wave_path, pairs_path;
    initial begin
        if (!$value$plusargs("wave=%s", wave_path) || !$value$plusargs("pairs=%s", pairs_path) ||
            !$value$plusargs("units=%d", UNITS) || !$value$plusargs("framing=%d", FRAMING)) $fatal;
        wave_file = $fopen(wave_path, "rb");
        pairs_file = $fopen(pairs_path, "rb");
        repeat (10) @(negedge clk);
        reset = 0;
        forever begin
            @(negedge clk);
            early = $fgetc(wave_file);
            early_fall = $fgetc(wave_file);
            late = $fgetc(wave_file);
            late_fall = $fgetc(wave_file);
            if (early < 0 || early_fall < 0 || late < 0 || late_fall < 0) begin
                pair_valid = 0;
                repeat (100) @(negedge clk);
                n = $fread(word, pairs_file);     // must be at the end
                if (units != UNITS || lasts != UNITS || announced != UNITS || framing_errors != FRAMING ||
                    checksum_errors || end_marker_errors || unpack_errors || n != 0)
                    $fatal(1, "units=%0d lasts=%0d announced=%0d errors=%0d/%0d/%0d/%0d", units, lasts,
                           announced, framing_errors, checksum_errors, end_marker_errors, unpack_errors);
                $display("PASS link: %0d units, %0d pairs, %0d headers rejected", units, received,
                         framing_errors);
                $finish;
            end
            samples = {late_fall[7:0], late[7:0], early_fall[7:0], early[7:0]};
            pair_valid = 1;
        end
    end

    always @(posedge clk) if (unit_valid) announced = announced + 1;

    always @(posedge clk) if (unpacked_valid) begin
        n = $fread(word, pairs_file);
        if (n != 4 || unpacked !== {word[11:8], word[23:16], word[31:24]})
            $fatal(1, "pair %0d: got %h", received, unpacked);
        received = received + 1;
        if (unpacked_last) lasts = lasts + 1;
    end
endmodule
