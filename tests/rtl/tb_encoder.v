`timescale 1ns/1ps
// Encoder: feeds a pair file with input bubbles and output back-pressure and
// writes the emitted records; codec_test check compares them to the reference.
module tb;
    reg clk = 0;
    always #4.166 clk = ~clk;

    reg reset = 1, in_valid = 0, flush = 0, out_ready = 0;
    reg [19:0] in_pair = 0;
    wire in_ready, out_valid, out_last;
    wire [63:0] out_data, accepted;
    wire [3:0] out_bytes;
    wire [31:0] records;
    encoder dut (
        .clk(clk), .reset(reset), .in_valid(in_valid), .in_ready(in_ready), .in_pair(in_pair),
        .flush(flush), .out_valid(out_valid), .out_ready(out_ready), .out_data(out_data),
        .out_bytes(out_bytes), .out_last(out_last), .accepted(accepted), .records(records));

    integer in_file, out_file, n, j, tick = 0, sent = 0;
    reg [31:0] word;
    reg [1023:0] in_path, out_path;
    reg [63:0] held_data;
    reg [3:0] held_bytes;
    reg held_last, holding = 0;

    initial begin
        if (!$value$plusargs("input=%s", in_path) || !$value$plusargs("output=%s", out_path)) $fatal;
        in_file = $fopen(in_path, "rb");
        out_file = $fopen(out_path, "wb");
        repeat (10) @(negedge clk);
        reset = 0;
        while (!$feof(in_file)) begin
            n = $fread(word, in_file);
            if (n == 4) begin
                in_pair = {word[11:8], word[23:16], word[31:24]};   // file is little-endian
                in_valid = 1;
                @(posedge clk);
                while (!in_ready) @(posedge clk);
                sent = sent + 1;
                @(negedge clk);
                in_valid = 0;
                if (sent % 2 == 0) @(negedge clk);   // 80 Msps average in 120 MHz bursts
            end
        end
        flush = 1;
        repeat (20000) @(negedge clk);
        if (accepted != sent || records != (sent + 1023) / 1024)
            $fatal(1, "accepted=%0d sent=%0d records=%0d", accepted, sent, records);
        $display("PASS encoder: %0d pairs, %0d records", sent, records);
        $fclose(out_file);
        $finish;
    end

    always @(negedge clk) begin
        tick = tick + 1;
        out_ready = (tick % 101 < 89) && (tick % 8192 < 8000);
    end

    always @(posedge clk) if (!reset) begin
        if (holding && (!out_valid || out_data !== held_data || out_bytes !== held_bytes || out_last !== held_last))
            $fatal(1, "output changed while stalled");
        holding = out_valid && !out_ready;
        held_data = out_data;
        held_bytes = out_bytes;
        held_last = out_last;
        if (out_valid && out_ready) begin
            if (out_bytes == 0 || out_bytes > 8 || out_bytes[0]) $fatal(1, "byte count");
            for (j = 0; j < out_bytes; j = j + 1) $fwrite(out_file, "%c", out_data[j * 8 +: 8]);
        end
    end

    initial begin #100000000; $fatal(1, "timeout"); end
endmodule
