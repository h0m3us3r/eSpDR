`timescale 1ns/1ps
// FT600 transmitter with TXE changing at the latest specified time after
// each FT edge; every accepted word must be the next word in sequence.
module tb;
    reg ft_clk = 0;
    always #5 ft_clk = ~ft_clk;
    wire control, launch, write_launch, work;
    assign #0.375 control = ft_clk;
    assign #4.75 launch = ft_clk;
    assign #0.875 write_launch = ~ft_clk;
    assign #4.75 work = ~ft_clk;

    reg reset = 1, valid = 0, txe = 1;
    reg [127:0] data = 0;
    wire pop, wr, data_clock_enable;
    wire [15:0] pads;
    wire [1:0] be;
    wire [31:0] underruns, max_stall;
    usb_tx #(.WORDS(2048)) dut (
        .control(control), .launch(launch), .data_launch(launch), .write_launch(write_launch),
        .work(work), .reset(reset), .valid(valid), .data(data), .available(10'd300), .pop(pop),
        .ft_txe(txe), .data_clock_enable(data_clock_enable), .ft_data(pads), .ft_be(be),
        .ft_wr(wr), .underruns(underruns), .max_stall(max_stall));

    integer produced = 0, consumed = 0, tick = 0, j;
    reg [31:0] rng = 123;
    initial begin
        #200 reset = 0;
        wait (consumed == 81920);
        #50;
        if (underruns) $fatal(1, "unexpected underrun");
        $display("PASS usb_tx: %0d words, longest stall %0d cycles", consumed, max_stall);
        $finish;
    end

    always @(negedge work) begin
        rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
        valid = produced < 10240;
        for (j = 0; j < 8; j = j + 1) data[j * 16 +: 16] = 16'(produced * 8 + j) ^ 16'hABCD;
    end
    always @(posedge work) if (!reset && pop) produced = produced + 1;

    always @(posedge ft_clk) begin
        if (!reset && !wr && !txe) begin
            if (pads !== (16'(consumed) ^ 16'hABCD))
                $fatal(1, "word %0d: got %h", consumed, pads);
            consumed = consumed + 1;
        end
        tick = tick + 1;
        #3.3 txe = tick % 1900 < 200 || rng[5:4] == 0;
    end

    initial begin #10000000; $fatal(1, "timeout"); end
endmodule
