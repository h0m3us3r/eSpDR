`timescale 1ns/1ps
// Whole stream path: two lanes -> reorder -> encoder -> packer -> DDR ring
// (MIG model with direction-turnaround delays) -> FT600 model with TXE
// stalls. Writes all chronological pairs and the USB bytes for codec_test
// check. LOSE_EVERY drops every n-th unit, which must become an index gap.
module tb #(parameter FIXED_LENGTH = 0, UNITS = 16, RING_BITS = 12, USB_WORDS = 2048,
            DRAIN_CYCLES = 50000, LOSE_EVERY = 0);
    reg clk = 0, ui_clk = 0, ft_clk = 0;
    always #4.166 clk = ~clk;
    always #6 ui_clk = ~ui_clk;
    always #5 ft_clk = ~ft_clk;

    reg reset = 1, flush = 0, txe = 1;
    reg [1:0] valid = 0, last = 0, unit_valid = 0;
    reg [19:0] pair0 = 0, pair1 = 0;
    reg [31:0] sequence0 = 0, sequence1 = 1;
    reg [13:0] first0 = 0, first1 = 0, count0 = 0, count1 = 0;
    wire [15:0] ft_data;
    wire [1:0] ft_be;
    wire ft_wr, app_en, app_wdf_wren, app_wdf_end, ended;
    wire [27:0] app_addr;
    wire [2:0] app_cmd;
    wire [127:0] app_wdf_data;
    wire [15:0] app_wdf_mask;
    reg app_rdy = 0, app_wdf_rdy = 0, app_rd_data_valid = 0;
    reg [127:0] app_rd_data = 0;
    wire [63:0] pairs, lost_pairs;
    wire [31:0] stream_crc, records, overflow0, overflow1, lost_units;
    wire [31:0] ring_input_overflow, ring_output_overflow, ring_errors, usb_underruns, usb_max_stall;
    wire [14:0] peak0, peak1;
    wire [RING_BITS:0] ring_used, ring_peak;

    stream #(.RING_ADDR_BITS(RING_BITS), .USB_TRANSFER_WORDS(USB_WORDS)) dut (
        .clk(clk), .reset(reset), .flush(flush), .valid(valid), .last(last), .pair0(pair0),
        .pair1(pair1), .unit_valid(unit_valid), .unit_sequence0(sequence0),
        .unit_sequence1(sequence1), .unit_first0(first0), .unit_first1(first1),
        .unit_count0(count0), .unit_count1(count1),
        .ui_clk(ui_clk), .ui_reset(reset), .calibrated(1'b1),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .ft_clk(ft_clk), .ft_txe(txe), .ft_data(ft_data), .ft_be(ft_be), .ft_wr(ft_wr),
        .pairs(pairs), .lost_pairs(lost_pairs), .lost_units(lost_units), .discarded_units(),
        .stream_crc(stream_crc), .records(records), .ended(ended),
        .reorder_overflow0(overflow0), .reorder_overflow1(overflow1),
        .reorder_peak0(peak0), .reorder_peak1(peak1),
        .ring_input_overflow(ring_input_overflow), .ring_output_overflow(ring_output_overflow),
        .ring_errors(ring_errors), .ring_used(ring_used), .ring_peak(ring_peak),
        .usb_underruns(usb_underruns), .usb_max_stall(usb_max_stall));

    // MIG model.
    reg [127:0] memory [0:(1 << RING_BITS) - 1], write_data [0:524287], read_data [0:524287];
    reg [RING_BITS-1:0] write_address [0:524287];
    integer commands_in = 0, commands_out = 0, data_in = 0, data_out = 0, reads_in = 0, reads_out = 0;
    integer turnaround = 0;
    reg last_direction = 0;

    integer unit0 = 0, unit1 = 1, at0 = 0, at1 = 0, tick = 0, ui_tick = 0, ft_tick = 0;
    integer lane_pairs = 0, usb_bytes = 0, j;
    integer pairs_file, usb_file;
    reg [1023:0] pairs_path, usb_path;
    reg [31:0] rng = 32'h127458;

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
    function lost(input integer unit);
        // A gap needs a later unit to measure it by, so the last two stay.
        lost = LOSE_EVERY != 0 && unit < UNITS - 2 && unit % LOSE_EVERY == LOSE_EVERY - 1;
    endfunction
    integer expected_lost_units = 0, expected_lost_pairs = 0;
    // Wide random values for the first units, then narrow ones that compress.
    function [19:0] pair_value(input integer unit, input integer at);
        reg [31:0] x;
        begin
            x = 32'(unit * 16289 + at + 1);
            x = x ^ (x >> 16); x = x * 32'h7FEB352D;
            x = x ^ (x >> 15); x = x * 32'h846CA68B;
            x = x ^ (x >> 16);
            pair_value = unit < 8 ? x[19:0] : {4'b0, x[5:0], 4'b0, x[11:6]};
        end
    endfunction

    initial begin
        if (!$value$plusargs("pairs=%s", pairs_path) || !$value$plusargs("usb=%s", usb_path)) $fatal;
        pairs_file = $fopen(pairs_path, "wb");
        usb_file = $fopen(usb_path, "wb");
        // Chronological pairs, which the stream must reproduce.
        for (unit0 = 0; unit0 < UNITS; unit0 = unit0 + 1) begin
            if (lost(unit0)) begin
                expected_lost_units = expected_lost_units + 1;
                expected_lost_pairs = expected_lost_pairs + unit_length(unit0);
            end
            for (at0 = 0; at0 < unit_length(unit0); at0 = at0 + 1)
                for (j = 0; j < 4; j = j + 1)
                    $fwrite(pairs_file, "%c", (pair_value(unit0, at0) >> (8 * j)) & 255);
        end
        $fclose(pairs_file);
        unit0 = 0;
        at0 = 0;
        repeat (100) @(negedge clk);
        reset = 0;
        wait (unit0 >= UNITS && unit1 >= UNITS);
        flush = 1;
        repeat (DRAIN_CYCLES) @(negedge clk);
        if (!ended || ring_used != 0 || usb_underruns || overflow0 || overflow1 ||
            ring_input_overflow || ring_output_overflow || ring_errors ||
            lost_units != expected_lost_units || lost_pairs != expected_lost_pairs)
            $fatal(1, "stream status: lost %0d units / %0d pairs", lost_units, lost_pairs);
        $display("PASS stream: %0d pairs, %0d lost in %0d gaps, %0d records, %0d USB bytes, lane peaks %0d/%0d, ring peak %0d",
                 pairs, lost_pairs, lost_units, records, usb_bytes, peak0, peak1, ring_peak);
        $fclose(usb_file);
        $finish;
    end

    always @(negedge clk) if (!reset) begin
        tick = tick + 1;
        valid[0] = unit0 < UNITS && (tick * 2) % 91 < 32;
        valid[1] = unit1 < UNITS && ((tick + 7) * 2) % 91 < 32;
        unit_valid[0] = valid[0] && at0 == 0 && !lost(unit0);
        unit_valid[1] = valid[1] && at1 == 0 && !lost(unit1);
        sequence0 = unit0; first0 = unit_first(unit0); count0 = unit_length(unit0);
        sequence1 = unit1; first1 = unit_first(unit1); count1 = unit_length(unit1);
        pair0 = pair_value(unit0, at0);
        pair1 = pair_value(unit1, at1);
        last[0] = at0 + 1 == unit_length(unit0);
        last[1] = at1 + 1 == unit_length(unit1);
        if (lost(unit0)) valid[0] = 0;  // never received, but its time passes
        if (lost(unit1)) valid[1] = 0;
    end
    always @(posedge clk) if (!reset) begin
        if (unit0 < UNITS && (tick * 2) % 91 < 32) begin
            if (at0 + 1 == unit_length(unit0)) begin unit0 = unit0 + 2; at0 = 0; end else at0 = at0 + 1;
        end
        if (unit1 < UNITS && ((tick + 7) * 2) % 91 < 32) begin
            if (at1 + 1 == unit_length(unit1)) begin unit1 = unit1 + 2; at1 = 0; end else at1 = at1 + 1;
        end
    end

    always @(negedge ui_clk) if (!reset) begin
        ui_tick = ui_tick + 1;
        rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
        if (turnaround > 0) turnaround = turnaround - 1;
        app_rdy = turnaround == 0 && ui_tick % 251 < 230;
        app_wdf_rdy = ui_tick % 199 < 180;
        app_rd_data_valid = turnaround == 0 && reads_in != reads_out && rng[7];
        if (app_rd_data_valid) begin
            app_rd_data = read_data[reads_out];
            reads_out = reads_out + 1;
        end
    end
    always @(posedge ui_clk) if (!reset) begin
        if (app_wdf_wren && app_wdf_rdy) begin
            write_data[data_in] = app_wdf_data;
            data_in = data_in + 1;
        end
        if (app_en && app_rdy) begin
            if (app_cmd[0] != last_direction) turnaround = 8;
            last_direction = app_cmd[0];
            if (app_cmd == 0) begin
                write_address[commands_in] = app_addr[RING_BITS+2:3];
                commands_in = commands_in + 1;
            end else begin
                read_data[reads_in] = memory[app_addr[RING_BITS+2:3]];
                reads_in = reads_in + 1;
            end
        end
        if (commands_in != commands_out && data_in != data_out) begin
            memory[write_address[commands_out]] = write_data[data_out];
            commands_out = commands_out + 1;
            data_out = data_out + 1;
        end
    end

    always @(posedge ft_clk) begin
        if (!reset && !ft_wr && !txe) begin
            $fwrite(usb_file, "%c%c", ft_data[7:0], ft_data[15:8]);
            usb_bytes = usb_bytes + 2;
        end
        ft_tick = ft_tick + 1;
        #3.3 txe = ft_tick % 5000 < 170 || ft_tick < 4000;
    end

    initial begin #100000000; $fatal(1, "timeout"); end
endmodule
