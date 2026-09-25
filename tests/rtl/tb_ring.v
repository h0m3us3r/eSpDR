`timescale 1ns/1ps
// DDR ring against a MIG model with independent command/data acceptance,
// random stalls and consumer pauses that fill the ring completely.
// CLEAR_AT > 0 clears the ring mid-stream (reads in flight, writes half
// accepted); it must then carry a fresh stream exactly.
module tb #(parameter CLEAR_AT = 0);
    reg clk = 0;
    always #6 clk = ~clk;

    reg reset = 1, clear = 0, in_valid = 0, in_start = 0;
    reg [127:0] in_data = 0;
    reg [4:0] in_commit = 0;
    reg [9:0] out_level = 0;
    wire in_ready, out_push, app_en, app_wdf_wren, app_wdf_end;
    wire [127:0] out_data, app_wdf_data;
    wire [27:0] app_addr;
    wire [2:0] app_cmd;
    wire [15:0] app_wdf_mask;
    reg app_rdy = 0, app_wdf_rdy = 0, app_rd_data_valid = 0;
    reg [127:0] app_rd_data = 0;
    wire [9:0] occupancy, peak;
    wire [31:0] errors;
    ddr_ring #(.ADDR_BITS(9)) dut (
        .clk(clk), .reset(reset), .clear(clear), .calibrated(1'b1), .in_level(10'd300), .in_valid(in_valid),
        .in_start(in_start), .in_data(in_data), .in_commit(in_commit), .in_ready(in_ready),
        .out_push(out_push), .out_data(out_data), .out_level(out_level), .out_full(out_level == 512),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid), .occupancy(occupancy), .occupancy_peak(peak),
        .errors(errors));

    reg [127:0] memory [0:511], write_data [0:32767], read_data [0:32767], fifo [0:32767];
    reg [8:0] write_address [0:32767];
    integer commands_in = 0, commands_out = 0, data_in = 0, data_out = 0, reads_in = 0, reads_out = 0;
    integer fifo_in = 0, fifo_out = 0, sent = 0, tick = 0, total = 16000, cleared = 0, clear_ticks = 0;
    reg [31:0] rng = 32'h87654321;
    function [127:0] pattern(input integer i);
        pattern = {32'hABC01234, 32'(i), 32'(~i), 32'(i * 3571)};
    endfunction

    initial begin
        repeat (10) @(negedge clk);
        reset = 0;
        wait (fifo_out == total);
        repeat (100) @(negedge clk);
        if (errors || occupancy || sent != total || peak < 500)
            $fatal(1, "errors=%0d occupancy=%0d sent=%0d peak=%0d", errors, occupancy, sent, peak);
        if (CLEAR_AT && !cleared) $fatal(1, "clear never happened");
        $display("PASS ring: %0d beats through %0d wraps, peak %0d%s", fifo_out, total / 512, peak,
                 CLEAR_AT ? ", after a mid-stream clear" : "");
        $finish;
    end

    // Mid-stream clear: the output FIFO is reset with it, and the source
    // restarts from beat 0.
    always @(negedge clk) if (!reset && CLEAR_AT && !cleared && fifo_out == CLEAR_AT) begin
        clear = 1;
        clear_ticks = clear_ticks + 1;
        if (clear_ticks == 11) begin
            clear = 0;
            cleared = 1;
            sent = 0;
            fifo_in = 0;
            fifo_out = 0;
        end
    end

    always @(negedge clk) if (!reset) begin
        tick = tick + 1;
        rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
        app_rdy = rng[0] && tick % 191 < 170;
        app_wdf_rdy = rng[3] && tick % 233 < 190;
        in_valid = sent < total;
        in_start = sent % 163 == 0;
        in_data = pattern(sent);
        in_commit = (sent % 163 == 162 || sent == total - 1) ? 16 : 0;
        app_rd_data_valid = reads_in != reads_out && rng[7];
        if (app_rd_data_valid) begin
            app_rd_data = read_data[reads_out];
            reads_out = reads_out + 1;
        end
        if (!clear && fifo_in != fifo_out && tick % 40000 > 17000 && rng[9]) begin
            if (fifo[fifo_out] !== pattern(fifo_out)) $fatal(1, "data at beat %0d", fifo_out);
            fifo_out = fifo_out + 1;
        end
        out_level = fifo_in - fifo_out;
    end

    always @(posedge clk) if (!reset) begin
        if (errors) $fatal(1, "ring error after %0d beats", sent);
        if (in_valid && in_ready) sent = sent + 1;
        if (clear && out_push) $fatal(1, "output while clearing");
        if (app_wdf_wren && app_wdf_rdy) begin
            write_data[data_in] = app_wdf_data;
            data_in = data_in + 1;
        end
        if (app_en && app_rdy) begin
            if (app_cmd == 0) begin
                write_address[commands_in] = app_addr[11:3];
                commands_in = commands_in + 1;
            end else begin
                read_data[reads_in] = memory[app_addr[11:3]];
                reads_in = reads_in + 1;
            end
        end
        if (commands_in != commands_out && data_in != data_out) begin
            memory[write_address[commands_out]] = write_data[data_out];
            commands_out = commands_out + 1;
            data_out = data_out + 1;
        end
        if (out_push) begin
            if (fifo_in - fifo_out >= 512) $fatal(1, "read credit exceeded");
            fifo[fifo_in] = out_data;
            fifo_in = fifo_in + 1;
        end
    end

    initial begin #100000000; $fatal(1, "timeout at beat %0d", fifo_out); end
endmodule
