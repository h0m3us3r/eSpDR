`timescale 1ns/1ps
`include "protocol.vh"
// Encoded stream path: lanes -> reorder -> encoder -> packer -> DDR ring ->
// FT600.
//
// When `flush` is set (the receiver was stopped) and the input has been idle
// for 8192 clocks, the encoder closes its partial block. Once that has
// drained, an END record carrying the total span (pairs delivered and lost)
// and the CRC of every delivered pair is appended, followed by zero bytes up to the next 1 MiB
// transfer boundary, so the host receives complete transfers only.
module stream #(
    parameter RING_ADDR_BITS = 24,
    parameter USB_TRANSFER_WORDS = 524288
) (
    input  wire         clk,            // 120 MHz processing clock
    input  wire         reset,
    input  wire         flush,
    input  wire [1:0]   valid,
    input  wire [1:0]   last,
    input  wire [19:0]  pair0,
    input  wire [19:0]  pair1,
    input  wire [1:0]   unit_valid,
    input  wire [31:0]  unit_sequence0,
    input  wire [31:0]  unit_sequence1,
    input  wire [13:0]  unit_first0,
    input  wire [13:0]  unit_first1,
    input  wire [13:0]  unit_count0,
    input  wire [13:0]  unit_count1,

    input  wire         ui_clk,         // MIG user interface
    input  wire         ui_reset,
    input  wire         calibrated,
    output wire [27:0]  app_addr,
    output wire [2:0]   app_cmd,
    output wire         app_en,
    output wire [127:0] app_wdf_data,
    output wire [15:0]  app_wdf_mask,
    output wire         app_wdf_end,
    output wire         app_wdf_wren,
    input  wire         app_rdy,
    input  wire         app_wdf_rdy,
    input  wire [127:0] app_rd_data,
    input  wire         app_rd_data_valid,

    input  wire         ft_clk,
    input  wire         ft_txe,
    output wire [15:0]  ft_data,
    output wire [1:0]   ft_be,
    output wire         ft_wr,

    // Status, in the clk domain. Counters from the DDR and FT600 domains
    // arrive as snapshots a few clocks old.
    output wire [63:0]  pairs,          // delivered
    output wire [63:0]  lost_pairs,
    output wire [31:0]  lost_units,
    output wire [31:0]  discarded_units,
    output wire [31:0]  stream_crc,
    output wire [31:0]  records,
    output wire         ended,
    output wire [31:0]  reorder_overflow0,
    output wire [31:0]  reorder_overflow1,
    output wire [14:0]  reorder_peak0,
    output wire [14:0]  reorder_peak1,
    output wire [31:0]  ring_input_overflow,
    output wire [31:0]  ring_output_overflow,
    output wire [31:0]  ring_errors,
    output wire [RING_ADDR_BITS:0] ring_used,
    output wire [RING_ADDR_BITS:0] ring_peak,
    output wire [31:0]  usb_underruns,
    output wire [31:0]  usb_max_stall
);
    function automatic [31:0] crc_byte(input [31:0] c, input [7:0] d);
        reg [31:0] v;
        integer j;
        begin
            v = c ^ d;
            for (j = 0; j < 8; j = j + 1)
                v = v[0] ? (v >> 1) ^ 32'hEDB88320 : v >> 1;
            crc_byte = v;
        end
    endfunction

    // CRC of a pair as a little-endian 32-bit word.
    function automatic [31:0] crc_pair(input [31:0] c, input [19:0] d);
        reg [31:0] v;
        integer j;
        begin
            v = c ^ {12'b0, d};
            for (j = 0; j < 32; j = j + 1)
                v = v[0] ? (v >> 1) ^ 32'hEDB88320 : v >> 1;
            crc_pair = v;
        end
    endfunction

    // ---- chronological order ----------------------------------------------------
    wire        ordered_valid, ordered_ready, skip_valid, skip_ready;
    wire [19:0] ordered_pair;
    wire [31:0] skip_pairs;
    reorder reorder (
        .clk(clk), .reset(reset), .valid(valid), .last(last), .pair0(pair0), .pair1(pair1),
        .unit_valid(unit_valid), .unit_sequence0(unit_sequence0), .unit_sequence1(unit_sequence1),
        .unit_first0(unit_first0), .unit_first1(unit_first1),
        .unit_count0(unit_count0), .unit_count1(unit_count1),
        .out_valid(ordered_valid), .out_ready(ordered_ready), .out_pair(ordered_pair),
        .skip_valid(skip_valid), .skip_ready(skip_ready), .skip_pairs(skip_pairs),
        .overflow0(reorder_overflow0), .overflow1(reorder_overflow1),
        .lost_units(lost_units), .discarded_units(discarded_units), .peak0(reorder_peak0), .peak1(reorder_peak1));

    reg [31:0] ordered_crc = 32'hFFFFFFFF;
    always @(posedge clk)
        if (reset) ordered_crc <= 32'hFFFFFFFF;
        else if (ordered_valid && ordered_ready) ordered_crc <= crc_pair(ordered_crc, ordered_pair);
    assign stream_crc = ~ordered_crc;

    // Input idle time, counted only once the receiver has stopped.
    reg [12:0] idle = 0;
    always @(posedge clk)
        if (reset || !flush || ordered_valid || skip_valid || (|valid)) idle <= 0;
        else if (!(&idle)) idle <= idle + 1'b1;

    // ---- encoder ----------------------------------------------------------------------
    wire        encoded_valid, encoded_ready, encoded_last;
    wire [63:0] encoded_data;
    wire [3:0]  encoded_bytes;
    encoder encoder (
        .clk(clk), .reset(reset), .in_valid(ordered_valid), .in_ready(ordered_ready),
        .in_pair(ordered_pair), .flush(&idle),
        .skip_valid(skip_valid), .skip_ready(skip_ready), .skip_pairs(skip_pairs),
        .out_valid(encoded_valid), .out_ready(encoded_ready), .out_data(encoded_data),
        .out_bytes(encoded_bytes), .out_last(encoded_last),
        .accepted(pairs), .skipped(lost_pairs), .records(records));

    // Two-entry queue: breaks the combinational path from a full DDR input
    // FIFO back into the encoder's output stage.
    reg [68:0] queue [0:1];             // {last, bytes, data}
    reg        queue_write = 0, queue_read = 0;
    reg [1:0]  queue_level = 0;
    wire       packer_ready;
    wire       queue_pop = queue_level != 0 && packer_ready;  // the trailer starts only once drained
    wire       queue_push = encoded_valid && encoded_ready;
    assign encoded_ready = queue_level < 2;
    always @(posedge clk) begin
        if (reset) begin
            queue_write <= 0;
            queue_read <= 0;
            queue_level <= 0;
        end else begin
            if (queue_push) begin
                queue[queue_write] <= {encoded_last, encoded_bytes, encoded_data};
                queue_write <= !queue_write;
            end
            if (queue_pop)
                queue_read <= !queue_read;
            case ({queue_push, queue_pop})
                2'b10: queue_level <= queue_level + 1'b1;
                2'b01: queue_level <= queue_level - 1'b1;
                default: ;
            endcase
        end
    end

    // ---- END record and transfer padding -------------------------------------------------
    localparam TRANSFER_BYTE_BITS = $clog2(USB_TRANSFER_WORDS * 2);
    localparam END_WAIT = 0, END_CRC = 1, END_READY = 2, END_SEND = 3, END_PAD = 4, END_DONE = 5;
    reg [2:0]   end_state = END_WAIT;
    reg [12:0]  drained = 0;
    reg [TRANSFER_BYTE_BITS-1:0] transfer_bytes = 0;   // bytes into the current transfer
    reg [255:0] end_header = 0;
    reg [31:0]  end_crc = 32'hFFFFFFFF;
    reg [4:0]   end_crc_at = 0;
    reg [1:0]   end_word = 0;

    always @(posedge clk)
        if (reset || !(&idle) || encoded_valid || queue_level != 0) drained <= 0;
        else if (!(&drained)) drained <= drained + 1'b1;

    wire trailer_valid = end_state == END_SEND || end_state == END_PAD;
    wire [3:0] trailer_bytes = (end_state == END_PAD && transfer_bytes[2:0] != 0)
                                   ? 4'd8 - {1'b0, transfer_bytes[2:0]} : 4'd8;
    wire [TRANSFER_BYTE_BITS-1:0] transfer_next = transfer_bytes + trailer_bytes;
    // Zero padding goes in 1 KiB "records" so the ring's reservation rule holds.
    wire trailer_last = end_state == END_SEND ? end_word == 3 : transfer_next[9:0] == 0;
    wire [63:0] trailer_data = end_state == END_SEND ? end_header[end_word * 64 +: 64] : 64'b0;
    assign ended = end_state == END_DONE;

    always @(posedge clk) begin
        if (reset) begin
            transfer_bytes <= 0;
            end_state <= END_WAIT;
            end_word <= 0;
            end_crc_at <= 0;
            end_crc <= 32'hFFFFFFFF;
            end_header <= 0;
        end else begin
            if (queue_pop)
                transfer_bytes <= transfer_bytes + queue[queue_read][67:64];
            case (end_state)
                END_WAIT: if (&drained) begin
                    end_header <= 0;
                    end_header[`IQR_OFF_MAGIC * 8 +: 32] <= `IQR_MAGIC;
                    end_header[`IQR_OFF_VERSION * 8 +: 8] <= `IQR_VERSION;
                    end_header[`IQR_OFF_MODE * 8 +: 8] <= `IQR_END;
                    end_header[`IQR_OFF_INDEX * 8 +: 64] <= pairs + lost_pairs;
                    end_header[`IQR_OFF_CRC * 8 +: 32] <= ~ordered_crc;
                    end_crc <= 32'hFFFFFFFF;
                    end_crc_at <= 0;
                    end_state <= END_CRC;
                end
                END_CRC: begin
                    end_crc <= crc_byte(end_crc, end_header[end_crc_at * 8 +: 8]);
                    if (end_crc_at == `IQR_OFF_HEADER_CRC - 1)
                        end_state <= END_READY;
                    else
                        end_crc_at <= end_crc_at + 1'b1;
                end
                END_READY: begin
                    end_header[`IQR_OFF_HEADER_CRC * 8 +: 32] <= ~end_crc;
                    end_word <= 0;
                    end_state <= END_SEND;
                end
                END_SEND: if (packer_ready) begin
                    transfer_bytes <= transfer_next;
                    if (end_word == 3)
                        end_state <= transfer_next == 0 ? END_DONE : END_PAD;
                    else
                        end_word <= end_word + 1'b1;
                end
                END_PAD: if (packer_ready) begin
                    transfer_bytes <= transfer_next;
                    if (transfer_next == 0)
                        end_state <= END_DONE;
                end
                default: ;
            endcase
        end
    end

    // ---- dense packing into DDR beats ------------------------------------------------------
    wire         beat_valid, beat_ready, beat_start;
    wire [127:0] beat_data;
    wire [4:0]   beat_commit;
    packer packer (
        .clk(clk), .reset(reset),
        .in_valid(queue_level != 0 || trailer_valid), .in_ready(packer_ready),
        .in_data(trailer_valid ? trailer_data : queue[queue_read][63:0]),
        .in_bytes(trailer_valid ? trailer_bytes : queue[queue_read][67:64]),
        .in_last(trailer_valid ? trailer_last : queue[queue_read][68]),
        .out_valid(beat_valid), .out_ready(beat_ready), .out_data(beat_data),
        .out_commit(beat_commit), .out_start(beat_start));

    // ---- DDR ring (MIG UI clock domain) ------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] clear_sync = 2'b11;
    always @(posedge ui_clk) clear_sync <= {clear_sync[0], reset};
    wire ring_reset = ui_reset || clear_sync[1];

    wire         ingress_full, ingress_valid, ingress_ready;
    wire [133:0] ingress_data;          // {start, commit, beat}
    wire [9:0]   ingress_level;
    assign beat_ready = !ingress_full;
    async_fifo #(.WIDTH(134), .ADDR_BITS(9), .RAM_STYLE("block"), .SHOW_AHEAD(1)) ingress (
        .wr_clk(clk), .wr_reset(reset), .wr_valid(beat_valid && beat_ready),
        .wr_data({beat_start, beat_commit, beat_data}),
        .full(ingress_full), .overflow(ring_input_overflow), .wr_level(),
        .rd_clk(ui_clk), .rd_reset(ring_reset), .rd_enable(ingress_valid && ingress_ready),
        .rd_valid(ingress_valid), .rd_data(ingress_data), .empty(), .rd_level(ingress_level));

    wire         egress_push, egress_full;
    wire [127:0] egress_data;
    wire [9:0]   egress_level;
    wire [RING_ADDR_BITS:0] ring_occupancy, ring_occupancy_peak;
    wire [31:0]  ring_error_count, egress_overflow;
    ddr_ring #(.ADDR_BITS(RING_ADDR_BITS), .BURST_BEATS(USB_TRANSFER_WORDS / 8)) ring (
        .clk(ui_clk), .reset(ui_reset), .clear(clear_sync[1]), .calibrated(calibrated),
        .in_level(ingress_level), .in_valid(ingress_valid), .in_start(ingress_data[133]),
        .in_data(ingress_data[127:0]), .in_commit(ingress_data[132:128]), .in_ready(ingress_ready),
        .out_push(egress_push), .out_data(egress_data), .out_level(egress_level),
        .out_full(egress_full),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .occupancy(ring_occupancy), .occupancy_peak(ring_occupancy_peak), .errors(ring_error_count));

    // ---- FT600 clocks: phases of the FT600's own 100 MHz clock -----------------------------------
    wire control, launch, data_launch, data_clock_enable, write_launch, work, ft_locked;
`ifdef SIMULATION
    assign #1.625 control = ft_clk;
    assign #4.75 launch = ft_clk;
    assign data_launch = launch;
    assign #0.25 write_launch = ~ft_clk;
    assign #4.75 work = ~ft_clk;
    assign ft_locked = 1;
`else
    wire feedback, feedback_buffered, control_raw, launch_raw, work_raw, write_raw;
    MMCME2_BASE #(
        .CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(10.0), .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(10.0), .CLKOUT0_PHASE(58.5),     // control: 1.625 ns
        .CLKOUT1_DIVIDE(10), .CLKOUT1_PHASE(171.0),        // launch: 4.75 ns
        .CLKOUT2_DIVIDE(10), .CLKOUT2_PHASE(351.0),        // work: 9.75 ns
        .CLKOUT3_DIVIDE(10), .CLKOUT3_PHASE(189.0)         // write_launch: 5.25 ns
    ) ft_clock (
        .CLKIN1(ft_clk), .CLKFBIN(feedback_buffered), .CLKFBOUT(feedback),
        .CLKOUT0(control_raw), .CLKOUT1(launch_raw), .CLKOUT2(work_raw), .CLKOUT3(write_raw),
        .LOCKED(ft_locked), .RST(1'b0), .PWRDWN(1'b0));
    BUFG feedback_buffer (.I(feedback), .O(feedback_buffered));
    BUFG control_buffer (.I(control_raw), .O(control));
    BUFG launch_buffer (.I(launch_raw), .O(launch));
    BUFGCE #(.SIM_DEVICE("7SERIES")) data_buffer (.I(launch_raw), .CE(data_clock_enable), .O(data_launch));
    BUFG work_buffer (.I(work_raw), .O(work));
    BUFG write_buffer (.I(write_raw), .O(write_launch));
`endif

    (* ASYNC_REG = "TRUE" *) reg [1:0] ft_reset = 2'b11;
    always @(posedge control) ft_reset <= {ft_reset[0], ring_reset || !ft_locked};

    wire         usb_valid, usb_pop;
    wire [127:0] usb_data;
    wire [9:0]   usb_available;
    wire [31:0]  underruns, max_stall;
    async_fifo #(.WIDTH(128), .ADDR_BITS(9), .RAM_STYLE("block"), .SHOW_AHEAD(1)) output_queue (
        .wr_clk(ui_clk), .wr_reset(ring_reset), .wr_valid(egress_push), .wr_data(egress_data),
        .full(egress_full), .overflow(egress_overflow), .wr_level(egress_level),
        .rd_clk(work), .rd_reset(ft_reset[1]), .rd_enable(usb_pop),
        .rd_valid(usb_valid), .rd_data(usb_data), .empty(), .rd_level(usb_available));

    usb_tx #(.WORDS(USB_TRANSFER_WORDS)) tx (
        .control(control), .launch(launch), .data_launch(data_launch),
        .write_launch(write_launch), .work(work), .reset(ft_reset[1]),
        .valid(usb_valid), .data(usb_data), .available(usb_available), .pop(usb_pop),
        .ft_txe(ft_txe), .data_clock_enable(data_clock_enable),
        .ft_data(ft_data), .ft_be(ft_be), .ft_wr(ft_wr),
        .underruns(underruns), .max_stall(max_stall));

    // ---- live status in the clk domain ------------------------------------------------------------
    snapshot #(.WIDTH(2 * RING_ADDR_BITS + 66)) ring_status (
        .source_clk(ui_clk),
        .source_value({ring_occupancy, ring_occupancy_peak, ring_error_count, egress_overflow}),
        .clk(clk), .value({ring_used, ring_peak, ring_errors, ring_output_overflow}));
    snapshot #(.WIDTH(64)) usb_status (
        .source_clk(work), .source_value({underruns, max_stall}),
        .clk(clk), .value({usb_underruns, usb_max_stall}));
endmodule
