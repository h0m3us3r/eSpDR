`include "protocol.vh"
// ESP32-S3 IQ link receiver and USB streamer for Alchitry Au + Ft (+ Br).
//
//   link (16 lines, 240 MHz sampling) -> two lane receivers -> stream.v
//   (reorder, lossless encoder, 256 MiB DDR3 ring) -> FT600 USB 3.
//
// Control is over the FT2232 UART (1 Mbaud, protocol/control.h). The FPGA
// also supplies the ESP32-S3's 40 MHz reference clock.
module iqstream_top (
    input  wire        clk100,
    input  wire        rst_n,
    input  wire        uart_rx,
    output wire        uart_tx,

    input  wire [15:0] link,
    inout  wire        esp_refin,

    input  wire        ft_clk,
    input  wire        ft_txe,
    output wire        ft_wr,
    output wire        ft_rd,
    output wire        ft_oe,
    inout  wire [15:0] ft_data,
    inout  wire [1:0]  ft_be,

    inout  wire [15:0] ddr3_dq,
    inout  wire [1:0]  ddr3_dqs_n,
    inout  wire [1:0]  ddr3_dqs_p,
    output wire [13:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_cke,
    output wire [0:0]  ddr3_cs_n,
    output wire [0:0]  ddr3_odt,
    output wire [1:0]  ddr3_dm
);
    localparam UART_DIVIDER = 120;      // 120 MHz / 1 Mbaud

    assign ft_rd = 1'b1;                // write-only FT600 channel
    assign ft_oe = 1'b1;

    // ---- clocks and reset ----------------------------------------------------------------
    // Link sampling phase at start-up, in MMCM steps; measured for the
    // documented wiring. `iqstream calibrate` measures it for other builds.
    localparam DEFAULT_PHASE = 162;

    wire oscillator, sample_clk, clk, delay_clk, esp_clk, clocks_locked, phase_ready;
    wire [8:0] phase;
    reg  [8:0] phase_target = DEFAULT_PHASE;
    IBUF oscillator_buffer (.I(clk100), .O(oscillator));
    clocks clocks (
        .oscillator(oscillator), .target(phase_target), .sample_clk(sample_clk),
        .process_clk(clk), .delay_clk(delay_clk), .esp_clk(esp_clk), .locked(clocks_locked),
        .phase_ready(phase_ready), .phase(phase));

    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync = 0;
    reg [5:0] boot = 0;
    always @(posedge clk) begin
        reset_sync <= {reset_sync[0], rst_n && clocks_locked};
        if (!reset_sync[1]) boot <= 0;
        else if (!(&boot)) boot <= boot + 1'b1;
    end
    wire reset = !(&boot);

    // ---- ESP reference clock ----------------------------------------------------------------
    reg  reference_enable = 0;
    wire reference_running;
    reference_output reference_clock (
        .clock40(esp_clk), .locked(clocks_locked), .enable(reference_enable),
        .running(reference_running), .pad(esp_refin));
    (* ASYNC_REG = "TRUE" *) reg [1:0] reference_sync = 0;
    always @(posedge clk) reference_sync <= {reference_sync[0], reference_running};

    // ---- arm / stop ----------------------------------------------------------------------------
    // ARM clears the whole receive and stream path for 16 clocks, then runs.
    // It opens the only stream; another ARM is refused until RELEASE.
    reg arm = 0, disarm = 0, armed = 0, stream_started = 0, stream_open = 0;
    reg [4:0] clear_count = 15;
    (* MAX_FANOUT = 64 *) reg clear = 1;
    always @(posedge clk) begin
        if (reset || arm) clear_count <= 15;
        else if (clear_count != 0) clear_count <= clear_count - 1'b1;
        clear <= reset || clear_count != 0;
        if (reset || disarm) armed <= 0;
        else if (arm) armed <= 1;
        if (reset) stream_started <= 0;
        else if (arm) stream_started <= 1;
    end

    // ---- link receive ------------------------------------------------------------------------------
    wire        deskew_ready, pair_valid;
    wire [63:0] sample_pair;
    wire [31:0] sample_overflow;
    link_input link_input (
        .link(link), .sample_clk(sample_clk), .delay_clk(delay_clk), .clk(clk),
        .reset(reset), .clear(clear), .run(armed), .deskew_ready(deskew_ready),
        .pair_valid(pair_valid), .pair(sample_pair), .overflow(sample_overflow));

    wire [1:0]  byte_valid, lane_valid, lane_last;
    wire [7:0]  byte_value [0:1];
    wire [15:0] byte_index [0:1];
    wire [31:0] lane_sequence [0:1];
    wire [1:0]  unit_valid;
    wire [13:0] unit_first [0:1], unit_count [0:1];
    wire [3:0]  head_pairs [0:1], tail_pairs [0:1];
    wire [10:0] total_groups [0:1];
    wire [19:0] lane_pair [0:1];
    wire [31:0] units [0:1], framing_errors [0:1], unpack_errors [0:1];
    wire [31:0] checksum_errors [0:1], end_marker_errors [0:1];
    genvar lane;
    generate
        for (lane = 0; lane < 2; lane = lane + 1) begin : lanes
            link_lane #(.LANE(lane)) receive (
                .clk(clk), .reset(clear), .run(armed), .pair_valid(pair_valid),
                .samples({sample_pair[48 + lane * 8 +: 8], sample_pair[32 + lane * 8 +: 8],
                          sample_pair[16 + lane * 8 +: 8], sample_pair[lane * 8 +: 8]}),
                .byte_valid(byte_valid[lane]), .byte_value(byte_value[lane]),
                .byte_index(byte_index[lane]), .sequence_word(lane_sequence[lane]),
                .unit_valid(unit_valid[lane]), .unit_first(unit_first[lane]),
                .unit_count(unit_count[lane]),
                .head_pairs(head_pairs[lane]), .tail_pairs(tail_pairs[lane]),
                .total_groups(total_groups[lane]), .units(units[lane]),
                .framing_errors(framing_errors[lane]), .checksum_errors(checksum_errors[lane]),
                .end_marker_errors(end_marker_errors[lane]));
            link_unpack unpack (
                .clk(clk), .reset(clear), .byte_valid(byte_valid[lane]),
                .byte_value(byte_value[lane]), .byte_index(byte_index[lane]),
                .head_pairs(head_pairs[lane]), .tail_pairs(tail_pairs[lane]),
                .total_groups(total_groups[lane]), .pair_valid(lane_valid[lane]),
                .pair(lane_pair[lane]), .last(lane_last[lane]), .errors(unpack_errors[lane]));
        end
    endgenerate

    // ---- DDR3 --------------------------------------------------------------------------------------------
    reg [7:0] ddr_reset_count = 255;
    always @(posedge clk)
        if (reset) ddr_reset_count <= 255;
        else if (ddr_reset_count != 0) ddr_reset_count <= ddr_reset_count - 1'b1;

    wire         ui_clk, ui_reset, ddr_calibrated;
    wire [27:0]  app_addr;
    wire [2:0]   app_cmd;
    wire         app_en, app_wdf_end, app_wdf_wren, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [127:0] app_wdf_data, app_rd_data;
    wire [15:0]  app_wdf_mask;
    ddr_memory ddr (
        .oscillator(oscillator), .reference_200(delay_clk), .reset(reset || ddr_reset_count != 0),
        .ui_clk(ui_clk), .ui_reset(ui_reset), .calibrated(ddr_calibrated),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_ras_n(ddr3_ras_n),
        .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke),
        .ddr3_cs_n(ddr3_cs_n), .ddr3_odt(ddr3_odt), .ddr3_dm(ddr3_dm));

    (* ASYNC_REG = "TRUE" *) reg [1:0] calibrated_sync = 0;
    always @(posedge clk) calibrated_sync <= {calibrated_sync[0], ddr_calibrated};
    wire memory_ready = calibrated_sync[1];

    // ---- stream -----------------------------------------------------------------------------------------------
    wire [63:0] stream_pairs, lost_pairs;
    wire [31:0] stream_crc, records, reorder_overflow0, reorder_overflow1, lost_units, discarded_units;
    wire [31:0] ring_input_overflow, ring_output_overflow, ring_errors, usb_underruns, usb_max_stall;
    wire [14:0] reorder_peak0, reorder_peak1;
    wire [24:0] ring_used, ring_peak;
    wire        stream_ended;
    stream stream (
        .clk(clk), .reset(clear || !memory_ready), .flush(stream_started && !armed),
        .valid(lane_valid), .last(lane_last), .pair0(lane_pair[0]), .pair1(lane_pair[1]),
        .unit_valid(unit_valid), .unit_sequence0(lane_sequence[0]),
        .unit_sequence1(lane_sequence[1]), .unit_first0(unit_first[0]),
        .unit_first1(unit_first[1]), .unit_count0(unit_count[0]), .unit_count1(unit_count[1]),
        .ui_clk(ui_clk), .ui_reset(ui_reset), .calibrated(ddr_calibrated),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .ft_clk(ft_clk), .ft_txe(ft_txe), .ft_data(ft_data), .ft_be(ft_be), .ft_wr(ft_wr),
        .pairs(stream_pairs), .lost_pairs(lost_pairs), .lost_units(lost_units),
        .discarded_units(discarded_units),
        .stream_crc(stream_crc), .records(records), .ended(stream_ended),
        .reorder_overflow0(reorder_overflow0), .reorder_overflow1(reorder_overflow1),
        .reorder_peak0(reorder_peak0),
        .reorder_peak1(reorder_peak1), .ring_input_overflow(ring_input_overflow),
        .ring_output_overflow(ring_output_overflow), .ring_errors(ring_errors),
        .ring_used(ring_used), .ring_peak(ring_peak), .usb_underruns(usb_underruns),
        .usb_max_stall(usb_max_stall));

    // ---- status -------------------------------------------------------------------------------------------------
    // Every statistic is in this clock domain (those from the sample, DDR and
    // FT600 domains arrive as snapshots), so any of them can be read live.
    wire [31:0] flags = (armed ? `FPGA_FLAG_ARMED : 0) |
                        (memory_ready ? `FPGA_FLAG_DDR_READY : 0) |
                        (deskew_ready ? `FPGA_FLAG_DESKEW_READY : 0) |
                        (phase_ready ? `FPGA_FLAG_PHASE_READY : 0) |
                        (reference_sync[1] ? `FPGA_FLAG_REFERENCE_ON : 0) |
                        (stream_ended ? `FPGA_FLAG_STREAM_ENDED : 0) |
                        (stream_open ? `FPGA_FLAG_STREAM_OPEN : 0);

    function automatic [31:0] statistic(input [15:0] index);
        case (index)
            `FPGA_STAT_FLAGS: statistic = flags;
            `FPGA_STAT_PAIRS_LO: statistic = stream_pairs[31:0];
            `FPGA_STAT_PAIRS_HI: statistic = stream_pairs[63:32];
            `FPGA_STAT_STREAM_CRC: statistic = stream_crc;
            `FPGA_STAT_RECORDS: statistic = records;
            `FPGA_STAT_UNITS0: statistic = units[0];
            `FPGA_STAT_UNITS1: statistic = units[1];
            `FPGA_STAT_FRAMING0: statistic = framing_errors[0] + unpack_errors[0];
            `FPGA_STAT_FRAMING1: statistic = framing_errors[1] + unpack_errors[1];
            `FPGA_STAT_CHECKSUM0: statistic = checksum_errors[0];
            `FPGA_STAT_CHECKSUM1: statistic = checksum_errors[1];
            `FPGA_STAT_END_MARK0: statistic = end_marker_errors[0];
            `FPGA_STAT_END_MARK1: statistic = end_marker_errors[1];
            `FPGA_STAT_SAMPLE_OVERFLOW: statistic = sample_overflow;
            `FPGA_STAT_REORDER_OVERFLOW0: statistic = reorder_overflow0;
            `FPGA_STAT_REORDER_OVERFLOW1: statistic = reorder_overflow1;
            `FPGA_STAT_LOST_UNITS: statistic = lost_units;
            `FPGA_STAT_RING_INPUT_OVERFLOW: statistic = ring_input_overflow;
            `FPGA_STAT_RING_OUTPUT_OVERFLOW: statistic = ring_output_overflow;
            `FPGA_STAT_RING_ERRORS: statistic = ring_errors;
            `FPGA_STAT_RING_USED: statistic = {7'b0, ring_used};
            `FPGA_STAT_RING_PEAK: statistic = {7'b0, ring_peak};
            `FPGA_STAT_USB_UNDERRUNS: statistic = usb_underruns;
            `FPGA_STAT_USB_MAX_STALL: statistic = usb_max_stall;
            `FPGA_STAT_REORDER_PEAK0: statistic = {17'b0, reorder_peak0};
            `FPGA_STAT_REORDER_PEAK1: statistic = {17'b0, reorder_peak1};
            `FPGA_STAT_PHASE: statistic = {23'b0, phase};
            `FPGA_STAT_LOST_PAIRS_LO: statistic = lost_pairs[31:0];
            `FPGA_STAT_LOST_PAIRS_HI: statistic = lost_pairs[63:32];
            `FPGA_STAT_DISCARDED_UNITS: statistic = discarded_units;
            default: statistic = 0;
        endcase
    endfunction

    // ---- control ------------------------------------------------------------------------------------------------
    wire        request_valid;
    wire [7:0]  op;
    wire [15:0] arg;
    reg  [7:0]  status = 0;
    wire [31:0] value = op == `CTL_INFO ? 32'(`CTL_FPGA_FIRMWARE_ID)
                      : op == `CTL_STATUS ? statistic(arg)
                      : flags;
    control_port #(.DIVIDER(UART_DIVIDER)) control (
        .clk(clk), .reset(reset), .rx(uart_rx), .tx(uart_tx), .request_valid(request_valid),
        .op(op), .arg(arg), .status(status), .value(value));

    always @(posedge clk) begin
        arm <= 0;
        disarm <= 0;
        if (reset) begin
            reference_enable <= 0;
            phase_target <= DEFAULT_PHASE;
            stream_open <= 0;
        end else if (request_valid) begin
            status <= `CTL_OK;
            case (op)
                `CTL_INFO: if (arg != 0) status <= `CTL_BAD_ARGUMENT;
                `CTL_SAFE: disarm <= 1;
                `CTL_STATUS: if (arg >= `FPGA_STAT_COUNT) status <= `CTL_BAD_ARGUMENT;
                `FPGA_REFERENCE:
                    if (arg == 0) reference_enable <= 0;
                    else if (arg != `FPGA_REFERENCE_KEY) status <= `CTL_BAD_ARGUMENT;
                    else if (!clocks_locked) status <= `CTL_NOT_READY;
                    else reference_enable <= 1;
                `FPGA_ARM:
                    if (stream_open) status <= `CTL_BUSY;
                    else if (memory_ready && deskew_ready && phase_ready) begin
                        arm <= 1;
                        stream_open <= 1;
                    end else status <= `CTL_NOT_READY;
                `FPGA_STOP: disarm <= 1;
                `FPGA_RELEASE: begin
                    disarm <= 1;
                    stream_open <= 0;
                end
                `FPGA_PHASE:
                    if (arg >= `FPGA_PHASE_STEPS) status <= `CTL_BAD_ARGUMENT;
                    else if (armed) status <= `CTL_BUSY;
                    else phase_target <= arg[8:0];
                default: status <= `CTL_UNKNOWN_OP;
            endcase
        end
    end
endmodule
