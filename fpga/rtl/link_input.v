// Link line capture: per-line input delays, then each line sampled on both
// edges of the 240 MHz sample clock in its input tile (IDDR). Two consecutive
// clock periods, four samples per line, are handed to the 120 MHz domain.
//
// The per-line IDELAY taps compensate the measured board skew; they are
// loaded once after IDELAYCTRL calibration and verified by reading back.
module link_input #(
    // Tap values (78 ps each) for link lines 15..0, measured with the ESP
    // driving 10 mA: lines whose data ends early are delayed so that every
    // line's data stays valid as long after the marker edge as possible.
    parameter [79:0] TAPS = {5'd0, 5'd0, 5'd5, 5'd4, 5'd2, 5'd5, 5'd2, 5'd5,
                             5'd2, 5'd3, 5'd3, 5'd1, 5'd3, 5'd4, 5'd7, 5'd3}
) (
    input  wire [15:0] link,
    input  wire        sample_clk,      // 240 MHz
    input  wire        delay_clk,       // 200 MHz IDELAY reference
    input  wire        clk,             // 120 MHz
    input  wire        reset,           // clk domain
    input  wire        clear,           // clk domain: clears the capture path
    input  wire        run,             // clk domain
    output wire        deskew_ready,    // clk domain
    output wire        pair_valid,
    // Per 120 MHz clock: {falling, rising} samples of the later period above
    // those of the earlier one; each sample is the 16 lines.
    output wire [63:0] pair,
    output wire [31:0] overflow         // clk domain: capture FIFO overflows
);
    // ---- fixed per-line delays ----
    wire calibrated;
    (* IODELAY_GROUP = "link" *) IDELAYCTRL delay_control (.REFCLK(delay_clk), .RST(reset), .RDY(calibrated));

    reg [2:0] settle = 0;
    always @(posedge clk)
        if (reset || !calibrated) settle <= 0;
        else if (settle != 7) settle <= settle + 1'b1;
    wire load_taps = !reset && calibrated && settle == 1;
    wire [79:0] tap_readback;
    assign deskew_ready = !reset && calibrated && settle == 7 && tap_readback == TAPS;

    wire [15:0] delayed, rise, fall;
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : lines
            wire raw;
            IBUF input_buffer (.I(link[i]), .O(raw));
            (* IODELAY_GROUP = "link" *) IDELAYE2 #(
                .DELAY_SRC("IDATAIN"), .IDELAY_TYPE("VAR_LOAD"), .HIGH_PERFORMANCE_MODE("TRUE"),
                .REFCLK_FREQUENCY(200.0), .SIGNAL_PATTERN("DATA")
            ) delay (
                .IDATAIN(raw), .DATAIN(1'b0), .DATAOUT(delayed[i]), .C(clk), .CE(1'b0),
                .INC(1'b0), .LD(load_taps), .CNTVALUEIN(TAPS[i * 5 +: 5]),
                .CNTVALUEOUT(tap_readback[i * 5 +: 5]), .LDPIPEEN(1'b0), .REGRST(1'b0),
                .CINVCTRL(1'b0));
            // rise at the rising edge, fall half a period later, presented
            // together at the next rising edge.
            IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), .SRTYPE("SYNC")) capture (
                .C(sample_clk), .CE(1'b1), .D(delayed[i]), .R(1'b0), .S(1'b0),
                .Q1(rise[i]), .Q2(fall[i]));
        end
    endgenerate

    // ---- pairing of sample periods ----
    reg [31:0] sampled = 0;             // {fall, rise}
    always @(posedge sample_clk)
        sampled <= {fall, rise};

    (* ASYNC_REG = "TRUE" *) reg [1:0] run_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] clear_sync = 2'b11;
    always @(posedge sample_clk) begin
        run_sync <= {run_sync[0], run};
        clear_sync <= {clear_sync[0], clear};
    end

    reg        second = 0;
    reg        push = 0;
    reg [31:0] previous = 0;
    reg [63:0] pair_data = 0;
    always @(posedge sample_clk) begin
        push <= 0;
        if (clear_sync[1]) begin
            second <= 0;
            previous <= 0;
        end else if (run_sync[1]) begin
            previous <= sampled;
            second <= !second;
            if (second) begin
                pair_data <= {sampled, previous};
                push <= 1;
            end
        end
    end

    wire [31:0] sample_overflow;
    async_fifo #(.WIDTH(64)) sample_fifo (
        .wr_clk(sample_clk), .wr_reset(clear_sync[1]), .wr_valid(push), .wr_data(pair_data),
        .full(), .overflow(sample_overflow), .wr_level(),
        .rd_clk(clk), .rd_reset(clear), .rd_enable(1'b1),
        .rd_valid(pair_valid), .rd_data(pair), .empty(), .rd_level());

    snapshot overflow_status (
        .source_clk(sample_clk), .source_value(sample_overflow), .clk(clk), .value(overflow));
endmodule
