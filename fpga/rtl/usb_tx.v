// FT600 245-mode transmitter (16-bit, 100 MHz, write only).
//
// Data is sent in whole transfers of WORDS 16-bit words. A transfer starts
// only once a complete block is prefetched (`available` >= 255 beats), and WR
// then stays asserted for the whole transfer, including across TXE stalls.
//
// A word on the pads is accepted at FT edge k if WR is low at k and TXE, as
// driven after edge k-1, is low. So TXE from edge k-1 decides what the pads
// show at edge k+1, within one cycle, and TXE's pin is on the other side of
// the die from the clock, WR and data pins.
//
// Clocking (all from the FT600's 100 MHz clock, see stream.v):
//   control       samples TXE in its input register, 1.625 ns after the FT
//                 edge, well inside TXE's valid window
//   launch        data launch, 4.75 ns
//   data_launch   `launch` gated by !TXE through a glitchless BUFGCE, so the
//                 data pads hold their word while the FT600 is not accepting
//   write_launch  WR launch, 5.25 ns
//   work          internal logic, 9.75 ns
// The data-pad clock enable and the WR decision take TXE straight from its
// pin (a registered copy could not cross the die in time); both use TXE from
// the preceding FT edge, and static timing checks their setup and hold.
module usb_tx #(
    parameter WORDS = 524288            // one transfer: 1 MiB
) (
    input  wire         control,
    input  wire         launch,
    input  wire         data_launch,
    input  wire         write_launch,
    input  wire         work,
    input  wire         reset,

    input  wire         valid,          // prefetch FIFO head (show-ahead)
    input  wire [127:0] data,
    input  wire [9:0]   available,
    output wire         pop,

    input  wire         ft_txe,
    output wire         data_clock_enable,
    output wire [15:0]  ft_data,
    output wire [1:0]   ft_be,
    output wire         ft_wr,

    output reg  [31:0]  underruns = 0,
    output reg  [31:0]  max_stall = 0   // longest TXE-high stall inside a transfer, cycles
);
    (* IOB = "TRUE" *) reg sampled_txe = 1;
    reg sampled_wr = 1;
    (* IOB = "TRUE" *) reg wr_pad = 1;

    reg         loaded = 0;             // a word was launched onto the pads this cycle
    reg         word_valid = 0;         // `word` holds the next word to launch
    reg         pad_valid = 0;          // the pads hold a word not yet accepted
    reg         running = 0;            // inside a transfer
    reg         halted = 0;             // stopped by an underrun until reset
    reg [15:0]  word = 0;
    reg [127:0] block = 0;
    reg [2:0]   beat = 0;               // 16-bit word within `block`
    reg [19:0]  left = 0;               // words left in the transfer
    reg         words_left = 0;         // left != 0, registered for the pad-side paths
    reg [31:0]  stall = 0;
    reg         pad_reset = 1;          // reset for the launch and WR registers

    wire need_block = !word_valid || (loaded && beat == 7);
    assign pop = !reset && need_block && valid;

    always @(posedge control) begin
        sampled_txe <= ft_txe;
        sampled_wr <= wr_pad;
    end

    // Retimed from the work clock, which leaves the launch and WR clocks
    // most of a cycle, whatever the TXE sample phase.
    always @(posedge work) pad_reset <= reset;

    always @(posedge work) begin
        if (reset) begin
            beat <= 0;
            word_valid <= 0;
            word <= 0;
            block <= 0;
            pad_valid <= 0;
            running <= 0;
            halted <= 0;
            left <= 0;
            words_left <= 0;
            stall <= 0;
            max_stall <= 0;
            underruns <= 0;
        end else begin
            if (!running && !halted && word_valid && available >= 255 && !sampled_txe) begin
                running <= 1;
                left <= WORDS;
                words_left <= 1;
            end
            if (!sampled_wr) begin
                if (!sampled_txe) begin
                    // The FT600 accepted the word on the pads.
                    stall <= 0;
                    if (!loaded) begin
                        pad_valid <= 0;
                        if (left == 0) begin
                            running <= 0;
                        end else begin
                            underruns <= underruns + 1;
                            halted <= 1;
                            running <= 0;
                        end
                    end
                end else begin
                    if (stall != 32'hFFFFFFFF)
                        stall <= stall + 1;
                    if (stall >= max_stall && stall != 32'hFFFFFFFF)
                        max_stall <= stall + 1;
                end
            end
            if (loaded) begin
                pad_valid <= 1;
                left <= left - 1;
                words_left <= left != 1;
            end
            if (need_block) begin
                word_valid <= valid;
                if (valid) begin
                    block <= data;
                    word <= data[15:0];
                    beat <= 0;
                end
            end else if (loaded) begin
                beat <= beat + 1;
                word <= block[({1'b0, beat} + 1) * 16 +: 16];
            end
        end
    end

    wire loading = running && word_valid && words_left;

    always @(posedge launch)
        loaded <= !pad_reset && loading && (!pad_valid || !sampled_txe);

    assign data_clock_enable = !ft_txe;

`ifdef SYNTHESIS
    // Data pads: clocked by the TXE-gated launch clock.
    (* IOB = "TRUE" *) reg [15:0] data_register = 0;
    always @(posedge data_launch)
        if (pad_reset) data_register <= 0;
        else data_register <= word;

    // The next FT edge's TXE reaches the WR register only after it has
    // captured the preceding edge's; static timing checks both.
    wire txe_for_wr = ft_txe;

    // WR next state: low while a word is being loaded, or while the pads
    // hold a word the FT600 has not accepted yet; released otherwise, which
    // ends the transfer. Instantiated so the placement constraint has a
    // stable cell to pin beside WR's IOB.
    wire wr_next;
    (* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h007F7F7F)) wr_next_lut (
        .I0(words_left), .I1(word_valid), .I2(running), .I3(txe_for_wr), .I4(pad_valid),
        .O(wr_next));
`else
    // Simulation model of the same timed paths.
    reg [15:0] data_register = 0;
    always @(posedge launch)
        if (pad_reset) data_register <= 0;
        else if (!sampled_txe) data_register <= word;
    wire txe_for_wr = sampled_txe;
    wire wr_next = !(pad_valid && txe_for_wr) && !loading;
`endif

    always @(posedge write_launch)
        if (pad_reset || !running) wr_pad <= 1;
        else wr_pad <= wr_next;

    assign ft_data = data_register;
    assign ft_wr = wr_pad;
    assign ft_be = 2'b11;
endmodule
