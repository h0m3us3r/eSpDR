`include "protocol.vh"
// Merges the two lanes back into chronological order.
//
// Units alternate between lanes (even sequence numbers on lane 0), and both
// lanes transmit concurrently, so each lane is buffered in its own queue and
// whole units are drained in sequence order. Each queue holds a complete
// maximum-length unit. An overflow drops the pair and is counted; the lanes
// are never back-pressured.
//
// A lane announces every unit it accepts (sequence, ring position, length)
// before its pairs. A sequence number that never arrives is a lost unit: the
// merge emits a skip of exactly the missing pairs and carries on. Units are
// contiguous in the ESP's 16384-pair ring, so the gap is the ring distance
// from the end of the last unit delivered to the start of the next one
// received, plus whole ring turns fixed by the number of units missing.
//
// A unit whose header was misread is dropped with its pairs: a sequence
// number already passed, a ring position that cannot follow the last unit
// delivered, a sequence number far from the other lane's, or one far ahead
// that the other lane does not confirm. Right after lost units, a number a
// few units off can still fit the ring position and be taken; that unit's
// header then fails its lane checksum, so the run reports it. Its pairs are dropped as they
// arrive, alongside the merge, so the other lane never waits for them.
module reorder #(
    parameter ADDR_BITS = 14,
    parameter FAR = 64,                 // units apart: the lanes disagree
    parameter CONFIRM_LEVEL = 3 << (ADDR_BITS - 2)  // stop waiting for the other lane
) (
    input  wire        clk,
    input  wire        reset,
    input  wire [1:0]  valid,
    input  wire [1:0]  last,
    input  wire [19:0] pair0,
    input  wire [19:0] pair1,
    input  wire [1:0]  unit_valid,      // a lane accepted a unit header
    input  wire [31:0] unit_sequence0,
    input  wire [31:0] unit_sequence1,
    input  wire [13:0] unit_first0,
    input  wire [13:0] unit_first1,
    input  wire [13:0] unit_count0,
    input  wire [13:0] unit_count1,

    output reg         out_valid = 0,
    input  wire        out_ready,
    output reg  [19:0] out_pair = 0,
    output reg         skip_valid = 0,  // skip_pairs are missing before the next pair
    input  wire        skip_ready,
    output reg  [31:0] skip_pairs = 0,

    output wire [31:0] overflow0,
    output wire [31:0] overflow1,
    output reg  [31:0] lost_units = 0,
    output reg  [31:0] discarded_units = 0,
    output reg  [ADDR_BITS:0] peak0 = 0,
    output reg  [ADDR_BITS:0] peak1 = 0
);
    localparam RING = 16384;

    // ---- per-lane pair queues ----
    wire [1:0]  available;
    wire [20:0] head [0:1];             // {last, pair}
    wire [ADDR_BITS:0] level [0:1];
    wire [1:0]  pop;

    async_fifo #(.WIDTH(21), .ADDR_BITS(ADDR_BITS), .RAM_STYLE("block"), .SHOW_AHEAD(1)) lane0 (
        .wr_clk(clk), .wr_reset(reset), .wr_valid(valid[0]), .wr_data({last[0], pair0}),
        .full(), .overflow(overflow0), .wr_level(level[0]),
        .rd_clk(clk), .rd_reset(reset), .rd_enable(pop[0]),
        .rd_valid(available[0]), .rd_data(head[0]), .empty(), .rd_level());

    async_fifo #(.WIDTH(21), .ADDR_BITS(ADDR_BITS), .RAM_STYLE("block"), .SHOW_AHEAD(1)) lane1 (
        .wr_clk(clk), .wr_reset(reset), .wr_valid(valid[1]), .wr_data({last[1], pair1}),
        .full(), .overflow(overflow1), .wr_level(level[1]),
        .rd_clk(clk), .rd_reset(reset), .rd_enable(pop[1]),
        .rd_valid(available[1]), .rd_data(head[1]), .empty(), .rd_level());

    // ---- per-lane unit announcements: {sequence, first, count} ----
    wire [1:0]  unit_available;
    wire [59:0] unit_head [0:1];
    wire [1:0]  unit_pop;

    async_fifo #(.WIDTH(60), .ADDR_BITS(3), .SHOW_AHEAD(1)) units0 (
        .wr_clk(clk), .wr_reset(reset), .wr_valid(unit_valid[0]),
        .wr_data({unit_sequence0, unit_first0, unit_count0}),
        .full(), .overflow(), .wr_level(),
        .rd_clk(clk), .rd_reset(reset), .rd_enable(unit_pop[0]),
        .rd_valid(unit_available[0]), .rd_data(unit_head[0]), .empty(), .rd_level());

    async_fifo #(.WIDTH(60), .ADDR_BITS(3), .SHOW_AHEAD(1)) units1 (
        .wr_clk(clk), .wr_reset(reset), .wr_valid(unit_valid[1]),
        .wr_data({unit_sequence1, unit_first1, unit_count1}),
        .full(), .overflow(), .wr_level(),
        .rd_clk(clk), .rd_reset(reset), .rd_enable(unit_pop[1]),
        .rd_valid(unit_available[1]), .rd_data(unit_head[1]), .empty(), .rd_level());

    wire [31:0] head_sequence [0:1];
    wire [13:0] head_first [0:1];
    wire [13:0] head_count [0:1];
    assign {head_sequence[0], head_first[0], head_count[0]} = unit_head[0];
    assign {head_sequence[1], head_first[1], head_count[1]} = unit_head[1];

    // ---- merge ----
    localparam SELECT = 0, DECIDE = 1, DRAIN = 2, GAP = 3, SKIP = 4;
    reg [2:0]  state = SELECT;
    reg [1:0]  dropping = 0;            // the lane's head unit is being dropped
    reg [31:0] next_sequence = 0;       // the unit to deliver next
    reg [13:0] last_end = 0;            // ring index just past the last delivered unit
    reg [31:0] resume_sequence = 0;     // first unit present after a gap
    reg [13:0] resume_first = 0;
    reg [31:0] gap_units = 0;
    reg [31:0] gap_pairs = 0;
    reg [31:0] gap_minimum = 0;

    wire lane = next_sequence[0];
    wire draining_pair = state == DRAIN && (!out_valid || out_ready) && available[lane];
    wire [1:0] drain_pop = draining_pair ? (lane ? 2'b10 : 2'b01) : 2'b00;
    wire [1:0] drop_pop = dropping & available;
    wire [1:0] head_last = {head[1][20], head[0][20]};
    assign pop = drain_pop | drop_pop;
    // A unit's announcement is released with its last pair.
    assign unit_pop = pop & head_last;
    wire [1:0] dropped = drop_pop & head_last;

    // Classification of the announced units that are not being dropped,
    // registered: the merge decides in DECIDE on what SELECT showed. A head
    // announcement changes only when popped, and neither DRAIN nor a drop
    // pops a lane that is present, so the registered view stays current.
    wire [1:0] present = unit_available & ~dropping;
    wire [1:0] behind, near, beyond;     // before next_sequence; at most 2 / over FAR ahead
    wire [1:0] placed;                   // ring position consistent with the sequence number
    genvar l;
    generate for (l = 0; l < 2; l = l + 1) begin : compare
        wire [31:0] ahead = head_sequence[l] - next_sequence;
        assign behind[l] = ahead[31];
        assign near[l] = ahead <= 2;
        assign beyond[l] = ahead > FAR;
        // A unit d units after the next one starts d whole ring turns, less
        // 96..1024 pairs per unit, after the last unit's end (each unit holds
        // LINK_MIN_PAIRS..LINK_MAX_PAIRS). Beyond 15 units any position fits.
        wire [3:0]  d = ahead[3:0];
        wire [13:0] back = last_end - head_first[l];
        assign placed[l] = ahead[31:4] != 0 ||
                           (d == 0 ? back == 0
                                   : back >= d * (RING - `LINK_MAX_PAIRS) &&
                                     back <= d * (RING - `LINK_MIN_PAIRS));
    end endgenerate
    wire [31:0] distance = head_sequence[0] - head_sequence[1];   // negative: lane 1 is ahead
    wire [31:0] reverse_distance = head_sequence[1] - head_sequence[0];
    wire far = distance > FAR && reverse_distance > FAR;

    reg  [1:0]  stale = 0;               // present but already passed, or misplaced
    reg         expected = 0;            // the next unit is present
    reg         lanes_disagree = 0;
    reg  [1:0]  outlier = 0;             // the lane with the larger sequence
    reg  [1:0]  any_present = 0;
    reg         both_present = 0;
    reg         next_dropping = 0;
    reg         earliest_near = 0;
    reg         earliest_filling = 0;       // its queue is filling up
    reg         earliest_beyond = 0;
    reg  [1:0]  earliest_lane = 0;
    reg  [31:0] earliest_sequence = 0;
    reg  [13:0] earliest_first = 0;
    always @(posedge clk) begin
        stale <= present & (behind | ~placed);
        expected <= present[lane] && head_sequence[lane] == next_sequence;
        lanes_disagree <= &present && far;
        outlier <= distance[31] ? 2'b10 : 2'b01;
        any_present <= |present;
        both_present <= &present;
        next_dropping <= dropping[lane];
        if (present[1] && (!present[0] || !distance[31])) begin
            earliest_near <= near[1];
            earliest_filling <= level[1] >= CONFIRM_LEVEL;
            earliest_beyond <= beyond[1];
            earliest_lane <= 2'b10;
            earliest_sequence <= head_sequence[1];
            earliest_first <= head_first[1];
        end else begin
            earliest_near <= near[0];
            earliest_filling <= level[0] >= CONFIRM_LEVEL;
            earliest_beyond <= beyond[0];
            earliest_lane <= 2'b01;
            earliest_sequence <= head_sequence[0];
            earliest_first <= head_first[0];
        end
    end
    // A lone unit far ahead is accepted only with the other lane's agreement:
    // once its queue fills without it, the unit is taken as misread.
    wire lone_beyond = any_present && !both_present && !next_dropping &&
                       !earliest_near && earliest_filling && earliest_beyond;
    wire [1:0] drop = state != DECIDE ? 2'b00
                    : |stale ? stale
                    : expected ? 2'b00
                    : lanes_disagree ? outlier
                    : lone_beyond ? earliest_lane : 2'b00;

    always @(posedge clk) begin
        if (reset) begin
            state <= SELECT;
            next_sequence <= 0;
            last_end <= 0;              // the ESP's first unit starts at ring index 0
            out_valid <= 0;
            skip_valid <= 0;
            lost_units <= 0;
            discarded_units <= 0;
            dropping <= 0;
            peak0 <= 0;
            peak1 <= 0;
        end else begin
            if (level[0] > peak0) peak0 <= level[0];
            if (level[1] > peak1) peak1 <= level[1];
            dropping <= (dropping & ~dropped) | drop;
            discarded_units <= discarded_units + dropped[0] + dropped[1];
            if (!out_valid || out_ready) out_valid <= 0;

            case (state)
                // A unit's header reaches us about 190 us before the next
                // unit's header on the other lane (the ESP cores alternate).
                // So once a later unit is announced while this one is not,
                // it is lost; resume at the earliest unit announced. A unit
                // further ahead is accepted at once only if both lanes agree.
                // Otherwise the merge waits for the silent lane while the
                // waiting lane's queue has room: until CONFIRM_LEVEL, about
                // 300 us into a unit, where waiting longer would overflow.
                // Then a unit up to FAR ahead is accepted and one beyond FAR
                // is dropped.
                // A unit that the lane is still dropping may hide the next
                // one behind it, so the merge waits for the drop to end.
                SELECT: if (!out_valid) state <= DECIDE;

                DECIDE: begin
                    state <= SELECT;
                    if (drop != 0) begin
                        // dropping is set below
                    end else if (expected) begin
                        state <= DRAIN;
                    end else if (any_present && !next_dropping) begin
                        if (earliest_near || both_present || earliest_filling) begin
                            resume_sequence <= earliest_sequence;
                            resume_first <= earliest_first;
                            state <= GAP;
                        end
                    end
                end

                DRAIN: if (draining_pair) begin
                    out_valid <= 1;
                    out_pair <= head[lane][19:0];
                    if (head[lane][20]) begin
                        last_end <= head_first[lane] + head_count[lane];
                        next_sequence <= next_sequence + 1;
                        state <= SELECT;
                    end
                end

                // gap = ring distance + whole turns, such that each of the
                // missing units has at least LINK_MIN_PAIRS.
                GAP: begin
                    gap_units <= resume_sequence - next_sequence;
                    gap_pairs <= {18'b0, resume_first - last_end};
                    gap_minimum <= (resume_sequence - next_sequence) * `LINK_MIN_PAIRS;
                    state <= SKIP;
                end

                SKIP: if (gap_pairs < gap_minimum) begin
                    gap_pairs <= gap_pairs + RING;
                end else if (!skip_valid) begin
                    skip_valid <= 1;
                    skip_pairs <= gap_pairs;
                end else if (skip_ready) begin
                    skip_valid <= 0;
                    lost_units <= lost_units + gap_units;
                    last_end <= resume_first;
                    next_sequence <= resume_sequence;
                    state <= SELECT;
                end
            endcase
        end
    end
endmodule
