`include "protocol.vh"
// Receiver for one 8-bit link lane (see protocol/link.h).
//
// Input: two consecutive 240 MHz periods of the lane, delivered at 120 MHz;
// each period brings a sample at the rising clock edge and one half a period
// later, at the falling edge. All cycle numbers below count 240 MHz periods
// (= ESP CPU cycles) from the unit's marker.
//
// The marker's falling edge fixes the integer alignment of each unit. It is
// found on the falling-edge samples, and the rising-edge sample of the same
// period is cycle 240. After that no data value influences timing: every
// byte is taken from the rising-edge sample at a cycle computed from the
// transmit kernel's fixed schedule, in the middle of the interval it is held
// for. So when the sampling phase puts the falling-edge samples half a period
// from the ESP's transitions, which keeps marker detection clear of any
// edge, the data samples sit exactly in the middle of every byte. Only the final byte of each 40-byte group is
// held longer (the kernel's loop and fragment overheads); its hold time is
// derived from the same schedule.
//
// The unit's samples are forwarded with byte_index; checksum and end marker
// errors are counted but do not discard data. A malformed header (wrong
// version, length out of range, a sequence number of the other lane, or a
// capture bank other than sequence % 4, where the ESP always puts it) is
// counted and the lane resumes at the next marker. Each accepted unit is
// announced on unit_valid before its first pair; the reorder stage judges
// its sequence number and turns units that never arrived into gaps.
module link_lane #(
    parameter LANE = 0
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        run,
    input  wire        pair_valid,
    input  wire [31:0] samples,         // {later fall, later rise, earlier fall, earlier rise}

    output reg         byte_valid = 0,
    output reg  [7:0]  byte_value = 0,
    output reg  [15:0] byte_index = 0,  // 0..7 header, 8.. payload and trailer
    output reg  [31:0] sequence_word = 0,
    output reg         unit_valid = 0,  // header accepted: sequence_word, unit_first, unit_count
    output reg  [13:0] unit_first = 0,  // ring index of the unit's first pair
    output reg  [13:0] unit_count = 0,
    output reg  [3:0]  head_pairs = 0,  // valid pairs in a leading partial group
    output reg  [3:0]  tail_pairs = 0,  // valid pairs in a trailing partial group
    output reg  [10:0] total_groups = 0,

    output reg  [31:0] units = 0,
    output reg  [31:0] framing_errors = 0,
    output reg  [31:0] checksum_errors = 0,
    output reg  [31:0] end_marker_errors = 0
);
    localparam IDLE = 0, HEADER = 1, PREP0 = 2, PREP1 = 3, PREP2 = 4, PAYLOAD = 5,
               TRAILER = 6;

    // Transmit kernel schedule (esp32s3/src/transmit.S).
    localparam MARKER_MIN_EARLY = 238;  // 0xFF periods before an edge in the earlier period
    localparam MARKER_MIN_LATE = 237;   // ... in the later period
    localparam FIRST_HEADER_CYCLE = 243;
    localparam FIRST_PAYLOAD_CYCLE = 263;
    localparam GROUP_GAP = 13;          // hold of a group's last byte inside a fragment

    reg [2:0]  state = IDLE;
    reg [16:0] cycle = 0;               // cycle of the earlier period
    reg [16:0] cycle_late = 1;
    reg [16:0] next_cycle = 0;          // header/trailer: next byte's sample cycle
    reg [16:0] write_cycle = 0;         // payload: cycle the current byte was written
    reg [16:0] select_cycle = 0;        // payload: cycle to sample the current byte
    reg [8:0]  marker_length = 0;

    wire [7:0] early = samples[7:0];        // data: rising-edge samples
    wire [7:0] late = samples[23:16];
    wire [7:0] early_fall = samples[15:8];  // marker: falling-edge samples
    wire [7:0] late_fall = samples[31:24];
    // The marker ends at the first falling-edge sample after the 0xFF run that
    // is not 0xFF. A sample taken while the lines are switching reads a mix of
    // old and new bits; it still marks the edge.
    wire marker_end_early = early_fall != 8'hFF && marker_length >= MARKER_MIN_EARLY;
    wire marker_end_late = early_fall == 8'hFF && late_fall != 8'hFF && marker_length >= MARKER_MIN_LATE;

    wire [16:0] wanted_cycle = (state == HEADER || state == TRAILER) ? next_cycle : select_cycle;
    wire take_early = cycle == wanted_cycle;
    wire take_late = cycle_late == wanted_cycle;
    wire [7:0] sampled = take_late ? late : early;

    reg [2:0]  header_at = 0;
    reg [2:0]  trailer_at = 0;
    reg [31:0] layout = 0;
    reg [1:0]  fragment = 0;            // 0 head, 1 bulk, 2 wrapped bulk, 3 tail
    reg [10:0] groups [0:3];
    reg [10:0] group_at = 0;
    reg [5:0]  group_byte = 0;          // 0..39 within the current group
    reg [15:0] payload_index = 8;
    reg [14:0] bulk_pairs = 0, aligned_start = 0, until_wrap = 0;
    reg [13:0] pair_count = 0;
    reg [31:0] checksum = 0, wire_checksum = 0;
    reg [7:0]  checksum_low = 0;
    reg        end_marker_bad = 0;

    // Hold time of the byte after the last group, per the kernel's trailer
    // path from each fragment.
    wire [6:0] trailer_gap = fragment == 0 ? 67 : fragment == 1 ? 46 : fragment == 2 ? 28 : 8;

    // Next non-empty fragment and the cost of moving to it.
    reg [1:0] following;
    reg       have_following;
    reg [6:0] dispatch_cost;
    reg [6:0] hold_cycles;
    reg [6:0] last_hold = 2;
    integer f;
    always @* begin
        following = 0;
        have_following = 0;
        dispatch_cost = 0;
        for (f = 3; f >= 1; f = f - 1)
            if (f > fragment && groups[f] != 0) begin
                following = 2'(f);
                have_following = 1;
            end
        if (have_following) begin
            // Last payload write, through fragment dispatch and priming, to
            // the next fragment's first write.
            case (following)
                1: dispatch_cost = 50;
                2: dispatch_cost = 47 + (fragment == 0 ? 21 : 0);
                3: dispatch_cost = 49 + (fragment < 1 ? 21 : 0) + (fragment < 2 ? 18 : 0);
            endcase
            if (groups[following] > 1)
                dispatch_cost = dispatch_cost + 3;
        end
        hold_cycles = 2;
        if (group_byte == 39) begin
            if (group_at + 1 < groups[fragment])
                hold_cycles = GROUP_GAP + ((group_at == 0 && groups[fragment] >= 3) ? 1 : 0);
            else if (have_following)
                hold_cycles = dispatch_cost;
        end
    end

    always @(posedge clk)
        last_hold <= group_at + 1 < groups[fragment]
                         ? 7'd13 + ((group_at == 0 && groups[fragment] >= 3) ? 1 : 0)
                     : have_following ? dispatch_cost
                     : trailer_gap;

    integer k;
    wire sequence_ok = sequence_word[0] == (LANE % 2) && layout[29:28] == sequence_word[1:0];

    always @(posedge clk) begin
        byte_valid <= 0;
        unit_valid <= 0;
        if (reset) begin
            state <= IDLE;
            marker_length <= 0;
            cycle <= 0;
            cycle_late <= 1;
            units <= 0;
            framing_errors <= 0;
            checksum_errors <= 0;
            end_marker_errors <= 0;
            sequence_word <= 0;
            layout <= 0;
            byte_index <= 0;
            checksum <= 0;
            wire_checksum <= 0;
            end_marker_bad <= 0;
            for (k = 0; k < 4; k = k + 1)
                groups[k] <= 0;
        end else if (run && pair_valid) begin
            if (state != IDLE) begin
                cycle <= cycle + 17'd2;
                cycle_late <= cycle_late + 17'd2;
            end
            case (state)
                IDLE: begin
                    // Track the 0xFF run on the falling-edge samples; the
                    // period whose falling-edge sample first shows the
                    // marker's end is cycle 240.
                    if (marker_end_early || marker_end_late) begin
                        cycle <= marker_end_late ? 241 : 242;
                        cycle_late <= marker_end_late ? 242 : 243;
                        next_cycle <= FIRST_HEADER_CYCLE;
                        header_at <= 0;
                        byte_index <= 0;
                        sequence_word <= 0;
                        layout <= 0;
                        checksum <= 0;
                        wire_checksum <= 0;
                        end_marker_bad <= 0;
                        marker_length <= 0;
                        state <= HEADER;
                    end else if (late_fall == 8'hFF) begin
                        if (early_fall == 8'hFF)
                            marker_length <= marker_length >= 509 ? 9'd511 : marker_length + 9'd2;
                        else
                            marker_length <= 1;
                    end else begin
                        marker_length <= 0;
                    end
                end

                HEADER: if (take_early || take_late) begin
                    if (header_at[0])
                        checksum <= checksum + {16'b0, sampled, checksum_low};
                    else
                        checksum_low <= sampled;
                    byte_valid <= 1;
                    byte_value <= sampled;
                    byte_index <= {13'b0, header_at};
                    next_cycle <= header_at == 6 ? FIRST_PAYLOAD_CYCLE : next_cycle + 17'd2;
                    if (header_at < 4)
                        sequence_word[header_at * 8 +: 8] <= sampled;
                    else
                        layout[(header_at - 4) * 8 +: 8] <= sampled;
                    if (header_at == 7)
                        state <= PREP0;
                    else
                        header_at <= header_at + 1'b1;
                end

                // Validate the header and derive the fragment geometry,
                // exactly as the ESP builds its transmit descriptor.
                PREP0: begin
                    if (layout[31:30] != `LINK_VERSION ||
                        layout[27:14] < `LINK_MIN_PAIRS || layout[27:14] > `LINK_MAX_PAIRS ||
                        !sequence_ok) begin
                        framing_errors <= framing_errors + 1;
                        marker_length <= 0;
                        state <= IDLE;
                    end else begin
                        unit_valid <= 1;
                        unit_first <= layout[13:0];
                        unit_count <= layout[27:14];
                        pair_count <= layout[27:14];
                        head_pairs <= 4'(-layout[3:0]);
                        tail_pairs <= 4'(layout[27:14] + layout[3:0]);
                        aligned_start <= {1'b0, layout[13:0]} + {11'b0, 4'(-layout[3:0])};
                        state <= PREP1;
                    end
                end

                PREP1: begin
                    bulk_pairs <= {1'b0, pair_count} - {11'b0, head_pairs} - {11'b0, tail_pairs};
                    until_wrap <= 15'd16384 - {1'b0, aligned_start[13:0]};
                    state <= PREP2;
                end

                PREP2: begin
                    groups[0] <= head_pairs != 0;
                    groups[3] <= tail_pairs != 0;
                    groups[1] <= (until_wrap > bulk_pairs ? bulk_pairs : until_wrap) >> 4;
                    groups[2] <= (bulk_pairs > until_wrap ? bulk_pairs - until_wrap : 15'b0) >> 4;
                    total_groups <= (bulk_pairs >> 4) + (head_pairs != 0) + (tail_pairs != 0);
                    fragment <= head_pairs != 0 ? 0 : 1;
                    group_at <= 0;
                    group_byte <= 0;
                    payload_index <= 8;
                    // A fragment of one group skips the loop setup cycle.
                    if (head_pairs != 0 || (until_wrap > bulk_pairs ? bulk_pairs : until_wrap) == 16) begin
                        write_cycle <= 270;
                        select_cycle <= 271;
                    end else begin
                        write_cycle <= 271;
                        select_cycle <= 272;
                    end
                    state <= PAYLOAD;
                end

                PAYLOAD: if (take_early || take_late) begin
                    if (payload_index[0])
                        checksum <= checksum + {16'b0, sampled, checksum_low};
                    else
                        checksum_low <= sampled;
                    byte_valid <= 1;
                    byte_value <= sampled;
                    byte_index <= payload_index;
                    payload_index <= payload_index + 1'b1;
                    write_cycle <= write_cycle + {10'b0, hold_cycles};
                    // Sample in the middle of each byte's hold interval.
                    if (group_byte == 38)
                        select_cycle <= write_cycle + 17'd2 + {11'b0, last_hold[6:1]};
                    else if (group_byte == 39)
                        select_cycle <= write_cycle + {10'b0, last_hold} + 17'd1;
                    else
                        select_cycle <= select_cycle + 17'd2;

                    if (group_byte != 39) begin
                        group_byte <= group_byte + 1'b1;
                    end else begin
                        group_byte <= 0;
                        if (group_at + 1 < groups[fragment]) begin
                            group_at <= group_at + 1'b1;
                        end else if (have_following) begin
                            fragment <= following;
                            group_at <= 0;
                        end else begin
                            trailer_at <= 0;
                            next_cycle <= write_cycle + {10'b0, trailer_gap} + 17'd1;
                            state <= TRAILER;
                        end
                    end
                end

                TRAILER: if (take_early || take_late) begin
                    byte_valid <= 1;
                    byte_value <= sampled;
                    byte_index <= payload_index;
                    payload_index <= payload_index + 1'b1;
                    next_cycle <= next_cycle + 17'd2;
                    if (trailer_at < 4)
                        wire_checksum[trailer_at * 8 +: 8] <= sampled;
                    else if (sampled != 8'(`LINK_END_MARKER >> ((trailer_at - 4) * 8)))
                        end_marker_bad <= 1;
                    if (trailer_at == 7) begin
                        if (wire_checksum != checksum)
                            checksum_errors <= checksum_errors + 1;
                        if (end_marker_bad || sampled != 8'(`LINK_END_MARKER >> 24))
                            end_marker_errors <= end_marker_errors + 1;
                        units <= units + 1;
                        marker_length <= 0;
                        state <= IDLE;
                    end else begin
                        trailer_at <= trailer_at + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
