`include "protocol.vh"
// Lossless block encoder (format: protocol/iq_record.h).
//
// Three block buffers rotate through three concurrent stages:
//   capture  accept up to 1024 pairs, tracking min/max/sum and the block CRC
//   score    replay the block and total the payload bits of every mode and
//            Rice parameter; pick the smallest (ties: RAW, MIN, CENTER, DELTA)
//   emit     send the 32-byte header, then the payload as 64-bit words
// A flush closes a partial block once the input goes idle. A skip (pairs lost
// upstream) closes the partial block too and advances the sample index, so
// the next record's index jumps over the missing pairs.
// Each record is byte-for-byte what iqr::encode(..., mode -1) produces.
module encoder (
    input  wire        clk,
    input  wire        reset,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [19:0] in_pair,
    input  wire        flush,
    input  wire        skip_valid,
    output wire        skip_ready,
    input  wire [31:0] skip_pairs,

    output wire        out_valid,
    input  wire        out_ready,
    output wire [63:0] out_data,
    output wire [3:0]  out_bytes,       // valid bytes in out_data (8, or 2/4/6 at a record end)
    output wire        out_last,        // last word of a record

    output reg  [63:0] accepted = 0,    // pairs accepted since reset
    output reg  [63:0] skipped = 0,     // pairs skipped since reset
    output reg  [31:0] records = 0      // records emitted
);
    // ---- helpers ---------------------------------------------------------------
    function automatic signed [10:0] sext(input [9:0] x);
        sext = {x[9], x};
    endfunction

    function automatic [9:0] zigzag(input [9:0] x);   // 10-bit two's complement
        zigzag = x[9] ? ((~x << 1) | 10'd1) : (x << 1);
    endfunction

    function automatic [3:0] bit_width(input [10:0] x);
        integer j;
        begin
            bit_width = 0;
            for (j = 0; j < 10; j = j + 1)
                if (x[j]) bit_width = j + 1;
        end
    endfunction

    function automatic [4:0] rice_bits(input [9:0] z, input [3:0] k);
        rice_bits = (z >> k) >= 15 ? 5'd26 : 5'((z >> k) + 1 + k);
    endfunction

    // {length[5:0], code[25:0]}: q zeros, a one, k low bits; or the escape.
    function automatic [31:0] rice_code(input [9:0] z, input [3:0] k);
        reg [9:0] q;
        begin
            q = z >> k;
            if (q >= 15)
                rice_code = {6'd26, z, 16'h8000};
            else
                rice_code = {6'(q + 1 + k),
                             (26'b1 << q[3:0]) | ({16'b0, z & ((10'b1 << k) - 10'd1)} << ({1'b0, q[3:0]} + 5'd1))};
        end
    endfunction

    function automatic [31:0] crc_byte(input [31:0] c, input [7:0] b);
        reg [31:0] v;
        integer j;
        begin
            v = c ^ b;
            for (j = 0; j < 8; j = j + 1)
                v = v[0] ? (v >> 1) ^ 32'hEDB88320 : v >> 1;
            crc_byte = v;
        end
    endfunction

    // CRC of a pair as a little-endian 32-bit word.
    function automatic [31:0] crc_pair(input [31:0] c, input [19:0] x);
        crc_pair = crc_byte(crc_byte(crc_byte(crc_byte(c, x[7:0]), x[15:8]), {4'b0, x[19:16]}), 8'd0);
    endfunction

    // Signed mean of a full block, truncated toward zero.
    function automatic signed [10:0] block_mean(input signed [20:0] sum);
        block_mean = sum < 0 ? -((-sum) >> 10) : (sum >> 10);
    endfunction

    // ---- block buffers and per-block metadata -------------------------------------
    reg [1:0] capture_bank = 0, score_bank = 0, emit_bank = 0;
    reg [2:0] filled = 0, scored = 0;

    reg [10:0] count [0:2];
    reg [63:0] first_index [0:2];
    reg [31:0] block_crc [0:2];
    reg signed [10:0] min_i [0:2], min_q [0:2], mean_i [0:2], mean_q [0:2];
    reg [3:0]  range_bits_i [0:2], range_bits_q [0:2];
    reg [19:0] first_pair [0:2];
    reg [1:0]  mode [0:2];
    reg [3:0]  k_i [0:2], k_q [0:2];
    reg [15:0] payload_bits [0:2];
    reg signed [10:0] base_i [0:2], base_q [0:2];

    // ---- capture state --------------------------------------------------------------------
    reg [10:0] capture_count = 0;
    reg        closing = 0;
    reg signed [10:0] low_i = 511, low_q = 511, high_i = -512, high_q = -512;
    reg signed [20:0] sum_i = 0, sum_q = 0;
    reg [19:0] capture_first = 0;
    reg [31:0] capture_crc = 32'hFFFFFFFF;


    wire [19:0] ram_q [0:2];
    wire        emit_read;
    reg  [3:0]  score_state = 0;
    reg  [10:0] score_read = 0, emit_read_at = 0, score_count = 0;

    genvar b;
    generate
        for (b = 0; b < 3; b = b + 1) begin : banks
            (* ram_style = "block" *) reg [19:0] ram [0:1023];
            reg [19:0] q;
            wire score_reading = score_bank == b && score_state == 1 && score_read < score_count;
            wire [9:0] address = score_reading ? score_read[9:0] : emit_read_at[9:0];
            always @(posedge clk) begin
                if (in_valid && in_ready && capture_bank == b)
                    ram[capture_count[9:0]] <= in_pair;
                if (score_reading || (emit_bank == b && emit_read))
                    q <= ram[address];
            end
            assign ram_q[b] = q;
        end
    endgenerate

    // ---- capture ----------------------------------------------------------------------
    wire take = in_valid && in_ready;
    assign in_ready = !closing && !filled[capture_bank] && !scored[capture_bank];
    wire close_full = take && capture_count == 1023;
    wire close_partial = (flush || skip_valid) && capture_count != 0 && !take && in_ready;
    assign skip_ready = skip_valid && capture_count == 0 && !closing;
    wire signed [10:0] in_i = sext(in_pair[9:0]), in_q = sext(in_pair[19:10]);

    // ---- scoring ------------------------------------------------------------------------
    // costs[s][k]: payload bits of stream s with Rice parameter k.
    // Streams: 0 CENTER I, 1 CENTER Q, 2 DELTA I, 3 DELTA Q.
    reg [15:0] costs [0:3][0:9];
    reg [15:0] best_cost [0:3];
    reg [3:0]  best_k [0:3];
    reg [3:0]  k_scan = 0;
    reg [10:0] score_done = 0;
    reg        score_read_valid = 0, score_pair_valid = 0;
    reg [19:0] score_pair = 0;
    reg [9:0]  score_mean_i = 0, score_mean_q = 0, previous_i = 0, previous_q = 0;
    reg [4:0]  score_range_bits = 0;
    reg [15:0] min_bits = 0;
    reg [1:0]  chosen_mode = 0;
    reg [15:0] chosen_bits = 0;
    reg [3:0]  chosen_k_i = 0, chosen_k_q = 0;
    reg signed [10:0] chosen_base_i = 0, chosen_base_q = 0;

    wire [9:0] center_z_i = zigzag(score_pair[9:0] - score_mean_i);
    wire [9:0] center_z_q = zigzag(score_pair[19:10] - score_mean_q);
    wire [9:0] delta_z_i = zigzag(score_pair[9:0] - previous_i);
    wire [9:0] delta_z_q = zigzag(score_pair[19:10] - previous_q);

    // Payload bits rounded up to 16-bit words, for mode comparison.
    function automatic [12:0] words(input [16:0] bits);
        words = 13'((bits + 17'd15) >> 4);
    endfunction

    // ---- emission -----------------------------------------------------------------------
    localparam EMIT_IDLE = 0, EMIT_HEADER_CRC = 1, EMIT_HEADER_DONE = 2, EMIT_HEADER = 3,
               EMIT_PAYLOAD = 4;
    reg [2:0]   emit_state = EMIT_IDLE;
    reg [255:0] header = 0;
    reg [31:0]  header_crc = 32'hFFFFFFFF;
    reg [4:0]   header_crc_at = 0;
    reg [1:0]   header_word = 0;
    reg [10:0]  emit_count = 0, emitted = 0;
    reg [15:0]  emit_bits = 0;
    reg [1:0]   emit_mode = 0;
    reg [3:0]   emit_k_i = 0, emit_k_q = 0;
    reg [9:0]   predictor_i = 0, predictor_q = 0;
    reg [127:0] reservoir = 0;          // pending payload bits, LSB first
    reg [7:0]   reservoir_bits = 0;
    reg         emit_done = 0;

    // Payload pipeline: RAM read -> residual -> code -> pair of codes -> reservoir.
    reg        read_valid = 0;
    reg        residual_valid = 0;
    reg [9:0]  residual_i = 0, residual_q = 0;
    reg        code_valid = 0;
    reg [31:0] code_i = 0, code_q = 0;
    reg        pair_code_valid = 0;
    reg [51:0] pair_code = 0;
    reg [6:0]  pair_code_bits = 0;

    wire [19:0] emit_q = ram_q[emit_bank];
    wire [9:0] offset_i = emit_q[9:0] - predictor_i;
    wire [9:0] offset_q = emit_q[19:10] - predictor_q;
    wire [31:0] rice_i = rice_code(residual_i, emit_k_i);
    wire [31:0] rice_q = rice_code(residual_q, emit_k_q);
    wire [51:0] joined_code = {26'b0, code_i[25:0]} | ({26'b0, code_q[25:0]} << code_i[31:26]);
    wire [6:0] joined_bits = {1'b0, code_i[31:26]} + {1'b0, code_q[31:26]};

    wire payload_word_ready = emit_state == EMIT_PAYLOAD &&
                              (reservoir_bits >= 64 || (emit_done && reservoir_bits != 0));
    assign out_valid = emit_state == EMIT_HEADER || payload_word_ready;
    assign out_data = emit_state == EMIT_HEADER ? header[header_word * 64 +: 64] : reservoir[63:0];
    assign out_bytes = (emit_state == EMIT_HEADER || reservoir_bits >= 64)
                           ? 4'd8 : 4'(((reservoir_bits + 15) >> 4) << 1);
    assign out_last = (emit_state == EMIT_HEADER && header_word == 3 && emit_bits == 0) ||
                      (payload_word_ready && emit_done && reservoir_bits <= 64);
    wire send = out_valid && out_ready;
    wire payload_send = payload_word_ready && out_ready;
    wire [7:0] bits_after_send = payload_send ? (reservoir_bits >= 64 ? reservoir_bits - 8'd64 : 8'd0)
                                              : reservoir_bits;

    // Each stage advances only when the next can take its output; a pair code
    // enters the reservoir only when its worst case (52 bits) fits.
    wire append_code = emit_state == EMIT_PAYLOAD && pair_code_valid && bits_after_send <= 76;
    wire join_codes = emit_state == EMIT_PAYLOAD && code_valid && (!pair_code_valid || append_code);
    wire make_codes = emit_state == EMIT_PAYLOAD && residual_valid && (!code_valid || join_codes);
    wire make_residuals = emit_state == EMIT_PAYLOAD && read_valid && (!residual_valid || make_codes);
    assign emit_read = emit_state == EMIT_PAYLOAD && emit_read_at < emit_count &&
                       (!read_valid || make_residuals);

    integer s, k;
    always @(posedge clk) begin
        if (reset) begin
            capture_bank <= 0;
            score_bank <= 0;
            emit_bank <= 0;
            filled <= 0;
            scored <= 0;
            capture_count <= 0;
            closing <= 0;
            low_i <= 511; low_q <= 511; high_i <= -512; high_q <= -512;
            sum_i <= 0; sum_q <= 0;
            capture_crc <= 32'hFFFFFFFF;
            accepted <= 0;
            skipped <= 0;
            records <= 0;
            score_state <= 0;
            score_read_valid <= 0;
            score_pair_valid <= 0;
            emit_state <= EMIT_IDLE;
            read_valid <= 0;
            residual_valid <= 0;
            code_valid <= 0;
            pair_code_valid <= 0;
            reservoir_bits <= 0;
            emit_done <= 0;
        end else begin
            // ---- capture ----
            if (take) begin
                if (capture_count == 0)
                    capture_first <= in_pair;
                capture_count <= capture_count + 1'b1;
                if (in_i < low_i) low_i <= in_i;
                if (in_q < low_q) low_q <= in_q;
                if (in_i > high_i) high_i <= in_i;
                if (in_q > high_q) high_q <= in_q;
                sum_i <= sum_i + in_i;
                sum_q <= sum_q + in_q;
                capture_crc <= crc_pair(capture_crc, in_pair);
                accepted <= accepted + 1;
            end
            if (close_full || close_partial)
                closing <= 1;
            if (skip_ready)
                skipped <= skipped + skip_pairs;
            // Publish the block one clock later, from registered statistics.
            if (closing) begin
                filled[capture_bank] <= 1;
                count[capture_bank] <= capture_count;
                first_index[capture_bank] <= accepted + skipped - capture_count;
                block_crc[capture_bank] <= ~capture_crc;
                first_pair[capture_bank] <= capture_first;
                min_i[capture_bank] <= low_i;
                min_q[capture_bank] <= low_q;
                range_bits_i[capture_bank] <= bit_width(high_i - low_i);
                range_bits_q[capture_bank] <= bit_width(high_q - low_q);
                // A partial final block uses zero as its CENTER base.
                mean_i[capture_bank] <= capture_count == 1024 ? block_mean(sum_i) : 11'sd0;
                mean_q[capture_bank] <= capture_count == 1024 ? block_mean(sum_q) : 11'sd0;
                capture_bank <= capture_bank == 2 ? 2'd0 : capture_bank + 2'd1;
                capture_count <= 0;
                closing <= 0;
                low_i <= 511; low_q <= 511; high_i <= -512; high_q <= -512;
                sum_i <= 0; sum_q <= 0;
                capture_crc <= 32'hFFFFFFFF;
            end

            // ---- score ----
            case (score_state)
                0: if (filled[score_bank] && !scored[score_bank]) begin
                    score_count <= count[score_bank];
                    score_read <= 0;
                    score_done <= 0;
                    score_read_valid <= 0;
                    score_pair_valid <= 0;
                    min_bits <= 0;
                    score_mean_i <= mean_i[score_bank][9:0];
                    score_mean_q <= mean_q[score_bank][9:0];
                    score_range_bits <= range_bits_i[score_bank] + range_bits_q[score_bank];
                    previous_i <= first_pair[score_bank][9:0];
                    previous_q <= first_pair[score_bank][19:10];
                    for (s = 0; s < 4; s = s + 1)
                        for (k = 0; k < 10; k = k + 1)
                            costs[s][k] <= 0;
                    score_state <= 1;
                end
                1: begin
                    // RAM read -> pair register -> cost accumulation.
                    score_read_valid <= score_read < score_count;
                    if (score_read < score_count)
                        score_read <= score_read + 1'b1;
                    score_pair_valid <= score_read_valid;
                    if (score_read_valid)
                        score_pair <= ram_q[score_bank];
                    if (score_pair_valid) begin
                        min_bits <= min_bits + score_range_bits;
                        for (k = 0; k < 10; k = k + 1) begin
                            costs[0][k] <= costs[0][k] + rice_bits(center_z_i, k);
                            costs[1][k] <= costs[1][k] + rice_bits(center_z_q, k);
                            costs[2][k] <= costs[2][k] + rice_bits(delta_z_i, k);
                            costs[3][k] <= costs[3][k] + rice_bits(delta_z_q, k);
                        end
                        previous_i <= score_pair[9:0];
                        previous_q <= score_pair[19:10];
                        score_done <= score_done + 1'b1;
                        if (score_done + 1 == score_count) begin
                            k_scan <= 0;
                            for (s = 0; s < 4; s = s + 1) begin
                                best_cost[s] <= 16'hFFFF;
                                best_k[s] <= 0;
                            end
                            score_state <= 2;
                        end
                    end
                end
                2: begin
                    for (s = 0; s < 4; s = s + 1)
                        if (costs[s][k_scan] < best_cost[s]) begin
                            best_cost[s] <= costs[s][k_scan];
                            best_k[s] <= k_scan;
                        end
                    if (k_scan == 9)
                        score_state <= 3;
                    else
                        k_scan <= k_scan + 1'b1;
                end
                3: begin
                    chosen_mode <= `IQR_RAW;
                    chosen_bits <= (score_count << 4) + (score_count << 2);   // 20 bits per pair
                    chosen_k_i <= 0;
                    chosen_k_q <= 0;
                    chosen_base_i <= 0;
                    chosen_base_q <= 0;
                    score_state <= 4;
                end
                4: begin
                    if (words(min_bits) < words(chosen_bits)) begin
                        chosen_mode <= `IQR_MIN;
                        chosen_bits <= min_bits;
                        chosen_k_i <= range_bits_i[score_bank];
                        chosen_k_q <= range_bits_q[score_bank];
                        chosen_base_i <= min_i[score_bank];
                        chosen_base_q <= min_q[score_bank];
                    end
                    score_state <= 5;
                end
                5: begin
                    if (words(best_cost[0] + best_cost[1]) < words(chosen_bits)) begin
                        chosen_mode <= `IQR_CENTER;
                        chosen_bits <= best_cost[0] + best_cost[1];
                        chosen_k_i <= best_k[0];
                        chosen_k_q <= best_k[1];
                        chosen_base_i <= mean_i[score_bank];
                        chosen_base_q <= mean_q[score_bank];
                    end
                    score_state <= 6;
                end
                6: begin
                    if (words(best_cost[2] + best_cost[3]) < words(chosen_bits)) begin
                        chosen_mode <= `IQR_DELTA;
                        chosen_bits <= best_cost[2] + best_cost[3];
                        chosen_k_i <= best_k[2];
                        chosen_k_q <= best_k[3];
                        chosen_base_i <= sext(first_pair[score_bank][9:0]);
                        chosen_base_q <= sext(first_pair[score_bank][19:10]);
                    end
                    score_state <= 7;
                end
                7: begin
                    mode[score_bank] <= chosen_mode;
                    payload_bits[score_bank] <= chosen_bits;
                    k_i[score_bank] <= chosen_k_i;
                    k_q[score_bank] <= chosen_k_q;
                    base_i[score_bank] <= chosen_base_i;
                    base_q[score_bank] <= chosen_base_q;
                    scored[score_bank] <= 1;
                    score_bank <= score_bank == 2 ? 2'd0 : score_bank + 2'd1;
                    score_state <= 0;
                end
            endcase

            // ---- emit ----
            case (emit_state)
                EMIT_IDLE: if (scored[emit_bank]) begin
                    emit_count <= count[emit_bank];
                    emit_bits <= payload_bits[emit_bank];
                    header <= 0;
                    header[`IQR_OFF_MAGIC * 8 +: 32] <= `IQR_MAGIC;
                    header[`IQR_OFF_VERSION * 8 +: 8] <= `IQR_VERSION;
                    header[`IQR_OFF_MODE * 8 +: 8] <= {6'b0, mode[emit_bank]};
                    header[`IQR_OFF_K_I * 8 +: 8] <= {4'b0, k_i[emit_bank]};
                    header[`IQR_OFF_K_Q * 8 +: 8] <= {4'b0, k_q[emit_bank]};
                    header[`IQR_OFF_COUNT * 8 +: 16] <= {5'b0, count[emit_bank]};
                    header[`IQR_OFF_BITS * 8 +: 16] <= payload_bits[emit_bank];
                    header[`IQR_OFF_BASE_I * 8 +: 16] <= {{5{base_i[emit_bank][10]}}, base_i[emit_bank]};
                    header[`IQR_OFF_BASE_Q * 8 +: 16] <= {{5{base_q[emit_bank][10]}}, base_q[emit_bank]};
                    header[`IQR_OFF_INDEX * 8 +: 64] <= first_index[emit_bank];
                    header[`IQR_OFF_CRC * 8 +: 32] <= block_crc[emit_bank];
                    header_crc <= 32'hFFFFFFFF;
                    header_crc_at <= 0;
                    emit_state <= EMIT_HEADER_CRC;
                end
                EMIT_HEADER_CRC: begin
                    header_crc <= crc_byte(header_crc, header[header_crc_at * 8 +: 8]);
                    if (header_crc_at == `IQR_OFF_HEADER_CRC - 1)
                        emit_state <= EMIT_HEADER_DONE;
                    else
                        header_crc_at <= header_crc_at + 1'b1;
                end
                EMIT_HEADER_DONE: begin
                    header[`IQR_OFF_HEADER_CRC * 8 +: 32] <= ~header_crc;
                    header_word <= 0;
                    emit_state <= EMIT_HEADER;
                end
                EMIT_HEADER: if (send) begin
                    if (header_word == 3) begin
                        emit_state <= EMIT_PAYLOAD;
                        emit_read_at <= 0;
                        emitted <= 0;
                        read_valid <= 0;
                        residual_valid <= 0;
                        code_valid <= 0;
                        pair_code_valid <= 0;
                        reservoir <= 0;
                        reservoir_bits <= 0;
                        emit_done <= 0;
                        predictor_i <= base_i[emit_bank][9:0];
                        predictor_q <= base_q[emit_bank][9:0];
                        emit_mode <= mode[emit_bank];
                        emit_k_i <= k_i[emit_bank];
                        emit_k_q <= k_q[emit_bank];
                    end else begin
                        header_word <= header_word + 1'b1;
                    end
                end
                EMIT_PAYLOAD: begin
                    if (payload_send) begin
                        reservoir <= reservoir >> 64;
                        reservoir_bits <= bits_after_send;
                    end
                    if (append_code) begin
                        reservoir <= (payload_send ? reservoir >> 64 : reservoir) |
                                     ({76'b0, pair_code} << bits_after_send);
                        reservoir_bits <= bits_after_send + pair_code_bits;
                        emitted <= emitted + 1'b1;
                        if (emitted + 1 == emit_count)
                            emit_done <= 1;
                        pair_code_valid <= 0;
                    end
                    if (join_codes) begin
                        pair_code <= joined_code;
                        pair_code_bits <= joined_bits;
                        pair_code_valid <= 1;
                        code_valid <= 0;
                    end
                    if (make_codes) begin
                        code_valid <= 1;
                        residual_valid <= 0;
                        case (emit_mode)
                            `IQR_RAW: begin
                                code_i <= {6'd10, 16'b0, residual_i};
                                code_q <= {6'd10, 16'b0, residual_q};
                            end
                            `IQR_MIN: begin
                                code_i <= {2'b0, emit_k_i, 16'b0, residual_i};
                                code_q <= {2'b0, emit_k_q, 16'b0, residual_q};
                            end
                            default: begin
                                code_i <= rice_i;
                                code_q <= rice_q;
                            end
                        endcase
                    end
                    if (make_residuals) begin
                        residual_valid <= 1;
                        read_valid <= 0;
                        case (emit_mode)
                            `IQR_RAW: begin
                                residual_i <= emit_q[9:0];
                                residual_q <= emit_q[19:10];
                            end
                            `IQR_MIN: begin
                                residual_i <= offset_i;
                                residual_q <= offset_q;
                            end
                            default: begin
                                residual_i <= zigzag(offset_i);
                                residual_q <= zigzag(offset_q);
                            end
                        endcase
                        if (emit_mode == `IQR_DELTA) begin
                            predictor_i <= emit_q[9:0];
                            predictor_q <= emit_q[19:10];
                        end
                    end
                    if (emit_read) begin
                        emit_read_at <= emit_read_at + 1'b1;
                        read_valid <= 1;
                    end
                end
            endcase

            if (send && out_last) begin
                filled[emit_bank] <= 0;
                scored[emit_bank] <= 0;
                emit_bank <= emit_bank == 2 ? 2'd0 : emit_bank + 2'd1;
                records <= records + 1;
                emit_state <= EMIT_IDLE;
                read_valid <= 0;
            end
        end
    end
endmodule
