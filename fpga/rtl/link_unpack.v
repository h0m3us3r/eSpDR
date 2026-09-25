// Rebuilds 20-bit IQ pairs from a lane's 40-byte groups.
//
// Each group carries 16 pairs: 32 bytes of low 16 bits (little-endian per
// pair), then 8 bytes holding the high nibbles of pairs 2k and 2k+1. Padding
// pairs of a partial head or tail group are dropped using the unit layout.
// `last` marks the unit's final pair.
module link_unpack (
    input  wire        clk,
    input  wire        reset,
    input  wire        byte_valid,
    input  wire [7:0]  byte_value,
    input  wire [15:0] byte_index,
    input  wire [3:0]  head_pairs,
    input  wire [3:0]  tail_pairs,
    input  wire [10:0] total_groups,

    output reg         pair_valid = 0,
    output reg  [19:0] pair = 0,
    output reg         last = 0,
    output reg  [31:0] errors = 0      // a group arrived before the previous one was emitted
);
    reg [19:0] group_pairs [0:15];
    reg [5:0]  group_byte = 0;
    reg [10:0] group = 0;
    reg        emitting = 0;
    reg [4:0]  emit_at = 0, emit_count = 0;
    reg        emit_last = 0;

    always @(posedge clk) begin
        pair_valid <= 0;
        last <= 0;
        if (reset) begin
            group_byte <= 0;
            group <= 0;
            emitting <= 0;
            errors <= 0;
        end else begin
            if (byte_valid && byte_index == 0) begin
                // New unit.
                group_byte <= 0;
                group <= 0;
                if (emitting)
                    errors <= errors + 1;
            end

            if (byte_valid && byte_index >= 8 && group < total_groups) begin
                if (group_byte < 32) begin
                    if (group_byte[0])
                        group_pairs[group_byte[4:1]][15:8] <= byte_value;
                    else
                        group_pairs[group_byte[4:1]][7:0] <= byte_value;
                end else begin
                    group_pairs[(group_byte - 32) * 2][19:16] <= byte_value[3:0];
                    group_pairs[(group_byte - 32) * 2 + 1][19:16] <= byte_value[7:4];
                end

                if (group_byte == 39) begin
                    group_byte <= 0;
                    group <= group + 1'b1;
                    emitting <= 1;
                    emit_at <= 0;
                    emit_last <= group + 1 == total_groups;
                    if (group == 0 && head_pairs != 0)
                        emit_count <= {1'b0, head_pairs};
                    else if (group + 1 == total_groups && tail_pairs != 0)
                        emit_count <= {1'b0, tail_pairs};
                    else
                        emit_count <= 5'd16;
                    if (emitting)
                        errors <= errors + 1;
                end else begin
                    group_byte <= group_byte + 1'b1;
                end
            end

            if (emitting) begin
                pair <= group_pairs[emit_at[3:0]];
                pair_valid <= 1;
                if (emit_at + 1 == emit_count) begin
                    emitting <= 0;
                    last <= emit_last;
                end else begin
                    emit_at <= emit_at + 1'b1;
                end
            end
        end
    end
endmodule
