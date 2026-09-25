// Packs variable-length encoder words (2..8 bytes) densely into 16-byte DDR
// beats, with no padding between records.
//
// Per beat it reports where record boundaries fall, which the ring needs:
//   out_start   a record starts in this beat
//   out_commit  byte offset just past the last record completed in this
//               beat (16 if one ended exactly at the beat end), 0 if none
module packer (
    input  wire         clk,
    input  wire         reset,
    input  wire         in_valid,
    output wire         in_ready,
    input  wire [63:0]  in_data,
    input  wire [3:0]   in_bytes,
    input  wire         in_last,

    output wire         out_valid,
    input  wire         out_ready,
    output wire [127:0] out_data,
    output wire [4:0]   out_commit,
    output wire         out_start
);
    reg [191:0] reservoir = 0;
    reg [5:0]   used = 0;               // bytes held
    reg [5:0]   commit_at = 0;          // end of the last completed record, in bytes
    reg [5:0]   start_at = 0;           // 1 + offset of a record start, 0 if none
    reg         record_start = 1;

    wire pop = out_valid && out_ready;
    wire [5:0] remaining = pop ? used - 6'd16 : used;
    assign in_ready = remaining <= 16;
    wire take = in_valid && in_ready;

    // Clear the unused high bytes of a short final word.
    reg [63:0] clean_data;
    always @* begin
        case (in_bytes)
            2: clean_data = {48'b0, in_data[15:0]};
            4: clean_data = {32'b0, in_data[31:0]};
            6: clean_data = {16'b0, in_data[47:0]};
            default: clean_data = in_data;
        endcase
    end

    assign out_valid = used >= 16;
    assign out_data = reservoir[127:0];
    assign out_start = start_at != 0 && start_at <= 16;
    assign out_commit = commit_at > 16 ? 5'd0 : commit_at[4:0];

    always @(posedge clk) begin
        if (reset) begin
            reservoir <= 0;
            used <= 0;
            commit_at <= 0;
            start_at <= 0;
            record_start <= 1;
        end else begin
            if (pop) begin
                reservoir <= reservoir >> 128;
                used <= remaining;
                commit_at <= commit_at > 16 ? commit_at - 6'd16 : 6'd0;
                start_at <= start_at > 16 ? start_at - 6'd16 : 6'd0;
            end
            if (take) begin
                reservoir <= (pop ? reservoir >> 128 : reservoir) |
                             ({128'b0, clean_data} << {remaining[4:1], 4'b0});
                used <= remaining + in_bytes;
                if (record_start)
                    start_at <= remaining + 6'd1;
                record_start <= in_last;
                if (in_last)
                    commit_at <= remaining + in_bytes;
            end
        end
    end
endmodule
