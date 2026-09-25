// 256 MiB DDR3 ring buffer between the encoder and USB.
//
// The input is a dense byte stream in 16-byte beats. Only whole records are
// ever made readable: `committed` advances to the end of the last complete
// record, so USB never sends half a record even while the encoder stalls.
//
// Pointers (all in beats):
//   allocated  beats accepted from the packer
//   written    beats whose MIG command and data have both been accepted
//   committed  readable prefix (end of the last complete record)
//   issued     read commands sent
//   reclaimed  read responses handed to the output FIFO
//
// Before admitting a beat in which a record starts, the ring requires room
// for a maximum-size record (RESERVE_BEATS) so a record is never left
// incomplete. If the ring is full the input stalls; the packer and encoder
// back up and, eventually, the reorder queues overflow and count it.
//
// `clear` starts a new stream without disturbing the MIG: the ring stops
// taking new work, completes a write whose command or data is already
// accepted, discards the read responses still in flight, and only then
// resets its pointers (or after CLEAR_TIMEOUT clocks). `reset` is immediate.
//
// Reads are issued only when the output FIFO has room for every response in
// flight. Writes and reads take turns in quanta of up to 128 beats to limit
// DDR bus turnarounds; writes preempt reads early when the input queue is
// half full and USB still has at least 4 KiB prefetched.
module ddr_ring #(
    parameter ADDR_BITS = 24,       // beats: 2^24 x 16 bytes = 256 MiB
    parameter OUT_ADDR_BITS = 9,
    parameter MAX_READS = 32,
    parameter RESERVE_BEATS = 163,  // 2592-byte raw record plus 14 bytes of a preceding one
    parameter BURST_BEATS = 1,      // reads start only once this many beats are readable
    parameter CLEAR_TIMEOUT = 65535
) (
    input  wire                     clk,        // MIG UI clock
    input  wire                     reset,      // with the MIG
    input  wire                     clear,      // new stream
    input  wire                     calibrated,

    input  wire [9:0]               in_level,   // input FIFO fill, beats
    input  wire                     in_valid,
    input  wire                     in_start,
    input  wire [127:0]             in_data,
    input  wire [4:0]               in_commit,
    output wire                     in_ready,

    output wire                     out_push,
    output wire [127:0]             out_data,
    input  wire [OUT_ADDR_BITS:0]   out_level,
    input  wire                     out_full,

    output wire [27:0]              app_addr,
    output wire [2:0]               app_cmd,
    output wire                     app_en,
    output wire [127:0]             app_wdf_data,
    output wire [15:0]              app_wdf_mask,
    output wire                     app_wdf_end,
    output wire                     app_wdf_wren,
    input  wire                     app_rdy,
    input  wire                     app_wdf_rdy,
    input  wire [127:0]             app_rd_data,
    input  wire                     app_rd_data_valid,

    output wire [ADDR_BITS:0]       occupancy,
    output reg  [ADDR_BITS:0]       occupancy_peak = 0,
    output reg  [31:0]              errors = 0
);
    localparam MIG_READ = 3'b001, MIG_WRITE = 3'b000;

    reg [ADDR_BITS:0] allocated = 0, written = 0, committed = 0, issued = 0, reclaimed = 0;
    reg [ADDR_BITS:0] reserved_end = 0;
    reg [24:0]        burst_left = 0;

    // The single write in flight.
    reg               pending = 0;
    reg               command_done = 0, data_done = 0;
    reg [127:0]       value = 0;
    reg [4:0]         commit_bytes = 0;
    reg [ADDR_BITS-1:0] address = 0;

    reg               write_turn = 1;
    reg [6:0]         quota = 0;
    reg               clear_pending = 0;
    reg [15:0]        clear_wait = 0;

    wire active = calibrated && !reset;                 // MIG handshakes may proceed
    wire clearing = clear || clear_pending;
    wire run = active && errors == 0 && !clearing;      // new work may start
    assign occupancy = allocated - reclaimed;
    wire [ADDR_BITS:0] outstanding = issued - reclaimed;
    wire [ADDR_BITS:0] readable = committed - issued;
    wire have_read_work = (burst_left != 0 && readable != 0) || outstanding != 0;
    wire want_read = burst_left != 0 && readable != 0 &&
                     (out_level + outstanding) < ((1 << OUT_ADDR_BITS) - 2) &&
                     outstanding < MAX_READS;
    wire choose_read = run && want_read && !write_turn;

    assign app_en = active && app_rdy &&
                    (choose_read || ((write_turn || clearing) && pending && !command_done));
    assign app_cmd = choose_read ? MIG_READ : MIG_WRITE;
    assign app_addr = choose_read ? {{(25 - ADDR_BITS){1'b0}}, issued[ADDR_BITS-1:0], 3'b0}
                                  : {{(25 - ADDR_BITS){1'b0}}, address, 3'b0};
    assign app_wdf_data = value;
    assign app_wdf_mask = 0;
    assign app_wdf_wren = active && pending && !data_done;
    assign app_wdf_end = app_wdf_wren;

    wire quiet = !pending && outstanding == 0;
    wire command_accepted = app_en && app_rdy;
    wire write_command = command_accepted && !choose_read;
    wire read_command = command_accepted && choose_read;
    wire data_accepted = app_wdf_wren && app_wdf_rdy;
    wire write_complete = pending && (command_done || write_command) && (data_done || data_accepted);

    assign in_ready = run && !pending && !occupancy[ADDR_BITS] &&
                      (!in_start || occupancy <= ((1 << ADDR_BITS) - RESERVE_BEATS));
    wire take = in_valid && in_ready;

    assign out_push = run && app_rd_data_valid && outstanding != 0 && !out_full;
    assign out_data = app_rd_data;

    task reset_pointers;
        begin
            allocated <= 0;
            reserved_end <= 0;
            burst_left <= 0;
            written <= 0;
            committed <= 0;
            issued <= 0;
            reclaimed <= 0;
            pending <= 0;
            command_done <= 0;
            data_done <= 0;
            write_turn <= 1;
            quota <= 0;
            occupancy_peak <= 0;
            errors <= 0;
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            reset_pointers;
            clear_pending <= 0;
        end else if (clearing) begin
            // Finish the MIG transactions already started, then reset.
            if (write_command) command_done <= 1;
            if (data_accepted) data_done <= 1;
            if (write_complete) pending <= 0;
            if (app_rd_data_valid && outstanding != 0) reclaimed <= reclaimed + 1;
            clear_wait <= clear ? 16'd0 : clear_wait + 1'b1;
            clear_pending <= 1;
            if (!clear && ((quiet && !app_rd_data_valid) || clear_wait == CLEAR_TIMEOUT)) begin
                reset_pointers;
                clear_pending <= 0;
            end
        end else if (run) begin
            if (occupancy > occupancy_peak)
                occupancy_peak <= occupancy;

            if (take) begin
                if (in_start)
                    reserved_end <= allocated + RESERVE_BEATS;
                else if (allocated == reserved_end)
                    errors <= errors + 1;       // record longer than its reservation
                value <= in_data;
                commit_bytes <= in_commit;
                address <= allocated[ADDR_BITS-1:0];
                allocated <= allocated + 1;
                pending <= 1;
                command_done <= 0;
                data_done <= 0;
            end
            if (write_command) command_done <= 1;
            if (data_accepted) data_done <= 1;

            // Arbitration between write and read quanta.
            if (write_turn) begin
                if (!pending && (!in_valid || !in_ready)) begin
                    write_turn <= 0;
                    quota <= 0;
                end else if (write_complete) begin
                    if (quota == 127) begin
                        write_turn <= 0;
                        quota <= 0;
                    end else begin
                        quota <= quota + 1'b1;
                    end
                end
            end else begin
                if (!have_read_work || (in_level >= 256 && out_level >= 256 && (pending || in_ready))) begin
                    write_turn <= 1;
                    quota <= 0;
                end else if (read_command) begin
                    if (quota == 127) begin
                        write_turn <= 1;
                        quota <= 0;
                    end else begin
                        quota <= quota + 1'b1;
                    end
                end
            end

            if (write_complete) begin
                pending <= 0;
                written <= written + 1;
                // A record ended inside this beat: everything before this
                // beat is complete; if it ended exactly at the beat end, so
                // is this beat.
                if (commit_bytes != 0)
                    committed <= written + (commit_bytes == 16 ? 1 : 0);
            end

            if (burst_left == 0 && readable >= BURST_BEATS)
                burst_left <= BURST_BEATS;
            if (read_command) begin
                issued <= issued + 1;
                burst_left <= burst_left - 1;
            end
            if (app_rd_data_valid) begin
                if (outstanding == 0 || out_full)
                    errors <= errors + 1;       // unexpected or unstorable response
                else
                    reclaimed <= reclaimed + 1;
            end
        end
    end
endmodule
