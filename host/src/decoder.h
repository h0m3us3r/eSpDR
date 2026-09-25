// Parallel, order-preserving decoding and checking of a record stream.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

class StreamError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

enum class OutputFormat {
    None,   // check only
    Iqc,    // the records as received, END record included, padding removed
    Cs16,   // interleaved little-endian int16 I, Q per pair; gaps as zero pairs
};

struct StreamTotals {
    uint64_t pairs = 0;         // pairs delivered
    uint64_t span = 0;          // sample index after the last pair (delivered + lost)
    uint64_t gaps = 0;          // index jumps: pairs lost upstream
    uint64_t records = 0;
    uint64_t record_bytes = 0;  // records including the END record
    uint32_t crc = 0;           // CRC32 of the delivered pairs as LE u32
    bool ended = false;         // END record seen and verified

    uint64_t lost() const { return span - pairs; }
};

// Receives decoded pairs in stream order, one batch at a time: the sample
// index of the first pair, then interleaved I, Q (lost pairs inside a batch
// are zeros, so the batch is contiguous in sample time). Called on the
// decoder's writer thread; it must not block for long.
using SampleSink = std::function<void(uint64_t first_index, std::vector<int16_t> &&samples)>;

class StreamDecoder {
public:
    // output may be null when format is None. threads: decoding workers.
    StreamDecoder(OutputFormat format, std::FILE *output, unsigned threads, SampleSink sink = {});
    ~StreamDecoder();
    StreamDecoder(const StreamDecoder &) = delete;
    StreamDecoder &operator=(const StreamDecoder &) = delete;

    // Consumes stream bytes. Blocks while the bounded queues are full.
    // After the END record only zero padding may follow.
    void add(const uint8_t *data, size_t size);
    // Waits for everything queued; returns the totals. Throws on any error.
    StreamTotals finish();

    // Fails the stream if a record reaches past the pairs the source can have
    // produced since `start`, so that a corrupt index never becomes a huge
    // zero fill. Call before the first add().
    void limit_to_elapsed(std::chrono::steady_clock::time_point start, double pairs_per_second,
                          double slack_seconds);

    bool ended() const { return end_seen_; }
    // Thread-safe progress: pairs delivered and written so far.
    uint64_t progress() const;

private:
    struct Result {
        uint64_t first_index = 0;  // sample index of the job's first record
        uint64_t span_end = 0;     // sample index after its last pair
        uint64_t gaps = 0;         // index jumps inside the job
        uint64_t pairs = 0;
        uint64_t records = 0;
        uint64_t record_bytes = 0;
        uint32_t crc = 0;  // combined CRC of this job's pairs
        bool has_end = false;
        uint64_t end_index = 0;
        uint32_t end_crc = 0;
        std::vector<uint8_t> raw;      // Iqc output
        std::vector<int16_t> samples;  // Cs16 output and the sample sink
    };

    struct Job {
        std::vector<uint8_t> records;
        std::promise<Result> result;
    };

    void submit_batch();
    void worker();
    void writer();
    Result decode(std::vector<uint8_t> records) const;
    void rethrow();

    OutputFormat format_;
    std::FILE *output_;
    SampleSink sink_;
    std::vector<uint8_t> partial_;      // bytes of an incomplete record
    std::vector<uint8_t> batch_;        // complete records awaiting submission
    bool end_seen_ = false;
    bool limited_ = false;
    std::chrono::steady_clock::time_point limit_start_;
    double limit_rate_ = 0, limit_slack_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Job> jobs_;
    std::deque<std::future<Result>> ordered_;
    bool stopping_ = false;
    std::exception_ptr error_;
    StreamTotals totals_;
    std::vector<std::thread> workers_;
    std::thread writer_thread_;
};
