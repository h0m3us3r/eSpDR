#include "decoder.h"

#include <algorithm>
#include <array>
#include <cstring>

#include <zlib.h>

#include "iq_record.h"

namespace {

constexpr size_t kBatchBytes = 1 << 20;  // records per decoding job
constexpr size_t kMaxQueuedJobs = 16;    // bounds memory; back-pressures USB

int16_t component(uint32_t pair, unsigned c)
{
    unsigned x = (pair >> (10 * c)) & 1023;
    return int16_t(int(x & 511) - int(x & 512));
}

}  // namespace

StreamDecoder::StreamDecoder(OutputFormat format, std::FILE *output, unsigned threads, SampleSink sink)
    : format_(format), output_(output), sink_(std::move(sink))
{
    for (unsigned i = 0; i < std::max(1u, threads); ++i) workers_.emplace_back([this] { worker(); });
    writer_thread_ = std::thread([this] { writer(); });
}

StreamDecoder::~StreamDecoder()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    changed_.notify_all();
    for (auto &thread : workers_) thread.join();
    writer_thread_.join();
}

void StreamDecoder::rethrow()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_) std::rethrow_exception(error_);
}

uint64_t StreamDecoder::progress() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return totals_.pairs;
}

void StreamDecoder::add(const uint8_t *data, size_t size)
{
    rethrow();
    if (end_seen_) {
        for (size_t i = 0; i < size; ++i)
            if (data[i]) throw StreamError("non-zero data after the END record");
        return;
    }
    partial_.insert(partial_.end(), data, data + size);
    size_t at = 0;
    while (partial_.size() - at >= IQR_HEADER_BYTES) {
        iqr::Header header;
        size_t record = iqr::inspect(partial_.data() + at, partial_.size() - at, &header);
        if (!record) throw StreamError("corrupt record header in the stream");
        if (record > partial_.size() - at) break;
        batch_.insert(batch_.end(), partial_.begin() + long(at), partial_.begin() + long(at + record));
        at += record;
        if (header.mode == IQR_END) {
            end_seen_ = true;
            for (size_t i = at; i < partial_.size(); ++i)
                if (partial_[i]) throw StreamError("non-zero data after the END record");
            at = partial_.size();
            break;
        }
        if (batch_.size() >= kBatchBytes) submit_batch();
    }
    partial_.erase(partial_.begin(), partial_.begin() + long(at));
    if (end_seen_) submit_batch();
}

void StreamDecoder::submit_batch()
{
    if (batch_.empty()) return;
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this] { return ordered_.size() < kMaxQueuedJobs || error_; });
    if (error_) std::rethrow_exception(error_);
    Job job;
    job.records = std::move(batch_);
    ordered_.push_back(job.result.get_future());
    jobs_.push_back(std::move(job));
    batch_.clear();
    changed_.notify_all();
}

StreamTotals StreamDecoder::finish()
{
    if (!partial_.empty()) throw StreamError("the stream ends inside a record");
    submit_batch();
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this] { return ordered_.empty() || error_; });
    if (error_) std::rethrow_exception(error_);
    if (output_ && std::fflush(output_) != 0) throw StreamError("output write failed");
    return totals_;
}

void StreamDecoder::limit_to_elapsed(std::chrono::steady_clock::time_point start, double pairs_per_second,
                                     double slack_seconds)
{
    limited_ = true;
    limit_start_ = start;
    limit_rate_ = pairs_per_second;
    limit_slack_ = slack_seconds;
}

void StreamDecoder::worker()
{
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
            if (jobs_.empty()) return;
            job = std::move(jobs_.front());
            jobs_.pop_front();
        }
        try {
            job.result.set_value(decode(std::move(job.records)));
        } catch (...) {
            job.result.set_exception(std::current_exception());
        }
    }
}

StreamDecoder::Result StreamDecoder::decode(std::vector<uint8_t> records) const
{
    Result result;
    std::array<uint32_t, IQR_BLOCK_PAIRS> pairs;
    const bool samples = format_ == OutputFormat::Cs16 || sink_;
    size_t at = 0;
    while (at < records.size()) {
        iqr::Header header;
        size_t size = iqr::inspect(records.data() + at, records.size() - at, &header);
        if (!size || !iqr::decode(records.data() + at, size, pairs.data(), pairs.size(), &header))
            throw StreamError("record failed its CRC or format check near pair " +
                              std::to_string(at ? result.span_end : header.index));
        if (limited_) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - limit_start_).count();
            if (double(header.index + header.count) > (elapsed + limit_slack_) * limit_rate_)
                throw StreamError("record index " + std::to_string(header.index) +
                                  " is ahead of the elapsed capture time");
        }
        if (at == 0) result.first_index = result.span_end = header.index;
        if (header.index < result.span_end)
            throw StreamError("record index went backwards at pair " + std::to_string(header.index));
        if (header.index > result.span_end) {
            // Pairs lost upstream: keep the output's time base.
            ++result.gaps;
            if (samples) result.samples.resize(result.samples.size() + 2 * (header.index - result.span_end), 0);
            result.span_end = header.index;
        }
        result.record_bytes += size;
        at += size;
        if (header.mode == IQR_END) {
            result.has_end = true;
            result.end_index = header.index;
            result.end_crc = header.crc;
            break;
        }
        result.crc = uint32_t(crc32_combine(result.crc, header.crc, 4 * header.count));
        result.pairs += header.count;
        result.span_end += header.count;
        ++result.records;
        if (samples)
            for (unsigned i = 0; i < header.count; ++i) {
                result.samples.push_back(component(pairs[i], 0));
                result.samples.push_back(component(pairs[i], 1));
            }
    }
    if (format_ == OutputFormat::Iqc) result.raw = std::move(records);
    return result;
}

void StreamDecoder::writer()
{
    for (;;) {
        std::future<Result> next;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] { return stopping_ || !ordered_.empty(); });
            if (ordered_.empty()) return;
            next = std::move(ordered_.front());
        }
        try {
            Result result = next.get();
            if (result.first_index < totals_.span)
                throw StreamError("record index went backwards at pair " + std::to_string(result.first_index));
            uint64_t gaps = result.gaps;
            bool written = true;
            if (result.first_index > totals_.span) {
                // A gap between jobs.
                ++gaps;
                if (format_ == OutputFormat::Cs16) {
                    std::vector<int16_t> zeros(2 * IQR_BLOCK_PAIRS, 0);
                    for (uint64_t left = result.first_index - totals_.span; left && written;) {
                        size_t n = size_t(std::min<uint64_t>(left, IQR_BLOCK_PAIRS));
                        written = std::fwrite(zeros.data(), 2 * sizeof(int16_t), n, output_) == n;
                        left -= n;
                    }
                }
            }
            uint32_t crc = uint32_t(crc32_combine(totals_.crc, result.crc, 4 * result.pairs));
            if (result.has_end && (result.end_index != result.span_end || result.end_crc != crc))
                throw StreamError("END record does not match the received stream");
            if (format_ == OutputFormat::Iqc)
                written = written && std::fwrite(result.raw.data(), 1, result.raw.size(), output_) == result.raw.size();
            else if (format_ == OutputFormat::Cs16)
                written = written && std::fwrite(result.samples.data(), sizeof(int16_t), result.samples.size(),
                                                 output_) == result.samples.size();
            if (!written) throw StreamError("output write failed");
            if (sink_ && !result.samples.empty()) sink_(result.first_index, std::move(result.samples));
            std::lock_guard<std::mutex> lock(mutex_);
            totals_.pairs += result.pairs;
            totals_.span = result.span_end;
            totals_.gaps += gaps;
            totals_.crc = crc;
            totals_.records += result.records;
            totals_.record_bytes += result.record_bytes;
            totals_.ended = totals_.ended || result.has_end;
            ordered_.pop_front();
            changed_.notify_all();
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!error_) error_ = std::current_exception();
            ordered_.clear();
            changed_.notify_all();
        }
    }
}
