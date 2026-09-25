#include "spectrum.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <fftw3.h>

namespace {

constexpr size_t kQueuedBatches = 2;  // beyond this, the oldest batch is dropped
constexpr double kFullScale = 512;    // 10-bit two's complement components

// Cosine-sum windows: w[n] = sum_k (-1)^k a_k cos(2 pi k n / N), periodic.
const std::vector<std::pair<std::string, std::vector<double>>> &window_table()
{
    static const std::vector<std::pair<std::string, std::vector<double>>> table = {
        {"rectangular", {1}},
        {"hann", {0.5, 0.5}},
        {"hamming", {0.54, 0.46}},
        {"blackman", {0.42, 0.5, 0.08}},
        {"blackman-harris-4", {0.35875, 0.48829, 0.14128, 0.01168}},
        {"blackman-harris-7",
         {0.27105140069342, 0.43329793923448, 0.21812299954311, 0.06592544638803, 0.01081174209837,
          0.00077658482522, 0.00001388721735}},
        {"flat-top", {0.21557895, 0.41663158, 0.277263158, 0.083578947, 0.006947368}},
    };
    return table;
}

}  // namespace

const std::vector<std::string> &spectrum_windows()
{
    static const std::vector<std::string> names = [] {
        std::vector<std::string> list;
        for (const auto &entry : window_table()) list.push_back(entry.first);
        return list;
    }();
    return names;
}

double spectrum_window_bandwidth(const std::string &window)
{
    const unsigned n = 4096;
    double sum = 0, squares = 0;
    for (const auto &entry : window_table()) {
        if (entry.first != window) continue;
        for (unsigned i = 0; i < n; ++i) {
            double w = 0, sign = 1;
            for (size_t k = 0; k < entry.second.size(); ++k, sign = -sign)
                w += sign * entry.second[k] * std::cos(2 * M_PI * double(k) * i / n);
            sum += w;
            squares += w * w;
        }
    }
    return sum ? n * squares / (sum * sum) : 1;
}

void SpectrumSettings::validate() const
{
    if (size < 512 || size > 262144 || (size & (size - 1)))
        throw std::runtime_error("FFT size must be a power of two from 512 to 262144");
    const auto &names = spectrum_windows();
    if (std::find(names.begin(), names.end(), window) == names.end())
        throw std::runtime_error("unknown FFT window " + window);
    if (!(rate >= 1 && rate <= 100)) throw std::runtime_error("FFT rate must be 1..100 frames per second");
    if (averaging > 100000) throw std::runtime_error("averaging must be at most 100000 blocks");
}

// One FFT size and window, with the accumulator for the current frame.
struct SpectrumEngine::Transform {
    unsigned size;
    std::vector<float> window;
    std::vector<float> power;  // accumulated |X|^2, FFT order
    double scale;              // |X|^2 of a full-scale complex tone -> 1
    fftwf_complex *in, *out;
    fftwf_plan plan;

    Transform(unsigned n, const std::string &name) : size(n), window(n), power(n, 0.0f)
    {
        const std::vector<double> *terms = nullptr;
        for (const auto &entry : window_table())
            if (entry.first == name) terms = &entry.second;
        double sum = 0;
        for (unsigned i = 0; i < n; ++i) {
            double w = 0, sign = 1;
            for (size_t k = 0; k < terms->size(); ++k, sign = -sign)
                w += sign * (*terms)[k] * std::cos(2 * M_PI * double(k) * i / n);
            window[i] = float(w);
            sum += w;
        }
        scale = 1 / ((kFullScale * sum) * (kFullScale * sum));
        in = fftwf_alloc_complex(n);
        out = fftwf_alloc_complex(n);
        // Measuring finds a faster plan; the time limit keeps a new size from
        // stalling the stream for long (FFTW remembers plans it has made).
        fftwf_set_timelimit(0.1);
        plan = fftwf_plan_dft_1d(int(n), in, out, FFTW_FORWARD, FFTW_MEASURE);
    }
    ~Transform()
    {
        fftwf_destroy_plan(plan);
        fftwf_free(in);
        fftwf_free(out);
    }
};

SpectrumEngine::SpectrumEngine(Output output) : output_(std::move(output))
{
    thread_ = std::thread([this] { work(); });
}

SpectrumEngine::~SpectrumEngine()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    changed_.notify_all();
    thread_.join();
}

void SpectrumEngine::configure(const SpectrumSettings &settings)
{
    std::lock_guard<std::mutex> lock(mutex_);
    settings_ = settings;
    ++settings_version_;
}

SpectrumSettings SpectrumEngine::settings() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return settings_;
}

SpectrumEngine::Stats SpectrumEngine::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void SpectrumEngine::begin_run(uint64_t run, double center_hz, double sample_rate)
{
    std::lock_guard<std::mutex> lock(mutex_);
    run_ = run;
    center_hz_ = center_hz;
    sample_rate_ = sample_rate;
}

void SpectrumEngine::add(uint64_t first_index, std::vector<int16_t> &&samples)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kQueuedBatches) {
            stats_.pairs_dropped += queue_.front().samples.size() / 2;
            queue_.pop_front();
        }
        queue_.push_back({run_, center_hz_, sample_rate_, first_index, std::move(samples)});
    }
    changed_.notify_all();
}

void SpectrumEngine::work()
{
    for (;;) {
        Batch batch;
        SpectrumSettings settings;
        uint64_t version;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (stopping_) return;
            batch = std::move(queue_.front());
            queue_.pop_front();
            settings = settings_;
            version = settings_version_;
        }

        if (version != applied_version_ || batch.run != current_run_) {
            if (!transform_ || transform_->size != settings.size || settings.window != active_.window)
                transform_ = std::make_unique<Transform>(settings.size, settings.window);
            active_ = settings;
            applied_version_ = version;
            current_run_ = batch.run;
            current_center_ = batch.center_hz;
            current_rate_ = batch.sample_rate;
            // Frames are periods of sample time; their blocks are spread evenly
            // over the period (overlapping when there are more than fit).
            frame_pairs_ = std::max<uint64_t>(settings.size, uint64_t(std::llround(batch.sample_rate / settings.rate)));
            blocks_per_frame_ = settings.averaging ? settings.averaging
                                                   : unsigned(std::max<uint64_t>(1, frame_pairs_ / settings.size));
            stride_ = blocks_per_frame_ > 1 ? (frame_pairs_ - settings.size) / (blocks_per_frame_ - 1) : 0;
            frame_ = (batch.first + frame_pairs_ - 1) / frame_pairs_;  // the first whole period
            slot_ = 0;
            accumulated_ = 0;
            std::fill(transform_->power.begin(), transform_->power.end(), 0.0f);
            carry_.clear();
        }
        process(batch);
    }
}

void SpectrumEngine::process(Batch &batch)
{
    const SpectrumSettings &settings = active_;
    const unsigned size = transform_->size;
    const uint64_t batch_pairs = batch.samples.size() / 2, end = batch.first + batch_pairs;

    // The previous batch's tail joins this one when they are contiguous.
    uint64_t available = batch.first;
    if (!carry_.empty() && carry_first_ + carry_.size() / 2 == batch.first) available = carry_first_;
    else carry_.clear();

    std::vector<int16_t> joined;
    uint64_t used = 0;
    for (;;) {
        uint64_t frame_start = frame_ * frame_pairs_;
        uint64_t position = frame_start + slot_ * stride_;
        if (position < available) {
            // This block's samples were lost (a dropped batch or a gap in
            // the stream): move on to the first block that can still be made.
            if (available >= frame_start + frame_pairs_) {
                finish_frame();
                frame_ = available / frame_pairs_;
                slot_ = 0;
            } else {
                slot_ = stride_ ? unsigned((available - frame_start + stride_ - 1) / stride_) : blocks_per_frame_;
                if (slot_ >= blocks_per_frame_) {
                    finish_frame();
                    ++frame_;
                    slot_ = 0;
                }
            }
            continue;
        }
        if (position + size > end) break;

        const int16_t *pairs;
        if (position >= batch.first) {
            pairs = batch.samples.data() + 2 * (position - batch.first);
        } else {
            size_t from_carry = size_t(batch.first - position);
            joined.assign(carry_.end() - 2 * long(from_carry), carry_.end());
            joined.insert(joined.end(), batch.samples.begin(), batch.samples.begin() + 2 * long(size - from_carry));
            pairs = joined.data();
        }

        // Window, transform and accumulate.
        float mean_i = 0, mean_q = 0;
        if (settings.remove_dc) {
            double sum_i = 0, sum_q = 0;
            for (unsigned n = 0; n < size; ++n) {
                sum_i += pairs[2 * n];
                sum_q += pairs[2 * n + 1];
            }
            mean_i = float(sum_i / size);
            mean_q = float(sum_q / size);
        }
        fftwf_complex *in = transform_->in;
        const float *window = transform_->window.data();
        for (unsigned n = 0; n < size; ++n) {
            in[n][0] = (pairs[2 * n] - mean_i) * window[n];
            in[n][1] = (pairs[2 * n + 1] - mean_q) * window[n];
        }
        fftwf_execute(transform_->plan);
        const fftwf_complex *out = transform_->out;
        float *power = transform_->power.data();
        if (settings.peak) {
            for (unsigned k = 0; k < size; ++k)
                power[k] = std::max(power[k], out[k][0] * out[k][0] + out[k][1] * out[k][1]);
        } else {
            for (unsigned k = 0; k < size; ++k) power[k] += out[k][0] * out[k][0] + out[k][1] * out[k][1];
        }
        ++accumulated_;
        used += size;

        if (++slot_ == blocks_per_frame_) {
            finish_frame();
            ++frame_;
            slot_ = 0;
        }
    }

    // Keep the tail for a block that straddles into the next batch.
    uint64_t keep = std::min<uint64_t>(size - 1, end - available);
    if (keep <= batch_pairs) {
        carry_.assign(batch.samples.end() - 2 * long(keep), batch.samples.end());
    } else {
        std::vector<int16_t> tail(carry_.end() - 2 * long(keep - batch_pairs), carry_.end());
        tail.insert(tail.end(), batch.samples.begin(), batch.samples.end());
        carry_.swap(tail);
    }
    carry_first_ = end - keep;

    std::lock_guard<std::mutex> lock(mutex_);
    stats_.pairs_used += used;
}

void SpectrumEngine::finish_frame()
{
    if (!accumulated_) return;
    const SpectrumSettings &settings = active_;
    const unsigned size = transform_->size;
    auto frame = std::make_shared<SpectrumFrame>();
    frame->run = current_run_;
    frame->index = frame_ * frame_pairs_;
    frame->center_hz = current_center_;
    frame->sample_rate = current_rate_;
    frame->blocks = accumulated_;
    frame->peak = settings.peak;
    frame->db.resize(size);
    // The ESP's I + jQ is LO minus RF, so RF offset f appears at FFT bin -f.
    // Display bin i is RF offset (i - size/2) * rate / size.
    double scale = transform_->scale / (settings.peak ? 1 : accumulated_);
    float *power = transform_->power.data();
    for (unsigned i = 0; i < size; ++i) {
        unsigned k = (size / 2 - i) & (size - 1);
        frame->db[i] = float(10 * std::log10(power[k] * scale + 1e-20));
    }
    std::fill(transform_->power.begin(), transform_->power.end(), 0.0f);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.frames;
        stats_.blocks += accumulated_;
    }
    accumulated_ = 0;
    output_(std::move(frame));
}
