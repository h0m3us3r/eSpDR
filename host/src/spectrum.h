// Live power spectrum of the decoded stream: windowed FFTs of evenly spaced
// blocks of samples, combined per frame and converted to dBFS.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct SpectrumSettings {
    unsigned size = 4096;      // FFT points: a power of two, 512..262144
    std::string window = "blackman-harris-4";
    double rate = 40;          // frames per second (of sample time)
    unsigned averaging = 1;    // FFT blocks per frame; 0: every sample
    bool peak = false;         // combine blocks by maximum, not mean power
    bool remove_dc = false;    // subtract each block's mean first

    // Throws std::runtime_error naming the first invalid field.
    void validate() const;
};

// FFT window names, in the order the UI offers them, and each one's
// equivalent noise bandwidth in bins.
const std::vector<std::string> &spectrum_windows();
double spectrum_window_bandwidth(const std::string &window);

struct SpectrumFrame {
    uint64_t run = 0;          // capture run the samples came from
    uint64_t index = 0;        // sample index of the frame's first block
    double center_hz = 0;      // LO
    double sample_rate = 0;    // span of the frame, Hz
    unsigned blocks = 0;       // FFT blocks combined
    bool peak = false;
    std::vector<float> db;     // dBFS per bin, from center - rate/2 upwards
};

class SpectrumEngine {
public:
    using Output = std::function<void(std::shared_ptr<const SpectrumFrame>)>;

    explicit SpectrumEngine(Output output);  // called on the engine's thread
    ~SpectrumEngine();
    SpectrumEngine(const SpectrumEngine &) = delete;
    SpectrumEngine &operator=(const SpectrumEngine &) = delete;

    void configure(const SpectrumSettings &settings);  // validated by the caller
    SpectrumSettings settings() const;

    // A new capture run: its sample indices start again at zero.
    void begin_run(uint64_t run, double center_hz, double sample_rate);
    // Decoded samples (a SampleSink). Never blocks: a batch that arrives
    // while the engine is still busy is dropped and counted.
    void add(uint64_t first_index, std::vector<int16_t> &&samples);

    struct Stats {
        uint64_t frames = 0, blocks = 0;
        uint64_t pairs_used = 0;     // pairs that went into FFT blocks
        uint64_t pairs_dropped = 0;  // pairs in batches dropped while busy
    };
    Stats stats() const;

private:
    struct Batch {
        uint64_t run;
        double center_hz, sample_rate;
        uint64_t first;
        std::vector<int16_t> samples;
    };
    struct Transform;

    void work();
    void process(Batch &batch);
    void finish_frame();

    Output output_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Batch> queue_;
    SpectrumSettings settings_;
    uint64_t settings_version_ = 1;
    uint64_t run_ = 0;
    double center_hz_ = 0, sample_rate_ = 0;
    bool stopping_ = false;
    Stats stats_;

    // Engine thread only.
    std::unique_ptr<Transform> transform_;
    SpectrumSettings active_;        // the settings in use
    uint64_t applied_version_ = 0;
    uint64_t current_run_ = ~0ull;
    double current_center_ = 0, current_rate_ = 0;
    uint64_t frame_pairs_ = 0, stride_ = 0;
    unsigned blocks_per_frame_ = 0;
    uint64_t frame_ = 0;             // frame being accumulated
    unsigned slot_ = 0;              // next block within it
    unsigned accumulated_ = 0;       // blocks in the accumulator
    std::vector<int16_t> carry_;     // tail of the previous batch
    uint64_t carry_first_ = 0;
    std::thread thread_;
};
