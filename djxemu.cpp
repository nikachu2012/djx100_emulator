#define _USE_MATH_DEFINES
// djxemu - DJ-X100 receiver emulator core (DLL).
//
//   SDR IQ (any rate) -> NCO + FIR decimation to fs1 (~96..192 kS/s) -> IF crystal filter model (SOS, from the GUI)
//   -> real IF at 18 kHz sampled at 72 kS/s (cubic interpolation, aliasing as in the radio) -> front-end AGC ->
//   12-bit ADC -> mode:
//   the DSP firmware executed instruction by instruction on a C55x simulator with the C5517 peripherals and the
//   AMBE-3000R (c55: c55sim + djxboard), in its own thread. Whole radio (djx_radio_load): the CPU firmware on the RX
//   simulator (rx/) boots and drives it, and the BK1068 broadcast receiver is a behavioural WFM model.
//   DSP alone (djx_load_firmware): the caller plays the CPU on the I2C link (djx_cpu_send / djx_cpu_line).
//   -> 24 kHz audio.
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "djxboard.h"
#include "djxcpu.h"

#ifdef _WIN32
#define DJX_API extern "C" __declspec(dllexport)
#else
#define DJX_API extern "C" __attribute__((visibility("default")))
#endif

typedef std::complex<double> cd;

// channel: MODE_FM = the main receiver (IF -> ADC -> DSP), MODE_WFM = the BK1068 broadcast receiver (WFM model)
enum { MODE_FM = 1, MODE_WFM = 4 };

struct Status {
    double level_db;          // channel power after the IF filter, dBFS of the SDR input
    double adc_rms;           // ADC RMS in LSB
    double agc_db;            // front-end AGC gain
    int adc_clip;             // clipped samples in the last second
    int mode;
    unsigned frames_to_ambe, speech_packets;   // AMBE-3000R channel packets, of them decoded
    // firmware backend
    int backend;              // 1: DSP firmware loaded and running on the C55x simulator
    int fw_queue;             // 10 ms ADC blocks waiting for the simulator (lag)
    double dsp_load, dsp_load_peak;   // simulated DSP: busy cycles / available (estimate), average and peak
    double mips;              // simulated instructions per second of wall time
    unsigned uart_tx, uart_rx, i2c_lines, halted;
};

struct Emu {
    std::mutex mtx;
    // input / channelizer
    double fs_in = 2400000, offset = 0, fs1 = 96000;
    int D = 25, L = 0;
    std::vector<double> h;                  // decimation FIR
    std::vector<cd> hist;                   // circular history (length L, duplicated)
    int hpos = 0, phase = 0;
    double nco_ph = 0, nco_step = 0;
    cd nco_z{1, 0}, nco_rot{1, 0};
    double nco_rot_step = 1e30;
    // IF filter (SOS, real coefficients, complex signal)
    std::vector<double> sos;                // nsec * 6: b0 b1 b2 a0 a1 a2
    std::vector<cd> z1, z2;
    // interpolation to 72 kS/s
    cd y[4];                                // last four fs1 samples
    double tpos = 0;                        // position of the next ADC sample in fs1 units, relative to y[1]
    uint32_t adc_n = 0;
    double agc = 1.0, pow_acc = 0;
    int pow_n = 0, clip = 0, clip_s = 0;
    double adc_ms = 0;
    double level = 0;
    // ADC block for the firmware
    uint16_t adc_buf[720];
    int adc_fill = 0;
    // BK1068 model (broadcast FM)
    double am_avg = 0;
    double sql_db = -200, sql_open = 0;
    double wfm_t = 0, wfm_deem = 0, wfm_lp = 0; cd wfm_prev = 0;
    int mode = MODE_FM;
    double volume = 1.0;
    // output
    std::deque<int16_t> audio;
    // status / events
    Status s{};
    std::deque<std::string> events;
    uint64_t adc_count = 0;
    // channel rings for the GUI spectrum (fs1, before / after the IF filter)
    std::vector<cd> ring_pre = std::vector<cd>(8192), ring_post = std::vector<cd>(8192);
    size_t ring_pos = 0;
    // 1 = firmware loaded (worker thread runs the simulators)
    int backend = 0;
    std::mutex omtx;                         // outputs: audio, events, status
    std::mutex fwmtx;                        // the simulator
    std::mutex qmtx;
    std::condition_variable qcv;
    std::deque<std::array<uint16_t, 720>> q;
    std::thread worker;
    bool stop = false;
    tsim_stats ts{};
    bool stock = false;                      // the loaded image is a stock DigRxx (all modes run in the DSP)
    bool if_invert = false;                  // spectrum inversion of the IF chain (as in the radio)
    double hb_hist[64] = {0};                // 48 -> 24 kHz half-band decimator
    int hb_pos = 0;
    double mips = 0;
    uint64_t mips_insns = 0;
    std::chrono::steady_clock::time_point mips_t;
    // whole-radio backend: the stock CPU firmware (RX65N simulator, rx/) drives the stock DSP firmware
    bool radio = false;
    std::string dsp_bin, dsp_dis;
    std::deque<std::string> dev_q;           // DSP I2C lines to other devices (codec), moved out by the worker (omtx)
    bool dsp_run = false;
    std::mutex lcdmtx;
    std::vector<uint32_t> lcd = std::vector<uint32_t>(320 * 240, 0xFF000000u);
    unsigned lcd_frames = 0, radio_blocks = 0;
    std::chrono::steady_clock::time_point last_blk = std::chrono::steady_clock::now();
    std::string radio_err;
    // panel requests from the GUI thread (lock-free for the caller; applied at the next slice boundary)
    std::mutex kmtx;
    std::deque<uint32_t> kq;                 // key mask changes, each held until the firmware has scanned it
    uint32_t kcur = 0;
    int khold = 100;                         // 10 ms steps the current mask has been applied
    uint32_t kscan0 = 0;                     // rxcpu_scan_passes() when it was applied
    std::atomic<int> enc_add[2] = {{0}, {0}};
    std::atomic<int> cpi_req{0};
    std::atomic<double> radio_t{0};
    std::atomic<uint32_t> tuned_hz{0}, tuned_n{0};
    // RF section model: channel level / SNR measured on the SDR channel (fs1), fed to the CPU's RSSI and noise
    // detector A/D inputs before every 10 ms slice
    std::vector<cd> rf_buf;
    double n0_wide = 0;                      // noise per fs1 FFT bin from the whole SDR band (0 = unknown)
    uint64_t wide_n = 0;
    std::atomic<double> rf_dbm{-127.0}, rf_snr{-10.0};
    double rf_floor_dbm = -127.0;            // receiver noise floor in the channel (12 kHz, NF ~6 dB)
    std::atomic<double> rf_gain_db{0.0};     // antenna level offset (GUI)
    std::atomic<bool> rf_quiet{true};        // no SDR data for 300 ms: receiver noise only
    std::atomic<int64_t> iq_ms{0};           // wall time of the last SDR block (djx_process)
    // BK1068 (broadcast FM): when the firmware powers and tunes it, the channel switches to the WFM model
    std::atomic<int> want_wfm{0};
    std::atomic<double> bk_gain{0.0};
    std::deque<int16_t> wfm_q;               // BK1068 audio (24 kHz), mixed with the codec stream (omtx)
    double beep_ph = 0;
    double af_gain = 0;                      // AF volume (DAC ch7 x ch8) applied to the codec stream
    std::atomic<int> af_on{0}, af_vol{-1};
};

static Emu *g;

static void push_event(const std::string &e)
{
    g->events.push_back(e);
    while (g->events.size() > 500) g->events.pop_front();
}

static void push_audio(const int16_t *a, int n)
{
    std::lock_guard<std::mutex> lk(g->omtx);
    for (int i = 0; i < n; i++) {
        double v = a[i] * g->volume;
        g->audio.push_back((int16_t)std::max(-32767.0, std::min(32767.0, v)));
    }
    while (g->audio.size() > 24000 * 4) g->audio.pop_front();
}

// 48 kHz codec stream -> 24 kHz (31-tap windowed-sinc half-band, cut-off 10 kHz)
static int decimate48(int16_t *a, int n)
{
    static double h[31];
    static bool init = false;
    if (!init) {
        double sum = 0;
        for (int i = 0; i < 31; i++) {
            double m = i - 15, fc = 10000.0 / 48000.0;
            double v = m == 0 ? 2 * fc : std::sin(2 * M_PI * fc * m) / (M_PI * m);
            v *= 0.54 - 0.46 * std::cos(2 * M_PI * i / 30);
            h[i] = v; sum += v;
        }
        for (double &v : h) v /= sum;
        init = true;
    }
    int k = 0;
    for (int i = 0; i < n; i++) {
        g->hb_hist[g->hb_pos] = g->hb_hist[g->hb_pos + 32] = a[i];
        g->hb_pos = (g->hb_pos + 1) & 31;
        if (i & 1) {
            double acc = 0;
            for (int j = 0; j < 31; j++) acc += h[j] * g->hb_hist[g->hb_pos + 1 + j];
            a[k++] = (int16_t)std::max(-32767.0, std::min(32767.0, acc));
        }
    }
    return k;
}

// whole radio. The CPU (RX65N simulator) and the DSP (C55x simulator) run their 10 ms slices in parallel on two
// threads and exchange the I2C command lines and the DSP's "ready" line at the slice boundaries (the radio's link
// protocol works in milliseconds, the CPU waits up to 0.5 s for an answer).
struct RadioCpuThread {
    std::thread th;
    std::mutex m;
    std::condition_variable cv;
    bool go = false, done = false, quit = false;
    const char *err = nullptr;
    std::vector<std::string> cmds;           // CPU -> DSP lines produced during the slice
    int ready = 0;                           // DSP ready line as seen by the CPU during the slice
    double cpu_s = 0;                        // wall time spent in the CPU simulator (DJX_PERF)
    void start()
    {
        th = std::thread([this] {
            for (;;) {
                std::unique_lock<std::mutex> lk(m);
                cv.wait(lk, [this] { return go || quit; });
                if (quit) return;
                go = false;
                lk.unlock();
                auto t0 = std::chrono::steady_clock::now();
                const char *e = rxcpu_run(0.01);
                cpu_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                lk.lock();
                err = e;
                done = true;
                cv.notify_all();
            }
        });
    }
    void run_async() { std::lock_guard<std::mutex> lk(m); done = false; go = true; cv.notify_all(); }
    const char *wait() { std::unique_lock<std::mutex> lk(m); cv.wait(lk, [this] { return done; }); return err; }
};
static RadioCpuThread *g_cpu_th;

static void radio_to_dsp(const uint8_t *d, int n)      // called by the CPU thread (inside rxcpu_run)
{
    std::string c((const char *)d, (size_t)n);
    while (!c.empty() && (c.back() == '\n' || c.back() == '\r')) c.pop_back();
    g_cpu_th->cmds.push_back(c);
}
static int radio_dsp_ready(void) { return g_cpu_th->ready; }

// whole radio: one 10 ms step of the CPU and of the DSP (fwmtx held). Returns the DSP's 48 kHz samples.
// Which receiver is in use: the firmware leaves the BK1068 powered and tuned (and its DAC volume set) after it
// leaves RADIO for VFO / memory, and the main receiver may already sit on the VFO frequency (tuned at boot), so no
// pin changes on RADIO -> VFO. The firmware's own state byte tells: RAM 0x84EAC2 high nibble 1 = RADIO (BK1068),
// 2 = VFO (CPU firmware 4.00; also kept at 0x82D528 = EEPROM settings 0x36).
static bool radio_bk_in_use(int powered, uint32_t bk_hz)
{
    return powered && bk_hz && (rxcpu_mem8(0x84EAC2u) >> 4) == 1;
}

static int radio_step(const uint16_t *blk, int16_t *a, int max)
{
    if (!g->radio_err.empty()) { memset(a, 0, sizeof(int16_t) * 480); return 480; }
    if (!g_cpu_th) { g_cpu_th = new RadioCpuThread(); g_cpu_th->start(); }
    {   // panel input
        std::lock_guard<std::mutex> lk(g->kmtx);
        // a key change stays applied until the firmware's matrix scan (one row per main-loop call, debounced over
        // two reads of each row) has seen it: >= 3 full scans and >= 60 ms. The scan rate depends on how busy the
        // main loop is (10 ms on the RADIO screen, much slower on heavy screens), so a fixed time dropped clicks.
        // No scan at all (e.g. another program on the CPU): plain 120 ms; never more than 1 s.
        g->khold++;
        uint32_t sp = rxcpu_scan_passes() - g->kscan0;
        bool seen = g->khold >= 6 && sp >= 3, noscan = g->khold >= 12 && sp == 0;
        if ((seen || noscan || g->khold >= 100) && !g->kq.empty()) {
            g->kcur = g->kq.front();
            g->kq.pop_front();
            g->khold = 0;
            g->kscan0 = rxcpu_scan_passes();
            rxcpu_set_keys(g->kcur);
            if (getenv("DJX_KEYLOG")) {
                char t[64];
                snprintf(t, sizeof t, "%.3f keys %05X", rxcpu_time(), (unsigned)g->kcur);
                std::lock_guard<std::mutex> ol(g->omtx);
                push_event(t);
            }
        }
    }
    for (int w = 0; w < 2; w++) if (int d = g->enc_add[w].exchange(0)) rxcpu_encoder(w, d);
    if (int c = g->cpi_req.exchange(0)) rxcpu_set_cpi(c);
    {
        int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        g->rf_quiet = now - g->iq_ms > 300;
    }
    if (g->rf_quiet) rxcpu_set_rf(g->rf_floor_dbm + g->rf_gain_db, -10.0);   // no antenna signal
    else rxcpu_set_rf(g->rf_dbm, g->rf_snr);
    g_cpu_th->ready = g->dsp_run ? tsim_dsp_ready() : 0;
    g_cpu_th->cmds.clear();
    g_cpu_th->run_async();
    int na = 480;
    static double dsp_s = 0;
    static int perf_n = 0;
    auto pt0 = std::chrono::steady_clock::now();
    if (g->dsp_run) na = tsim_step2(blk, a, max);                               // in parallel with the CPU
    else memset(a, 0, sizeof(int16_t) * 480);                                   // DSP in reset: silence
    dsp_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - pt0).count();
    static double dsp_e = 0;
    for (int j = 0; j < na; j++) dsp_e += (double)a[j] * a[j];
    if (++perf_n == 500) {                                                      // DJX_PERF: load per 5 s of radio time
        if (getenv("DJX_PERF")) fprintf(stderr, "perf: CPU %.0f%% DSP %.0f%% of one core, DSP audio rms %.0f\n", g_cpu_th->cpu_s / 5 * 100, dsp_s / 5 * 100, std::sqrt(dsp_e / 240000));
        perf_n = 0; dsp_s = 0; g_cpu_th->cpu_s = 0; dsp_e = 0;
    }
    const char *e = g_cpu_th->wait();
    if (e) { g->radio_err = e; return na; }
    {   // audio path after the codec: AF volume = two DAC channels (ch7, ch8: the firmware's 33-step table at
        // 0xFFE0C552), AF power amplifier on/off (TCA6424 port 2 bits 6/7: squelch open, monitor, beeps)
        int d7 = rxcpu_dac(7), d8 = rxcpu_dac(8);
        bool amp = (rxcpu_expander_out(2) & 0xC0) != 0;
        double want = (amp && d7 > 0 && d8 > 0) ? (d7 / 255.0) * (d8 / 255.0) * 8.0 : 0.0;
        g->af_on = amp ? 1 : 0;
        g->af_vol = d7 < 0 || d8 < 0 ? -1 : d7 * 256 + d8;
        if (getenv("DJX_AFLOG")) {
            static int k = 0;
            double e2 = 0;
            for (int i = 0; i < na; i++) e2 += (double)a[i] * a[i];
            if (k++ % 50 == 0) fprintf(stderr, "af: t %.2f quiet %d na %d dsp rms %.0f amp %d d7 %d d8 %d want %.3f run %d\n", rxcpu_time(), (int)(blk[0] == 2048 && blk[1] == 2048 && blk[300] == 2048), na, std::sqrt(e2 / std::max(1, na)), (int)amp, d7, d8, want, (int)g->dsp_run);
        }
        double g0 = g->af_gain, step = (want - g0) / std::max(1, na);
        for (int i = 0; i < na; i++) {
            g0 += step;
            a[i] = (int16_t)std::max(-32767.0, std::min(32767.0, a[i] * g0));
        }
        g->af_gain = want;
        // beeps: TPU4 square wave on P2.5 into the AF amplifier; level = DAC ch1 (the firmware's beep level)
        double beep_hz = rxcpu_beep_hz();
        int d1 = rxcpu_dac(1);
        if (beep_hz > 0 && amp && d1 > 0) {
            double lv = 9000.0 * d1 / 255.0, w = 2 * M_PI * beep_hz / 48000.0;
            for (int i = 0; i < na; i++) {
                g->beep_ph += w;
                if (g->beep_ph > 2 * M_PI) g->beep_ph -= 2 * M_PI;
                double v = std::sin(g->beep_ph) + std::sin(3 * g->beep_ph) / 3;   // softened square
                a[i] = (int16_t)std::max(-32767.0, std::min(32767.0, a[i] + lv * v));
            }
        }
        // BK1068: its own audio (WFM model in the channelizer) through the AF amplifier, radio volume = DAC ch5
        int pw = 0, bvol = 0;
        uint32_t bcnt = 0;
        uint32_t bhz = rxcpu_bk1068(&pw, &bvol, &bcnt);
        bool bk = radio_bk_in_use(pw, bhz);
        g->want_wfm = bk ? 1 : 0;
        int d5 = rxcpu_dac(5);
        double bg = (amp && d5 > 0) ? (d5 / 255.0) * (d5 / 255.0) * 2.0 * std::pow(10.0, -(31 - bvol) * 1.5 / 20) : 0.0;
        g->bk_gain = bk ? bg : 0.0;
        if (bk) {   // its RSSI register (unit ~dBuV: the firmware's S-meter spans 50..79) and quality register
            double rel = g->rf_quiet ? 0.0 : g->rf_dbm - (g->rf_floor_dbm + 12.0);
            rxcpu_bk1068_rf((int)std::lround(45 + 0.75 * rel), g->rf_quiet ? 0 : (int)std::lround(std::max(0.0, g->rf_snr.load())));
        }
    }
    // slice boundary: exchange the link lines, follow the DSP reset line
    if (g->dsp_run) {
        char line[256];
        while (tsim_cpu_line(line, sizeof line)) {
            std::string l = std::string(line) + "\r\n";
            rxcpu_dsp_line((const uint8_t *)l.data(), (int)l.size());
            std::lock_guard<std::mutex> lk(g->omtx);
            push_event(std::string("DSP>CPU ") + line);
        }
    }
    bool rst = (rxcpu_port_out(8) & 0x04) && (rxcpu_port_dir(8) & 0x04);     // P8.2: DSP reset (low = reset)
    if (rst && !g->dsp_run) {                                                  // boots again from its SPI flash
        const char *ee = tsim_init(g->dsp_bin.c_str(), g->dsp_dis.c_str(), nullptr);
        if (ee) { g->radio_err = ee; return na; }
        std::lock_guard<std::mutex> lk(g->omtx);
        push_event("DSP reset released (boot)");
    }
    g->dsp_run = rst;
    rxcpu_set_dsp_clock(rst);
    for (const std::string &c : g_cpu_th->cmds) {
        if (rst) tsim_cpu_send(c.c_str());
        std::lock_guard<std::mutex> lk(g->omtx);
        push_event("CPU>DSP " + c);
    }
    {
        uint32_t n = 0, bn = 0;
        int pw = 0;
        uint32_t hz = rxcpu_tuned_hz(&n), bhz = rxcpu_bk1068(&pw, nullptr, &bn);
        bool bk = radio_bk_in_use(pw, bhz);
        g->tuned_hz = bk ? bhz : hz;                       // the receiver that is in use
        g->tuned_n = n + bn + (bk ? 0x40000000u : 0);
        g->radio_t = rxcpu_time();
    }
    if (++g->radio_blocks % 4 == 0) {                                           // LCD at 25 frames/s
        static std::vector<uint32_t> fr(320 * 240);
        if (rxcpu_lcd_rgba(fr.data())) {
            std::lock_guard<std::mutex> lk(g->lcdmtx);
            g->lcd.swap(fr);
            g->lcd_frames++;
        }
    }
    return na;
}

static void worker_main()
{
    for (;;) {
        std::array<uint16_t, 720> blk;
        {
            std::unique_lock<std::mutex> lk(g->qmtx);
            if (g->radio) {
                // the radio keeps running without SDR data (quiet ADC input), paced by the wall clock: one 10 ms
                // block per 10 ms deadline (as fast as possible while behind, at most 0.2 s of catch-up)
                static auto due = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (now - due > std::chrono::milliseconds(200)) due = now;
                g->qcv.wait_until(lk, due, [] { return g->stop || !g->q.empty(); });
                if (g->stop) return;
                if (g->q.empty()) {
                    if (std::chrono::steady_clock::now() - g->last_blk < std::chrono::milliseconds(300)) { due += std::chrono::milliseconds(10); continue; }
                    blk.fill(2048);
                    due += std::chrono::milliseconds(10);
                } else {
                    due = std::chrono::steady_clock::now();
                    blk = g->q.front();
                    g->q.pop_front();
                    g->last_blk = std::chrono::steady_clock::now();
                }
            } else {
                g->qcv.wait(lk, [] { return g->stop || !g->q.empty(); });
                if (g->stop) return;
                blk = g->q.front();
                g->q.pop_front();
            }
        }
        g->qcv.notify_all();
        int16_t a[960];
        int na;
        tsim_stats ts;
        {
            std::lock_guard<std::mutex> lk(g->fwmtx);
            na = g->radio ? radio_step(blk.data(), a, 960) : tsim_step2(blk.data(), a, 960);
            tsim_stats_get(&ts);               // a few memory reads: published after every 10 ms step
            char dl[256];
            std::vector<std::string> dls;
            while (tsim_dev_line(dl, sizeof dl)) dls.emplace_back(dl);
            if (!dls.empty()) {
                std::lock_guard<std::mutex> ol(g->omtx);
                for (auto &x : dls) if (g->dev_q.size() < 1000) g->dev_q.push_back(x);
            }
        }
        if (g->stock) na = decimate48(a, na);
        if (g->mode == MODE_WFM && !g->radio) na = 0;
        {
            std::lock_guard<std::mutex> lk(g->omtx);
            if (g->radio) {                    // the BK1068's audio joins the codec stream at the amplifier
                if (g->wfm_q.size() > 4800) g->wfm_q.erase(g->wfm_q.begin(), g->wfm_q.end() - 2400);
                for (int i = 0; i < na && !g->wfm_q.empty(); i++) {
                    a[i] = (int16_t)std::max(-32767, std::min(32767, a[i] + g->wfm_q.front()));
                    g->wfm_q.pop_front();
                }
            }
            g->ts = ts;
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - g->mips_t).count();
            if (dt > 0.5) {
                g->mips = (ts.insns - g->mips_insns) / dt / 1e6;
                g->mips_insns = ts.insns;
                g->mips_t = now;
            }
        }
        push_audio(a, na);
    }
}

// ---- channelizer ----------------------------------------------------------------------------------------
static void design_decimator(Emu *e)
{
    double target = e->mode == MODE_WFM ? 240000.0 : 96000.0;
    e->D = (int)std::floor(e->fs_in / target);
    if (e->D < 1) e->D = 1;
    e->fs1 = e->fs_in / e->D;
    int taps = e->D == 1 ? 1 : 8 * e->D + 1;
    e->L = taps;
    e->h.assign((size_t)taps, 0.0);
    double fc = (e->mode == MODE_WFM ? 110000.0 : 40000.0) / e->fs_in, sum = 0;   // cut-off (normalised to fs_in)
    for (int i = 0; i < taps; i++) {
        double m = i - (taps - 1) / 2.0;
        double sinc = m == 0 ? 2 * fc : std::sin(2 * M_PI * fc * m) / (M_PI * m);
        double w = taps == 1 ? 1 : 0.42 - 0.5 * std::cos(2 * M_PI * i / (taps - 1)) + 0.08 * std::cos(4 * M_PI * i / (taps - 1));
        e->h[(size_t)i] = sinc * w;
        sum += e->h[(size_t)i];
    }
    for (double &v : e->h) v /= sum;
    e->hist.assign((size_t)(2 * taps), cd(0, 0));
    e->hpos = 0;
    e->phase = 0;
    e->nco_step = -2 * M_PI * e->offset / e->fs_in;
}

static void push_fs1(Emu *e, cd x);

static void channelize(Emu *e, const float *iq, int n)
{
    for (int k = 0; k < n; k++) {
        cd x(iq[2 * k], iq[2 * k + 1]);
        if ((k & 1023) == 0 || e->nco_rot_step != e->nco_step) {   // exact phasor now and then (no drift)
            e->nco_z = cd(std::cos(e->nco_ph), std::sin(e->nco_ph));
            e->nco_rot = cd(std::cos(e->nco_step), std::sin(e->nco_step));
            e->nco_rot_step = e->nco_step;
        }
        x *= e->nco_z;
        e->nco_z *= e->nco_rot;
        e->nco_ph += e->nco_step;
        if (e->nco_ph > M_PI) e->nco_ph -= 2 * M_PI;
        if (e->nco_ph < -M_PI) e->nco_ph += 2 * M_PI;
        e->hist[(size_t)e->hpos] = e->hist[(size_t)(e->hpos + e->L)] = x;
        e->hpos = (e->hpos + 1) % e->L;
        if (++e->phase >= e->D) {
            e->phase = 0;
            cd acc(0, 0);
            const cd *w = &e->hist[(size_t)e->hpos];      // oldest .. newest
            for (int j = 0; j < e->L; j++) acc += w[j] * e->h[(size_t)j];
            push_fs1(e, acc);
        }
    }
}

// ---- IF filter, real IF sampling, ADC ------------------------------------------------------------------
static cd if_filter(Emu *e, cd x)
{
    size_t ns = e->sos.size() / 6;
    for (size_t i = 0; i < ns; i++) {
        const double *c = &e->sos[6 * i];
        cd yv = c[0] * x + e->z1[i];
        e->z1[i] = c[1] * x - c[4] * yv + e->z2[i];
        e->z2[i] = c[2] * x - c[5] * yv;
        x = yv;
    }
    return x;
}

static void adc_sample(Emu *e, double r)
{
    // front-end AGC (the radio's RF/IF AGC): keep the ADC around 300 LSB RMS, slow
    double v = r * e->agc * 2048.0;
    e->adc_ms += v * v;
    if (++e->pow_n >= 7200) {                        // 100 ms
        double rms = std::sqrt(e->adc_ms / e->pow_n);
        e->s.adc_rms = rms;
        double want = 300.0;
        if (rms > 1e-9) {
            double ratio = want / rms;
            ratio = std::min(4.0, std::max(0.25, ratio));
            e->agc *= std::pow(ratio, 0.3);
            e->agc = std::min(1e7, std::max(1e-3, e->agc));
        }
        e->s.agc_db = 20 * std::log10(e->agc);
        e->adc_ms = 0;
        e->pow_n = 0;
    }
    long q = std::lround(v) + 2048;
    if (q < 0 || q > 4095) { e->clip++; q = q < 0 ? 0 : 4095; }
    e->adc_buf[e->adc_fill++] = (uint16_t)q;
    e->adc_count++;
    if (e->adc_fill == 720) {
        e->adc_fill = 0;
        {
            if (e->backend == 1) {
                std::array<uint16_t, 720> blk;
                memcpy(blk.data(), e->adc_buf, sizeof e->adc_buf);
                std::unique_lock<std::mutex> lk(e->qmtx);
                // back-pressure: at most 0.5 s of signal ahead of the simulated DSP
                e->qcv.wait(lk, [e] { return e->q.size() < 50 || e->stop; });
                e->q.push_back(blk);
                lk.unlock();
                e->qcv.notify_all();
            }
        }
        if (e->adc_count % 72000 < 720) { e->s.adc_clip = e->clip_s = e->clip; e->clip = 0; }
    }
}

static cd cubic(const cd *p, double t)               // p[0..3] = y[-1], y[0], y[1], y[2]; t in [0,1)
{
    cd a0 = p[1];
    cd a1 = 0.5 * (p[2] - p[0]);
    cd a2 = p[0] - 2.5 * p[1] + 2.0 * p[2] - 0.5 * p[3];
    cd a3 = 0.5 * (p[3] - p[0]) + 1.5 * (p[1] - p[2]);
    return ((a3 * t + a2) * t + a1) * t + a0;
}

static void wfm_sample(Emu *e, cd x)            // BK1068 behavioural model
{
    double d = std::arg(x * std::conj(e->wfm_prev));
    e->wfm_prev = x;
    double a = d * e->fs1 / (2 * M_PI) / 75000.0;  // +-1 at +-75 kHz
    double k = 1.0 - std::exp(-1.0 / (50e-6 * e->fs1));
    e->wfm_deem += k * (a - e->wfm_deem);          // 50 us de-emphasis
    e->wfm_lp += 0.35 * (e->wfm_deem - e->wfm_lp);
    e->wfm_t += 24000.0 / e->fs1;
    if (e->wfm_t >= 1.0) {
        e->wfm_t -= 1.0;
        double p = std::norm(x);
        e->am_avg = 0.999 * e->am_avg + 0.001 * p;
        e->sql_open = 10 * std::log10(e->am_avg * e->agc * e->agc + 1e-20) > e->sql_db;
        double gk = e->radio ? e->bk_gain.load() : 1.0;
        int16_t o16 = e->sql_open ? (int16_t)std::max(-32767.0, std::min(32767.0, e->wfm_lp * 20000.0 * gk)) : 0;
        if (e->radio) { std::lock_guard<std::mutex> lk(e->omtx); e->wfm_q.push_back(o16); }
        else push_audio(&o16, 1);
    }
}

// ---- RF section model ------------------------------------------------------------------------------------
static void fft_inplace(std::vector<cd> &a)
{
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = -2 * M_PI / (double)len;
        cd wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            cd w(1, 0);
            for (size_t k = 0; k < len / 2; k++) {
                cd u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// Signal level and SNR of the receive channel (fs1 baseband, centred on the tuned frequency): power within
// +-6 kHz against the noise density (median of the bins 9..36 kHz off the channel). The level is referred to the
// receiver's own noise floor, so any SDR gain maps to the same S-meter reading for the same SNR.
static void rf_measure(Emu *e)
{
    const size_t N = e->rf_buf.size();
    std::vector<cd> a(e->rf_buf);
    for (size_t i = 0; i < N; i++) a[i] *= 0.5 - 0.5 * std::cos(2 * M_PI * (double)i / (double)N);
    fft_inplace(a);
    double df = e->fs1 / (double)N, pin = 0;
    int nin = 0;
    std::vector<double> nb;
    bool wfm = e->mode == MODE_WFM;
    // narrow: signal +-6 kHz, noise = median of 9..36 kHz off; WFM (BK1068): signal +-100 kHz, noise = 10th percentile
    // of the whole +-115 kHz (a broadcast channel fills it)
    double bw = wfm ? 100000.0 : 6000.0, nlo = wfm ? 0.0 : 9000.0, nhi = wfm ? 115000.0 : std::min(36000.0, 0.42 * e->fs1);
    for (size_t k = 0; k < N; k++) {
        double f = (k < N / 2 ? (double)k : (double)k - (double)N) * df, p = std::norm(a[k]);
        double af = std::fabs(f);
        if (af <= bw) { pin += p; nin++; }
        if (af >= nlo && af <= nhi && (wfm || af > bw)) nb.push_back(p);
    }
    if (nb.size() < 16 || nin == 0) return;
    double q = wfm ? 0.10 : 0.5;
    size_t qi = (size_t)((double)nb.size() * q);
    std::nth_element(nb.begin(), nb.begin() + (long)qi, nb.end());
    double n0 = nb[qi] / -std::log(1.0 - q);                         // quantile of an exponential -> mean
    if (e->n0_wide > 0 && (wfm || e->n0_wide < n0)) n0 = e->n0_wide;  // the SDR band's floor when it is lower
    double rel_db, snr_db;
    if (n0 <= 1e-30) { rel_db = pin > 1e-30 ? 60 : 0; }
    else rel_db = 10 * std::log10(std::max(1.0, pin / (n0 * nin)));
    rel_db = std::min(rel_db, 90.0);
    double snr = std::pow(10.0, rel_db / 10) - 1;
    snr_db = 10 * std::log10(std::max(snr, 1e-3));
    double dbm = e->rf_floor_dbm + (wfm ? 12.0 : 0.0) + rel_db + e->rf_gain_db;   // 200 kHz: floor +12 dB
    // light smoothing (the CPU averages 5 readings itself)
    e->rf_dbm = 0.5 * e->rf_dbm + 0.5 * dbm;
    e->rf_snr = 0.5 * e->rf_snr + 0.5 * snr_db;
}

// Noise floor from the whole SDR band: 20th percentile of a 4096-point spectrum of the raw input every ~0.25 s
// (most of a 1-2.4 MHz band is empty), scaled to a bin of the fs1 channel FFT (same length, same window).
static void wide_floor(Emu *e, const float *iq, int n)
{
    const int N = 4096;
    e->wide_n += (uint64_t)n;
    if (n < N || e->wide_n < (uint64_t)(e->fs_in / 4)) return;
    e->wide_n = 0;
    std::vector<cd> a(N);
    for (int i = 0; i < N; i++) a[i] = cd(iq[2 * i], iq[2 * i + 1]) * (0.5 - 0.5 * std::cos(2 * M_PI * i / N));
    fft_inplace(a);
    std::vector<double> pw(N);
    for (int i = 0; i < N; i++) pw[i] = std::norm(a[i]);
    pw[0] = pw[1];                                                    // DC spike of the SDR
    size_t qi = (size_t)(N * 0.2);
    std::nth_element(pw.begin(), pw.begin() + (long)qi, pw.end());
    double n0 = pw[qi] / -std::log(0.8) * (e->fs1 / e->fs_in);
    e->n0_wide = e->n0_wide > 0 ? 0.7 * e->n0_wide + 0.3 * n0 : n0;
}

static void push_fs1(Emu *e, cd x)
{
    if (e->radio) {
        e->rf_buf.push_back(x);
        if (e->rf_buf.size() >= 2048) { rf_measure(e); e->rf_buf.clear(); }
    }
    if (e->mode == MODE_WFM) {
        e->ring_pre[e->ring_pos] = x;
        e->ring_post[e->ring_pos] = x;
        e->ring_pos = (e->ring_pos + 1) & 8191;
        e->level = 0.999 * e->level + 0.001 * std::norm(x);
        wfm_sample(e, x);
        return;
    }
    cd yv = e->sos.empty() ? x : if_filter(e, x);
    e->ring_pre[e->ring_pos] = x;
    e->ring_post[e->ring_pos] = yv;
    e->ring_pos = (e->ring_pos + 1) & 8191;
    e->level = 0.999 * e->level + 0.001 * std::norm(yv);
    e->y[0] = e->y[1]; e->y[1] = e->y[2]; e->y[2] = e->y[3]; e->y[3] = yv;
    // ADC samples (72 kS/s) between y[1] and y[2]
    double step = e->fs1 / 72000.0;
    while (e->tpos < 1.0) {
        cd v = cubic(e->y, e->tpos);
        static const double c[4] = {1, 0, -1, 0}, s[4] = {0, 1, 0, -1};   // e^{j pi n / 2}
        int ph = (int)(e->adc_n & 3);
        // Re{v e^{j 2 pi 18k t}}; inverted: Re{conj(v) e^{j 2 pi 18k t}} - the radio's IF chain inverts the spectrum
        // (the stock DSP decodes 4FSK only this way round)
        double r = e->if_invert ? v.real() * c[ph] + v.imag() * s[ph] : v.real() * c[ph] - v.imag() * s[ph];
        e->adc_n++;
        adc_sample(e, r);
        e->tpos += step;
    }
    e->tpos -= 1.0;
}

// ---- API -------------------------------------------------------------------------------------------------
DJX_API int djx_init(double fs_in, double offset_hz)
{
    if (!g) g = new Emu();
    std::lock_guard<std::mutex> lk(g->mtx);
    g->fs_in = fs_in;
    g->offset = offset_hz;
    design_decimator(g);
    return (int)g->fs1;
}

DJX_API double djx_fs1(void) { return g ? g->fs1 : 0; }

DJX_API void djx_set_offset(double offset_hz)
{
    std::lock_guard<std::mutex> lk(g->mtx);
    g->offset = offset_hz;
    g->nco_step = -2 * M_PI * offset_hz / g->fs_in;
}

DJX_API void djx_set_if_sos(const double *sos, int nsec)
{
    std::lock_guard<std::mutex> lk(g->mtx);
    g->sos.assign(sos, sos + 6 * nsec);
    g->z1.assign((size_t)nsec, cd(0, 0));
    g->z2.assign((size_t)nsec, cd(0, 0));
}

DJX_API void djx_set_mode(int mode)
{
    std::lock_guard<std::mutex> lk(g->mtx);
    bool redesign = (mode == MODE_WFM) != (g->mode == MODE_WFM);
    g->mode = mode;
    if (redesign) design_decimator(g);
}
DJX_API void djx_set_volume(double v) { std::lock_guard<std::mutex> lk(g->mtx); g->volume = v; }
DJX_API void djx_set_squelch_db(double db) { std::lock_guard<std::mutex> lk(g->mtx); g->sql_db = db; }

DJX_API int djx_process(const float *iq, int n)
{
    {   // back-pressure first, without g->mtx: channelize() would otherwise wait for the simulator while holding it,
        // and every GUI call that needs g->mtx (spectrum, retune, volume) stalled with it
        std::unique_lock<std::mutex> ql(g->qmtx);
        g->qcv.wait(ql, [] { return g->q.size() < 15 || g->stop || g->backend != 1; });
    }
    std::lock_guard<std::mutex> lk(g->mtx);
    if (g->radio) wide_floor(g, iq, n);
    g->iq_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (g->radio) {                                  // the radio switched between its receivers (main / BK1068)
        int m = g->want_wfm ? MODE_WFM : (g->mode == MODE_WFM ? MODE_FM : g->mode);
        if (m != g->mode) { g->mode = m; design_decimator(g); g->rf_buf.clear(); }
    }
    channelize(g, iq, n);
    return (int)g->audio.size();
}

DJX_API int djx_read_audio(int16_t *out, int max)
{
    std::lock_guard<std::mutex> lk(g->omtx);
    int n = 0;
    while (n < max && !g->audio.empty()) { out[n++] = g->audio.front(); g->audio.pop_front(); }
    return n;
}

DJX_API void djx_status(Status *out)
{
    // g->ts is published by the simulator thread after every 10 ms step
    std::lock_guard<std::mutex> lk(g->omtx);
    Status &st = g->s;
    st.mode = g->mode;
    st.level_db = 10 * std::log10(g->level + 1e-20);
    st.backend = g->backend;
    if (g->backend == 1) {
        const tsim_stats &t = g->ts;
        st.frames_to_ambe = t.ambe_channel_pkts;
        st.speech_packets = t.ambe_channel_pkts - t.ambe_bad;
        st.dsp_load = t.load_pct; st.dsp_load_peak = t.load_peak_pct;
        st.mips = g->mips;
        st.uart_tx = t.uart_tx_bytes; st.uart_rx = t.uart_rx_bytes; st.i2c_lines = t.i2c_lines_rx; st.halted = (unsigned)t.halted;
        std::lock_guard<std::mutex> ql(g->qmtx);
        st.fw_queue = (int)g->q.size();
    }
    *out = st;
}

// DSP alone: load a DSP firmware image (+ its dis55 listing; map only for a modular overlay image) on the C55x
// simulator. The caller plays the CPU (djx_cpu_send / djx_cpu_line).
// Returns nullptr or an error text.
DJX_API const char *djx_load_firmware(const char *bin, const char *dis, const char *map)
{
    static std::string err;
    {
        std::lock_guard<std::mutex> lk(g->fwmtx);
        const char *e = tsim_init(bin, dis, map);
        if (e) { err = e; g->backend = 0; return err.c_str(); }
        g->stock = tsim_is_stock() != 0;
        g->if_invert = g->stock;
        g->mips_insns = 0;
        g->mips_t = std::chrono::steady_clock::now();
    }
    {
        std::lock_guard<std::mutex> ql(g->qmtx);
        g->q.clear();
    }
    if (!g->worker.joinable()) g->worker = std::thread(worker_main);
    std::lock_guard<std::mutex> lk(g->mtx);
    g->backend = 1;
    return nullptr;
}

DJX_API const char *djx_fw_error(void)
{
    std::lock_guard<std::mutex> lk(g->fwmtx);
    return tsim_last_error();
}

DJX_API int djx_status_size(void) { return (int)sizeof(Status); }

DJX_API int djx_event(char *buf, int max)
{
    std::lock_guard<std::mutex> lk(g->omtx);
    if (g->events.empty()) return 0;
    std::string e = g->events.front();
    g->events.pop_front();
    std::strncpy(buf, e.c_str(), (size_t)max - 1);
    buf[max - 1] = 0;
    return 1;
}

DJX_API int djx_cpu_send(const char *cmd)
{
    if (g->backend == 1) { std::lock_guard<std::mutex> lk(g->fwmtx); return tsim_cpu_send(cmd); }
    return 0;
}

DJX_API int djx_cpu_line(char *buf, int max)
{
    if (g->backend == 1) { std::lock_guard<std::mutex> lk(g->fwmtx); return tsim_cpu_line(buf, max); }
    return 0;
}

DJX_API int djx_channel_iq(float *pre, float *post, int n)   // latest n fs1 samples (oldest first), interleaved
{
    std::lock_guard<std::mutex> lk(g->mtx);
    if (n > 8192) n = 8192;
    for (int i = 0; i < n; i++) {
        size_t k = (g->ring_pos + 8192 - (size_t)n + (size_t)i) & 8191;
        pre[2 * i] = (float)g->ring_pre[k].real(); pre[2 * i + 1] = (float)g->ring_pre[k].imag();
        post[2 * i] = (float)g->ring_post[k].real(); post[2 * i + 1] = (float)g->ring_post[k].imag();
    }
    return n;
}

DJX_API int djx_fw_is_stock(void) { return g && g->stock ? 1 : 0; }

// I2C transfers of the DSP to other devices than the CPU (the WM8731 codec at 0x1A)
DJX_API int djx_dev_line(char *buf, int max)
{
    // not under fwmtx: the simulator thread holds that for whole 10 ms steps (the GUI stalled up to 2 s here)
    std::lock_guard<std::mutex> lk(g->omtx);
    if (g->dev_q.empty()) return 0;
    std::strncpy(buf, g->dev_q.front().c_str(), (size_t)max - 1);
    buf[max - 1] = 0;
    g->dev_q.pop_front();
    return 1;
}

DJX_API void djx_set_if_invert(int on) { std::lock_guard<std::mutex> lk(g->mtx); g->if_invert = on != 0; }
DJX_API int djx_get_if_invert(void) { return g && g->if_invert ? 1 : 0; }

// ---- whole radio (stock CPU firmware + stock DSP firmware) ------------------------------------------------
// cpu_flash: cpu_flash_FFE00000-FFFFFFFF.bin, cpu_option: cpu_option_FE7F5D00.bin (may be ""), eeprom: 256 KiB
// image (read, and written back by djx_radio_save), dsp_bin / dsp_dis: DigRxx107.bin and its listing.
DJX_API const char *djx_radio_load(const char *cpu_flash, const char *cpu_option, const char *eeprom,
                                   const char *dsp_bin, const char *dsp_dis)
{
    static std::string err;
    {
        std::lock_guard<std::mutex> lk(g->fwmtx);
        const char *e = tsim_init(dsp_bin, dsp_dis, nullptr);   // checks the files; the CPU boots it later
        if (e) { err = e; return err.c_str(); }
        e = rxcpu_init(cpu_flash, cpu_option, eeprom);
        if (e) { err = e; return err.c_str(); }
        rxcpu_set_dsp_link(radio_to_dsp, radio_dsp_ready);
        g->dsp_bin = dsp_bin;
        g->dsp_dis = dsp_dis;
        g->dsp_run = false;
        g->stock = true;
        g->if_invert = true;
        g->radio_err.clear();
        g->mips_insns = 0;
        g->mips_t = std::chrono::steady_clock::now();
        g->radio = true;
    }
    {
        std::lock_guard<std::mutex> ql(g->qmtx);
        g->q.clear();
    }
    if (!g->worker.joinable()) g->worker = std::thread(worker_main);
    std::lock_guard<std::mutex> lk(g->mtx);
    g->backend = 1;
    return nullptr;
}
DJX_API int djx_radio_active(void) { return g && g->radio ? 1 : 0; }
DJX_API unsigned djx_radio_lcd(uint32_t *rgba)            // 320 x 240 0xAARRGGBB; returns the frame counter
{
    std::lock_guard<std::mutex> lk(g->lcdmtx);
    memcpy(rgba, g->lcd.data(), g->lcd.size() * 4);
    return g->lcd_frames;
}
// key mask (bit = key code pressed). Every change is queued and held until the firmware's debounced key scan has
// seen it (radio_step), so a quick click is never lost.
DJX_API void djx_radio_keys(uint32_t mask)
{
    std::lock_guard<std::mutex> lk(g->kmtx);
    uint32_t last = g->kq.empty() ? g->kcur : g->kq.back();
    if (mask != last) g->kq.push_back(mask);
}
DJX_API void djx_radio_encoder(int which, int steps) { g->enc_add[which & 1] += steps; }
DJX_API uint32_t djx_radio_tuned(uint32_t *count) { if (count) *count = g->tuned_n; return g->tuned_hz; }
DJX_API double djx_radio_time(void) { return g->radio_t; }
DJX_API void djx_radio_set_cpi(int cpi) { g->cpi_req = cpi; }
DJX_API void djx_radio_save(void) { std::lock_guard<std::mutex> lk(g->fwmtx); rxcpu_save(); }
// RF model: measured antenna level (dBm) / SNR (dB) of the tuned channel, AF amplifier state and volume DAC
DJX_API void djx_radio_rf(double *dbm, double *snr_db, int *af_on, int *af_vol)
{
    if (dbm) *dbm = g->rf_quiet ? g->rf_floor_dbm + g->rf_gain_db : g->rf_dbm.load();
    if (snr_db) *snr_db = g->rf_quiet ? -10.0 : g->rf_snr.load();
    if (af_on) *af_on = g->af_on;
    if (af_vol) *af_vol = g->af_vol;
}
DJX_API void djx_radio_set_rf_gain(double db) { g->rf_gain_db = db; }
// PC link (the radio's USB serial port, high level): bytes to / from the CPU firmware's PC command interpreter
DJX_API void djx_radio_usb_write(const uint8_t *d, int n) { rxcpu_usb_write(d, n); }
DJX_API int djx_radio_usb_read(uint8_t *d, int max) { return rxcpu_usb_read(d, max); }
DJX_API int djx_radio_wfm(void) { return g && g->radio && g->want_wfm ? 1 : 0; }   // BK1068 broadcast FM in use
DJX_API const char *djx_radio_error(void) { return g ? g->radio_err.c_str() : ""; }
