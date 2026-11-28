// djxboard.cpp - see djxboard.h
#include "djxboard.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "c55sim.h"
extern "C" {
#include "blip25.h"
}
#include "dstar.h"

namespace {

const double SYSCLK = 140.76e6;
const uint64_t TICK = 1407600;                 // 10 ms
enum { VEC_TINT = 4, VEC_UART = 6, VEC_DMA = 8, VEC_I2C = 23 };

std::string read_file(const char *p, bool &ok)
{
    std::ifstream f(p, std::ios::binary);
    ok = (bool)f;
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

struct Board : c55::Bus {
    c55::Cpu cpu;
    uint16_t io[0x10000];
    std::map<std::string, uint32_t> sym;
    std::string err;
    uint64_t tick_start = 0;
    bool first_tick = true;
    int adc_half = 0, audio_half = 0;
    double load = 0, load_peak = 0;
    uint64_t busy_mark = 0;

    // ---- UART ----
    std::deque<uint8_t> tx_fifo, rx_fifo, rx_line;   // rx_line: bytes the AMBE is sending (not yet on the wire)
    uint64_t tx_next = 0, rx_next = 0;               // completion time of the byte on the wire
    bool tx_shift = false;
    uint8_t tx_shift_byte = 0;
    unsigned uart_tx_bytes = 0, uart_rx_bytes = 0, uart_rx_over = 0, uart_tx_full = 0;
    bool uart_oe = false;
    uint64_t byte_cycles() const
    {
        uint32_t div = (uint32_t)io[0x1B10] | ((uint32_t)io[0x1B12] << 8);
        if (!div) div = 1;
        return 10ull * 16 * div;                      // 8N1 = 10 bit times
    }

    // ---- AMBE-3000R ----
    uint8_t amb_pkt[400];
    int amb_len = 0, amb_need = 0;
    blip25_decoder amb_dec;
    dstar_decoder *amb_dstar = dstar_new();             // D-STAR（AMBE-2000 互換）は ambe2-c に無いので mbelib
    ~Board() { dstar_free(amb_dstar); }
    unsigned amb_rate = 0, amb_channel = 0, amb_bad = 0;
    bool amb_ready_sent = false;
    int amb_log_n = 0;

    // ---- I2C ----
    uint16_t i2c_pending = 0;                         // bit n: INTCODE n pending
    std::deque<std::string> cpu_tx;                   // commands waiting for the bus
    std::string cur_cmd;
    size_t cur_pos = 0;
    int i2c_state = 0;                                // 0 idle, 1 slave rx (CPU writing), 2 master tx (DSP writing)
    uint64_t i2c_next = 0;
    bool rrdy = false, xrdy = false, dxr_full = false;
    uint16_t dxr = 0;
    unsigned m_count = 0;
    std::string cpu_rx_cur;
    std::deque<std::string> cpu_lines;
    unsigned i2c_lines_rx = 0, i2c_cmds = 0;
    uint16_t m_sar = 0x7F;                            // slave address of the current master transfer
    std::vector<uint8_t> dev_cur;                     // bytes of the current transfer to another device
    std::deque<std::string> dev_log;                  // "aa: xx xx ..." per transfer (codec register writes)
    std::map<int, int> codec_regs;                    // 2-byte writes to 0x1A: reg (7 bit) -> value (9 bit)
    // I2C bit time from the firmware's own clock setup (ICCLKL 0x1A0C, ICCLKH 0x1A10, ICPSC 0x1A30):
    // SCL = SYSCLK / ((IPSC + 1) (ICCL + d + ICCH + d)), d = 7 / 6 / 5 for IPSC = 0 / 1 / more. DigRxx107 sets
    // 0x3B / 0x3B / 0 -> ~950 kHz. TSIM_I2C100K=1: the old fixed 100 kHz.
    uint64_t i2c_byte_cycles() const
    {
        static const bool slow = getenv("TSIM_I2C100K") != nullptr;
        unsigned l = io[0x1A0C], h = io[0x1A10], p = io[0x1A30] & 0xFF;
        if (slow || !l || !h) return (uint64_t)(9 * SYSCLK / 100000.0);
        unsigned d = p == 0 ? 7 : p == 1 ? 6 : 5;
        return (uint64_t)9 * (p + 1) * (l + h + 2 * d);
    }

    bool stock = false;
    // modular overlay firmware: SPI boot flash with the overlays, code bank switch when one is loaded, 48 kHz audio
    bool dspx = false;
    std::vector<uint8_t> flash;
    std::vector<std::string> ovl_dis;                 // listing per directory index
    uint32_t a_ovl_hook = 0;
    unsigned ovl_loads = 0;
    uint32_t spi_pos = 0, spi_len = 0, spi_addr = 0;
    uint16_t spi_cmd = 0, spi_rx = 0;
    void spi_char(uint16_t cmd2)
    {
        if (spi_pos == 0) { spi_len = (io[0x3004] & 0x0FFF) + 1u; spi_cmd = 0; spi_addr = 0; }
        uint16_t out = (uint16_t)(io[0x3009] >> 8);
        if ((cmd2 & 3) == 2) {                          // write a character
            if (spi_pos == 0) spi_cmd = out;
            else if (spi_pos < 4) spi_addr = (spi_addr << 8) | out;
            spi_rx = 0xFF;
        } else {                                        // read a character
            spi_rx = (spi_cmd == 0x03 && spi_pos >= 4 && !flash.empty()) ? flash[spi_addr++ % flash.size()] : 0xFF;
        }
        if (++spi_pos >= spi_len) spi_pos = 0;          // chip select released after FLEN + 1 characters
    }
    uint32_t iolog_lo = 0, iolog_hi = 0;
    bool io_stats = false;
    uint64_t n_slices = 0;
    int16_t audio_out[960];
    int audio_n = 0;                               // stock DigRxx image: no symbols, board-level observation only
    std::map<uint16_t, unsigned> io_rd_seen, io_wr_seen;

    // ---- general-purpose timers GPT0..2 (0x1810 / 0x1850 / 0x1890): 32-bit down counters, SYSCLK / 2^(PSCDIV+1),
    //      TIAFR 0x1C14 (bit n = timer n expired), all three share the TINT interrupt
    struct Tmr { bool run = false; uint64_t t0 = 0; uint64_t prd = 0; uint32_t div = 2; bool reload = false; } tmr[3];
    static int tmr_of(uint32_t a) { return a >= 0x1810 && a <= 0x1815 ? 0 : a >= 0x1850 && a <= 0x1855 ? 1 : a >= 0x1890 && a <= 0x1895 ? 2 : -1; }
    static uint32_t tmr_base(int n) { return 0x1810u + 0x40u * (uint32_t)n; }
    uint32_t tmr_count(int n, uint64_t now)
    {
        Tmr &t = tmr[n];
        uint32_t b = tmr_base(n);
        if (!t.run) return ((uint32_t)io[b + 5] << 16) | io[b + 4];
        uint64_t ticks = (now - t.t0) / t.div;
        if (ticks > t.prd) return 0;
        return (uint32_t)(t.prd - ticks);
    }
    void tmr_tcr(int n, uint16_t v, uint64_t now)
    {
        Tmr &t = tmr[n];
        uint32_t b = tmr_base(n);
        bool was = t.run;
        if (was && !(v & 1)) {                            // stop: freeze the count
            uint32_t c = tmr_count(n, now);
            io[b + 4] = (uint16_t)c; io[b + 5] = (uint16_t)(c >> 16);
            t.run = false;
        }
        io[b] = v;
        t.div = 2u << ((v >> 2) & 15);
        t.reload = (v & 2) != 0;
        if (!was && (v & 1) && (v & 0x8000)) {
            t.run = true;
            t.prd = ((uint64_t)io[b + 3] << 16) | io[b + 2];
            t.t0 = now;
        }
    }
    uint64_t tmr_expiry(int n) const { const Tmr &t = tmr[n]; return t.run ? t.t0 + (t.prd + 1) * t.div : UINT64_MAX; }
    void tmr_service(uint64_t now)
    {
        for (int n = 0; n < 3; n++) {
            Tmr &t = tmr[n];
            while (t.run && now >= tmr_expiry(n)) {
                io[0x1C14] |= (uint16_t)(1u << n);
                cpu.raise(VEC_TINT);
                if (t.reload) { t.t0 = tmr_expiry(n); t.prd = ((uint64_t)io[tmr_base(n) + 3] << 16) | io[tmr_base(n) + 2]; }
                else { t.run = false; io[tmr_base(n)] &= (uint16_t)~1u; io[tmr_base(n) + 4] = io[tmr_base(n) + 5] = 0; }
            }
        }
    }
    // ---- DMA, software-started (memory to memory) transfers
    struct DmaJob { uint64_t done; int ctl, ch; };
    std::vector<DmaJob> dma_jobs;
    static bool dma_tcru(uint32_t a) { return a >= 0x0C00 && a < 0x1000 && (a & 0x1F) == 5 && (a & 0xFF) < 0x80; }
    void dma_start(uint32_t a, uint16_t v)
    {
        int ctl = (int)((a - 0x0C00) >> 8), ch = (int)((a & 0xFF) >> 5);
        uint32_t b = 0x0C00u + (uint32_t)ctl * 0x100u + (uint32_t)ch * 0x20u;
        uint32_t src = ((uint32_t)io[b + 1] << 16) | io[b], dst = ((uint32_t)io[b + 3] << 16) | io[b + 2];
        uint32_t bytes = io[b + 4];
        bool src_io = src < 0x10000, dst_io = dst < 0x10000;
        uint32_t n = bytes / 2;
        uint32_t sw = dma_word(io[b], io[b + 1]), dw = dma_word(io[b + 2], io[b + 3]);
        for (uint32_t i = 0; i < n; i++) {
            uint16_t w = src_io ? io_rd(src >> 1) : cpu.mem[(sw + i) % c55::Cpu::MEM_WORDS];
            if (dst_io) io_wr(dst >> 1, w); else cpu.mem[(dw + i) % c55::Cpu::MEM_WORDS] = w;
        }
        dma_jobs.push_back({cpu.cycles + 40 + n, ctl, ch});
        (void)v;
    }
    void dma_service(uint64_t now)
    {
        for (size_t i = 0; i < dma_jobs.size();) {
            if (now < dma_jobs[i].done) { i++; continue; }
            DmaJob j = dma_jobs[i];
            dma_jobs.erase(dma_jobs.begin() + (long)i);
            uint32_t t = 0x0C00u + (uint32_t)j.ctl * 0x100u + (uint32_t)j.ch * 0x20u + 5;
            uint16_t tcru = io[t];
            if (!(tcru & 0x1000)) io[t] = tcru & (uint16_t)~0x8000;       // not auto-reload: channel stops
            int bit = j.ctl * 4 + j.ch;
            if (tcru & 0x2000) {
                io[0x1C30] |= (uint16_t)(1u << bit);
                if (io[0x1C31] & (1u << bit)) cpu.raise(VEC_DMA);
            }
        }
    }
    uint64_t dma_next_event() const
    {
        uint64_t t = UINT64_MAX;
        for (auto &j : dma_jobs) t = std::min(t, j.done);
        return t;
    }
    uint64_t tmr_next_event() const
    {
        uint64_t t = UINT64_MAX;
        for (int n = 0; n < 3; n++) t = std::min(t, tmr_expiry(n));
        return t;
    }
    Board() { memset(io, 0, sizeof io); }

    uint32_t S(const char *n) { auto it = sym.find(n); return it == sym.end() ? 0 : it->second; }

    // ------------------------------------------------------------------ I/O space
    uint16_t io_rd(uint32_t a) override
    {
        a &= 0xFFFF;
        if (io_stats) io_rd_seen[(uint16_t)a]++;
        if (iolog_lo && a >= iolog_lo && a <= iolog_hi) { uint16_t v_ = io_rd2(a); fprintf(stderr, "%.6f R %04X -> %04X pc %06X\n", cpu.cycles / SYSCLK, a, v_, cpu.pc); return v_; }
        return io_rd2(a);
    }
    uint16_t io_rd2(uint32_t a)
    {
        switch (a) {
        case 0x1B00: {                                       // RBR
            if (io[0x1B06] & 0x80) return io[0x1B10];       // DLAB: DLL
            if (rx_fifo.empty()) return 0;
            uint8_t b = rx_fifo.front();
            rx_fifo.pop_front();
            if (!rx_fifo.empty() && (io[0x1B02] & 1)) cpu.raise(VEC_UART);   // level: still data available
            return b;
        }
        case 0x1B04: {                                       // IIR: RDA > THRE; reading THRE clears it
            bool rx = !rx_fifo.empty() && (io[0x1B02] & 1);
            uint16_t ff = (io[0x1B04] & 1) ? 0xC0 : 0x00;       // FIFOs enabled (FCR bit 0)
            if (rx) return ff | 0x04;
            if (thre_pending && (io[0x1B02] & 2)) { thre_pending = false; return ff | 0x02; }
            return ff | 0x01;
        }
        case 0x1B0A: {                                       // LSR
            uint16_t v = 0;
            if (!rx_fifo.empty()) v |= 0x01;
            if (uart_oe) { v |= 0x02; uart_oe = false; }
            if (tx_fifo.empty()) v |= 0x20;
            if (tx_fifo.empty() && !tx_shift) v |= 0x40;
            return v;
        }
        case 0x1A08: {                                       // ICSTR
            uint16_t v = io[0x1A08] & ~(0x0008 | 0x0010 | 0x1000);
            if (rrdy) v |= 0x0008;
            if (xrdy) v |= 0x0010;
            if (i2c_state) v |= 0x1000;
            return v;
        }
        case 0x1A18: rrdy = false; return io[0x1A18];         // ICDRR
        case 0x1A28: {                                       // ICIVR: highest priority pending code, cleared on read
            for (int c = 1; c <= 7; c++)
                if (i2c_pending & (1u << c)) {
                    i2c_pending &= (uint16_t)~(1u << c);
                    if (i2c_pending & i2c_enabled_mask()) cpu.raise(VEC_I2C);
                    return (uint16_t)c;
                }
            return 0;
        }
        case 0x1C08: return io[0x1C0A] & io[0x1C06];         // GPIO in: outputs read back
        case 0x1C09: return io[0x1C0B] & io[0x1C07];
        case 0x1C22: return (uint16_t)(io[0x1C22] | 0x0001); // PLL: report lock
        case 0x1C3A: return (uint16_t)(io[0x1C3A] | ((io[0x1C3A] & 0x0555) << 1));   // CLKSTOP1: request -> ack
        case 0x1C3B: return (uint16_t)(io[0x1C3B] | ((io[0x1C3B] & 0x0049) << 1));   // CLKSTOP2
        case 0x1814: case 0x1854: case 0x1894: return (uint16_t)tmr_count(tmr_of(a), cpu.cycles);
        case 0x1815: case 0x1855: case 0x1895: return (uint16_t)(tmr_count(tmr_of(a), cpu.cycles) >> 16);
        case 0x3006: return 0x0002;                          // SPI STAT1: character complete, not busy (instant)
        case 0x3008: return spi_rx;                          // SPI DR1: received character
        default: return io[a];
        }
    }

    void io_wr(uint32_t a, uint16_t v) override
    {
        a &= 0xFFFF;
        if (io_stats) io_wr_seen[(uint16_t)a]++;
        if (iolog_lo && a >= iolog_lo && a <= iolog_hi) fprintf(stderr, "%.6f W %04X <- %04X pc %06X\n", cpu.cycles / SYSCLK, a, v, cpu.pc);
        // TSIM_GPIOTRACE=mask: on a GPIO1 output write that changes a bit in mask, print the call path (CALL/RET
        // instructions among the last 64k executed addresses; needs TSIM_RING)
        if (a == 0x1C0A && cpu.ring && getenv("TSIM_GPIOTRACE") && ((io[0x1C0A] ^ v) & (uint16_t)strtoul(getenv("TSIM_GPIOTRACE"), 0, 16))) {
            fprintf(stderr, "%.6f GPIO1 %04X -> %04X, call path:\n", cpu.cycles / SYSCLK, io[0x1C0A], v);
            int shown = 0;
            for (uint32_t i = cpu.ring_i - 1; i != cpu.ring_i - 65535 && shown < 60; i--) {
                uint32_t pa = cpu.ring[i & 65535];
                c55::Insn *q = cpu.insn_at(pa);
                if (q && (q->text.find("CALL") != std::string::npos || q->text.find("RET") != std::string::npos ||
                          (q->par && q->par->text.find("CALL") != std::string::npos))) {
                    fprintf(stderr, "    %06X %s%s%s\n", pa, q->text.c_str(), q->par ? " || " : "", q->par ? q->par->text.c_str() : "");
                    shown++;
                }
            }
        }
        switch (a) {
        case 0x1B00:                                         // THR (or DLL with DLAB)
            if (io[0x1B06] & 0x80) { io[0x1B10] = v; return; }
            if (tx_fifo.size() >= 16) { uart_tx_full++; return; }
            tx_fifo.push_back((uint8_t)v);
            thre_pending = false;
            if (!tx_shift) start_tx(cpu.cycles);
            return;
        case 0x1B04:                                         // FCR
            if (v & 2) rx_fifo.clear();
            if (v & 4) tx_fifo.clear();
            io[0x1B04] = v;
            return;
        case 0x1B02:                                         // IER (or DLM with DLAB)
            if ((v & 2) && !(io[0x1B02] & 2) && tx_fifo.empty()) { thre_pending = true; cpu.raise(VEC_UART); }
            io[0x1B02] = v;
            return;
        case 0x1C30: io[0x1C30] &= (uint16_t)~v; return;     // DMAIFR: write 1 to clear
        case 0x1C14: io[0x1C14] &= (uint16_t)~v; return;     // TIAFR: write 1 to clear
        case 0x1810: case 0x1850: case 0x1890: tmr_tcr(tmr_of(a), v, cpu.cycles); return;
        case 0x0C05: case 0x0C25: case 0x0C45: case 0x0C65: case 0x0D05: case 0x0D25: case 0x0D45: case 0x0D65:
        case 0x0E05: case 0x0E25: case 0x0E45: case 0x0E65: case 0x0F05: case 0x0F25: case 0x0F45: case 0x0F65: {
            bool was = io[a] & 0x8000;
            io[a] = v;
            if (stock && (v & 0x8000) && !was && !(v & 0x0004)) dma_start(a, v);   // software-started transfer
            return;
        }
        case 0x1A20: dxr = v; dxr_full = true; xrdy = false; io[0x1A20] = v; return;   // ICDXR
        case 0x3005: io[0x3005] = v; if (v & 3) spi_char(v); return;   // SPI CMD2: one character
        case 0x1A24:                                         // ICMDR
            io[0x1A24] = v;
            if (!(v & 0x0020)) { i2c_state = 0; i2c_pending = 0; rrdy = xrdy = dxr_full = false; return; }
            if ((v & 0x2400) == 0x2400 && (v & 0x0200)) {    // STT + MST + TRX: master write
                if (i2c_state == 1) { set_code(1); io[0x1A24] &= (uint16_t)~0x2400; return; }   // arbitration lost
                i2c_state = 2;
                m_count = 0;
                m_sar = io[0x1A1C] & 0x3FF;
                dev_cur.clear();
                if (m_sar == 0x7F) cpu_rx_cur.clear();
                xrdy = !dxr_full;
                if (xrdy) set_code(5);
                i2c_next = cpu.cycles + i2c_byte_cycles();     // address byte
                io[0x1A24] &= (uint16_t)~0x2000;
            }
            return;
        default:
            io[a] = v;
        }
    }

    uint16_t i2c_enabled_mask() const
    {
        // ICIMR bits: 0 AL 1 NACK 2 ARDY 3 RRDY 4 XRDY 5 SCD 6 AAS -> INTCODEs 1..7
        return (uint16_t)((io[0x1A04] & 0x7F) << 1);
    }
    void set_code(int c)
    {
        i2c_pending |= (uint16_t)(1u << c);
        if (i2c_pending & i2c_enabled_mask() & (1u << c)) cpu.raise(VEC_I2C);
    }

    // ------------------------------------------------------------------ UART timing
    bool thre_pending = false;
    void start_tx(uint64_t now)
    {
        if (tx_fifo.empty()) { tx_shift = false; return; }
        tx_shift_byte = tx_fifo.front();
        tx_fifo.pop_front();
        tx_shift = true;
        tx_next = now + byte_cycles();
        if (tx_fifo.empty() && (io[0x1B02] & 2)) { thre_pending = true; cpu.raise(VEC_UART); }
    }

    void uart_service(uint64_t now)
    {
        while (tx_shift && now >= tx_next) {
            amb_rx(tx_shift_byte);
            uart_tx_bytes++;
            uint64_t t = tx_next;
            start_tx(t);
        }
        if (rx_next == 0 || rx_next < now - byte_cycles()) rx_next = now;
        while (!rx_line.empty() && now >= rx_next) {
            if (rx_fifo.size() >= 16) { uart_rx_over++; uart_oe = true; }
            else rx_fifo.push_back(rx_line.front());
            rx_line.pop_front();
            uart_rx_bytes++;
            rx_next += byte_cycles();
            if (io[0x1B02] & 1) cpu.raise(VEC_UART);
        }
    }
    uint64_t uart_next_event(uint64_t now) const
    {
        uint64_t t = UINT64_MAX;
        if (tx_shift) t = tx_next;
        if (!rx_line.empty()) t = std::min(t, std::max(rx_next, now));
        return t;
    }

    // ------------------------------------------------------------------ AMBE-3000R (ambe2-c)
    void amb_out(uint8_t b) { rx_line.push_back(b); }
    void amb_rx(uint8_t b)
    {
        if (amb_len == 0 && b != 0x61) { amb_bad++; return; }
        amb_pkt[amb_len++] = b;
        if (amb_len == 3) amb_need = 4 + ((amb_pkt[1] << 8) | amb_pkt[2]);
        else if (amb_len > 3 && amb_len == amb_need) { amb_packet(); amb_len = 0; }
        if (amb_len >= (int)sizeof amb_pkt) amb_len = 0;
    }
    uint16_t amb_rcw[6] = {0};
    unsigned amb_decoded = 0, amb_last_bits = 0;
    int amb_codec = -1;
    FILE *amb_dump = getenv("TSIM_AMBEDUMP") ? fopen(getenv("TSIM_AMBEDUMP"), "wb") : nullptr;                               // decoder chosen from RATET / RATEP: 17 26 33 34 0
    void amb_ctrl_reply(const std::vector<uint8_t> &r, uint8_t type)
    {
        amb_out(0x61); amb_out((uint8_t)(r.size() >> 8)); amb_out((uint8_t)r.size()); amb_out(type);
        for (uint8_t b : r) amb_out(b);
    }
    void amb_set_ratep()
    {
        uint16_t *w = amb_rcw;
        if (w[0] == 0x0431 && w[1] == 0x0754) amb_codec = (w[2] == 0 && w[3] == 0) ? 34 : 33;   // AMBE+2 2450
        else if ((w[0] & 0xFF00) == 0x0100 || w[0] == 0x0130) amb_codec = 0;                    // AMBE 2400 (D-STAR)
        else amb_codec = -1;
        amb_rate = 0x100 | (unsigned)(amb_codec & 0xFF);
    }
    void amb_set_ratet(int t)
    {
        amb_rate = (unsigned)t;
        amb_codec = (t == 33 || t == 34 || t == 17 || t == 26 || t == 0 || t == 22) ? t : -1;
    }
    void amb_speech(const int16_t *pcm)
    {
        amb_out(0x61); amb_out(0x01); amb_out(0x42); amb_out(0x02); amb_out(0x00); amb_out(0xA0);
        for (int i = 0; i < 160; i++) { amb_out((uint8_t)((uint16_t)pcm[i] >> 8)); amb_out((uint8_t)pcm[i]); }
    }
    void amb_packet()
    {
        const char *amb_log = getenv("TSIM_AMBELOG");       // "2": every packet, all bytes
        bool amb_full = amb_log && amb_log[0] == '2';
        if (amb_log && (amb_pkt[3] != 0x01 || amb_full || amb_log_n++ < 20)) {
            fprintf(stderr, "%.4f AMBE<-DSP", cpu.cycles / SYSCLK);
            for (int i = 0; i < amb_len && (amb_full || i < 40); i++) fprintf(stderr, " %02X", amb_pkt[i]);
            fprintf(stderr, "\n");
        }
        int len = (amb_pkt[1] << 8) | amb_pkt[2], type = amb_pkt[3];
        const uint8_t *f = &amb_pkt[4], *end = &amb_pkt[4 + len];
        if (type == 0x00) {                                   // control: answer every field
            std::vector<uint8_t> r;
            while (f < end) {
                uint8_t id = *f++;
                static const struct { uint8_t id, n; } fl[] = {{0x05, 2}, {0x06, 2}, {0x07, 1}, {0x09, 1}, {0x0A, 12},
                    {0x0B, 1}, {0x10, 1}, {0x2A, 1}, {0x2B, 0}, {0x2F, 1}, {0x30, 0}, {0x31, 0}, {0x33, 6}, {0x34, 0},
                    {0x3F, 1}, {0x44, 2}, {0x46, 5}, {0x49, 2}, {0x4B, 2}, {0x4C, 1}};
                int n = -1;
                for (auto &e : fl) if (e.id == id) n = e.n;
                if (n < 0 || f + n > end) { r.push_back(id); r.push_back(0); break; }
                if (id == 0x09) amb_set_ratet(f[0]);
                if (id == 0x0A) { for (int k = 0; k < 6; k++) amb_rcw[k] = (uint16_t)((f[2 * k] << 8) | f[2 * k + 1]); amb_set_ratep(); }
                if (id == 0x0B) { blip25_init(&amb_dec); dstar_reset(amb_dstar); }
                if (id == 0x31 || id == 0x30) {
                    static const char v[] = "V120.E100.XXXX.C106.G514.R009.B0010411.C0020208", pid[] = "AMBE3000R";
                    const char *t = id == 0x31 ? v : pid;
                    r.push_back(id);
                    for (const char *c = t;; c++) { r.push_back((uint8_t)*c); if (!*c) break; }
                } else if (id != 0x2F) {
                    r.push_back(id); r.push_back(0);
                }
                f += n;
            }
            amb_ctrl_reply(r, 0x00);
            return;
        }
        if (type != 0x01) { amb_bad++; return; }
        // channel packet: optional rate fields, then CHAND (hard bits) or CHAND4 (4-bit soft decisions)
        while (f < end) {
            uint8_t id = *f++;
            if (id == 0x09 && f < end) { amb_set_ratet(*f++); continue; }
            if (id == 0x0A && f + 12 <= end) { for (int k = 0; k < 6; k++) amb_rcw[k] = (uint16_t)((f[2 * k] << 8) | f[2 * k + 1]); amb_set_ratep(); f += 12; continue; }
            if (id == 0x2F) { f++; continue; }
            if (id != 0x01 && id != 0x17) { amb_bad++; return; }
            int nbits = *f++;
            int nbytes = id == 0x01 ? (nbits + 7) / 8 : (nbits + 1) / 2;
            if (f + nbytes > end) { amb_bad++; return; }
            uint8_t hard[32] = {0};
            if (id == 0x01) memcpy(hard, f, (size_t)nbytes);
            else for (int b = 0; b < nbits && b < 255; b++) {                  // soft -> hard
                int nib = (b & 1) ? (f[b >> 1] & 15) : (f[b >> 1] >> 4);
                if (nib >= 8) hard[b >> 3] |= (uint8_t)(0x80 >> (b & 7));
            }
            f += nbytes;
            if (amb_dump) { fputc(nbits, amb_dump); fwrite(hard, 1, (size_t)((nbits + 7) / 8), amb_dump); fflush(amb_dump); }
            amb_channel++;
            amb_last_bits = (unsigned)nbits;
            int16_t pcm[160];
            int c = amb_codec;
            if (c < 0) c = nbits == 49 ? 34 : nbits == 72 ? 33 : nbits == 48 ? 0 : nbits == 80 ? 17 : -1;
            int err = 0;
            switch (c) {
            case 34: blip25_decode_2450_thumbdv(&amb_dec, hard, pcm); break;
            case 33: blip25_decode_3600(&amb_dec, hard, pcm, &err); break;
            case 0:
                if (nbits == 72) {          // D-STAR（RATEP 0x0130 ... 0x0048）: 空中のバイトそのままの 72 ビット
                    int fe = dstar_decode(amb_dstar, hard, pcm);
                    if (amb_log) fprintf(stderr, "%.4f AMBE D-STAR fec %d\n", cpu.cycles / SYSCLK, fe);
                } else {
                    blip25_decode_ambeplus_r0(&amb_dec, hard, pcm);
                }
                break;
            case 17: blip25_decode_ambeplus_r17(&amb_dec, hard, pcm); break;
            case 26: blip25_decode_ambeplus_r26_chand(&amb_dec, hard, nbits, pcm, &err); break;
            case 22: blip25_decode_ambeplus_r22(&amb_dec, hard, pcm, &err); break;
            default: amb_bad++; blip25_decode_lost(&amb_dec, pcm); break;
            }
            amb_decoded++;
            amb_speech(pcm);
            return;
        }
    }

    // ------------------------------------------------------------------ I2C timing
    bool dsp_ready() const { return (io[0x1C06] & (1u << 13)) && (io[0x1C0A] & (1u << 13)); }

    void i2c_service(uint64_t now)
    {
        if (!(io[0x1A24] & 0x0020)) return;                   // module in reset
        if (i2c_state == 0 && (!cpu_tx.empty()) && dsp_ready() && now >= i2c_next) {
            cur_cmd = cpu_tx.front();
            cpu_tx.pop_front();
            if (getenv("TSIM_I2CLOG")) fprintf(stderr, "DSP %.3f: I2C slave receives '%s' (IMR %04X IER1 %04X)\n", cpu.cycles / SYSCLK, cur_cmd.substr(0, cur_cmd.size() - 2).c_str(), io[0x1A04], cpu.r.ier1);
            cur_pos = 0;
            i2c_state = 1;
            set_code(7);                                       // addressed as slave
            i2c_next = now + i2c_byte_cycles();
        }
        while (i2c_state == 1 && now >= i2c_next) {
            if (rrdy) { i2c_next = now + i2c_byte_cycles() / 9; break; }   // clock stretching until ICDRR is read
            if (cur_pos < cur_cmd.size()) {
                io[0x1A18] = (uint8_t)cur_cmd[cur_pos++];
                rrdy = true;
                set_code(4);
                i2c_next += i2c_byte_cycles();
            } else {
                set_code(6);                                   // stop
                i2c_state = 0;
                i2c_next = now + i2c_byte_cycles();
            }
        }
        while (i2c_state == 2 && now >= i2c_next) {
            if (!dxr_full) {                                   // master waits for data (clock held low)
                if (m_count >= io[0x1A14]) { finish_master(); break; }
                i2c_next = now + i2c_byte_cycles() / 9;
                break;
            }
            dxr_full = false;
            char c = (char)(dxr & 0xFF);
            if (m_sar == 0x7F) cpu_rx(c); else dev_cur.push_back((uint8_t)c);
            m_count++;
            if (m_count >= io[0x1A14]) { finish_master(); break; }
            xrdy = true;
            set_code(5);
            i2c_next += i2c_byte_cycles();
        }
    }
    void finish_master()
    {
        if (m_sar != 0x7F && !dev_cur.empty()) {
            char b[16];
            std::string l;
            snprintf(b, sizeof b, "%02X:", m_sar); l = b;
            for (uint8_t x : dev_cur) { snprintf(b, sizeof b, " %02X", x); l += b; }
            dev_log.push_back(l);
            while (dev_log.size() > 256) dev_log.pop_front();
            if (m_sar == 0x1A && dev_cur.size() == 2) codec_regs[dev_cur[0] >> 1] = ((dev_cur[0] & 1) << 8) | dev_cur[1];
            dev_cur.clear();
        }
        io[0x1A24] &= (uint16_t)~(0x0400 | 0x0200 | 0x0800 | 0x2000);
        xrdy = false;
        i2c_state = 0;
        set_code(6);
        i2c_next = cpu.cycles + i2c_byte_cycles();
    }
    void cpu_rx(char c)
    {
        if (c == '\n') {
            std::string l = cpu_rx_cur;
            if (!l.empty() && l.back() == '\r') l.pop_back();
            cpu_lines.push_back(l);
            while (cpu_lines.size() > 64) cpu_lines.pop_front();
            i2c_lines_rx++;
            cpu_rx_cur.clear();
        } else if (cpu_rx_cur.size() < 200) {
            cpu_rx_cur += c;
        }
    }
    uint64_t i2c_next_event(uint64_t now) const
    {
        if (i2c_state || (!cpu_tx.empty() && dsp_ready())) return std::max(i2c_next, now + 1);
        return UINT64_MAX;
    }

    // ------------------------------------------------------------------ DMA
    uint32_t dma_word(uint16_t lo, uint16_t hi)
    {
        uint32_t b = ((uint32_t)hi << 16) | lo;
        if (b >= 0x80000) return (b - 0x80000) >> 1;
        if (b >= 0x10000) return (b - 0x10000) >> 1;
        return b >> 1;
    }
    void dma_tick(const uint16_t *adc720)
    {
        // DMA3 ch0: McBSP -> adc buffer (ping-pong), 32-bit elements, sample in the odd word
        uint16_t tcru = io[0x0C00 + 0x300 + 5];
        if (tcru & 0x8000) {
            uint32_t base = dma_word(io[0x0F02], io[0x0F03]);
            uint32_t half_words = io[0x0F04] / 4;            // bytes (ping + pong) -> words per half
            for (int i = 0; i < 720 && (uint32_t)(2 * i + 1) < half_words; i++) {
                cpu.mem[(base + adc_half * half_words + 2 * i) % c55::Cpu::MEM_WORDS] = 0;
                cpu.mem[(base + adc_half * half_words + 2 * i + 1) % c55::Cpu::MEM_WORDS] = adc720[i];
            }
            io[0x0F05] = (uint16_t)((tcru & ~0x0002) | (adc_half ? 0x0002 : 0));
            io[0x1C30] |= 1u << 12;
            if (io[0x1C31] & (1u << 12)) cpu.raise(VEC_DMA);
            adc_half ^= 1;
        }
        {                                                     // 48 kHz, ch1 = left, ch3 = right, packed 16-bit
            audio_n = 0;
            uint16_t t1 = io[0x0D25], t3 = io[0x0D65];
            if ((t1 & 0x8000) || (t3 & 0x8000)) {
                uint32_t b1 = dma_word(io[0x0D20], io[0x0D21]), b3 = dma_word(io[0x0D60], io[0x0D61]);
                uint32_t h1 = io[0x0D24] / 4, h3 = io[0x0D64] / 4;   // words per half
                uint32_t n = std::min<uint32_t>(std::max(h1, h3), 960);
                for (uint32_t i = 0; i < n; i++) {
                    int32_t l = (t1 & 0x8000) && i < h1 ? (int16_t)cpu.mem[(b1 + audio_half * h1 + i) % c55::Cpu::MEM_WORDS] : 0;
                    int32_t rr = (t3 & 0x8000) && i < h3 ? (int16_t)cpu.mem[(b3 + audio_half * h3 + i) % c55::Cpu::MEM_WORDS] : 0;
                    // TSIM_AUDIOCH=L / R: one channel only (default: the sum, as the stock image was measured)
                    static const char *ach = getenv("TSIM_AUDIOCH");
                    int32_t m = ach && ach[0] == 'L' ? l : ach && ach[0] == 'R' ? rr : l + rr;
                    audio_out[i] = (int16_t)(m > 32767 ? 32767 : m < -32768 ? -32768 : m);
                }
                audio_n = (int)n;
                io[0x0D25] = (uint16_t)((t1 & ~0x0002) | (audio_half ? 0x0002 : 0));
                io[0x0D65] = (uint16_t)((t3 & ~0x0002) | (audio_half ? 0x0002 : 0));
                io[0x1C30] |= (1u << 5) | (1u << 7);
                if (io[0x1C31] & ((1u << 5) | (1u << 7))) cpu.raise(VEC_DMA);
                audio_half ^= 1;
            }
        }
    }

    // ------------------------------------------------------------------ run one 10 ms step
    void step(const uint16_t *adc720)
    {
        // the DMA period is fixed by the McBSP frame clock, whatever the CPU is doing
        if (stock && !amb_ready_sent && (io[0x1B02] & 1)) {
            amb_ready_sent = true;
            const uint8_t rdy[] = {0x61, 0x00, 0x01, 0x00, 0x39};
            for (uint8_t x : rdy) amb_out(x);
        }
        tick_start = first_tick ? cpu.cycles : tick_start + TICK;
        first_tick = false;
        uint64_t busy0 = cpu.busy_cycles;
        dma_tick(adc720);
        uint64_t end = tick_start + TICK;
        while (!cpu.halted && cpu.cycles < end) {
            uint64_t now = cpu.cycles;
            uart_service(now);
            i2c_service(now);
            tmr_service(now);
            dma_service(now);
            uint64_t nx = std::min(std::min(std::min(end, tmr_next_event()), dma_next_event()),
                                   std::min(uart_next_event(now), i2c_next_event(now)));
            if (nx <= now) nx = now + 1;
            // run in slices of at most 2000 cycles so peripheral timing stays close
            uint64_t lim = std::min(nx, now + 2000);
            n_slices++;
            cpu.run(lim);
            if (cpu.idle && cpu.cycles < lim) cpu.cycles = lim;
            if (cpu.poll_yield) {                             // polling loop with nothing to do: to the next event
                cpu.poll_yield = false;
                if (cpu.cycles < nx) cpu.cycles = nx;
            }
        }
        double l = (double)(cpu.busy_cycles - busy0) / (double)TICK * 100.0;
        {
            // TSIM_LOADLOG=file: busy percentage of every 10 ms tick
            static FILE *ll = getenv("TSIM_LOADLOG") ? fopen(getenv("TSIM_LOADLOG"), "w") : nullptr;
            if (ll) fprintf(ll, "%.1f\n", l);
        }
        load = 0.98 * load + 0.02 * l;
        if (l > load_peak) load_peak = l;
    }

    // ------------------------------------------------------------------ observation
    struct Dump { uint32_t pc, addr, len; int n; };
    std::vector<Dump> dumps;
    static void hook(void *u, c55::Cpu &c, uint32_t addr)
    {
        Board *b = (Board *)u;
        for (auto &d : b->dumps)                   // TSIM_DUMPAT=pc:addr:len,... (debugging): memory words to files
            if (d.pc == addr && d.n < (getenv("TSIM_DUMPN") ? 1000000 : 4)) {
                char fn[64];
                bool app = getenv("TSIM_DUMPN") != nullptr;      // all hits appended to one file
                if (app) snprintf(fn, sizeof fn, "tsim_dump_%06X_%05X.bin", addr, d.addr); else snprintf(fn, sizeof fn, "tsim_dump_%06X_%d.bin", addr, d.n);
                d.n++;
                if (FILE *f = fopen(fn, app ? "ab" : "wb")) {
                    for (uint32_t i = 0; i < d.len; i++) { uint16_t w = c.mem[(d.addr + i) % c55::Cpu::MEM_WORDS]; fwrite(&w, 2, 1, f); }
                    fclose(f);
                }
            }
        if (addr == b->a_ovl_hook && b->dspx) {           // ovl_loaded_hook(idx): image now in memory
            int idx = (int16_t)c.treg(0);
            b->ovl_loads++;
            if (idx >= 0 && idx < (int)b->ovl_dis.size() && !b->ovl_dis[idx].empty()) {
                size_t w0 = c.warnings.size();
                if (!c.load_disassembly(b->ovl_dis[idx])) {
                    fprintf(stderr, "tsim: overlay %d listing: %s\n", idx, c.error.c_str());
                    for (size_t i = w0; i < c.warnings.size(); i++) fprintf(stderr, "    %s\n", c.warnings[i].c_str());
                    c.halted = true;
                }
                for (auto &d : b->dumps) c.set_hook(d.pc, Board::hook, b);   // TSIM_DUMPAT pcs inside the new overlay
            }
        }
    }

    void stats(tsim_stats *s)
    {
        memset(s, 0, sizeof *s);
        if (dspx) {
            s->frames_sent = cpu.rd32(S("_ambe_stats"));
            s->speech_packets = cpu.rd32(S("_ambe_stats") + 2);
            s->audio_underruns = cpu.rd32(S("_audio_underruns"));
            s->adc_overruns = cpu.rd32(S("_fw_adc_overruns"));
        }
        s->load_pct = load;
        s->load_peak_pct = load_peak;
        s->insns = cpu.insns;
        s->cycles = cpu.cycles;
        s->uart_tx_bytes = uart_tx_bytes; s->uart_rx_bytes = uart_rx_bytes;
        s->uart_rx_overruns = uart_rx_over; s->uart_tx_fifo_full = uart_tx_full;
        s->ambe_channel_pkts = amb_channel; s->ambe_bad = amb_bad; s->ambe_rate = amb_rate;
        s->i2c_lines_rx = i2c_lines_rx; s->i2c_cmds_tx = i2c_cmds;
        s->snapshot_pairs = (unsigned)n_slices;
        s->halted = cpu.halted;
    }
};

Board *B;
char errbuf[512];

}  // namespace

extern "C" const char *tsim_init(const char *bin, const char *dis, const char *map)
{
    delete B;
    B = new Board();
    Board &b = *B;
    b.cpu.bus = &b;
    bool ok;
    std::string img = read_file(bin, ok);
    if (!ok) { snprintf(errbuf, sizeof errbuf, "cannot read %s", bin); return errbuf; }
    std::vector<uint8_t> v(img.begin(), img.end());
    if (!b.cpu.load_boot_image(v)) { snprintf(errbuf, sizeof errbuf, "%s: %s", bin, b.cpu.error.c_str()); return errbuf; }
    // modular overlay firmware run without a map (whole radio, radio_test): its map sits next to the core listing
    std::string dspx_map;
    if ((!map || !*map) && dis) {
        std::string dm(dis);
        size_t sl = dm.find_last_of("/\\");
        if (dm.substr(sl == std::string::npos ? 0 : sl + 1) == "dspx_core_r.dis") {
            dspx_map = (sl == std::string::npos ? std::string() : dm.substr(0, sl + 1)) + "dspx.map";
            map = dspx_map.c_str();
        }
    }
    b.stock = !map || !*map;
    b.cpu.tolerant = b.stock;
    std::string d = read_file(dis, ok);
    if (!ok) { snprintf(errbuf, sizeof errbuf, "cannot read %s", dis); return errbuf; }
    if (!b.cpu.load_disassembly(d)) {
        snprintf(errbuf, sizeof errbuf, "%s: %s %s", dis, b.cpu.error.c_str(),
                 b.cpu.warnings.empty() ? "" : b.cpu.warnings[0].c_str());
        return errbuf;
    }
    if (!b.stock) {
        // map: "            0000306c    _adc_dma" (data: word address, code: byte address)
        std::istringstream ms(read_file(map, ok));
        if (!ok) { snprintf(errbuf, sizeof errbuf, "cannot read %s", map); return errbuf; }
        std::string line;
        while (std::getline(ms, line)) {
            std::istringstream ls(line);
            std::string a, n;
            if (ls >> a >> n && a.size() == 8 && n[0] == '_' && a.find_first_not_of("0123456789abcdef") == std::string::npos)
                b.sym[n] = (uint32_t)strtoul(a.c_str(), nullptr, 16);
        }
        b.dspx = b.sym.count("_ovl_loaded_hook") != 0;
        if (!b.dspx) { snprintf(errbuf, sizeof errbuf, "%s: no _ovl_loaded_hook (not a modular overlay image)", map); return errbuf; }
        {
            // modular firmware: the image file is the whole SPI flash; the overlay listings sit next to the core
            // listing, indexed by dspx_ovl.txt ("<directory index> <name> <listing>")
            b.flash = v;
            std::string dm(dis);
            size_t sl = dm.find_last_of("/\\");
            std::string dir = sl == std::string::npos ? std::string() : dm.substr(0, sl + 1);
            std::istringstream ix(read_file((dir + "dspx_ovl.txt").c_str(), ok));
            if (!ok) { snprintf(errbuf, sizeof errbuf, "cannot read %sdspx_ovl.txt", dir.c_str()); return errbuf; }
            int idx;
            std::string nm, fn;
            while (ix >> idx >> nm >> fn) {
                if (idx < 0 || idx > 64) continue;
                if ((int)b.ovl_dis.size() <= idx) b.ovl_dis.resize((size_t)idx + 1);
                b.ovl_dis[(size_t)idx] = read_file((dir + fn).c_str(), ok);
                if (!ok) { snprintf(errbuf, sizeof errbuf, "cannot read %s%s", dir.c_str(), fn.c_str()); return errbuf; }
            }
            b.a_ovl_hook = b.S("_ovl_loaded_hook");
            b.cpu.set_hook(b.a_ovl_hook, Board::hook, &b);
        }
        // a polling main loop `for (;;) fw_step();` never IDLEs - skip its empty passes
        if (b.sym.count("_fw_step") && b.sym.count("_fw_adc_ready") && b.sym.count("_fw_audio_ready") && !getenv("TSIM_NOIDLE")) {
            uint32_t w[2] = {b.S("_fw_adc_ready"), b.S("_fw_audio_ready")};
            b.cpu.set_poll_point(b.S("_fw_step"), w, 2);
        }
}
    if (b.stock) {
        // SYS/BIOS idle task loop of DigRxx107 ("MOV dbl(*(#0739eh)),XAR0 ; CALL idle functions ; B back")
        c55::Insn *q = b.cpu.insn_at(0x04BB2B);
        if (q && q->text.find("0739eh") != std::string::npos && !getenv("TSIM_NOIDLE")) b.cpu.set_idle_point(0x04BB2B);
        // iircas32 section loop of DigRxx107 (verified bit-exact against the interpreter)
        q = b.cpu.insn_at(0x0349D8);
        if (q && q->text.find("SFTS AC0,#-1") != std::string::npos && b.cpu.insn_at(0x034A54) && !getenv("TSIM_NONATIVE"))
            b.cpu.set_native(0x0349D8);
        q = b.cpu.insn_at(0x034987);
        if (q && q->text.find("ST0_55") != std::string::npos && !getenv("TSIM_NONATIVE") && !getenv("TSIM_NONATIVE2"))
            b.cpu.set_native(0x034987);
    }
    if (const char *dd = getenv("TSIM_DUMPAT")) {
        for (const char *q = dd; *q;) {
            unsigned pc, ad, ln; int n = 0;
            if (sscanf(q, "%x:%x:%u%n", &pc, &ad, &ln, &n) != 3) break;
            b.dumps.push_back({pc, ad, ln, 0});
            b.cpu.set_hook(pc, Board::hook, &b);
            q += n;
            if (*q == ',') q++;
        }
    }
    blip25_init(&b.amb_dec);
    b.cpu.reset();
    b.cpu.irqlog = getenv("TSIM_IRQLOG") != nullptr;
    b.io_stats = getenv("TSIM_IOSTATS") != nullptr;
    if (getenv("TSIM_IOLOG")) sscanf(getenv("TSIM_IOLOG"), "%x,%x", &b.iolog_lo, &b.iolog_hi);
    if (getenv("TSIM_RING")) b.cpu.ring = new uint32_t[65536]();
    // boot: run until the firmware idles waiting for its first interrupt (board_init, PLL lock delay, ...)
    for (int i = 0; i < 400 && !b.cpu.halted; i++) {
        b.cpu.run(b.cpu.cycles + 100000);
        b.uart_service(b.cpu.cycles);
        b.i2c_service(b.cpu.cycles);
        b.tmr_service(b.cpu.cycles);
        if (b.cpu.idle) break;
    }
    if (b.cpu.halted) { snprintf(errbuf, sizeof errbuf, "firmware stopped during boot: %s (pc 0x%06X)", b.cpu.error.c_str(), b.cpu.pc); return errbuf; }
    if (!b.cpu.idle && !b.stock && getenv("TSIM_REQUIRE_IDLE")) { snprintf(errbuf, sizeof errbuf, "firmware did not reach its main loop (pc 0x%06X)", b.cpu.pc); return errbuf; }
    b.first_tick = true;
    return nullptr;
}

extern "C" void tsim_step(const uint16_t *adc720) { if (B) B->step(adc720); }

extern "C" int tsim_cpu_send(const char *cmd)
{
    if (!B) return 0;
    B->cpu_tx.push_back(std::string(cmd) + "\r\n");
    B->i2c_cmds++;
    if (getenv("TSIM_I2CLOG"))
        fprintf(stderr, "DSP %.3f: queued '%s': i2c_state %d MDR %04X next %.3f ready %d\n", B->cpu.cycles / SYSCLK, cmd,
                B->i2c_state, B->io[0x1A24], B->i2c_next / SYSCLK, B->dsp_ready());
    return 1;
}

extern "C" int tsim_cpu_line(char *out, int max)
{
    if (!B || B->cpu_lines.empty()) return 0;
    std::string l = B->cpu_lines.front();
    B->cpu_lines.pop_front();
    strncpy(out, l.c_str(), (size_t)max - 1);
    out[max - 1] = 0;
    return 1;
}

extern "C" void tsim_stats_get(tsim_stats *s) { if (B) B->stats(s); else memset(s, 0, sizeof *s); }

// debugging: a firmware variable by its map symbol (data symbols are word addresses), word offset off; -1 if unknown
extern "C" long tsim_peek_sym(const char *sym, int off, int words32)
{
    if (!B || !B->S(sym)) return -1;
    uint32_t a = B->S(sym) + (uint32_t)off;
    return words32 ? (long)B->cpu.rd32(a) : (long)B->cpu.rd16(a);
}

extern "C" const char *tsim_last_error(void)
{
    if (!B) return "not initialised";
    if (!B->cpu.error.empty()) {
        snprintf(errbuf, sizeof errbuf, "%s (pc 0x%06X)", B->cpu.error.c_str(), B->cpu.pc);
        return errbuf;
    }
    return "";
}

extern "C" void tsim_debug(void)
{
    if (!B) return;
    c55::Cpu &c = B->cpu;
    fprintf(stderr, "pc %06X idle %d halted %d err '%s' st0 %04X st1 %04X st2 %04X st3 %04X ier0 %04X ifr0 %04X ier1 %04X ifr1 %04X ivpd %04X ivph %04X sp %04X ssp %04X sph %X\n",
            c.pc, c.idle, c.halted, c.error.c_str(), c.r.st0, c.r.st1, c.r.st2, c.r.st3, c.r.ier0, c.r.ifr0, c.r.ier1,
            c.r.ifr1, c.r.ivpd, c.r.ivph, c.r.sp, c.r.ssp, c.r.sph);
    fprintf(stderr, "DMA3ch0 TCRU %04X DSA %04X:%04X TCRL %04X | DMA1ch1 TCRU %04X | DMAIFR %04X DMAIER %04X | UART IER %04X LCR %04X DLL %04X | I2C MDR %04X OAR %04X IMR %04X | GPIO DIR1 %04X OUT1 %04X\n",
            B->io[0x0F05], B->io[0x0F03], B->io[0x0F02], B->io[0x0F04], B->io[0x0D25], B->io[0x1C30], B->io[0x1C31],
            B->io[0x1B02], B->io[0x1B06], B->io[0x1B10], B->io[0x1A24], B->io[0x1A00], B->io[0x1A04], B->io[0x1C06], B->io[0x1C0A]);
    if (getenv("TSIM_WARN")) for (auto &w : c.warnings) fprintf(stderr, "warn: %s\n", w.c_str());
    if (c.ring) {
        fprintf(stderr, "last instructions:\n");
        for (uint32_t i = c.ring_i - (uint32_t)atoi(getenv("TSIM_RING")); i != c.ring_i; i++) {
            uint32_t a = c.ring[i & 65535];
            c55::Insn *q = c.insn_at(a);
            fprintf(stderr, "  %06X %s%s%s\n", a, q ? q->text.c_str() : "?", q && q->par ? " || " : "", q && q->par ? q->par->text.c_str() : "");
        }
    }
}

extern "C" void tsim_profile(int on)
{
    if (!B) return;
    if (on) { B->cpu.profile = new uint32_t[0x80000](); return; }
    if (!B->cpu.profile) return;
    std::vector<std::pair<uint32_t, uint32_t>> v;
    uint64_t tot = 0;
    for (uint32_t a = 0; a < 0x80000; a++) if (B->cpu.profile[a]) { v.push_back({B->cpu.profile[a], a}); tot += B->cpu.profile[a]; }
    std::sort(v.rbegin(), v.rend());
    if (FILE *pf = fopen("tsim_profile.txt", "w")) {
        for (auto &e : v) { c55::Insn *q = B->cpu.insn_at(e.second); fprintf(pf, "%06X %u %d %d\n", e.second, e.first, q ? q->fast : -1, q && q->par ? q->par->fast : -1); }
        fclose(pf);
    }
    for (size_t i = 0; i < v.size() && i < 40; i++) {
        c55::Insn *in = B->cpu.insn_at(v[i].second);
        fprintf(stderr, "%6.2f%% %06X %s%s%s\n", 100.0 * v[i].first / tot, v[i].second, in ? in->text.c_str() : "?",
                in && in->par ? " || " : "", in && in->par ? in->par->text.c_str() : "");
    }
}

extern "C" void tsim_io_report(void)
{
    if (!B) return;
    fprintf(stderr, "I/O read:");
    for (auto &e : B->io_rd_seen) fprintf(stderr, " %04X(%u)", e.first, e.second);
    fprintf(stderr, "\nI/O write:");
    for (auto &e : B->io_wr_seen) fprintf(stderr, " %04X=%04X(%u)", e.first, B->io[e.first], e.second);
    fprintf(stderr, "\n");
}

extern "C" int tsim_is_stock(void) { return B && B->stock; }

extern "C" unsigned tsim_peek(unsigned word_addr) { return B ? B->cpu.mem[word_addr % c55::Cpu::MEM_WORDS] : 0; }
extern "C" void tsim_poke(unsigned word_addr, unsigned v) { if (B) B->cpu.mem[word_addr % c55::Cpu::MEM_WORDS] = (uint16_t)v; }

extern "C" int tsim_dev_line(char *out, int max)
{
    if (!B || B->dev_log.empty()) return 0;
    std::string l = B->dev_log.front();
    B->dev_log.pop_front();
    strncpy(out, l.c_str(), (size_t)max - 1);
    out[max - 1] = 0;
    return 1;
}

extern "C" unsigned tsim_prof_count(unsigned byte_addr) { return (B && B->cpu.profile) ? B->cpu.profile[byte_addr & 0x7FFFF] : 0; }

// one 10 ms step; audio: up to 960 samples at tsim_audio_rate(); returns the number of samples
extern "C" int tsim_step2(const uint16_t *adc720, int16_t *audio, int max)
{
    if (!B) return 0;
    B->step(adc720);
    int n = std::min(B->audio_n, max);
    memcpy(audio, B->audio_out, (size_t)n * sizeof(int16_t));
    return n;
}

extern "C" int tsim_audio_rate(void) { return 48000; }

extern "C" volatile uint32_t *tsim_pc_ptr(void) { return B ? (volatile uint32_t *)&B->cpu.pc : nullptr; }

extern unsigned long g_idle_insns;
extern "C" unsigned long tsim_idle_insns(void) { return g_idle_insns; }
extern "C" int tsim_dsp_ready(void) { return B && B->dsp_ready() ? 1 : 0; }
// DSP GPIO output level (0..15 IODATAOUT1, 16..31 IODATAOUT2; -1 when the pin is not an output)
extern "C" int tsim_dsp_gpio(int pin)
{
    if (!B || pin < 0 || pin > 31) return -1;
    uint16_t dir = B->io[pin < 16 ? 0x1C06 : 0x1C07], out = B->io[pin < 16 ? 0x1C0A : 0x1C0B];
    uint16_t m = (uint16_t)(1u << (pin & 15));
    return (dir & m) ? ((out & m) ? 1 : 0) : -1;
}
