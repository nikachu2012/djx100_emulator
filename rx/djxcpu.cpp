// djxcpu - the DJ-X100 CPU board (Renesas RX65N) around the GNU RX simulator core (rx/gnu, from binutils-gdb
// sim/rx + opcodes/rx-decode.c), running the stock CPU firmware image (cpu_flash_FFE00000-FFFFFFFF.bin).
//
// Board model: clock generator status, ICU (IR / IER / IPR, selectable vectors, group interrupts), MTU1 1 ms tick,
// RIIC0..2 (FIT driver style: master transmit / receive, slave receive) with the devices on the buses (serial
// EEPROM, I/O expanders on RIIC0; the DSP on RIIC1), I/O ports (keys, DSP handshake), ...
#include "djxcpu.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

extern "C" {
#include "gnu/cpu.h"
#include "gnu/mem.h"
#include "gnu/err.h"
int rx_step_block(int n, int *done);
extern unsigned char *rx_hook_map;
extern void (*rx_hook_fn)(unsigned int addr);
extern unsigned int rx_watch_lo, rx_watch_hi;
extern void (*rx_watch_fn)(unsigned int address, unsigned char value);
extern "C" unsigned int rx_rwatch_lo[8], rx_rwatch_hi[8];
extern "C" int rx_rwatch_n;
extern "C" void (*rx_rwatch_fn)(unsigned int address);
void init_regs(void);
void execution_error_ignore_all(void);
int rx_syscall(int id) { (void)id; return RX_MAKE_STEPPED(); }
int sim_get_current_source_location(const char **a, const char **b, unsigned int *c) { (void)a; (void)b; (void)c; return 0; }
}

namespace {

const double ICLK = 120e6;             // one instruction is counted as one ICLK cycle (rough)

// ---------------------------------------------------------------------------------------------- I2C devices
struct I2CDev {
    virtual ~I2CDev() {}
    virtual bool start(bool read) { (void)read; return true; }     // addressed: ACK?
    virtual bool write(uint8_t b) { (void)b; return true; }         // ACK?
    virtual uint8_t read() { return 0xFF; }
    virtual void stop() {}
};

// EEPROM access map (RX_EELOG=file: one byte per EEPROM byte, bit0 read, bit1 written; written at exit)
static uint8_t g_eeacc[0x40000];
static void eelog_save(void)
{
    const char *f = getenv("RX_EELOG");
    FILE *o = f ? fopen(f, "wb") : 0;
    if (o) { fwrite(g_eeacc, 1, sizeof g_eeacc, o); fclose(o); }
}
static struct EelogInit { EelogInit() { if (getenv("RX_EELOG")) atexit(eelog_save); } } g_eelog_init;

// 24xx512-type serial EEPROM (64 KiB per device address, 16-bit word address, auto increment)
struct Eeprom : I2CDev {
    uint8_t *mem;
    uint32_t base = 0;                            // EEPROM offset of this device (access map)
    int nwr = 0;
    uint16_t addr = 0;
    bool *dirty;
    Eeprom(uint8_t *m, bool *d) : mem(m), dirty(d) {}
    // RX_EETRACE=1: one stderr line per transfer ("ee rd|wr <addr> <len>")
    int tr_n = 0, tr_rd = 0; uint32_t tr_a = 0;
    void tr_flush() { if (tr_n && getenv("RX_EETRACE")) fprintf(stderr, "ee %s %05X %d\n", tr_rd ? "rd" : "wr", tr_a, tr_n); tr_n = 0; }
    void stop() override { tr_flush(); }
    bool start(bool read) override { tr_flush(); if (!read) nwr = 0; else { tr_rd = 1; tr_a = base + addr; } return true; }
    bool write(uint8_t b) override
    {
        if (nwr >= 2) { if (!tr_n) { tr_rd = 0; tr_a = base + addr; } tr_n++; }
        if (nwr == 0) addr = (uint16_t)((addr & 0x00FF) | (b << 8));
        else if (nwr == 1) addr = (uint16_t)((addr & 0xFF00) | b);
        else { mem[addr] = b; g_eeacc[base + addr] |= 2; addr = (uint16_t)((addr & 0xFF00) | ((addr + 1) & 0xFF)); *dirty = true; }   // 256-byte pages (M24M02-class part: the stock firmware writes 140-byte records at 0x100-aligned addresses)
        nwr++;
        return true;
    }
    uint8_t read() override { uint8_t v = mem[addr]; g_eeacc[base + addr] |= 1; addr++; tr_n++; return v; }
};

// I/O expander: register file, first written byte = register pointer, auto increment
struct Expander : I2CDev {
    uint8_t reg[256];
    int nwr = 0;
    uint8_t ptr = 0;
    std::function<uint8_t(uint8_t)> input;        // input registers (keys)
    std::function<void(uint8_t, uint8_t)> output;
    Expander() { memset(reg, 0, sizeof reg); }
    bool start(bool read) override { if (!read) nwr = 0; return true; }
    bool write(uint8_t b) override
    {
        if (nwr++ == 0) { ptr = b; return true; }
        reg[ptr] = b;
        reg[ptr & 0x7F] = b;
        if (log) fprintf(stderr, "exp%02X wr reg %02X <- %02X\n", addr7, ptr, b);
        if (output) output(ptr, b);
        ptr++;
        return true;
    }
    uint8_t read() override
    {
        ptr = (uint8_t)((ptr & 0x80) ? ptr : ptr);
        uint8_t v;
        int r = ptr & 0x7F;
        if (r < nport) {                                  // input port: output latch on outputs, pins on inputs
            uint8_t cfg = reg[(ptr & 0x80) | (cfg0 + r)], out = reg[(ptr & 0x80) | (out0 + r)];
            uint8_t pins = input ? input((uint8_t)r) : 0xFF;
            v = (uint8_t)((out & ~cfg) | (pins & cfg));
        } else v = reg[ptr];
        if (log) fprintf(stderr, "exp%02X rd reg %02X -> %02X\n", addr7, ptr, v);
        ptr++;
        return v;
    }
    int addr7 = 0;
    int nport = 2, out0 = 2, cfg0 = 6;                   // TCA6416 layout; TCA6424: 3 ports, 4, 0x0C
    bool log = getenv("RX_EXPLOG") != nullptr;
};

struct Board;

// ---- BK1068 broadcast FM receiver: 16-bit registers on a bit-banged 2-wire bus (FUN_ffe1b6e9 / FUN_ffe1b778):
// START, 0x80, ACK, (reg << 1) | R, ACK, data high, data low (ACK between, auto increment), NACK / STOP.
// SCL = P3.0, SDA = P2.6, both open drain (PDR = 1 with PODR = 0 pulls low). Registers used by the CPU firmware:
// 3 = channel (bits 9..0, 100 kHz from 64 MHz, bit 15 = tune), 5 = volume (bits 4..0), 9 = signal quality (SNR),
// 10 = status (bit 14 seek/tune complete, bits 6..0 RSSI), 11 = channel read back.
struct Bk1068 {
    uint16_t reg[32] = {0};
    bool scl = true, sda = true;            // bus lines
    bool sda_low = false;                   // slave pulls SDA low
    enum { IDLE, RX, RX_ACK, TX, TX_ACK } st = IDLE;
    int bits = 0, nbyte = 0, r = 0;
    uint8_t sr = 0, hib = 0;
    bool rd = false, selected = false, hi = true;
    uint32_t tune_count = 0;
    bool log = getenv("RX_BKLOG") != nullptr;
    uint8_t tx_byte() { return hi ? (uint8_t)(reg[r & 31] >> 8) : (uint8_t)reg[r & 31]; }
    void got_byte(uint8_t b)
    {
        if (nbyte == 0) { selected = b == 0x80; }
        else if (nbyte == 1) { r = b >> 1; rd = b & 1; hi = true; }
        else if (hi) { hib = b; hi = false; return; }        // high byte, wait for the low byte
        else {
            uint16_t v = (uint16_t)(hib << 8 | b);
            write_reg(r, v);
            r = (r + 1) & 31;
            hi = true;
        }
        if (nbyte == 0) return;
    }
    void write_reg(int n, uint16_t v)
    {
        if (log) fprintf(stderr, "BK1068 reg %02X <- %04X\n", n, v);
        reg[n & 31] = v;
        if (n == 3 && (v & 0x8000)) {                        // tune: done at once
            reg[11] = v & 0x3FF;
            reg[10] |= 0x4000;
            tune_count++;
        } else if (n == 3) reg[10] &= (uint16_t)~0x4000;
    }
    // bus lines after a port write: start / stop / clock edges
    void lines(bool nscl, bool nsda)
    {
        bool rise = nscl && !scl, fall = !nscl && scl;
        if (scl && nscl && sda && !nsda) { st = RX; bits = 0; nbyte = 0; selected = false; sda_low = false; }  // START
        else if (scl && nscl && !sda && nsda) { st = IDLE; sda_low = false; }                                     // STOP
        else if (rise) {
            if (st == RX) { sr = (uint8_t)(sr << 1 | (nsda ? 1 : 0)); bits++; }
            else if (st == TX_ACK) { if (nsda) st = IDLE; }                 // NACK: master is done
        } else if (fall) {
            if (st == RX && bits == 8) {
                uint8_t b = sr;
                got_byte(b);
                if (nbyte == 0 && !selected) { st = IDLE; }
                else { st = RX_ACK; sda_low = true; }
                nbyte++;
            } else if (st == RX_ACK) {
                sda_low = false;
                bits = 0;
                if (nbyte == 2 && rd) { st = TX; bits = 0; hi = true; sr = tx_byte(); sda_low = !(sr & 0x80); }
                else st = RX;
            } else if (st == TX) {
                bits++;
                if (bits == 8) { st = TX_ACK; sda_low = false; }
                else sda_low = !(sr >> (7 - bits) & 1);
            } else if (st == TX_ACK) {                                     // ACK: next byte
                if (hi) hi = false; else { hi = true; r = (r + 1) & 31; }
                st = TX; bits = 0; sr = tx_byte(); sda_low = !(sr & 0x80);
            }
        }
        scl = nscl; sda = nsda;
    }
};

// ---------------------------------------------------------------------------------------------- RIIC
struct Riic {
    Board *b = nullptr;
    int ch = 0;
    uint32_t base = 0;
    int vec_rxi = 0, vec_txi = 0;
    int grp_tei = 0, grp_eei = 0;          // GRPBL1 bit numbers
    std::map<int, I2CDev *> devs;          // 7-bit address -> device (master mode targets)
    I2CDev *cur = nullptr;
    bool addr_phase = false, rx_last = false, busy = false;
    // the slave was ACKed after the byte the firmware meant to be the last one: it drives the next byte and holds
    // SDA low, so no STOP / START can be generated until extra SCL clocks (ICCR1.CLO) let it finish (real-bus model)
    bool sda_held = false;
    int clo_count = 0;
    // slave reception (another master writes to us)
    std::deque<std::vector<uint8_t>> slave_in;
    std::vector<uint8_t> slave_cur;
    size_t slave_pos = 0;
    int slave_state = 0;                   // 0 idle, 1 receiving
    std::function<void(int addr, const std::vector<uint8_t> &)> on_master_write;   // completed write transfers
    std::vector<uint8_t> mw;               // bytes of the current master write
    int mw_addr = -1;
    uint8_t drr = 0xFF;
    bool shift_busy = false, tx_pending = false;
    uint8_t pend = 0;
    void start_byte(uint8_t byte);

    uint8_t &reg(int off);
    void schedule(double us, std::function<void()> f);
    void irq(int vec);
    uint64_t byte_us() const { return 23; }    // ~400 kHz, 9 bits
    // START -> TDRE: the FIT driver must see START (EEI) first; ~500 instructions whatever the CPI
    double start_tdre_us() const;
    uint8_t sr2() { return reg(9); }
    void set2(uint8_t bits) { reg(9) |= bits; }
    void clr2(uint8_t bits) { reg(9) &= (uint8_t)~bits; }
    uint8_t ier() { return reg(7); }
    void tdre() { set2(0x80); if (ier() & 0x80) irq(vec_txi); }
    void rdrf(uint8_t v) { drr = v; set2(0x20); if (ier() & 0x20) irq(vec_rxi); }

    void write_reg(int off, uint8_t v);
    uint8_t read_reg(int off);
    void service_slave();
};

// ---------------------------------------------------------------------------------------------- board
struct Board {
    std::vector<uint8_t> io = std::vector<uint8_t>(0x90000);   // 0x00080000 .. 0x0010FFFF
    uint64_t cycles = 0, insns = 0;
    bool waiting = false, halted = false;
    bool stock_app = true;                  // the stock application (else cpufw): stock-only hooks
    bool swreset = false;                   // SWRR = 0xA501 written: software reset at the end of the slice
    // ---- PC link (the radio's USB CDC serial port), high level: the firmware's byte receiver FUN_ffe527af gets the
    // PC's bytes, its USB transmit routine (RAM 0x355E9, called by FUN_ffe5a070 with the 0x3DC6F buffer) is caught.
    // The USB driver's poll routine (RAM 0x352FE, ~70 % of all instructions while idle) is skipped: no USB host
    // controller is modelled, and its cycles are still accounted (USB_POLL_CYCLES).
    std::mutex usbm;
    std::deque<uint8_t> usb_rx, usb_tx;
    bool usb_skip = getenv("RX_NOUSBSKIP") == nullptr;
    uint64_t usb_calls = 0;
    // 8-channel 8-bit DAC on a 3-wire bus (FUN_ffe17e32): P5.4 data, P5.6 clock (rising edge), P5.7 load (rising
    // edge latches). 4 address bits LSB first, then 8 data bits MSB first. ch1 = AF volume (volume step * 8).
    uint8_t p5_last = 0xFF;
    uint32_t scan_passes = 0;                    // completed key-matrix scans (the firmware debounces over 2 of them)
    uint32_t dac_sr = 0;
    int dac_bits = 0;
    int dac[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    void port5_write(uint8_t v)
    {
        uint8_t rise = (uint8_t)(v & ~p5_last);
        if (rise & 0x08) {                       // key-matrix row 0 (P5.3) released: one full 4-row scan done (FUN_ffe36fa0)
            scan_passes++;
            if (getenv("RX_SCANLOG")) fprintf(stderr, "%10.4f SCAN pass\n", cycles / ICLK);
        }
        if (!(v & 0x80) && (p5_last & 0x80)) { dac_sr = 0; dac_bits = 0; }            // load low: new word
        if (rise & 0x40) { dac_sr = (dac_sr << 1) | (v >> 4 & 1); dac_bits++; }
        if ((rise & 0x40) && !(R(0x8C032) & 0x02)) { lmx_sr = (lmx_sr << 1) | (v >> 4 & 1); lmx_bits++; }
        if ((rise & 0x80) && dac_bits >= 12) {
            uint32_t w = dac_sr & 0xFFF;
            int a = (int)((w >> 11 & 1) | (w >> 9 & 2) | (w >> 7 & 4) | (w >> 5 & 8));  // address sent LSB first
            if (a >= 1 && a <= 8) dac[a] = (int)(w & 0xFF);
            if (getenv("RX_DACLOG")) fprintf(stderr, "%10.4f DAC bits %d word %03X -> ch %d = %d\n", cycles / ICLK, dac_bits, (unsigned)w, a, (int)(w & 0xFF));
            dac_bits = 0;
        }
        p5_last = v;
    }
    // LMX2571 first LO on the same bus (LE = PJ1, low during the frame): the receive frequency follows from the
    // fractional-N words when R0 (VCO calibration) is written - generic for any firmware (cpufw has no hook
    // address). LO = f + 243.95 MHz, fPD = 19.2 MHz * MULT / R_PRE (cpufw/docs/hw_rf.md 2.)
    uint32_t lmx_sr = 0; int lmx_bits = 0;
    uint16_t lmx_reg[64] = {0};
    uint32_t lmx_freq = 0;
    void portj_write(uint8_t v)
    {
        uint8_t old = R(0x8C032);
        if (!(v & 0x02) && (old & 0x02)) { lmx_sr = 0; lmx_bits = 0; }
        if ((v & 0x02) && !(old & 0x02) && lmx_bits >= 24) {
            int r = (int)(lmx_sr >> 16 & 0x7F);
            lmx_reg[r & 63] = (uint16_t)lmx_sr;
            if (r == 0) {
                static const int cd1[4] = {4, 5, 6, 7}, cd2[8] = {1, 2, 4, 8, 16, 32, 64, 64}, pre[2] = {2, 4};
                uint16_t r6 = lmx_reg[6];
                double fpd = 19.2e6 / ((lmx_reg[5] & 0xFF) ? (lmx_reg[5] & 0xFF) : 1) * (r6 & 0x1F);
                double den = (double)(((lmx_reg[1] >> 8) << 16) | lmx_reg[3]);
                double n = (lmx_reg[4] & 0xFFF) + (den > 0 ? (double)(((lmx_reg[1] & 0xFF) << 16) | lmx_reg[2]) / den : 0);
                double lo = n * fpd * pre[lmx_reg[4] >> 15] / (cd1[(r6 >> 8) & 3] * cd2[(r6 >> 10) & 7]);
                double f = lo - 243.95e6;
                if (f > 1e6 && f < 2e9 && !stock_app) { tuned_hz = (uint32_t)(f + 0.5); tune_count++; }
                lmx_freq = (uint32_t)(f + 0.5);
                if (getenv("RX_LMXLOG")) fprintf(stderr, "%10.4f LMX R0: f = %.0f Hz\n", cycles / ICLK, f);
            }
            lmx_bits = 0;
        }
    }
    std::string err;
    std::map<uint32_t, uint64_t> rd_seen, wr_seen;
    bool log_io = false;
    uint32_t log_lo = 0, log_hi = 0;
    std::multimap<uint64_t, std::function<void()>> events;

    // EEPROM 4 x 64 KiB (0x50..0x53 on RIIC0), persisted
    std::vector<uint8_t> eeprom = std::vector<uint8_t>(0x40000, 0xFF);
    bool eeprom_dirty = false;
    std::string eeprom_path;
    std::vector<std::unique_ptr<I2CDev>> owned;
    Expander *exp20 = nullptr, *exp23 = nullptr;
    Bk1068 bk;
    void bk_bus()
    {
        bool scl = !((R(0x8C003) & 1) && !(R(0x8C023) & 1));
        bool sda = !((R(0x8C002) & 0x40) && !(R(0x8C022) & 0x40)) && !bk.sda_low;
        bk.lines(scl, sda);
    }
    Riic riic[3];
    // inputs of the board
    uint8_t port_in[0x13];                  // PIDR values for input pins (P0..PF, PG, PH, PJ = 0x12)
    std::function<bool()> dsp_ready;
    bool dsp_clock = false;
    std::function<int()> dsp_heartbeat;     // DSP GPIO10 level (-1 = input); unset: modelled as a toggling clock
    std::function<int(int)> dsp_pins;       // DSP GPIO14/15 levels (scan stop inputs PC0 / P7.3); unset: board input (high)
    // ---- front panel. Key matrix (FUN_ffe36fa0): rows driven low one at a time on P5.3 / P1.5 / P8.6 / P1.0,
    // sense lines (pull-up, low = pressed) P2.2 / P8.7 / P2.3 / P1.7 -> key codes 0..15. Keys 16/17 on P4.6/P4.0.
    // Rotary encoders: IRQ9 (vec 73) A=P4.1 B=P4.7, IRQ11 (vec 75) A=P4.3 B=P4.5 (B==A -> one way, else other).
    uint32_t keys = 0;
    uint8_t enc_pins = 0xAA;
    int enc_q[2] = {0, 0};
    uint32_t tuned_hz = 0, tune_count = 0;
    unsigned cpi = getenv("RX_CPI") ? (unsigned)atoi(getenv("RX_CPI")) : 4;   // ICLK cycles per instruction (the idle main loop redraws the LCD: fewer instructions per second = less host load). 8 made busy screens (menus) run their main loop - and so the key scan - at ~7 passes/s instead of 100: keys got lost
    bool prof = getenv("RX_PROF") != nullptr;
    std::map<uint32_t, uint64_t> prof_hist;                   // P4.1/P4.3/P4.5/P4.7 levels
    uint8_t panel_pins(int n, uint8_t in)
    {
        static const int rowpin[4][2] = {{5, 3}, {1, 5}, {8, 6}, {1, 0}};
        static const int colpin[4][2] = {{2, 2}, {8, 7}, {2, 3}, {1, 7}};
        static const int code[4][4] = {{1, 4, 7, 0}, {2, 5, 8, 10}, {3, 6, 9, 11}, {12, 13, 14, 15}};
        for (int c = 0; c < 4; c++) {
            if (colpin[c][0] != n) continue;
            bool low = false;
            for (int r = 0; r < 4; r++) {
                int rp = rowpin[r][0], rb = rowpin[r][1];
                bool driven_low = (R(0x8C000 + (uint32_t)rp) >> rb & 1) && !(R(0x8C020 + (uint32_t)rp) >> rb & 1);
                if (driven_low && (keys >> code[r][c] & 1)) low = true;
            }
            in = (uint8_t)(low ? in & ~(1 << colpin[c][1]) : in | (1 << colpin[c][1]));
        }
        if (n == 4) {
            in = (uint8_t)((in & ~0xEB) | (enc_pins & 0xAA) | 0x41);
            if (keys >> 16 & 1) in &= (uint8_t)~0x40;
            if (keys >> 17 & 1) in &= (uint8_t)~0x01;
        }
        return in;
    }
    void encoder(int which, int dir)
    {
        int a = which ? 3 : 1, bb = which ? 5 : 7;
        enc_pins ^= (uint8_t)(1 << a);
        bool av = enc_pins >> a & 1;
        bool bv = dir > 0 ? !av : av;
        enc_pins = (uint8_t)(bv ? enc_pins | (1 << bb) : enc_pins & ~(1 << bb));
        set_ir(which ? 75 : 73);
    }
    uint16_t adc_in[2][32];                 // S12AD0 / S12AD1 analogue inputs (12-bit)
    std::map<int, unsigned> nack_addr;

    uint8_t &R(uint32_t a) { return io[(a - 0x80000) % io.size()]; }

    Board()
    {
        memset(port_in, 0xFF, 16);
        memset(port_in + 16, 0x00, sizeof port_in - 16);   // PG / PH / PJ: Low（実機の通常状態に合わせる）
        // P60: 主画面のアイコン選択にだけ使う入力（FUN_ffe217bb 0xFFE2565C）。PG5 = 0 かつ P60 = 1 で
        // "⊘ENCR" を描く。実機は何も出ないので Low
        port_in[6] &= (uint8_t)~0x01;
        // PJ0: 起動時に読む入力（FUN_ffe1bf81 0xFFE1C0B4）。DJX_FULLBAND=1 で High
        if (const char *fb = getenv("DJX_FULLBAND"); fb && strcmp(fb, "0") != 0) port_in[0x12] |= 0x01;
        for (auto &u : adc_in) for (auto &c : u) c = 0x800;
        for (int k = 0; k < 3; k++) {
            Riic &r = riic[k];
            r.b = this; r.ch = k; r.base = 0x88300u + 0x20u * (uint32_t)k;
        }
        riic[0].vec_rxi = 52; riic[0].vec_txi = 53; riic[0].grp_tei = 13; riic[0].grp_eei = 14;
        if (getenv("RX_I2C_HELD")) riic[0].sda_held = true;       // start with SDA held low (EEPROM left mid-read)
        riic[1].vec_rxi = 50; riic[1].vec_txi = 51; riic[1].grp_tei = 28; riic[1].grp_eei = 29;
        riic[2].vec_rxi = 54; riic[2].vec_txi = 55; riic[2].grp_tei = 15; riic[2].grp_eei = 16;
        for (int a = 0; a < 4; a++) {
            owned.emplace_back(new Eeprom(&eeprom[(size_t)a * 0x10000], &eeprom_dirty));
            ((Eeprom *)owned.back().get())->base = (uint32_t)a * 0x10000;
            riic[0].devs[0x50 + a] = owned.back().get();
        }
        exp20 = new Expander(); owned.emplace_back(exp20); riic[0].devs[0x20] = exp20; exp20->addr7 = 0x20;
        exp23 = new Expander(); owned.emplace_back(exp23); riic[0].devs[0x23] = exp23; exp23->addr7 = 0x23;
        exp23->nport = 3; exp23->out0 = 4; exp23->cfg0 = 0x0C;
        memset(exp20->reg + 6, 0xFF, 2); memset(exp23->reg + 0x0C, 0xFF, 3); memset(exp23->reg + 0x8C, 0xFF, 3);
    }

    void at(uint64_t t, std::function<void()> f) { events.emplace(t, std::move(f)); }

    // ---- ICU
    uint64_t irm[4] = {0, 0, 0, 0};          // mirror of the IR flags (fast interrupt search)
    void set_ir(int vec) { if (vec > 0 && vec < 256) { R(0x87000 + (uint32_t)vec) = 1; irm[vec >> 6] |= 1ull << (vec & 63); } }
    int sel_vec_a(int source) { for (int v = 208; v < 256; v++) if (R(0x87900 + (uint32_t)v) == source) return v; return -1; }
    int sel_vec_b(int source) { for (int v = 128; v < 208; v++) if (R(0x87700 + (uint32_t)v) == source) return v; return -1; }
    bool ier(int vec) { return (R(0x87200 + (uint32_t)(vec >> 3)) >> (vec & 7)) & 1; }
    int ipr(int vec) { return R(0x87300 + (uint32_t)vec) & 15; }
    uint32_t rd32(uint32_t a) { return (uint32_t)R(a) | ((uint32_t)R(a + 1) << 8) | ((uint32_t)R(a + 2) << 16) | ((uint32_t)R(a + 3) << 24); }
    void wr32(uint32_t a, uint32_t v) { for (int i = 0; i < 4; i++) R(a + (uint32_t)i) = (uint8_t)(v >> (8 * i)); }
    // group interrupts (level): GRPxx status from the sources, vector requested while (GRP & GEN) != 0
    void update_groups()
    {
        uint32_t bl1 = 0;
        for (Riic &r : riic) {
            uint8_t s2 = r.reg(9), ie = r.reg(7);
            if ((s2 & 0x40) && (ie & 0x40)) bl1 |= 1u << r.grp_tei;
            if ((((s2 & 0x10) && (ie & 0x10)) || ((s2 & 0x08) && (ie & 0x08)) || ((s2 & 0x04) && (ie & 0x04)) ||
                 ((s2 & 0x02) && (ie & 0x02)) || ((s2 & 0x01) && (ie & 0x01))))
                bl1 |= 1u << r.grp_eei;
        }
        wr32(0x87634, bl1);
        if (bl1 & rd32(0x87674)) set_ir(111);
    }
    uint64_t irq_count = 0;
    std::map<int, uint64_t> irq_hist;
    bool take_interrupt()
    {
        update_groups();
        if (!(regs.r_psw & FLAGBIT_I)) return false;
        int ipl = (int)((regs.r_psw >> 24) & 15), best = -1, bp = 0;
        for (int w = 0; w < 4; w++)
            for (uint64_t m = irm[w]; m; m &= m - 1) {
                int v = w * 64 + __builtin_ctzll(m);
                if (v < 16) continue;
                if (!R(0x87000 + (uint32_t)v)) { irm[w] &= ~(1ull << (v & 63)); continue; }
                if (!ier(v)) continue;
                int p = ipr(v);
                if (p > ipl && p > bp) { bp = p; best = v; }
            }
        if (best < 0) return false;
        R(0x87000 + (uint32_t)best) = 0;
        irm[best >> 6] &= ~(1ull << (best & 63));
        static int irqlog = getenv("RX_IRQLOG") ? atoi(getenv("RX_IRQLOG")) : 0;
        if (irqlog > 0) { irqlog--; fprintf(stderr, "%10.6f IRQ %d pc %08X sr2 %02X ier %02X\n", cycles / ICLK, best, (unsigned)regs.r_pc, R(0x88309), R(0x88307)); }
        SI old_psw = regs.r_psw;
        regs.r_psw &= ~(FLAGBIT_I | FLAGBIT_U | FLAGBIT_PM);
        regs.r_psw = (regs.r_psw & ~0x0F000000u) | ((SI)bp << 24);
        SI st = get_reg(sp);
        st -= 4; mem_put_si((int)st, (unsigned long)old_psw);
        st -= 4; mem_put_si((int)st, (unsigned long)regs.r_pc);
        put_reg(sp, st);
        regs.r_pc = (SI)mem_get_si((int)(regs.r_intb + 4u * (uint32_t)best));
        waiting = false;
        irq_count++;
        irq_hist[best]++;
        return true;
    }

    // ---- MTU1: TGIA1 periodic compare match (the 1 ms system tick of this firmware), PCLKA = ICLK
    uint64_t mtu1_next = 0;
    bool mtu1_run() { return (R(0xC1280) & 0x02) != 0; }
    uint64_t mtu1_period()
    {
        static const uint32_t tpsc[8] = {1, 4, 16, 64, 1, 1, 256, 1};
        static const uint32_t tpsc2[8] = {1, 2, 8, 32, 1, 1, 1, 1};
        uint32_t d = tpsc[R(0xC1380) & 7];
        if ((R(0xC1380) & 7) == 0) d = tpsc2[R(0xC1394) & 7];
        uint32_t tgra = (uint32_t)R(0xC1388) | ((uint32_t)R(0xC1389) << 8);
        return (uint64_t)(tgra + 1) * d;
    }

    uint64_t next_event()
    {
        uint64_t t = UINT64_MAX;
        if (!events.empty()) t = events.begin()->first;
        if (mtu1_run()) t = std::min(t, mtu1_next);
        return t;
    }
    void service()
    {
        while (!events.empty() && events.begin()->first <= cycles) {
            auto f = std::move(events.begin()->second);
            events.erase(events.begin());
            f();
        }
        if (mtu1_run()) {
            if (mtu1_next == 0) mtu1_next = cycles + mtu1_period();
            while (cycles >= mtu1_next) {
                mtu1_next += mtu1_period();
                if (R(0xC1384) & 1) set_ir(sel_vec_a(8));      // TGIA1 (selectable interrupt A source 8)
            }
        } else mtu1_next = 0;
        for (Riic &r : riic) r.service_slave();
    }

    // ---- I/O
    uint8_t rd(uint32_t a)
    {
        if (log_io) rd_seen[a]++;
        uint8_t v;
        if (a >= 0x88300 && a < 0x88360 && (a & 0x1F) < 0x14) v = riic[(a - 0x88300) >> 5].read_reg((int)(a & 0x1F));
        else if ((a == 0xC1386 || a == 0xC1387) && mtu1_run() && mtu1_next) {   // MTU1.TCNT (counts up to TGRA)
            uint64_t per = mtu1_period(), left = mtu1_next > cycles ? mtu1_next - cycles : 0;
            uint64_t div = per / ((uint64_t)((uint32_t)R(0xC1388) | ((uint32_t)R(0xC1389) << 8)) + 1);
            uint32_t cnt = (uint32_t)((per - std::min(left, per)) / (div ? div : 1));
            v = (uint8_t)(a == 0xC1386 ? cnt : cnt >> 8);
        }
        else if (a >= 0x8C040 && a < 0x8C053) {               // PIDRn (P0..PJ): outputs read back, inputs from the board
            int n = (int)(a - 0x8C040);
            uint8_t dir = R(0x8C000 + (uint32_t)n), out = R(0x8C020 + (uint32_t)n);
            uint8_t in = port_in[n];
            if (n == 7 && dsp_ready) in = (uint8_t)((in & ~0x20) | (dsp_ready() ? 0x20 : 0));
            // PC5: audio frame clock from the DSP side (48 kHz while the DSP runs) - the CPU's DSP watchdog
            // With a heartbeat source (radio_test): the real wiring, DSP GPIO10 (stock toggles it every 250 ms).
            if (n == 12 && dsp_heartbeat) { int h = dsp_clock ? dsp_heartbeat() : 0; in = (uint8_t)((in & ~0x20) | (h > 0 ? 0x20 : 0)); }
            else if (n == 12) { static uint32_t lfsr = 0xACE1u; lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u); in = (uint8_t)((in & ~0x20) | ((dsp_clock && (lfsr & 1)) ? 0x20 : 0)); }
            // Scan stop inputs (stock DSP traces): DSP GPIO15 -> P7.3 (sync / carrier), GPIO14 -> PC0 (decoding / tone match)
            if (dsp_pins && n == 7) in = (uint8_t)((in & ~0x08) | (dsp_clock && dsp_pins(15) > 0 ? 0x08 : 0));
            if (dsp_pins && n == 12) in = (uint8_t)((in & ~0x01) | (dsp_clock && dsp_pins(14) > 0 ? 0x01 : 0));
            in = panel_pins(n, in);
            if (n == 2) in = (uint8_t)(bk.sda_low ? in & ~0x40 : in | 0x40);   // BK1068 SDA
            if (n == 3) in |= 1;                                                // SCL released: high
            if (n == 1 && riic[0].sda_held) in = (uint8_t)(in & ~0x08);        // SDA0 held low by the EEPROM
            v = (uint8_t)((out & dir) | (in & ~dir));
        } else switch (a) {
        case 0x8003C: {                                // OSCOVFSR
            v = 0x10;
            if (!(R(0x80032) & 1)) v |= 1;
            if (!(R(0x80033) & 1)) v |= 2;
            if (!(R(0x8002A) & 1)) v |= 4;
            if (!(R(0x80036) & 1)) v |= 8;
            break;
        }
        default: v = R(a);
        }
        if (log_hi && a >= log_lo && a <= log_hi) fprintf(stderr, "%10.6f R %08X -> %02X pc %08X\n", cycles / ICLK, a, v, regs.r_pc);
        return v;
    }
    void wr(uint32_t a, uint8_t v)
    {
        if (log_io) wr_seen[a]++;
        if (log_hi && a >= log_lo && a <= log_hi) fprintf(stderr, "%10.6f W %08X <- %02X pc %08X\n", cycles / ICLK, a, v, regs.r_pc);
        if (a == 0x800C2 || a == 0x800C3) {                         // SYSTEM.SWRR: 0xA501 -> software reset
            R(a) = v;
            if (R(0x800C2) == 0x01 && R(0x800C3) == 0xA5) swreset = true;
            return;
        }
        if (a >= 0x87000 && a < 0x87100) {                          // IR
            R(a) = v & 1;
            int vec = (int)(a - 0x87000);
            if (v & 1) irm[vec >> 6] |= 1ull << (vec & 63); else irm[vec >> 6] &= ~(1ull << (vec & 63));
            return;
        }
        if (a == 0x8C025) port5_write(v);
        if (a == 0x8C032) portj_write(v);
        if (a == 0x8C001 || a == 0x8C021 || a == 0x8C061) {        // P1.2 = SCL0 as GPIO: bus recovery clocks
            auto scl_low = [this] { return !(R(0x8C061) & 0x04) && (R(0x8C001) & 0x04) && !(R(0x8C021) & 0x04); };
            bool was_low = scl_low();
            R(a) = v;
            if (was_low && !scl_low() && riic[0].sda_held && ++riic[0].clo_count >= 9) {
                riic[0].sda_held = false;                           // the slave finished its byte, sees the NACK
                riic[0].reg(1) &= (uint8_t)~0xE8;
                riic[0].cur = nullptr;
            }
            return;
        }
        if (a == 0x8C002 || a == 0x8C022 || a == 0x8C003 || a == 0x8C023) { R(a) = v; bk_bus(); return; }
        if (a >= 0x88300 && a < 0x88360 && (a & 0x1F) < 0x14) { riic[(a - 0x88300) >> 5].write_reg((int)(a & 0x1F), v); return; }
        R(a) = v;
        if ((a == 0x89001 || a == 0x89101) && (v & 0x80)) {      // S12ADn ADCSR.ADST: conversion of the selected channels
            int u = a == 0x89001 ? 0 : 1;
            uint32_t base = u ? 0x89100u : 0x89000u;
            at(cycles + 360, [this, u, base] {
                uint32_t sel = (uint32_t)R(base + 4) | ((uint32_t)R(base + 5) << 8) | ((uint32_t)R(base + 6) << 16) | ((uint32_t)R(base + 7) << 24);
                for (int ch = 0; ch < 32; ch++)
                    if (sel & (1u << ch)) {
                        uint16_t d = adc_in[u][ch];
                        R(base + 0x20 + 2u * (uint32_t)ch) = (uint8_t)d;
                        R(base + 0x21 + 2u * (uint32_t)ch) = (uint8_t)(d >> 8);
                    }
                R(base + 1) &= (uint8_t)~0x80;
            });
        }
    }
};

// ---- RIIC implementation (needs Board)
uint8_t &Riic::reg(int off) { return b->R(base + (uint32_t)off); }
void Riic::schedule(double us, std::function<void()> f) { b->at(b->cycles + (uint64_t)(us * ICLK / 1e6), std::move(f)); }
void Riic::irq(int vec) { b->set_ir(vec); }
double Riic::start_tdre_us() const { return 500.0 * b->cpi / (ICLK / 1e6); }

uint8_t Riic::read_reg(int off)
{
    switch (off) {
    case 0: return (uint8_t)(reg(0) | (sda_held ? 0x02 : 0x03));   // ICCR1: SCLI high, SDAI low while held
    case 0x13: {                                                    // ICDRR: read starts the next byte
        uint8_t v = drr;
        if (sr2() & 0x20) {
            clr2(0x20);
            if ((reg(1) & 0x40) && !(reg(1) & 0x20) && cur && !rx_last) {    // master receive: next byte
                schedule(byte_us(), [this] {
                    rx_last = (reg(4) & 0x08) != 0;                // ACKBT at the 9th clock
                    rdrf(cur ? cur->read() : 0xFF);
                });
            }
        }
        return v;
    }
    default: return reg(off);
    }
}

void Riic::write_reg(int off, uint8_t v)
{
    switch (off) {
    case 1: {                                                       // ICCR2: ST / RS / SP requests
        reg(1) = (uint8_t)((reg(1) & 0xF1) | (v & 0x0E));
        if ((v & (0x02 | 0x04 | 0x08)) && sda_held) {               // SDA held low by the slave: nothing happens
            fprintf(stderr, "RIIC%u: ICCR2 %02X ignored, SDA held low by the slave\n", (unsigned)ch, v);
            return;
        }
        if ((v & 0x08) && (reg(1) & 0x40) && !(reg(1) & 0x20) && cur && !rx_last) {    // stop after an ACKed byte
            fprintf(stderr, "RIIC%u: STOP after an ACKed master-receive byte: slave holds SDA\n", (unsigned)ch);
            sda_held = true;
            clo_count = 0;
            return;
        }
        if ((v & 0x02) && !(reg(1) & 0x80)) {                       // start
            schedule(byte_us() / 2, [this] {
                reg(1) = (uint8_t)((reg(1) & ~0x02) | 0xE0);        // BBSY MST TRS
                set2(0x04);                                         // START (EEI first, TDRE a little later)
                addr_phase = true; rx_last = false; cur = nullptr; mw.clear(); mw_addr = -1;
                shift_busy = false; tx_pending = false;
                schedule(start_tdre_us(), [this] { tdre(); });
            });
        } else if (v & 0x04) {                                      // restart
            schedule(byte_us() / 2, [this] {
                reg(1) = (uint8_t)((reg(1) & ~0x04) | 0xE0);
                set2(0x04);
                if (on_master_write && mw_addr >= 0 && !mw.empty()) on_master_write(mw_addr, mw);
                addr_phase = true; rx_last = false; mw.clear(); mw_addr = -1;
                shift_busy = false; tx_pending = false;
                schedule(start_tdre_us(), [this] { tdre(); });
            });
        } else if (v & 0x08) {                                      // stop
            schedule(byte_us() / 2, [this] {
                reg(1) &= (uint8_t)~(0x08 | 0xE0);                  // SP, BBSY MST TRS
                clr2(0x80 | 0x40);
                set2(0x08);                                         // STOP
                if (cur) cur->stop();
                if (on_master_write && mw_addr >= 0 && !mw.empty()) on_master_write(mw_addr, mw);
                cur = nullptr; mw.clear(); mw_addr = -1;
                busy = false;
            });
        }
        return;
    }
    case 9: reg(9) = (uint8_t)(reg(9) & (v | 0xE0)); return;      // ICSR2: flags cleared by writing 0
    case 0:                                                         // ICCR1
        if ((v & 0x20) && sda_held && ++clo_count >= 9) {           // CLO: extra clocks free the slave
            sda_held = false;
            reg(1) &= (uint8_t)~0xE8;
            cur = nullptr;
        }
        if (!(v & 0x80) && (v & 0x40)) {                            // ICE = 0, IICRST = 1: module reset
            reg(1) = 0; reg(9) = 0; reg(4) = 0;
        }
        reg(0) = (uint8_t)(v & ~0x20);
        return;
    case 4:                                                         // ICMR3: ACKBT only with ACKWP already 1
        if (!(reg(4) & 0x10))
            v = (uint8_t)((v & ~0x08) | (reg(4) & 0x08));
        reg(4) = v;
        return;
    case 8: reg(8) = (uint8_t)(reg(8) & v); return;               // ICSR1
    case 0x12:                                                      // ICDRT: transmit
        if (reg(1) & 0x04) {                                        // RS still pending: the real RIIC does not
            // send this byte as the address after the restart (seen on the radio: the read then times out).
            // Firmware must wait for ICSR2.START after setting RS.
            fprintf(stderr, "RIIC%u: ICDRT written while RS is pending (byte %02X lost)\n",
                    (unsigned)((base - 0x88300u) / 0x20u), v);
            return;
        }
        reg(0x12) = v;
        clr2(0x80 | 0x40);
        if (!shift_busy) start_byte(v);
        else { tx_pending = true; pend = v; }
        return;
    default: reg(off) = v;
    }
}

// a byte moves from ICDRT to the shift register: ICDRT is empty again (TDRE), the byte is on the bus for 9 clocks
void Riic::start_byte(uint8_t byte)
{
    shift_busy = true;
    if (!addr_phase) schedule(1, [this] { if (!tx_pending && shift_busy) tdre(); });
    schedule(byte_us(), [this, byte] {
        bool ack;
        if (addr_phase) {
            addr_phase = false;
            int a = byte >> 1;
            bool rd = byte & 1;
            auto it = devs.find(a);
            cur = it == devs.end() ? nullptr : it->second;
            ack = cur ? cur->start(rd) : (on_master_write && !rd);
            mw_addr = rd ? -1 : a;
            if (!ack) b->nack_addr[a]++;
            if (ack && rd) {                                        // master receive: RDRF with the dummy byte
                shift_busy = false;
                reg(1) &= (uint8_t)~0x20;
                rx_last = false;
                rdrf(0xFF);
                return;
            }
            if (ack && !tx_pending) tdre();                         // ready for the first data byte
        } else {
            ack = cur ? cur->write(byte) : (mw_addr >= 0);
            if (mw_addr >= 0) mw.push_back(byte);
        }
        if (!ack) { shift_busy = false; tx_pending = false; set2(0x10); reg(4) |= 0x04; return; }   // NACKF, ACKBR
        reg(4) &= (uint8_t)~0x04;
        if (tx_pending) { tx_pending = false; start_byte(pend); return; }
        shift_busy = false;
        set2(0x40);                                                 // TEND: nothing more to send
    });
}

// slave reception: queued write transfers from another master (the DSP) addressed to SARL0
void Riic::service_slave()
{
    if (slave_state == 0) {
        if (slave_in.empty() || (reg(1) & 0x80) || !(reg(0) & 0x80)) return;   // bus busy or module off
        slave_cur = slave_in.front();
        slave_in.pop_front();
        slave_pos = 0;
        slave_state = 1;
        reg(1) = (uint8_t)((reg(1) & 0x1F) | 0x80);                 // BBSY, slave receiver
        set2(0x04);                                                 // START
        reg(8) |= 0x01;                                             // AAS0
        rdrf(reg(0x0A));                                            // address byte (dummy read)
        return;
    }
    if (sr2() & 0x20) return;                                       // wait until ICDRR is read
    if (slave_pos < slave_cur.size()) {
        uint8_t v = slave_cur[slave_pos++];
        schedule(0, [] {});
        rdrf(v);
        return;
    }
    slave_state = 0;
    reg(1) &= (uint8_t)~0x80;
    reg(8) &= (uint8_t)~0x01;
    set2(0x08);                                                     // STOP
}

Board *B;
char errbuf[512];

extern "C" {
extern unsigned int rx_ram_hooks[8];
extern int rx_n_ram_hooks;
}
static unsigned USB_POLL_INSNS = getenv("RX_USBPOLL") ? (unsigned)atoi(getenv("RX_USBPOLL")) : 500;        // ~instructions of one RAM 0x352FE call (765 per idle-loop pass measured, ~70 % in it)

static void emulate_rts(void)
{
    unsigned int spv = get_reg(sp);
    regs.r_pc = (SI)mem_get_si((int)spv);
    put_reg(sp, spv + 4);
}

// FUN_ffe2813c(src R1, count R2, x R3, y R4, [sp+4] width bits, [sp+5] font): the LCD glyph blitter, ~40 % of
// the firmware's instructions while idle (the main loop redraws the screen). Native, same memory writes, the
// cycles of the original (~44 instructions per glyph) are still accounted. RX_NOFASTBLIT=1 runs the original.
static bool fast_blit = getenv("RX_NOFASTBLIT") == nullptr;
static void glyph_blit(void)
{
    static const uint32_t font[6] = {0, 0xFFE09552u, 0xFFE0A552u, 0xFFE0AD52u, 0xFFE0B552u, 0xFFE0BD52u};
    uint32_t src = (uint32_t)regs.r[1], n = (uint32_t)regs.r[2] & 0xFFFF;
    uint32_t x = (uint32_t)regs.r[3] & 0xFFFF, y = (uint32_t)regs.r[4] & 0xFFFF;
    uint32_t spv = get_reg(sp);
    uint8_t wbits = (uint8_t)mem_get_qi((int)(spv + 4)), f = (uint8_t)mem_get_qi((int)(spv + 5));
    uint32_t last = wbits & 7, cols = wbits >> 3;
    if (last == 0) { last = 8; cols = (cols - 1) & 0xFFFF; }
    uint32_t fb = (uint32_t)mem_get_si(0x85e798);
    uint32_t off = (y << 8) + x, c = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (f >= 1 && f <= 5) {
            uint32_t g = font[f] + 8u * (uint8_t)mem_get_qi((int)(src + i));
            uint32_t len = (c == cols) ? last : 8;
            for (uint32_t k = 0; k < len; k++) mem_put_qi((int)(fb + off + k), (unsigned char)mem_get_qi((int)(g + k)));
        }
        c = (c + 1) & 0xFF;
        if (c > cols) { c = 0; off += 0x100 - cols * 8; } else off += 8;
    }
    B->cycles += (uint64_t)(20 + 44 * n) * B->cpi;
    regs.r_pc = (SI)mem_get_si((int)spv);
    put_reg(sp, spv + 4);
}

// FUN_ffe29104(string R1, x R2, y R3, colour R4): the large 12-pixel font (frequency digits, labels). Native as
// above; ~260 instructions per character are accounted.
static void big_text(void)
{
    static const uint32_t font[6] = {0, 0xFFE09552u, 0xFFE0A552u, 0xFFE0AD52u, 0xFFE0B552u, 0xFFE0BD52u};
    uint32_t str = (uint32_t)regs.r[1];
    uint16_t x = (uint16_t)regs.r[2];
    uint32_t y = (uint32_t)regs.r[3] & 0xFFFF;
    uint8_t col = (uint8_t)regs.r[4];
    uint32_t fb = (uint32_t)mem_get_si(0x85e798);
    uint64_t insns = 20;
    auto put = [&](uint32_t a, uint8_t v) { mem_put_qi((int)(fb + a), v); };
    for (uint32_t i = 0; i < 4096; i++) {
        uint8_t c = (uint8_t)mem_get_qi((int)(str + i));
        if (!c) break;
        if (c == 0x2E) {
            uint32_t a = (0x13Eu - x) * 0x100u + y + 0xE;
            put(a, col); put(a + 1, col); put(a + 0x100, col); put(a + 0x101, col);
            x = (uint16_t)(x + 4); insns += 40;
        } else if (c == 0x3A) {
            uint32_t a = (0x13Eu - x) * 0x100u + y + 3;
            for (uint32_t d : {0u, 1u, 8u, 9u, 0x100u, 0x101u, 0x108u, 0x109u}) put(a + d, col);
            x = (uint16_t)(x + 4); insns += 60;
        } else if (c == 0x5F) {
            x = (uint16_t)(x + 4); insns += 20;
        } else {
            uint32_t g;
            if (c >= 0x20 && c <= 0x39) g = (uint32_t)c * 0x14u - 0x1FF1FBu;
            else if (c >= 0x41 && c <= 0x5A) g = (uint32_t)c * 0x14u - 0x1FF20Fu;
            else if (c == 0x6B) g = 0xFFE012B5u;
            else if (c == 0x64) g = 0xFFE012C9u;
            else if (c == 0x6D) g = 0xFFE012A1u;
            else if (c == 0x7A) g = 0xFFE012F1u;
            else if (c == 0x5B) g = 0xFFE0105Du;
            else if (c == 0x5D) g = 0xFFE01071u;
            else if (c == 0x3C) g = 0xFFE01049u;
            else if (c == 0x3F) g = 0xFFE012DDu;
            else if (c == 0xDF) g = 0xFFE0128Du;
            else g = 0xFFE01085u;
            uint32_t off = (0x136u - x) * 0x100u + y;
            int k = 0;
            for (uint32_t j = 0; j < 0x14; j++) {
                if (col >= 1 && col <= 5) {
                    uint32_t t = font[col] + 8u * (uint8_t)mem_get_qi((int)(g + j));
                    for (uint32_t b = 0; b < 8; b++) put(off + b, (uint8_t)mem_get_qi((int)(t + b)));
                }
                if (++k < 2) off += 8; else { k = 0; off += 0xF8; }
            }
            x = (uint16_t)(x + 12); insns += 260;
        }
    }
    B->cycles += insns * B->cpi;
    uint32_t spv = get_reg(sp);
    regs.r_pc = (SI)mem_get_si((int)spv);
    put_reg(sp, spv + 4);
}

void cpu_hook(unsigned int addr)
{
    if (addr == 0xFFE2813Cu) { glyph_blit(); return; }
    if (addr == 0xFFE29104u && fast_blit && !getenv("RX_TEXTLOG")) { big_text(); return; }
    if (addr == 0x352FEu) {                          // USB driver poll
        B->usb_calls++;
        if (!B->usb_skip) return;
        int b = -1;
        {
            std::lock_guard<std::mutex> lk(B->usbm);
            if (!B->usb_rx.empty()) { b = B->usb_rx.front(); B->usb_rx.pop_front(); }
        }
        if (b >= 0) {                                // -> the application's byte receiver (ABI slot 0xFFF7F040, as the
                                                     // driver calls it; stock: BRA FUN_ffe527af), returns to the caller
            mem_put_qi(0x377E2, 1);                  // USB configured (the CDC write routine checks it)
            mem_put_qi(0x36ED8, 1);                  // PC link on USB: FUN_ffe5a070 then skips the SCI port
            regs.r[1] = (SI)b; regs.r_pc = (SI)0xFFF7F040u; return;
        }
        emulate_rts();
        B->cycles += (uint64_t)USB_POLL_INSNS * B->cpi;
        return;
    }
    if (addr == 0x355E9u) {                          // USB CDC write (buffer R1, length R2)
        if (!B->usb_skip) return;
        uint32_t a = (uint32_t)regs.r[1], n = (uint32_t)regs.r[2] & 0xFFFF;
        {
            std::lock_guard<std::mutex> lk(B->usbm);
            for (uint32_t i = 0; i < n && i < 4096; i++) B->usb_tx.push_back((uint8_t)mem_get_qi((int)(a + i)));
            while (B->usb_tx.size() > 65536) B->usb_tx.pop_front();
        }
        regs.r[1] = 0;
        emulate_rts();
        mem_put_qi(0x36ED7, 0);                      // transfer complete (FUN_ffe5a0aa waits for this flag)
        return;
    }
    if (addr == 0xFFE420ECu) {                     // FUN_ffe420ec(freq_hz): tunes the receiver (1st IF 243.95 MHz)
        B->tuned_hz = (uint32_t)regs.r[1];
        B->tune_count++;
        if (!getenv("RX_HOOKS")) return;
    }
    if (addr == 0xFFE288EEu || addr == 0xFFE29104u || addr == 0xFFE290EFu) {                              // text draw (string R1, x R2, y R3)
        char t[80];
        int n = 0;
        for (; n < 79; n++) { t[n] = (char)mem_get_qi((int)(regs.r[1] + (unsigned)n)); if (!t[n]) break; }
        t[n] = 0;
        fprintf(stderr, "%10.4f ret %08X TEXT x%u y%u '%s'", B->cycles / ICLK, (unsigned)mem_get_si((int)get_reg(sp)), (unsigned)regs.r[2], (unsigned)regs.r[3], t);
        for (int i = 0; i < n && i < 24; i++) fprintf(stderr, " %02X", (uint8_t)t[i]);
        fputc(10, stderr);
        return;
    }
    static std::map<uint32_t, unsigned> cnt;
    static unsigned hookn = getenv("RX_HOOKN") ? (unsigned)atoi(getenv("RX_HOOKN")) : 5;
    if (cnt[addr]++ < hookn)
        fprintf(stderr, "%10.4f HOOK %08X ret %08X R1 %08X R2 %08X eb8f %02X d53a %02X ebe8 %02X\n", B->cycles / ICLK, addr,
                (unsigned)mem_get_si((int)get_reg(sp)), (unsigned)regs.r[1], (unsigned)regs.r[2],
                (unsigned)mem_get_qi(0x84eb8f), (unsigned)mem_get_qi(0x82d539), (unsigned)mem_get_qi(0x84ebe8));
}

std::vector<uint8_t> read_file(const char *p)
{
    std::vector<uint8_t> d;
    FILE *f = fopen(p, "rb");
    if (!f) return d;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    d.resize((size_t)n);
    if (fread(d.data(), 1, (size_t)n, f) != (size_t)n) d.clear();
    fclose(f);
    return d;
}

}  // namespace

extern "C" int rxio_is(unsigned int a) { return (a >= 0x00080000u && a < 0x00110000u) || (a >= 0x007E0000u && a < 0x00800000u); }
extern "C" unsigned char rxio_rd(unsigned int a) { return (a >= 0x00080000u && a < 0x00110000u) ? B->rd(a) : 0; }
extern "C" void rxio_wr(unsigned int a, unsigned char v) { if (a >= 0x00080000u && a < 0x00110000u) B->wr(a, v); }

// ---- RF section model -------------------------------------------------------------------------------------
// The receiver's level detectors are two A/D inputs of S12AD1 (read every main-loop pass, FUN_ffe31581):
//   AN115 (ch15) RSSI: averaged over 5 passes (FUN_ffe47273) and compared with 14 thresholds made from the factory
//        calibration record (S1 point, full scale) of the band / mode class -> S-meter 0..14 (0x84EB95)
//   AN116 (ch16) noise detector (noise squelch): compared with a threshold made from the squelch calibration
//        record and the SQL setting (FUN_ffe47763) -> squelch open / BUSY (0x85DE00)
// Calibration (EEPROM 0x1100 squelch, 0x1200 S-meter; 10 mode classes x 4 bands x 2 words, version word first;
// the firmware falls back to its all-zero flash defaults when the version word differs) - written by the factory
// adjustment, so an emulated radio gets this model's calibration.
static const double RSSI_DBM0 = -130.0, RSSI_PER_DB = 30.0;        // RSSI ADC = (dBm + 130) * 30
static int rssi_adc(double dbm) { return (int)std::lround(std::min(4095.0, std::max(0.0, (dbm - RSSI_DBM0) * RSSI_PER_DB))); }
static int noise_adc(double snr_db) { return (int)std::lround(std::min(4095.0, 3500.0 / std::sqrt(1.0 + std::pow(10.0, snr_db / 10.0)))); }

static bool apply_rf_cal(std::vector<uint8_t> &ee)
{
    auto w16 = [&](size_t a, int v) { ee[a] = (uint8_t)v; ee[a + 1] = (uint8_t)(v >> 8); };
    auto r16 = [&](size_t a) { return ee[a] | ee[a + 1] << 8; };
    bool changed = false;
    if (r16(0x1100) == 0xFFFF) {                 // squelch: noise level at SQL level d563 (3) and at level 32
        w16(0x1100, 3);
        for (int c = 0; c < 10; c++)
            for (int b = 0; b < 4; b++) { w16(0x1102 + (size_t)(b * 4 + c * 16), noise_adc(8.0)); w16(0x1104 + (size_t)(b * 4 + c * 16), noise_adc(30.0)); }
        changed = true;
    }
    if (r16(0x1200) == 0xFFFF) {                 // S-meter: S1 = -121 dBm, full scale = -43 dBm (6 dB / step, S9 = -73)
        w16(0x1200, 2);
        for (int c = 0; c < 10; c++)
            for (int b = 0; b < 4; b++) { w16(0x1202 + (size_t)(b * 4 + c * 16), rssi_adc(-121.0)); w16(0x1204 + (size_t)(b * 4 + c * 16), rssi_adc(-43.0)); }
        changed = true;
    }
    return changed;
}

extern "C" const char *rxcpu_init(const char *flash_bin, const char *option_bin, const char *eeprom_bin)
{
    delete B;
    B = new Board();
    rxcpu_set_rf(-127.0, -10.0);                          // no antenna signal: receiver noise only
    init_mem();
    execution_error_ignore_all();
    init_regs();
    std::vector<uint8_t> f = read_file(flash_bin);
    if (f.size() != 0x200000) { snprintf(errbuf, sizeof errbuf, "cannot read %s (2 MiB flash image)", flash_bin); return errbuf; }
    mem_put_blk((int)0xFFE00000u, f.data(), (int)f.size());
    if (option_bin && *option_bin) {
        std::vector<uint8_t> o = read_file(option_bin);
        if (!o.empty()) mem_put_blk((int)0xFE7F5D00u, o.data(), (int)o.size());
    }
    std::vector<uint8_t> z(0x80000, 0);
    mem_put_blk(0x00000000 + 4, z.data(), 0x40000 - 4);
    mem_put_blk(0x00800000, z.data(), 0x80000);
    std::vector<uint8_t> ff(0x8000, 0xFF);
    mem_put_blk(0x00100000, ff.data(), 0x8000);
    if (eeprom_bin && *eeprom_bin) {
        B->eeprom_path = eeprom_bin;
        std::vector<uint8_t> e = read_file(eeprom_bin);
        if (e.size() == B->eeprom.size()) B->eeprom = e;
        else {
            // new radio: blank EEPROM with the three markers the firmware checks at boot
            //   0x3FF00 = A5     -> FUN_ffe1e4e5(1) writes the default settings (and clears the marker)
            //   0x0DF0  = A5 5A  -> factory set-up done (else factory / DSP-flash mode)
            //   0x0F00  = 01 A5  -> adjustment done (else the adjust menu, d539 = 1)
            B->eeprom[0x3FF00] = 0xA5;
            B->eeprom[0xDF0] = 0xA5; B->eeprom[0xDF1] = 0x5A;
            B->eeprom[0xF00] = 0x01; B->eeprom[0xF01] = 0xA5;
            B->eeprom_dirty = true;
        }
        if (apply_rf_cal(B->eeprom)) B->eeprom_dirty = true;
    }
    regs.r_pc = (SI)mem_get_si((int)0xFFFFFFFCu);
    regs.r_psw = 0;
    static std::vector<uint8_t> hooks(0x200000, 0);
    rx_hook_map = hooks.data();
    rx_hook_fn = cpu_hook;
    // the stock application's own routines (tuning, the LCD fast paths) are hooked only when it is the stock
    // application: cpufw (cpufw/, ABI string "cpufw" at 0xFFF7F020) has other code at these addresses
    B->stock_app = memcmp(f.data() + 0x17F020, "cpufw", 5) != 0;
    if (B->stock_app) {
        hooks[0xFFE420ECu - 0xFFE00000u] = 1;
        if (fast_blit) { hooks[0xFFE2813Cu - 0xFFE00000u] = 1; hooks[0xFFE29104u - 0xFFE00000u] = 1; }
    }
    rx_ram_hooks[0] = 0x352FEu;
    rx_ram_hooks[1] = 0x355E9u;
    rx_n_ram_hooks = 2;
    if (getenv("RX_TEXTLOG") && B->stock_app) { hooks[0xFFE288EEu - 0xFFE00000u] = 1; hooks[0xFFE29104u - 0xFFE00000u] = 1; hooks[0xFFE290EFu - 0xFFE00000u] = 1; }
    if (getenv("RX_HOOKS")) {                             // RX_HOOKS=ffe1d65b,ffe1d736,...
        std::string h = getenv("RX_HOOKS");
        size_t p0 = 0;
        while (p0 < h.size()) {
            size_t q = h.find(',', p0);
            if (q == std::string::npos) q = h.size();
            uint32_t a = (uint32_t)strtoul(h.substr(p0, q - p0).c_str(), nullptr, 16);
            if (a >= 0xFFE00000u) hooks[a - 0xFFE00000u] = 1;
            p0 = q + 1;
        }
    }
    if (getenv("RX_WATCH")) {
        sscanf(getenv("RX_WATCH"), "%x,%x", &rx_watch_lo, &rx_watch_hi);
        rx_watch_fn = [](unsigned int a, unsigned char v) {
            static std::map<uint32_t, unsigned> seen;
            if (seen[(uint32_t)regs.r_pc]++ == 0)
                fprintf(stderr, "%10.4f WATCH %08X <- %02X pc %08X\n", B->cycles / ICLK, a, v, (unsigned)regs.r_pc);
        };
    }
    if (const char *w = getenv("RX_RWATCH")) {           // RX_RWATCH=lo,hi[,lo,hi..]: first read per (pc, addr)
        unsigned lo, hi; int n = 0;
        while (rx_rwatch_n < 8 && sscanf(w, "%x,%x%n", &lo, &hi, &n) == 2) {
            rx_rwatch_lo[rx_rwatch_n] = lo; rx_rwatch_hi[rx_rwatch_n++] = hi;
            w += n; if (*w == ',') w++; else break;
        }
        rx_rwatch_fn = [](unsigned int a) {
            static std::map<uint64_t, unsigned> seen;
            if (seen[((uint64_t)regs.r_pc << 32) | a]++ == 0)
                fprintf(stderr, "%10.4f RWATCH %08X pc %08X\n", B->cycles / ICLK, a, (unsigned)regs.r_pc);
        };
    }
    if (const char *a = getenv("RX_ADC")) {              // RX_ADC=unit:ch:value,...
        for (const char *q = a; *q;) {
            int u, c, v, n = 0;
            if (sscanf(q, "%d:%d:%d%n", &u, &c, &v, &n) != 3) break;
            rxcpu_set_adc(u, c, v);
            q += n;
            if (*q == ',') q++;
        }
    }
    B->log_io = getenv("RX_IOSTATS") != nullptr;
    if (getenv("RX_IOLOG")) sscanf(getenv("RX_IOLOG"), "%x,%x", &B->log_lo, &B->log_hi);
    return nullptr;
}


// software reset (SWRR) or power cycle: CPU registers, I/O and RAM back to the reset state, the EEPROM, the panel
// inputs and the connections to the DSP stay. All pins become inputs -> the DSP is held in reset until the
// firmware releases it again.
static void cpu_reset(void)
{
    Board *o = B;
    Board *n = new Board();
    n->eeprom = o->eeprom; n->eeprom_dirty = o->eeprom_dirty; n->eeprom_path = o->eeprom_path;
    memcpy(n->adc_in, o->adc_in, sizeof n->adc_in);
    n->keys = o->keys; n->cpi = o->cpi; n->cycles = o->cycles; n->insns = o->insns;
    n->tuned_hz = o->tuned_hz; n->tune_count = o->tune_count;
    n->dsp_ready = o->dsp_ready;
    n->dsp_heartbeat = o->dsp_heartbeat; n->dsp_clock = o->dsp_clock; n->dsp_pins = o->dsp_pins;
    memcpy(n->dac, o->dac, sizeof n->dac);
    n->bk.tune_count = o->bk.tune_count;
    n->riic[1].on_master_write = o->riic[1].on_master_write;
    n->log_io = o->log_io; n->log_lo = o->log_lo; n->log_hi = o->log_hi;
    B = n;
    delete o;
    init_regs();
    std::vector<uint8_t> z(0x80000, 0);
    mem_put_blk(0x00000000 + 4, z.data(), 0x40000 - 4);
    mem_put_blk(0x00800000, z.data(), 0x80000);
    regs.r_pc = (SI)mem_get_si((int)0xFFFFFFFCu);
    regs.r_psw = 0;
    fprintf(stderr, "%10.4f CPU reset (SWRR)\n", B->cycles / ICLK);
}

extern "C" const char *rxcpu_run(double seconds)
{
    Board &b = *B;
    for (int w = 0; w < 2; w++)
        if (b.enc_q[w]) { int d = b.enc_q[w] > 0 ? 1 : -1; b.encoder(w, d); b.enc_q[w] -= d; }
    uint64_t end = b.cycles + (uint64_t)(seconds * ICLK);
    while (b.cycles < end && !b.halted) {
        if (b.waiting) {
            uint64_t t = std::min(end, b.next_event());
            if (t > b.cycles) b.cycles = t;
            b.service();
            if (!b.take_interrupt()) continue;
        }
        int done = 0;
        int rc = rx_step_block(128, &done);
        b.insns += (uint64_t)done;
        if (b.prof && b.cycles > (uint64_t)(atof(getenv("RX_PROF")) * ICLK)) b.prof_hist[(uint32_t)regs.r_pc & ~(getenv("RX_PROFMASK") ? (uint32_t)strtoul(getenv("RX_PROFMASK"), 0, 16) : 0xFu)]++;
        b.cycles += (uint64_t)done * b.cpi;
        if (!RX_STEPPED(rc)) {
            if (RX_STOPPED(rc) && RX_STOP_SIG(rc) == 0) {
                b.waiting = true;                      // WAIT
            } else {
                snprintf(errbuf, sizeof errbuf, "simulator stop rc %d (sig %d) at pc %08X", rc,
                         RX_STOPPED(rc) ? RX_STOP_SIG(rc) : -1, (unsigned)regs.r_pc);
                b.halted = true;
                b.err = errbuf;
                return errbuf;
            }
        }
        b.service();
        b.take_interrupt();
        if (b.swreset) break;
    }
    if (b.swreset) cpu_reset();
    return nullptr;
}

extern "C" void rxcpu_save(void)
{
    if (!B || B->eeprom_path.empty() || !B->eeprom_dirty) return;
    FILE *f = fopen(B->eeprom_path.c_str(), "wb");
    if (f) { fwrite(B->eeprom.data(), 1, B->eeprom.size(), f); fclose(f); B->eeprom_dirty = false; }
}

extern "C" void rxcpu_report(void)
{
    if (B && B->prof) {
        std::vector<std::pair<uint64_t, uint32_t>> v;
        uint64_t tot = 0;
        for (auto &kv : B->prof_hist) { v.push_back({kv.second, kv.first}); tot += kv.second; }
        std::sort(v.rbegin(), v.rend());
        for (size_t i = 0; i < v.size() && i < 25; i++)
            fprintf(stderr, "PROF %08X %5.1f%%\n", v[i].second, 100.0 * (double)v[i].first / (double)tot);
    }
    Board &b = *B;
    fprintf(stderr, "cpu: %llu insns, %.3f s, pc %08X psw %08X sp %08X isp %08X usp %08X intb %08X, irqs %llu, waiting %d\n",
            (unsigned long long)b.insns, b.cycles / ICLK, (unsigned)regs.r_pc, (unsigned)regs.r_psw, (unsigned)get_reg(sp),
            (unsigned)regs.r_isp, (unsigned)regs.r_usp, (unsigned)regs.r_intb, (unsigned long long)b.irq_count, b.waiting);
    fprintf(stderr, "  USB polls %llu (skipped: %d)\n", (unsigned long long)b.usb_calls, (int)b.usb_skip);
    for (auto &e : b.irq_hist) fprintf(stderr, "  irq vec %d x%llu\n", e.first, (unsigned long long)e.second);
    for (auto &e : b.nack_addr) fprintf(stderr, "  I2C NACK (no device) addr 0x%02X x%u\n", e.first, e.second);
    if (b.log_io) {
        fprintf(stderr, "I/O written:");
        for (auto &e : b.wr_seen) fprintf(stderr, " %05X=%02X", e.first, b.R(e.first));
        fprintf(stderr, "\nI/O read:");
        for (auto &e : b.rd_seen) fprintf(stderr, " %05X(%llu)", e.first, (unsigned long long)e.second);
        fprintf(stderr, "\n");
        fprintf(stderr, "enabled vectors:");
        for (int v = 16; v < 256; v++) if (b.ier(v)) fprintf(stderr, " %d(p%d)", v, b.ipr(v));
        fprintf(stderr, "\n");
    }
}

extern "C" uint32_t rxcpu_pc(void) { return (uint32_t)regs.r_pc; }
extern "C" uint32_t rxcpu_mem32(uint32_t a) { return (uint32_t)mem_get_si((int)a); }
extern "C" uint8_t rxcpu_mem8(uint32_t a) { return (uint8_t)mem_get_qi((int)a); }
extern "C" void rxcpu_write8(uint32_t a, uint8_t v) { mem_put_qi((int)a, v); }

extern "C" uint32_t rxcpu_ioreg32(uint32_t a) { return B ? B->rd32(a) : 0; }
extern "C" void rxcpu_read(uint32_t a, uint8_t *buf, int n) { mem_get_blk((int)a, buf, n); }

// ---- connections to the DSP board (RIIC1: the CPU writes commands to 0x17, the DSP writes lines to us at 0x7F)
extern "C" void rxcpu_set_dsp_link(void (*cmd)(const uint8_t *data, int n), int (*ready)(void))
{
    B->riic[1].on_master_write = [cmd](int addr, const std::vector<uint8_t> &d) { if (addr == 0x17 && cmd) cmd(d.data(), (int)d.size()); };
    B->dsp_ready = [ready] { return ready ? ready() != 0 : true; };
}
extern "C" void rxcpu_dsp_line(const uint8_t *data, int n) { B->riic[1].slave_in.emplace_back(data, data + n); }
extern "C" uint8_t rxcpu_port_out(int port) { return B->R(0x8C020 + (uint32_t)port); }
extern "C" uint8_t rxcpu_port_dir(int port) { return B->R(0x8C000 + (uint32_t)port); }

extern "C" void rxcpu_set_dsp_clock(int on) { if (B) B->dsp_clock = on != 0; }
extern "C" void rxcpu_set_dsp_heartbeat(int (*level)(void)) { if (B) B->dsp_heartbeat = level ? std::function<int()>(level) : nullptr; }
extern "C" void rxcpu_set_dsp_pins(int (*level)(int pin)) { if (B) B->dsp_pins = level ? std::function<int(int)>(level) : nullptr; }
extern "C" void rxcpu_set_keys(uint32_t mask) { if (B) B->keys = mask; }
extern "C" uint32_t rxcpu_scan_passes(void) { return B ? B->scan_passes : 0; }
extern "C" void rxcpu_encoder(int which, int steps)
{
    if (!B) return;
    B->enc_q[which & 1] += steps;            // one detent per rxcpu_run() slice (see there)
}
extern "C" uint32_t rxcpu_tuned_hz(uint32_t *count) { if (count) *count = B ? B->tune_count : 0; return B ? B->tuned_hz : 0; }
extern "C" void rxcpu_set_cpi(int cpi) { if (B && cpi > 0) B->cpi = (unsigned)cpi; }
extern "C" double rxcpu_time(void) { return B ? B->cycles / ICLK : 0; }
// The LCD as the user sees it: GLCDC graphics plane 2 (CLUT8, 240 x 320 portrait in memory) rotated to 320 x 240
// landscape, 0xAARRGGBB per pixel. Returns 0 while the plane is not set up.
// analogue input of the A/D converters (12-bit). S12AD1: AN114 battery, AN115 RSSI, AN116 noise detector
// (noise squelch), AN117 (temperature?), AN120 external power
extern "C" void rxcpu_set_adc(int unit, int ch, int value)
{
    if (B && unit >= 0 && unit < 2 && ch >= 0 && ch < 32) B->adc_in[unit][ch] = (uint16_t)std::min(4095, std::max(0, value));
}
// audio path: 8-channel DAC value (ch 1 = AF volume, -1 = never written) and the TCA6424 outputs
// (port 2 bits 6/7 = AF power amplifier on: squelch open or a beep)
extern "C" int rxcpu_dac(int ch) { return B && ch >= 1 && ch <= 8 ? B->dac[ch] : -1; }
extern "C" int rxcpu_expander_out(int port) { return B && B->exp23 && port >= 0 && port < 3 ? B->exp23->reg[4 + port] : 0; }
// key / alarm beeps: TPU4 toggles TIOCA4 (P2.5, PMR2 bit 5) at compare match A (TGRA4 0x88158, PCLKB / 4,
// counter cleared by TGRA): f = 60 MHz / 4 / (TGRA4 + 1) / 2 while TSTR.CST4 (0x88100 bit 4) runs
extern "C" double rxcpu_beep_hz(void)
{
    if (!B || !(B->R(0x88100) & 0x10) || !(B->R(0x8C062) & 0x20)) return 0;
    unsigned tgra = B->R(0x88158) | B->R(0x88159) << 8;
    return 60e6 / 4 / (tgra + 1.0) / 2;
}
// BK1068 (broadcast FM): receive frequency (0 = not tuned yet), powered (TCA6424 port 0 bit 6), volume 0..31,
// tune counter; and its RSSI / quality registers (the RF model)
extern "C" uint32_t rxcpu_bk1068(int *powered, int *volume, uint32_t *count)
{
    if (!B) return 0;
    if (powered) *powered = (B->exp23 && (B->exp23->reg[4] & 0x40)) ? 1 : 0;
    if (volume) *volume = B->bk.reg[5] & 0x1F;
    if (count) *count = B->bk.tune_count;
    return B->bk.tune_count ? 64000000u + 100000u * (B->bk.reg[11] & 0x3FFu) : 0;
}
extern "C" void rxcpu_bk1068_rf(int rssi, int quality)
{
    if (!B) return;
    B->bk.reg[10] = (uint16_t)((B->bk.reg[10] & 0xFF80) | std::min(127, std::max(0, rssi)));
    B->bk.reg[9] = (uint16_t)((B->bk.reg[9] & 0xFF80) | std::min(127, std::max(0, quality)));
}
// PC link (USB CDC serial of the radio, high level): bytes from the PC / to the PC
extern "C" void rxcpu_usb_write(const uint8_t *d, int n)
{
    if (!B) return;
    std::lock_guard<std::mutex> lk(B->usbm);
    for (int i = 0; i < n; i++) B->usb_rx.push_back(d[i]);
}
extern "C" int rxcpu_usb_read(uint8_t *d, int max)
{
    if (!B) return 0;
    std::lock_guard<std::mutex> lk(B->usbm);
    int n = 0;
    while (n < max && !B->usb_tx.empty()) { d[n++] = B->usb_tx.front(); B->usb_tx.pop_front(); }
    return n;
}
extern "C" uint64_t rxcpu_usb_polls(void) { return B ? B->usb_calls : 0; }
// receiver input: signal level at the antenna (dBm, channel bandwidth) and the channel's signal-to-noise ratio
extern "C" void rxcpu_set_rf(double level_dbm, double snr_db)
{
    rxcpu_set_adc(1, 15, rssi_adc(level_dbm));
    rxcpu_set_adc(1, 16, noise_adc(snr_db));
}
extern "C" int rxcpu_lcd_rgba(uint32_t *out)
{
    if (!B) return 0;
    uint32_t base = B->rd32(0xE120C);
    uint32_t stride = B->rd32(0xE1210) >> 16;           // GR2FLM3.LNOFF
    if (base < 0x800000 || base >= 0x860000) return 0;
    if (stride == 0 || stride > 1024) stride = 256;
    uint32_t clut[256];
    for (int i = 0; i < 256; i++) clut[i] = B->rd32(0xE0C00 + 4u * (uint32_t)i) | 0xFF000000u;
    static std::vector<uint8_t> fb;
    fb.resize((size_t)stride * 320);
    mem_get_blk((int)base, fb.data(), (int)fb.size());
    // memory row r (0..319), column c (0..239)  ->  screen x = 319 - r, y = c
    for (int r = 0; r < 320; r++)
        for (int c = 0; c < 240; c++) out[c * 320 + (319 - r)] = clut[fb[(size_t)r * stride + (size_t)c]];
    return 1;
}
