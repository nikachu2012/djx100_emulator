"""Test-signal generators for the stock DSP firmware on the C55x simulator (digital modes).

Every generator returns the FM / AM baseband of one transmission; adc_words() turns it into the radio's ADC stream
(72 kS/s, 12-bit offset binary, real IF at 18 kHz, spectrum as the radio's IF chain delivers it: a positive frequency
deviation is cos(w t - phi)). Only what a receiver needs is generated; the protocols' own FEC / CRC are computed
from their specifications so the DSP's decoders can check them.
"""
import numpy as np
from scipy.signal import lfilter

FS = 72000


def adc_words(dev_hz, amp=424.0, am=None, noise=0.0, seed=1):
    """dev_hz: instantaneous frequency deviation (Hz) at FS; am: optional AM envelope (0..1)"""
    ph = 2 * np.pi * np.cumsum(dev_hz) / FS
    t = np.arange(len(dev_hz)) / FS
    env = 1.0 if am is None else am
    x = amp * env * np.cos(2 * np.pi * 18000 * t - ph)
    if noise:
        x = x + np.random.default_rng(seed).normal(0, noise, len(x))
    return np.clip(np.round(2048 + x), 0, 4095).astype(np.uint16)


def nrz_upsample(symbols, rate, fs=FS):
    """symbols (any values) held for 1/rate seconds each"""
    n = int(len(symbols) * fs / rate)
    idx = (np.arange(n) * rate / fs).astype(int)
    return np.asarray(symbols, float)[np.minimum(idx, len(symbols) - 1)]


def gaussian_filter(x, bt, rate, fs=FS):
    sps = fs / rate
    span = 4
    t = np.arange(-span * sps, span * sps + 1) / sps
    a = np.sqrt(np.log(2) / 2) / bt
    h = np.sqrt(np.pi) / a * np.exp(-(np.pi * t / a) ** 2)
    h /= h.sum()
    y = np.convolve(x, h, mode='full')[int(span * sps):int(span * sps) + len(x)]
    return y


def rrc_taps(alpha, sps, span=8):
    t = np.arange(-span * sps, span * sps + 1) / sps
    h = np.zeros_like(t)
    for i, ti in enumerate(t):
        if abs(ti) < 1e-9:
            h[i] = 1 - alpha + 4 * alpha / np.pi
        elif alpha and abs(abs(4 * alpha * ti) - 1) < 1e-9:
            h[i] = alpha / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * alpha)) + (1 - 2 / np.pi) * np.cos(np.pi / (4 * alpha)))
        else:
            h[i] = (np.sin(np.pi * ti * (1 - alpha)) + 4 * alpha * ti * np.cos(np.pi * ti * (1 + alpha))) / (np.pi * ti * (1 - (4 * alpha * ti) ** 2))
    return h / np.sqrt(np.sum(h ** 2))


def fsk4(dibits, rate, dev_outer, shaping='rrc', alpha=0.2, fs=FS):
    """4FSK: dibit 01 -> +3, 00 -> +1, 10 -> -1, 11 -> -3 (TIA-102 / DMR / NXDN mapping); deviation of +3 = dev_outer"""
    m = {1: 3.0, 0: 1.0, 2: -1.0, 3: -3.0}
    sym = np.array([m[d] for d in dibits])
    sps = fs / rate
    if shaping == 'rrc':
        # symbol impulses at the fractional sample rate, Nyquist (RRC x RRC at the receiver) shaping approximated by
        # an RRC on the transmit side
        n = int(len(sym) * sps)
        x = np.zeros(n)
        pos = (np.arange(len(sym)) * sps).astype(int)
        x[pos] = sym
        h = rrc_taps(alpha, int(round(sps)), 8)
        y = np.convolve(x, h)[len(h) // 2:len(h) // 2 + n]
        y = y / (np.max(np.abs(y)) / 3.0) if np.max(np.abs(y)) else y
    else:
        y = nrz_upsample(sym, rate, fs)
    return y * dev_outer / 3.0


def silence(sec, fs=FS):
    return np.zeros(int(sec * fs))


def bits_from_bytes(bs, msb_first=True):
    out = []
    for b in bs:
        rng = range(7, -1, -1) if msb_first else range(8)
        out += [(b >> i) & 1 for i in rng]
    return out


# ---------------------------------------------------------------------------------------------------- POCSAG
POCSAG_SYNC = 0x7CD215D8
POCSAG_IDLE = 0x7A89C197


def _bch_31_21(d21):
    g = 0b11101101001                     # x^10+x^9+x^8+x^6+x^5+x^3+1
    r = d21 << 10
    for i in range(30, 9, -1):
        if r >> i & 1:
            r ^= g << (i - 10)
    cw = (d21 << 10) | (r & 0x3FF)
    return (cw << 1) | (bin(cw).count('1') & 1)    # even parity bit


def pocsag_codewords(ric, text, func=3):
    """alphanumeric message to RIC; returns the batch codewords (sync first)"""
    addr = _bch_31_21(((ric >> 3) << 2) | func)
    frame = ric & 7
    bits = []
    for ch in text:
        c = ord(ch) & 0x7F
        bits += [(c >> i) & 1 for i in range(7)]          # 7-bit chars, LSB first
    c = 0x04                                              # EOT
    bits += [(c >> i) & 1 for i in range(7)]
    while len(bits) % 20:
        bits.append(0)
    msgs = []
    for k in range(0, len(bits), 20):
        v = 0
        for b in bits[k:k + 20]:
            v = (v << 1) | b
        msgs.append(_bch_31_21((1 << 20) | v))
    cws = [POCSAG_IDLE] * (2 * frame) + [addr] + msgs
    batches = []
    while cws:
        b = cws[:16]
        cws = cws[16:]
        b += [POCSAG_IDLE] * (16 - len(b))
        batches.append([POCSAG_SYNC] + b)
    batches.append([POCSAG_SYNC] + [POCSAG_IDLE] * 16)
    return batches


def pocsag(ric=1234567, text='DJ-X100 EMU TEST', baud=1200, dev=4500.0):
    bits = [1, 0] * 288                                   # preamble 576 bits
    for b in pocsag_codewords(ric, text):
        for cw in b:
            bits += [(cw >> i) & 1 for i in range(31, -1, -1)]
    # bit 1 = negative deviation (lower frequency), bit 0 = positive
    sym = [(-1.0 if b else 1.0) for b in bits]
    x = nrz_upsample(sym, baud) * dev
    return np.concatenate([silence(0.3), x, silence(0.5)])


# ---------------------------------------------------------------------------------------------------- DMR
def _rem(d, g, nd, npar):
    r = d << npar
    for i in range(nd + npar - 1, npar - 1, -1):
        if r >> i & 1:
            r ^= g << (i - npar)
    return r & ((1 << npar) - 1)


def golay2087(byte):
    """DMR slot type: 8 data bits + 11 bits of the (23,12) Golay remainder (g = 0xC75) + overall parity"""
    p = _rem(byte, 0xC75, 12, 11)
    par = (bin(byte).count('1') + bin(p).count('1')) & 1
    v = (byte << 12) | (p << 1) | par
    return [(v >> i) & 1 for i in range(19, -1, -1)]


_GF_EXP = [0] * 512
_GF_LOG = [0] * 256
_x = 1
for _i in range(255):
    _GF_EXP[_i] = _x
    _GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
for _i in range(255, 512):
    _GF_EXP[_i] = _GF_EXP[_i - 255]


def _gmul(a, b):
    return 0 if a == 0 or b == 0 else _GF_EXP[_GF_LOG[a] + _GF_LOG[b]]


def rs129(msg9, mask):
    """DMR RS(12,9) parity (generator x^3 + 14 x^2 + 56 x + 64 over GF(256)/0x11D), XOR mask"""
    poly = [64, 56, 14, 1]
    par = [0, 0, 0]
    for b in msg9:
        d = b ^ par[2]
        par[2] = par[1] ^ _gmul(poly[2], d)
        par[1] = par[0] ^ _gmul(poly[1], d)
        par[0] = _gmul(poly[0], d)
    return list(msg9) + [par[2] ^ mask[0], par[1] ^ mask[1], par[0] ^ mask[2]]


def _ham15113(d):
    d = list(d)
    d.append(d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ d[8])
    d.append(d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[9])
    d.append(d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[9] ^ d[10])
    d.append(d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[6] ^ d[7] ^ d[10])
    return d


def _ham1393(d):
    d = list(d)
    d.append(d[0] ^ d[1] ^ d[3] ^ d[5] ^ d[6])
    d.append(d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[6] ^ d[7])
    d.append(d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ d[8])
    d.append(d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[8])
    return d


def bptc19696(bits96):
    """BPTC(196,96) encode + interleave (ETSI TS 102 361-1 B.1.1)"""
    m = [0] * 196
    pos = 0
    rows = [(4, 11)] + [(1 + 15 * r, 11 + 15 * r) for r in range(1, 9)]
    for lo, hi in rows:
        for a in range(lo, hi + 1):
            m[a] = bits96[pos]
            pos += 1
    for r in range(9):                                     # row Hamming(15,11)
        base = 1 + 15 * r
        row = _ham15113(m[base:base + 11])
        m[base:base + 15] = row
    for c in range(15):                                    # column Hamming(13,9)
        col = [m[1 + c + 15 * r] for r in range(9)]
        col = _ham1393(col)
        for r in range(13):
            m[1 + c + 15 * r] = col[r]
    out = [0] * 196
    for a in range(196):
        out[(a * 181) % 196] = m[a]
    return out


DMR_SYNC = {'bs_data': 0xDFF57D75DF5D, 'bs_voice': 0x755FD7DF75F7, 'ms_data': 0xD5D7F77FD757,
            'ms_voice': 0x7F7D5DD57DFD}


def dmr_burst(info196, slot_type20, sync48):
    s = [(sync48 >> i) & 1 for i in range(47, -1, -1)]
    return info196[:98] + slot_type20[:10] + s + slot_type20[10:] + info196[98:]


def dmr_lc(flco, src, dst, fid=0, svc=0):
    return [flco & 0x3F, fid, svc, (dst >> 16) & 255, (dst >> 8) & 255, dst & 255, (src >> 16) & 255,
            (src >> 8) & 255, src & 255]


def dmr_data_burst(lc_bytes12, cc, dt, sync='ms_data'):
    bits = bits_from_bytes(lc_bytes12)
    return dmr_burst(bptc19696(bits), golay2087((cc << 4) | dt), DMR_SYNC[sync])


def dmr_ms_call(src=1234567, dst=1, cc=1, headers=6, voice_superframes=3, terms=3, rng=None):
    """MS-sourced (direct mode) group call: voice LC headers, voice bursts (sync + random AMBE bits; embedded
    signalling of bursts B-F not modelled), terminators; 30 ms bursts every 60 ms"""
    rng = rng or np.random.default_rng(7)
    lc = dmr_lc(0, src, dst)
    bursts = [dmr_data_burst(rs129(lc, (0x96, 0x96, 0x96)), cc, 1)] * headers
    for _ in range(voice_superframes):
        for k in range(6):
            v = list(rng.integers(0, 2, 216))
            if k == 0:
                s = [(DMR_SYNC['ms_voice'] >> i) & 1 for i in range(47, -1, -1)]
            else:
                s = [0] * 48                               # EMB + embedded data (not modelled)
            bursts.append(v[:108] + s + v[108:])
    bursts += [dmr_data_burst(rs129(lc, (0x99, 0x99, 0x99)), cc, 2)] * terms
    out = []
    for b in bursts:
        dib = [(b[i] << 1) | b[i + 1] for i in range(0, 264, 2)]
        out.append(fsk4(dib, 4800, 1944.0))
        out.append(silence(0.0325))                        # the other slot: no carrier from this MS
    return np.concatenate(out)


# ---------------------------------------------------------------------------------------------------- D-STAR
def _crc_ccitt_dstar(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    crc = ~crc & 0xFFFF
    return [crc & 0xFF, crc >> 8]


def dstar_header_bits(my='JA1EMU  ', my2='X100', ur='CQCQCQ  ', rpt1='DIRECT  ', rpt2='DIRECT  ', flags=(0, 0, 0),
                      interleave_variant=0):
    h = list(flags) + [ord(c) for c in rpt2] + [ord(c) for c in rpt1] + [ord(c) for c in ur] + \
        [ord(c) for c in my] + [ord(c) for c in my2]
    h += _crc_ccitt_dstar(h)
    bits = bits_from_bytes(h, msb_first=False) + [0, 0]      # 328 + tail
    enc = []
    s1 = s2 = 0
    for b in bits:                                          # K=3 rate 1/2: g0 = 1+D+D^2, g1 = 1+D^2
        enc += [b ^ s1 ^ s2, b ^ s2]
        s2, s1 = s1, b
    enc = enc[:660]
    out = [0] * 660
    if interleave_variant == 0:                             # rows of 28/27 written row-wise, read column-wise
        rows = [list(range(r * 28, r * 28 + 28)) if r < 12 else list(range(336 + (r - 12) * 27, 336 + (r - 12) * 27 + 27)) for r in range(24)]
        k = 0
        for c in range(28):
            for r in range(24):
                if c < len(rows[r]):
                    out[k] = enc[rows[r][c]]
                    k += 1
    else:                                                   # the inverse permutation
        rows = [list(range(r * 28, r * 28 + 28)) if r < 12 else list(range(336 + (r - 12) * 27, 336 + (r - 12) * 27 + 27)) for r in range(24)]
        k = 0
        for c in range(28):
            for r in range(24):
                if c < len(rows[r]):
                    out[rows[r][c]] = enc[k]
                    k += 1
    reg = 0x7F                                              # scrambler x^7 + x^4 + 1, all ones
    for i in range(660):
        fb = ((reg >> 6) ^ (reg >> 3)) & 1
        reg = ((reg << 1) | fb) & 0x7F
        out[i] ^= fb
    return out


# 音声フレームのインターリーブ（DSD と同じ表）: 空中のビット j -> C(W[j]) のビット X[j]
_DSTAR_W = [0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2] * 2 + \
           [0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 3, 2, 1, 0, 0, 3, 3]
_DSTAR_X = [10, 22, 11, 9, 10, 22, 11, 23, 8, 20, 9, 21, 10, 8, 9, 21, 8, 6, 7, 19, 8, 20, 9, 7,
            6, 18, 7, 5, 6, 18, 7, 19, 4, 16, 5, 17, 6, 4, 5, 17, 4, 2, 3, 15, 4, 16, 5, 3,
            2, 14, 3, 1, 2, 14, 3, 15, 0, 12, 1, 13, 2, 0, 1, 13, 0, 12, 10, 11, 0, 12, 1, 13]
_GOLAY_GEN = [0x63A, 0x31D, 0x7B4, 0x3DA, 0x1ED, 0x6CC, 0x366, 0x1B3, 0x6E3, 0x54B, 0x49F, 0x475]


def _golay2312(u):                                          # ビット 22..11 がデータ、10..0 がパリティ
    p = 0
    for i in range(12):
        if u & (0x800 >> i):
            p ^= _GOLAY_GEN[i]
    return (u << 11) | p


# 無音フレーム（空中の 9E8D3288261A3F61E8）の音声ビット。mbelib の ambe_d[49] の順
DSTAR_SILENCE = [int(c) for c in '1111100001010100101010100100111011110100000001010']


def dstar_voice_frame(d):
    """音声ビット d（mbelib の ambe_d[49] の順。b0 = d0-5,48、ゲイン b2 = d6-9,42,43）-> FEC 1200 付きの空中の 9 バイト"""
    u0 = int(''.join(map(str, d[0:12])), 2)
    u1 = int(''.join(map(str, d[12:24])), 2)
    c0, c1 = _golay2312(u0), _golay2312(u1)
    fr = [[0] * 24 for _ in range(4)]
    for i in range(23):
        fr[0][i + 1] = (c0 >> i) & 1
        fr[1][i] = (c1 >> i) & 1
    fr[0][0] = bin(c0).count('1') & 1                       # Golay(24,12) の全体パリティ
    pr = 16 * u0                                            # C1 は u0 から作る PN で変調
    for j in range(22, -1, -1):
        pr = (173 * pr + 13849) & 0xFFFF
        fr[1][j] ^= pr >> 15
    for i in range(11):
        fr[2][10 - i] = d[24 + i]
    for i in range(14):
        fr[3][13 - i] = d[35 + i]
    air = [fr[w][x] for w, x in zip(_DSTAR_W, _DSTAR_X)]
    return bytes(sum(air[8 * k + i] << i for i in range(8)) for k in range(9))


def dstar_tx(interleave_variant=0, frames=42, rng=None, voice=None):
    """voice: 音声フレームの空中の 9 バイト（None なら乱数）"""
    rng = rng or np.random.default_rng(3)
    bits = [1, 0] * 64 + [1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0]
    bits += dstar_header_bits(interleave_variant=interleave_variant)
    sync = [1, 0] * 5 + [1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0]
    for f in range(frames):                                 # 72 AMBE bits + 24 slow data bits
        bits += list(rng.integers(0, 2, 72)) if voice is None else [(voice[i // 8] >> (i % 8)) & 1 for i in range(72)]
        bits += sync if f % 21 == 0 else list(rng.integers(0, 2, 24))
    bits += [1, 0] * 16 + [0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0]   # end pattern
    nrz = nrz_upsample([1.0 if b else -1.0 for b in bits], 4800)
    return gaussian_filter(nrz, 0.5, 4800) * 1200.0


def _ham743(d):                                          # TACT Hamming(7,4,3)
    return list(d) + [d[0] ^ d[1] ^ d[2], d[1] ^ d[2] ^ d[3], d[0] ^ d[1] ^ d[3]]


def dmr_cach(tc, at=1, lcss=0):
    tact = _ham743([at, tc, (lcss >> 1) & 1, lcss & 1])
    c = [0] * 24
    tpos = [0, 4, 8, 12, 14, 18, 22]
    for i, p in enumerate(tpos):
        c[p] = tact[i]
    return c


def dmr_bs_call(src=1234567, dst=1, cc=1, headers=8, terms=4):
    """BS-sourced (repeater output) continuous TDMA: CACH + burst, the call in slot 1, idle-like headers in slot 2"""
    lc = dmr_lc(0, src, dst)
    hdr = dmr_data_burst(rs129(lc, (0x96, 0x96, 0x96)), cc, 1, 'bs_data')
    term = dmr_data_burst(rs129(lc, (0x99, 0x99, 0x99)), cc, 2, 'bs_data')
    bits = []
    seq = [hdr] * headers + [term] * terms
    for k, b in enumerate(seq):
        for tc in (0, 1):
            bits += dmr_cach(tc) + (b if tc == 0 else hdr)
    dib = [(bits[i] << 1) | bits[i + 1] for i in range(0, len(bits), 2)]
    return fsk4(dib, 4800, 1944.0)


# ---------------------------------------------------------------------------------------------------- AIS
def _crc16_x25(data_bits):
    crc = 0xFFFF
    for b in data_bits:
        if (crc ^ b) & 1:
            crc = (crc >> 1) ^ 0x8408
        else:
            crc >>= 1
    crc = ~crc & 0xFFFF
    return [(crc >> i) & 1 for i in range(16)]           # sent LSB first


def ais_type1(mmsi=431000001, lat=35.6, lon=139.7, sog=12.3, cog=45.0, hdg=45, nav=0):
    """AIS position report class A (type 1): 168 bits, MSB first fields"""
    f = [(1, 6), (0, 2), (mmsi, 30), (nav, 4), (0, 8), (int(round(sog * 10)), 10), (1, 1),
         (int(round(lon * 600000)) & 0xFFFFFFF, 28), (int(round(lat * 600000)) & 0x7FFFFFF, 27),
         (int(round(cog * 10)), 12), (hdg, 9), (30, 6), (0, 2), (0, 3), (0, 1), (0, 19)]
    bits = []
    for v, n in f:
        bits += [(v >> (n - 1 - i)) & 1 for i in range(n)]
    assert len(bits) == 168
    return bits


def ais_tx(msg_bits=None):
    msg = msg_bits or ais_type1()
    # bytes are transmitted LSB first: reverse each 8-bit group of the MSB-first message
    data = []
    for i in range(0, len(msg), 8):
        data += msg[i:i + 8][::-1]
    data += _crc16_x25(data)
    stuffed, ones = [], 0
    for b in data:
        stuffed.append(b)
        ones = ones + 1 if b else 0
        if ones == 5:
            stuffed.append(0)
            ones = 0
    flag = [0, 1, 1, 1, 1, 1, 1, 0]
    raw = [0, 1] * 12 + flag + stuffed + flag + [0] * 8
    lvl, out = 1, []
    for b in raw:                                         # NRZI: 0 = change
        if b == 0:
            lvl ^= 1
        out.append(1.0 if lvl else -1.0)
    return gaussian_filter(nrz_upsample(out, 9600), 0.4, 9600) * 2400.0
