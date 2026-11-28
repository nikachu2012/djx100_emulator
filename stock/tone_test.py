"""Tone squelch test of the stock DSP firmware on the C55x simulator (CTCSS and DCS).

    python stock/tone_test.py [workdir]

Synthesises FM at the radio's IF (18 kHz, 72 kS/s ADC words, spectrum as the radio's IF chain delivers it) with a
1 kHz tone plus a CTCSS tone or a DCS code word and runs build/stock_test.exe:
  * tone search (rs code 000): the DSP must report the tone / code in `ci1nnn` (CTCSS 1..50 = the CPU's 50-tone
    table, DCS 101 + index in the DSP's 105-code table)
  * tone squelch (rs code set): audio with the matching code, silence with another code.
PASS / FAIL. The DCS words use the DSP's own Golay check matrix (0x021A7C).
"""
import os
import subprocess
import sys
import tempfile
import wave

import numpy as np
from scipy.signal import butter, lfilter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..'))
import djxcore as D  # noqa: E402

EXE = os.path.join(ROOT, 'build', 'stock_test' + D.EXE_EXT)
CTCSS = [67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8,
         118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3, 173.8,
         177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5, 203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6,
         241.8, 250.3, 254.1]
DCS = [0o17, 0o23, 0o25, 0o26, 0o31, 0o32, 0o36, 0o43, 0o47, 0o50, 0o51, 0o53, 0o54, 0o65, 0o71, 0o72, 0o73, 0o74,
       0o114, 0o115, 0o116, 0o122, 0o125, 0o131, 0o132, 0o134, 0o143, 0o145, 0o152, 0o155, 0o156, 0o162, 0o165,
       0o172, 0o174, 0o205, 0o212, 0o223, 0o225, 0o226, 0o243, 0o244, 0o245, 0o246, 0o251, 0o252, 0o255, 0o261,
       0o263, 0o265, 0o266, 0o271, 0o274, 0o306, 0o311, 0o315, 0o325, 0o331, 0o332, 0o343, 0o346, 0o351, 0o356,
       0o364, 0o365, 0o371, 0o411, 0o412, 0o413, 0o423, 0o431, 0o432, 0o445, 0o446, 0o452, 0o454, 0o455, 0o462,
       0o464, 0o465, 0o466, 0o503, 0o506, 0o516, 0o523, 0o526, 0o532, 0o546, 0o565, 0o606, 0o612, 0o624, 0o627,
       0o631, 0o632, 0o654, 0o662, 0o664, 0o703, 0o712, 0o723, 0o731, 0o732, 0o734, 0o743]
H = [0x00149F, 0x00293E, 0x0046E3, 0x008DC6, 0x010F13, 0x020AB9, 0x0401ED, 0x0803DA, 0x1007B4, 0x200F68, 0x400A4F]


def dcs_word(code):
    d = 0x800 | code                                   # bits 8..0 code, 11..9 = 100, 22..12 parity
    for p in range(2048):
        w = d | (p << 12)
        if not any(bin(w & r).count('1') & 1 for r in H):
            return w
    raise ValueError(code)


def make(path, ctcss=None, dcs=None, secs=6.0):
    fs = 72000
    t = np.arange(int(fs * secs)) / fs
    a = 2500 * np.sin(2 * np.pi * 1000 * t)
    if ctcss:
        a += 600 * np.sin(2 * np.pi * ctcss * t)
    if dcs is not None:
        w = dcs_word(dcs)
        bits = np.array([(w >> i) & 1 for i in range(23)])          # sent LSB first, 134.4 bit/s
        b, aa = butter(2, 300 / (fs / 2))
        a += 700 * lfilter(b, aa, bits[(t * 134.4).astype(int) % 23] * 2 - 1.0)
    ph = 2 * np.pi * np.cumsum(a) / fs
    # the radio's IF chain inverts the spectrum: positive deviation = cos(w t - phi) at the ADC
    np.clip(np.round(2048 + 424 * np.cos(2 * np.pi * 18000 * t - ph)), 0, 4095).astype(np.uint16).tofile(path)


def run(u16, code, wav, secs=6):
    r = subprocess.run([EXE, D.stock_bin(), D.STOCK_DIS, u16, str(secs), 'vr', 'vr',
                        'rsfm1000%03d0000000000000' % code, 'vl02048', 'sl00'],
                       env=dict(os.environ, TSIM_WAV=wav), capture_output=True, text=True, timeout=900)
    ci = [l.split()[-1] for l in r.stdout.splitlines() if 'DSP->CPU: ci' in l]
    w = wave.open(wav)
    x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float)
    rms = float(np.sqrt(np.mean(x[-w.getframerate():] ** 2)))
    return ci, rms


def main():
    work = sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp()
    os.makedirs(work, exist_ok=True)
    ok = True
    cases = [('CTCSS %.1f Hz' % f, dict(ctcss=f), CTCSS.index(f) + 1) for f in (67.0, 88.5, 131.8, 254.1)]
    cases += [('DCS %03o' % c, dict(dcs=c), 101 + DCS.index(c)) for c in (0o23, 0o205, 0o631)]
    for name, kw, want in cases:
        u = os.path.join(work, 'sig.u16')
        make(u, **kw)
        wav = os.path.join(work, 'out.wav')
        ci, _ = run(u, 0, wav)
        got = int(ci[0][3:6]) if ci else None
        _, on = run(u, want, wav)
        _, off = run(u, want + 1 if want not in (50, 205) else want - 1, wav)
        good = got == want and on > 500 and off < 5
        ok &= good
        print(f'{name:16s} detected {got} (want {want})  open with code {want}: rms {on:6.0f}  other code: rms {off:5.0f}'
              f'  {"ok" if good else "FAIL"}', flush=True)
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
