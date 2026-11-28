"""Digital-mode test of the stock DSP firmware on the C55x simulator with synthesised signals (stock/siggen.py).

    python stock/digital_test.py [workdir]

  D-STAR  GMSK 4800: radio header (callsigns, CRC, K=3 FEC, interleave, scrambler) -> "dcMY JA1EMU  UR CQCQCQ",
          call start / end "ci"
  AIS     GMSK 9600 HDLC NRZI, type 1 position report -> "ps <MMSI>,N35.599998,E139.699996,..."
  POCSAG  1200 bit/s FSK: sync / BCH decoding reached (the firmware only reports one special message format)
  DMR     BS-sourced voice LC header bursts: data sync found and BPTC decoding entered (informational: the LC of
          these synthetic bursts is not accepted yet)
Real recordings: T102 - stock/stock_offline.py with the SDR# recordings (see README).
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import djxcore as D  # noqa: E402
import siggen as G  # noqa: E402

EXE = os.path.join(ROOT, 'build', 'stock_test' + D.EXE_EXT)


def run(u16, rs, secs, env=None, hits=None):
    e = dict(os.environ)
    if hits:
        e.update(TSIM_PROF='1', TSIM_HITS=','.join('%06x' % h for h in hits))
    r = subprocess.run([EXE, D.stock_bin(), D.STOCK_DIS, u16, str(secs), 'vr', 'vr', rs], env=e, capture_output=True,
                       text=True, timeout=900)
    lines = [l.split('DSP->CPU: ', 1)[1] for l in r.stdout.splitlines() if 'DSP->CPU: ' in l]
    lines = [l for l in lines if not l.startswith(('sr', 'vr', 'ok'))]
    h = {}
    for l in r.stdout.splitlines():
        if l.startswith('hits '):
            a, n = l[5:].split(':')
            h[int(a, 16)] = int(n)
    return lines, h


def main():
    work = sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp()
    os.makedirs(work, exist_ok=True)
    ok = True
    f = os.path.join(work, 'sig.u16')

    x = np.concatenate([G.silence(1.5), G.dstar_tx(1), G.silence(0.3), G.dstar_tx(1)])
    G.adc_words(x).tofile(f)
    lines, _ = run(f, 'rsdr10009990000000000000', 5)
    good = any(l.startswith('dcMY JA1EMU  UR CQCQCQ') for l in lines) and any(l.startswith('ci1') for l in lines)
    ok &= good
    print(f'D-STAR  {"ok  " if good else "FAIL"} {lines[:3]}')

    x = np.concatenate([G.silence(1.5)] + [np.concatenate([G.ais_tx(), G.silence(0.3)]) for _ in range(4)])
    G.adc_words(x).tofile(f)
    lines, _ = run(f, 'rsai10000000000000000000', 3.5)
    good = any(l.startswith('ps 431000001,N35.599998,E139.699996') for l in lines)
    ok &= good
    print(f'AIS     {"ok  " if good else "FAIL"} {lines[:1]}')

    x = np.concatenate([G.silence(1.5), G.pocsag(), G.silence(0.5)])
    G.adc_words(x).tofile(f)
    _, h = run(f, 'rsps10000000000000000000', 5, hits=[0x01f825, 0x01f648])
    good = h.get(0x01f825, 0) > 0 and h.get(0x01f648, 0) > 0
    ok &= good
    print(f'POCSAG  {"ok  " if good else "FAIL"} sync found {h.get(0x01f825, 0)}x, code words decoded {h.get(0x01f648, 0)}'
          f' (no report: the firmware reports a special message format only)')

    sym_nrz = G.fsk4
    G.fsk4 = lambda d, r, dev, **k: sym_nrz(d, r, dev, shaping='nrz')
    x = np.concatenate([G.silence(1.5), G.dmr_bs_call()])
    G.fsk4 = sym_nrz
    G.adc_words(x).tofile(f)
    lines, h = run(f, 'rsdm10009990000000000000', 3.5, hits=[0x02326d])
    print(f'DMR     info data sync -> BPTC decoder entered {h.get(0x02326d, 0)}x, reports {lines[:2]}')
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
