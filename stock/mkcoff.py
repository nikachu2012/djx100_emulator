"""Wrap a stock DigRxx boot image into a COFF .out (cl55 assemble + link) so TI's dis55 can disassemble it for the
C55x simulator (c55/). Every section becomes a data section at its word address holding the image words;
dis55 -i disassembles them as code (code byte address = 2 x word address).
usage: mkcoff.py image.bin outdir      -> outdir/<name>_r.dis"""
import os, subprocess, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import bootimg

CG = os.environ.get('C5500_CGT', os.path.expanduser('~/tools/ti-cgt-c5500'))
img = bootimg.parse(open(sys.argv[1], 'rb').read())
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
base = os.path.splitext(os.path.basename(sys.argv[1]))[0]
asm, cmd = [], ['MEMORY', '{', '  ALL: o = 0x000000 l = 0x080000', '}', 'SECTIONS', '{']
for s in img['sections']:
    name = 's_%06X' % (s['addr'] * 2)
    asm.append('\t.sect "%s"' % name)
    d = s['data']
    w = [(d[i] << 8) | d[i + 1] for i in range(0, len(d) - 1, 2)]
    for i in range(0, len(w), 12):
        asm.append('\t.word ' + ','.join('0x%04X' % x for x in w[i:i + 12]))
    cmd.append('  %s : load = 0x%06X' % (name, s['addr'] * 2))
cmd.append('}')
open(os.path.join(out, base + '.asm'), 'w').write('\n'.join(asm) + '\n')
open(os.path.join(out, base + '.cmd'), 'w').write('\n'.join(cmd) + '\n')


def run(*a):
    # Linux 版 dis55 は不正な SWAP のオペランドに未初期化のバイトを出すので、デコードできない文字は置換する
    r = subprocess.run(a, cwd=out, capture_output=True, text=True, errors='replace')
    if r.returncode:
        sys.exit(r.stdout + r.stderr)
    return r.stdout


run(os.path.join(CG, 'bin', 'cl55'), '-vcpu:3.3', '--memory_model=large', '-c', base + '.asm')
run(os.path.join(CG, 'bin', 'cl55'), '-z', '-q', '-a', base + '.obj', base + '.cmd', '-o', base + '.out')
dis = run(os.path.join(CG, 'bin', 'dis55'), '-r', '-q', '-i', base + '.out')
open(os.path.join(out, base + '_r.dis'), 'w').write(dis)
print('wrote', os.path.join(out, base + '_r.dis'), len(dis.splitlines()), 'lines')
