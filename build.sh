#!/bin/bash
# Build the DJ-X100 emulator with zig (https://ziglang.org): C55x simulator + DSP board, RX65N simulator + CPU
# board, ambe2-c, the emulator DLL and the command-line drivers. Output in build/.
#   bash build.sh            everything
#   bash build.sh dll        only relink build/djxemu.dll (after one full build)
set -e
cd "$(dirname "$0")"
AMBE=third_party/ambe2-c/decoder
CXX="zig c++ -std=c++17 -target x86_64-windows-gnu"
mkdir -p build/obj/rx

dll() {
  $CXX -O2 -shared -msse2 -Ic55 -Irx -I$AMBE/include \
    djxemu.cpp build/obj/c55_*.o build/obj/rx/*.o build/obj/ambe_*.o -o build/djxemu.dll
  echo built build/djxemu.dll
}
if [ "$1" = dll ]; then dll; exit; fi

# ambe2-c (AMBE-3000R bit-compatible speech decoder)
for f in $AMBE/src/*.c; do
  b=$(basename "$f" .c)
  [ build/obj/ambe_$b.o -nt "$f" ] || zig cc -std=c99 -O2 -w -c "$f" -I$AMBE/include -I$AMBE/src -o build/obj/ambe_$b.o
done
# D-STAR（AMBE-2000 互換）の音声デコーダ: mbelib の 3600x2400。下のリンクで拾われるよう ambe_ を付ける
zig cc -std=gnu99 -O2 -w -c third_party/mbelib/dstar.c -o build/obj/ambe_mbelib_dstar.o
# C55x simulator + DJ-X100 DSP board
for f in c55sim djxboard; do
  $CXX -O3 -w -Ic55 -I$AMBE/include -Ithird_party/mbelib -c c55/$f.cpp -o build/obj/c55_$f.o
done
$CXX -O2 -w -Ic55 c55/stock_test.cpp build/obj/c55_*.o build/obj/ambe_*.o -o build/stock_test.exe
echo built build/stock_test.exe
# RX simulator (GNU sim/rx core) + DJ-X100 CPU board
for f in rx-decode rx mem reg fpu err misc rxstep; do
  [ build/obj/rx/$f.o -nt rx/gnu/$f.c ] || zig cc -std=gnu99 -O3 -w -Irx/gnu/stub -Irx/gnu -c rx/gnu/$f.c -o build/obj/rx/$f.o
done
$CXX -O3 -w -Irx -Irx/gnu/stub -c rx/djxcpu.cpp -o build/obj/rx/djxcpu.o
$CXX -O2 -w -Irx rx/cpu_test.cpp build/obj/rx/*.o -o build/cpu_test.exe
echo built build/cpu_test.exe
# whole radio: CPU + DSP simulators
$CXX -O2 -w -Irx -Ic55 rx/radio_test.cpp build/obj/rx/*.o build/obj/c55_*.o build/obj/ambe_*.o -o build/radio_test.exe
echo built build/radio_test.exe
dll
# instruction listing of the stock DSP image (needs TI's C5500 code generation tools, C5500_CGT)
STOCK_BIN=${DJX_STOCK_BIN:-${DJX_FW_DIR:-firmware}/DigRxx107.bin}
if [ ! -f build/stock/DigRxx107_r.dis ] && [ -f "$STOCK_BIN" ]; then
  python stock/mkcoff.py "$STOCK_BIN" build/stock || echo "stock listing: needs the TI C5500 CGT (C5500_CGT)"
fi
