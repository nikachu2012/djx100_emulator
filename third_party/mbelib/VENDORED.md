# mbelib (vendored, D-STAR only)

D-STAR speech decoder (AMBE 2400 + 1200 FEC, 72-bit frames; the AMBE-3000R's AMBE-2000 compatible RATEP
0x0130 0x0763 0x4000 0x0000 0x0000 0x0048), used by the AMBE-3000R model of the DSP board (`c55/djxboard.cpp`).
ambe2-c has no decoder for this rate: its rate 0 is the AMBE-3000R's own AMBE+ 2400, a different codec.

* Source: https://github.com/szechyjs/mbelib, commit 9a04ed5; only `mbelib.c`, `mbelib.h`, `mbelib_const.h`,
  `ecc.c`, `ecc_const.h`, `ambe3600x2400.c`, `ambe3600x2400_const.h` and `COPYRIGHT` are kept here, unchanged.
* `dstar.c` / `dstar.h` (this project): compile the files above as one unit with the symbols that ambe2-c's
  mbelib-compatible API also defines renamed, deinterleave the 72 air bits with the table of DSD
  (https://github.com/szechyjs/dsd, `dstar_const.h`, ISC) and decode with `mbe_processAmbe3600x2400Frame`.
* License: ISC (`COPYRIGHT`).
