# djx100_emulator

> [!WARNING]
> This document and source code were not written with human readers in mind. Have an LLM read them.

[日本語](README.ja.md)

An emulator of the Alinco DJ-X100 receiver that runs **the radio's own firmware, instruction by instruction**:

* the CPU firmware (Renesas RX65N) on an RX simulator, with the radio's CPU board around it - LCD, key matrix, dial
  and volume encoders, EEPROM, DACs, port expander, the BK1068 broadcast receiver and the PC link,
* the DSP firmware (TI TMS320C5517) on a C55x simulator, with the DSP board around it - IF ADC, audio codec, the I2C
  link to the CPU and the AMBE-3000R vocoder chip (emulated with [ambe2-c](https://github.com/tallcat4/ambe2-c); D-STAR voice with
  [mbelib](https://github.com/szechyjs/mbelib)),
* an SDR as the antenna: SDR# / SDR++ IQ recordings, an rtl_tcp server or an RTL-SDR dongle on USB.

The CPU firmware boots, reads its settings from the EEPROM, releases the DSP reset and sets the DSP up, draws the
real LCD, reads the real keys and tunes the receiver; the SDR follows the frequency it tunes to. The DSP firmware
demodulates every mode of the DJ-X100 (FM NFM AM NAM, T98, T102/B54, DMR, NXDN(N), T61, ICDU, dPMR, D-STAR, C4FM,
AIS, ACARS, POCSAG, 12 kHz IF) with the radio's own filters and decoders.

**No firmware is included.** You need the files from Alinco's official DJ-X100 firmware updater.

## Requirements

* Windows (x86-64)
* [zig](https://ziglang.org) (C/C++ compiler; tested with 0.17) and a bash shell (Git Bash) for `build.sh`
* Python 3 (tested with 3.14) with `pip install numpy scipy pillow sounddevice`
* TI's **C5500 code generation tools** (free from TI; `C5500_CGT`, default `~/tools/ti-cgt-c5500`): used once to
  disassemble the DSP firmware, because the DSP simulator decodes instructions from TI's own disassembler listing
* the firmware files (below)
* optional: an RTL-SDR dongle (`rtlsdr.dll` + `libusb-1.0.dll` from SDR# / SDR++ / rtl-sdr releases, WinUSB driver
  via Zadig)

## Setup

1. Put the firmware files from the official updater in `firmware/`:

   | File | Content |
   |---|---|
   | `cpu_flash_FFE00000-FFFFFFFF.bin` | CPU firmware (code flash, 2 MiB) |
   | `cpu_option_FE7F5D00.bin` | CPU option-setting memory (optional) |
   | `DigRxx107.bin` | DSP firmware 1.07 (C5517 boot image) |

2. Build:

   ```
   bash build.sh
   ```

   This builds `build/djxemu.dll` (the emulator core used by the GUI) and the command-line tools
   (`radio_test.exe`, `cpu_test.exe`, `stock_test.exe`), and makes the DSP listing
   `build/stock/DigRxx107_r.dis` with the TI tools (or by hand:
   `python stock/mkcoff.py firmware/DigRxx107.bin build/stock`). Every instruction decoded from the listing is
   checked byte for byte against the firmware image.

## Use

```
python tools/make_test_iq.py analog                          # test signal: data/test_145000000Hz.wav (NFM 145.100, AM 144.800)
python gui.py data/test_145000000Hz.wav --radio-freq 145.1   # boots, then types 145.1 + SET on the keypad
python gui.py --usb                                          # antenna: RTL-SDR on USB (--usb-gain dB, default AGC)
python gui.py --rtl 127.0.0.1:1234                           # antenna: rtl_tcp server
python gui.py --usb --fullband                               # boots with the CPU input PJ0 high
```

The left side of the window is the radio. Buttons are held like real keys (mouse down = key down).

| Key | Keyboard / mouse |
|---|---|
| 0-9, `.`, SET | 0-9, `.`, Enter |
| MODE, STEP, V/M, SCAN | M, T, V, S |
| FUNC, MONI/SQL | F, Esc |
| dial | ← / →, wheel on the LCD |
| volume ring | ↑ / ↓, Shift + wheel |

A new EEPROM starts with the volume at 0: turn the volume up first. The radio rejects a frequency that is off its
STEP grid. The settings are kept in `data/radio/eeprom.bin` (saved on close).

The right side shows the IQ source, the IF filter model, the channel spectrum before and after the IF crystal filter,
the audio output, the CPU <-> DSP link log and the simulator load. **REC audio** records to `djx_*.wav`.
The radio's USB serial port (PC link) is available as TCP 127.0.0.1:7373 (`--pc-port`, 0 = off).

IQ WAV files: SDR# / SDR++ recordings (RIFF/RF64, uint8 / int16 / float32); the centre frequency is read from the
file name (`..._145000000Hz...`).

### Environment variables

| Variable | Meaning |
|---|---|
| `DJX_FW_DIR` | folder of the firmware files (default `firmware/`) |
| `DJX_CPU_FLASH`, `DJX_CPU_OPTION`, `DJX_STOCK_BIN` | individual firmware files; `DJX_STOCK_BIN` may also be another DSP image (see [docs/internals.md](docs/internals.md)) |
| `DJX_EEPROM` | EEPROM image (default `data/radio/eeprom.bin`) |
| `C5500_CGT` | TI C5500 code generation tools |
| `DJX_RTLSDR_DLL` | `rtlsdr.dll` (file or folder) |
| `DJX_FULLBAND` | `1`: boots with the CPU input PJ0 high (same as `gui.py --fullband`) |

## Tests

| Command | Checks |
|---|---|
| `python radio_headless.py outdir [s]` | boot, keys, dial; LCD to PNG |
| `python rf_test.py [iq.wav] [outdir]` | S-meter, squelch, AF on / off, volume |
| `python wfm_test.py station.wav MHz [outdir]` | broadcast FM (BK1068) |
| `python stock/tone_test.py [workdir]` | CTCSS / DCS tone squelch of the DSP firmware |
| `python stock/digital_test.py [workdir]` | D-STAR / AIS / POCSAG / DMR with synthesised signals |
| `python stock/stock_offline.py iq.wav Hz MODE s [start] [out.wav]` | a recording through the DSP firmware alone |

The test scripts print PASS / FAIL (`radio_headless` and `stock_offline` print a log).

## Repository layout

| Path | Content |
|---|---|
| `gui.py`, `djxcore.py` | GUI and the Python interface to the DLL (IQ sources, RTL-SDR, PC link) |
| `djxemu.cpp` | emulator core DLL: SDR channel, IF filter / ADC model, RF level model, threads, audio path |
| `rx/` | CPU board (`djxcpu.cpp`) and the RX simulator core (`rx/gnu`, from GNU binutils-gdb) |
| `c55/` | C55x simulator (`c55sim.cpp`) and the DSP board (`djxboard.cpp`) |
| `stock/` | DSP listing tool (`mkcoff.py`), DSP tests, signal generator |
| `tools/` | test signal maker, ADC model, boot image tool, LCD dump to PNG |
| `third_party/ambe2-c` | AMBE+ / AMBE+2 decoder (MIT) |
| `third_party/mbelib` | D-STAR voice decoder (ISC) |
| `docs/internals.md` | how the radio is modelled, verification, debugging options |

## Limits

* The RF front end is a level model (RSSI and noise detector from the SDR's channel level and SNR), not the tuner,
  PLL and IF ICs. The IF filter and ADC models are estimates.
* Timing is approximate (fixed cycles per instruction on the RX, estimated cycles on the C55x).
* The PC link is modelled at the level of the firmware's serial routines, not as a USB device controller.

More in [docs/internals.md](docs/internals.md).

## License

**GPL-3.0-or-later** ([LICENSE](LICENSE)), because the RX simulator core is GPL code that every program here links:

| Code | Origin | License |
|---|---|---|
| `rx/gnu/*.c`, `*.h`, `rx/gnu/opcode/rx.h` | GNU binutils-gdb (`sim/rx`, `opcodes/rx-decode.c`, `include/opcode/rx.h`), Free Software Foundation; `mem.c`, `reg.c`, `rx.c` modified ([rx/gnu/README.md](rx/gnu/README.md)) | GPL-3.0-or-later |
| `third_party/ambe2-c` | ambe2-c / blip25-vocoder; the mbelib-compatible API mirrors mbelib (ISC) | MIT |
| `third_party/mbelib` | mbelib (D-STAR 3600x2400 only, [third_party/mbelib/VENDORED.md](third_party/mbelib/VENDORED.md)); `dstar.c` / `dstar.h` this project, with DSD's D-STAR interleave table (ISC) | ISC (`dstar.c`, `dstar.h`: GPL-3.0-or-later) |
| everything else | this project | GPL-3.0-or-later |

## Disclaimer

DJ-X100 and Alinco are trademarks of Alinco, Inc. This project is not affiliated with or endorsed by Alinco. No
firmware or other copyrighted material of Alinco is distributed here; firmware addresses and behaviour noted in the
sources were found by analysing the firmware for interoperability. Japan's Radio Act (電波法) forbids disclosing or
misusing the contents of communications not addressed to you - use the emulator accordingly.
