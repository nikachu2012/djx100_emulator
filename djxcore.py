"""Python side of the DJ-X100 emulator: ctypes binding of djxemu.dll, IF filter models, IQ sources."""
import ctypes as C
import os
import re
import sys
import socket
import struct
import time

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
# build.sh の出力名: Windows は DLL/EXE、macOS は dylib と拡張子なしの実行ファイル
LIB_EXT, EXE_EXT = ('.dylib', '') if sys.platform == 'darwin' else ('.dll', '.exe')
DLL = os.environ.get('DJX_DLL') or os.path.join(HERE, 'build', 'djxemu' + LIB_EXT)

# receiver channel of the core: the main receiver (IF -> ADC -> DSP) or the BK1068 broadcast receiver (WFM model)
MODE_FM, MODE_WFM = 1, 4

# IF crystal filter models (KDS 50815GR1 ~15 kHz). 8 poles is the default; other models show what happens when the
# DSP's equaliser does not match the filter.
IF_MODELS = {
    'MCF 8pole +-7.5k (default)': ('cheby', 8, 7500, 0.5),
    'MCF 6pole +-7.5k': ('cheby', 6, 7500, 0.5),
    'MCF 4pole +-7.5k': ('cheby', 4, 7500, 0.5),
    'Butterworth 8pole +-8.5k': ('butter', 8, 8500, 0),
    'none (wide)': None,
}


class Status(C.Structure):
    _fields_ = [('level_db', C.c_double), ('adc_rms', C.c_double), ('agc_db', C.c_double), ('adc_clip', C.c_int),
                ('mode', C.c_int), ('frames_to_ambe', C.c_uint), ('speech_packets', C.c_uint),
                ('backend', C.c_int), ('fw_queue', C.c_int), ('dsp_load', C.c_double), ('dsp_load_peak', C.c_double),
                ('mips', C.c_double), ('uart_tx', C.c_uint), ('uart_rx', C.c_uint), ('i2c_lines', C.c_uint),
                ('halted', C.c_uint)]


ROOT = HERE
# firmware files are not part of this repository: put them in firmware/ (DJX_FW_DIR) or name them with the DJX_* vars
FW_DIR = os.environ.get('DJX_FW_DIR') or os.path.join(ROOT, 'firmware')

# ---- stock DSP firmware (Alinco DigRxx107.bin from the official updater) ------------------------------------------
# The image runs unmodified on the C55x simulator; TI's disassembler output of it (dis55 -r -i of a COFF wrapper made
# by stock/mkcoff.py) is the instruction listing the simulator decodes from.
STOCK_BIN_CANDIDATES = [os.environ.get('DJX_STOCK_BIN', ''),
                        os.path.join(FW_DIR, 'DigRxx107.bin'),
                        os.path.join(ROOT, 'data', 'DigRxx107.bin')]
STOCK_DIS = os.path.join(ROOT, 'build', 'stock', 'DigRxx107_r.dis')

# CPU mode enum (stock CPU 4.00) -> label and the DSP mode-select command the CPU sends ("rs" + mode + 20 digits)
STOCK_MODES = [
    ('FM', 'rsfm10000000000000000000'), ('NFM', 'rsfn10000000000000000000'), ('AM', 'rsam10000000000000000000'),
    ('NAM', 'rsan10000000000000000000'), ('T98', 'rsdc19990000000000000000'), ('T102/B54', 'rsnx19990000000000000000'),
    ('DMR', 'rsdm10009990000000000000'), ('NXDN(N)', 'rsnw19990000000000000000'),
    ('T61(TYP1)', 'rsnm10000000000000000000'), ('T61(TYP2)', 'rsft10000000000000000000'),
    ('T61(TYP3)', 'rsnt10000000000000000000'), ('T61(TYP4)', 'rshk10000000000000000000'),
    ('T61(TYPX)', 'rsxp10000000000000000000'), ('ICDU', 'rsic10000000000000000000'), ('dPMR', 'rsdp10000000000000000000'),
    ('D-STAR', 'rsdr10009990000000000000'), ('C4FM(DN)', 'rsys10009990000000000000'), ('AIS', 'rsai10000000000000000000'),
    ('ACARS', 'rsac10000000000000000000'), ('POCSAG', 'rsps10000000000000000000'), ('12kIF(W)', 'rsiw10000000000000000000'),
    ('12kIF(N)', 'rsin10000000000000000000'), ('WFM', None)]


# the radio's CPU firmware (RX65N) and its EEPROM image: whole-radio emulation (Core.load_radio)
CPU_FLASH_CANDIDATES = [os.environ.get('DJX_CPU_FLASH'), os.path.join(FW_DIR, 'cpu_flash_FFE00000-FFFFFFFF.bin')]
CPU_OPTION_CANDIDATES = [os.environ.get('DJX_CPU_OPTION'), os.path.join(FW_DIR, 'cpu_option_FE7F5D00.bin')]
EEPROM_IMAGE = os.environ.get('DJX_EEPROM') or os.path.join(ROOT, 'data', 'radio', 'eeprom.bin')

# front panel key codes of the CPU firmware's key scan (rx/djxcpu.cpp: matrix codes 0..15, direct 16/17)
KEY_CODES = {'0': 0, '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '.': 10,
             'SET': 11, 'MODE': 12, 'STEP': 13, 'V/M': 14, 'SCAN': 15, 'MONI/SQL': 16, 'FUNC': 17}


def _first_file(cands):
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def stock_bin():
    for c in STOCK_BIN_CANDIDATES:
        if c and os.path.isfile(c):
            return c
    return None


def stock_listing(binpath):
    """the dis55 listing of the stock image (made once with the TI C5500 tools, stock/mkcoff.py)"""
    if os.path.isfile(STOCK_DIS):
        return STOCK_DIS
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(HERE, 'stock', 'mkcoff.py'), binpath, os.path.dirname(STOCK_DIS)],
                       capture_output=True, text=True)
    if r.returncode or not os.path.isfile(STOCK_DIS):
        raise RuntimeError('cannot make the stock listing (needs TI C5500 CGT, C5500_CGT): ' + (r.stdout + r.stderr)[-300:])
    return STOCK_DIS


class Core:
    def __init__(self, fs_in, offset_hz=0.0, stock=False):
        """stock=True: load the stock DSP firmware alone (the caller plays the CPU); otherwise call load_radio()"""
        self.lib = C.CDLL(os.path.abspath(DLL))
        L = self.lib
        L.djx_init.argtypes = [C.c_double, C.c_double]
        L.djx_fs1.restype = C.c_double
        L.djx_set_offset.argtypes = [C.c_double]
        L.djx_set_if_sos.argtypes = [C.POINTER(C.c_double), C.c_int]
        L.djx_set_mode.argtypes = [C.c_int]
        L.djx_set_volume.argtypes = [C.c_double]
        L.djx_set_squelch_db.argtypes = [C.c_double]
        L.djx_process.argtypes = [C.POINTER(C.c_float), C.c_int]
        L.djx_read_audio.argtypes = [C.POINTER(C.c_int16), C.c_int]
        L.djx_status.argtypes = [C.POINTER(Status)]
        L.djx_event.argtypes = [C.c_char_p, C.c_int]
        L.djx_cpu_send.argtypes = [C.c_char_p]
        L.djx_cpu_line.argtypes = [C.c_char_p, C.c_int]
        L.djx_channel_iq.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_float), C.c_int]
        L.djx_load_firmware.argtypes = [C.c_char_p] * 3
        L.djx_load_firmware.restype = C.c_char_p
        L.djx_fw_error.restype = C.c_char_p
        L.djx_fw_is_stock.restype = C.c_int
        L.djx_set_if_invert.argtypes = [C.c_int]
        L.djx_dev_line.argtypes = [C.c_char_p, C.c_int]
        L.djx_radio_load.argtypes = [C.c_char_p] * 5
        L.djx_radio_load.restype = C.c_char_p
        L.djx_radio_lcd.argtypes = [C.POINTER(C.c_uint32)]
        L.djx_radio_lcd.restype = C.c_uint
        L.djx_radio_keys.argtypes = [C.c_uint32]
        L.djx_radio_encoder.argtypes = [C.c_int, C.c_int]
        L.djx_radio_tuned.argtypes = [C.POINTER(C.c_uint32)]
        L.djx_radio_tuned.restype = C.c_uint32
        L.djx_radio_time.restype = C.c_double
        L.djx_radio_set_cpi.argtypes = [C.c_int]
        L.djx_radio_rf.argtypes = [C.POINTER(C.c_double), C.POINTER(C.c_double), C.POINTER(C.c_int),
                                   C.POINTER(C.c_int)]
        L.djx_radio_set_rf_gain.argtypes = [C.c_double]
        L.djx_radio_wfm.restype = C.c_int
        L.djx_radio_usb_write.argtypes = [C.c_char_p, C.c_int]
        L.djx_radio_usb_read.argtypes = [C.c_char_p, C.c_int]
        L.djx_radio_error.restype = C.c_char_p
        if L.djx_status_size() != C.sizeof(Status):
            raise RuntimeError(f'Status layout mismatch {L.djx_status_size()} vs {C.sizeof(Status)}')
        self.fs1 = L.djx_init(fs_in, offset_hz)
        self.fs_in = fs_in
        self.set_if_model('MCF 8pole +-7.5k (default)')
        self.fw_error = None
        self.fw_name = None
        if stock:
            self.load_stock()

    def load_stock(self, binpath=None):
        """run the stock DSP firmware (DigRxx107.bin) on the C55x simulator: all of the radio's modes"""
        binpath = binpath or stock_bin()
        if not binpath:
            self.fw_error = 'DigRxx107.bin not found (set DJX_STOCK_BIN)'
            return self.fw_error
        try:
            dis = stock_listing(binpath)
        except Exception as ex:
            self.fw_error = str(ex)
            return self.fw_error
        e = self.lib.djx_load_firmware(binpath.encode(), dis.encode(), b'')
        self.fw_error = e.decode() if e else None
        self.fw_name = None if e else os.path.basename(binpath)
        return self.fw_error

    # ---- whole radio: stock CPU firmware (RX65N simulator) + stock DSP firmware -----------------------------
    def load_radio(self, cpu_flash=None, cpu_option=None, eeprom=None, dsp_bin=None):
        """run the radio's own CPU firmware too: it boots and commands the DSP, draws the LCD, reads the keys"""
        cpu_flash = cpu_flash or _first_file(CPU_FLASH_CANDIDATES)
        cpu_option = cpu_option or _first_file(CPU_OPTION_CANDIDATES) or ''
        eeprom = os.path.abspath(eeprom or EEPROM_IMAGE)
        dsp_bin = dsp_bin or stock_bin()
        if not cpu_flash:
            self.fw_error = 'cpu_flash_FFE00000-FFFFFFFF.bin not found (set DJX_CPU_FLASH)'
            return self.fw_error
        if not dsp_bin:
            self.fw_error = 'DigRxx107.bin not found (set DJX_STOCK_BIN)'
            return self.fw_error
        try:
            if os.path.isfile(os.path.join(os.path.dirname(os.path.abspath(dsp_bin)), 'dspx_core_r.dis')):
                # a modular overlay firmware (DigRxx_dspx.bin): its core listing; the board finds dspx.map and the
                # overlay listings next to it
                dis = os.path.join(os.path.dirname(os.path.abspath(dsp_bin)), 'dspx_core_r.dis')
            else:
                dis = stock_listing(dsp_bin)
        except Exception as ex:
            self.fw_error = str(ex)
            return self.fw_error
        os.makedirs(os.path.dirname(eeprom), exist_ok=True)
        e = self.lib.djx_radio_load(cpu_flash.encode(), cpu_option.encode(), eeprom.encode(), dsp_bin.encode(),
                                    dis.encode())
        self.fw_error = e.decode() if e else None
        self.fw_name = None if e else 'CPU ' + os.path.basename(cpu_flash) + ' + ' + os.path.basename(dsp_bin)
        self.lcd_buf = np.zeros(320 * 240, np.uint32)
        return self.fw_error

    def radio_active(self): return bool(self.lib.djx_radio_active())

    def lcd(self):
        """(frame counter, 240 x 320 x 4 uint8 RGBA-ordered array) of the radio's LCD"""
        n = self.lib.djx_radio_lcd(self.lcd_buf.ctypes.data_as(C.POINTER(C.c_uint32)))
        v = self.lcd_buf.view(np.uint8).reshape(240, 320, 4)          # little endian 0xAARRGGBB -> B G R A
        return n, v[:, :, [2, 1, 0, 3]]

    def keys(self, mask): self.lib.djx_radio_keys(mask)

    def encoder(self, which, steps): self.lib.djx_radio_encoder(which, steps)

    def tuned(self):
        n = C.c_uint32()
        hz = self.lib.djx_radio_tuned(C.byref(n))
        return hz, n.value

    def radio_time(self): return self.lib.djx_radio_time()

    def radio_error(self): return self.lib.djx_radio_error().decode()

    def set_cpi(self, cpi): self.lib.djx_radio_set_cpi(int(cpi))

    def radio_save(self): self.lib.djx_radio_save()

    def radio_rf(self):
        """RF model: (antenna level dBm, channel SNR dB, AF amplifier on, volume DAC ch7*256+ch8 or -1)"""
        d, sn, on, vol = C.c_double(), C.c_double(), C.c_int(), C.c_int()
        self.lib.djx_radio_rf(C.byref(d), C.byref(sn), C.byref(on), C.byref(vol))
        return d.value, sn.value, bool(on.value), vol.value

    def set_rf_gain(self, db): self.lib.djx_radio_set_rf_gain(float(db))

    def usb_write(self, data: bytes):
        """PC link: bytes from the PC to the radio (its USB serial port)"""
        self.lib.djx_radio_usb_write(bytes(data), len(data))

    def usb_read(self, max_n=4096) -> bytes:
        """PC link: bytes the radio sent to the PC"""
        b = C.create_string_buffer(max_n)
        n = self.lib.djx_radio_usb_read(b, max_n)
        return b.raw[:n]

    def radio_wfm(self):
        """True while the radio uses its BK1068 broadcast-FM receiver (WFM channel model)"""
        return bool(self.lib.djx_radio_wfm())

    def is_stock(self): return bool(self.lib.djx_fw_is_stock())

    def set_if_invert(self, on): self.lib.djx_set_if_invert(1 if on else 0)

    def if_invert(self): return bool(self.lib.djx_get_if_invert())

    def dev_lines(self):
        out, buf = [], C.create_string_buffer(256)
        while self.lib.djx_dev_line(buf, 256):
            out.append(buf.value.decode('latin1'))
        return out

    def set_input_rate(self, fs_in, offset_hz):
        self.fs1 = self.lib.djx_init(fs_in, offset_hz)
        self.fs_in = fs_in
        self.set_if_model(self.if_model)

    def set_if_model(self, name):
        self.if_model = name
        m = IF_MODELS[name]
        if m is None:
            sos = np.zeros((0, 6))
        elif m[0] == 'cheby':
            sos = signal.cheby1(m[1], m[3], m[2], fs=self.fs1, output='sos')
        else:
            sos = signal.butter(m[1], m[2], fs=self.fs1, output='sos')
        arr = np.ascontiguousarray(sos, dtype=np.float64)
        self.lib.djx_set_if_sos(arr.ctypes.data_as(C.POINTER(C.c_double)), len(sos))

    def set_offset(self, hz): self.lib.djx_set_offset(hz)
    def set_mode(self, m):
        self.lib.djx_set_mode(m)
        fs1 = self.lib.djx_fs1()
        if fs1 != self.fs1:
            self.fs1 = fs1
            self.set_if_model(self.if_model)
    def set_volume(self, v): self.lib.djx_set_volume(v)
    def set_squelch(self, db): self.lib.djx_set_squelch_db(db)

    def process(self, iq):
        x = np.ascontiguousarray(iq, dtype=np.complex64).view(np.float32)
        return self.lib.djx_process(x.ctypes.data_as(C.POINTER(C.c_float)), len(iq))

    def read_audio(self, max_n=48000):
        buf = (C.c_int16 * max_n)()
        n = self.lib.djx_read_audio(buf, max_n)
        return np.frombuffer(buf, dtype=np.int16, count=n).copy()

    def status(self):
        s = Status()
        self.lib.djx_status(C.byref(s))
        return s

    def events(self):
        out, buf = [], C.create_string_buffer(4096)
        while self.lib.djx_event(buf, 4096):
            raw = buf.value
            try:
                out.append(raw.decode('utf-8'))
            except UnicodeDecodeError:
                out.append(raw.decode('cp932', 'replace'))
        return out

    def channel_iq(self, n=4096):
        pre = np.zeros(2 * n, np.float32); post = np.zeros(2 * n, np.float32)
        self.lib.djx_channel_iq(pre.ctypes.data_as(C.POINTER(C.c_float)), post.ctypes.data_as(C.POINTER(C.c_float)), n)
        return pre.view(np.complex64), post.view(np.complex64)

    def cpu(self, cmd):
        """send a command over the simulated CPU<->DSP I2C link; returns the DSP's reply lines"""
        self.lib.djx_cpu_send(cmd.encode())
        return self.cpu_lines()

    def cpu_lines(self):
        out, buf = [], C.create_string_buffer(256)
        while self.lib.djx_cpu_line(buf, 256):
            out.append(buf.value.decode('latin1'))
        return out


# ---- IQ sources -------------------------------------------------------------------------------------------
class WavSource:
    """SDR# / SDR++ IQ recording (RIFF or RF64, 2 channels, uint8 / int16 / float32). Center frequency from the
    file name (..._274331250Hz.wav) if present."""

    def __init__(self, path, loop=True):
        self.path, self.loop = path, loop
        f = open(path, 'rb')
        hdr = f.read(12)
        if hdr[:4] not in (b'RIFF', b'RF64') or hdr[8:12] != b'WAVE':
            raise ValueError('not a WAV file')
        fmt = None
        while True:
            h = f.read(8)
            if len(h) < 8:
                raise ValueError('no data chunk')
            cid, sz = h[:4], struct.unpack('<I', h[4:])[0]
            if cid == b'fmt ':
                fmt = struct.unpack('<HHIIHH', f.read(16))
                f.seek(sz - 16, 1)
            elif cid == b'data':
                self.data_off = f.tell()
                break
            else:
                f.seek(sz + (sz & 1), 1)
        self.f = f
        _, ch, self.rate, _, self.block_align, bits = fmt
        if ch != 2:
            raise ValueError('IQ WAV needs 2 channels')
        self.dtype = {8: np.uint8, 16: np.int16, 32: np.float32}[bits]
        self.bits = bits
        self.size = os.path.getsize(path) - self.data_off
        m = re.search(r'(\d{6,})Hz', os.path.basename(path))
        self.center = float(m.group(1)) if m else 0.0
        self.pos = 0

    def duration(self): return self.size / self.block_align / self.rate

    def seek(self, seconds):
        self.pos = int(seconds * self.rate) * self.block_align
        self.f.seek(self.data_off + self.pos)

    def read(self, n):
        raw = self.f.read(n * self.block_align)
        if len(raw) < n * self.block_align:
            if not self.loop:
                if not raw:
                    return None
            else:
                self.f.seek(self.data_off)
                raw += self.f.read(n * self.block_align - len(raw))
        self.pos = self.f.tell() - self.data_off
        x = np.frombuffer(raw[:len(raw) // self.block_align * self.block_align], dtype=self.dtype).astype(np.float32)
        if self.bits == 8:
            x = (x - 127.5) / 128
        elif self.bits == 16:
            x /= 32768
        return (x[0::2] + 1j * x[1::2]).astype(np.complex64)

    def tune(self, hz): return False      # recording: centre is fixed

    def close(self): self.f.close()


class RtlTcpSource:
    """rtl_tcp client (RTL-SDR). Commands: 1 freq, 2 rate, 3 gain mode, 4 gain (0.1 dB), 8 AGC."""

    def __init__(self, host='127.0.0.1', port=1234, rate=1024000, center=145.0e6, gain_db=None):
        self.s = socket.create_connection((host, port), timeout=5)
        self.s.settimeout(2)
        hdr = self._recv(12)
        if hdr[:4] != b'RTL0':
            raise ValueError('not an rtl_tcp server')
        self.rate, self.center = rate, center
        self._cmd(2, rate)
        self._cmd(1, int(center))
        if gain_db is None:
            self._cmd(3, 0); self._cmd(8, 1)
        else:
            self._cmd(3, 1); self._cmd(4, int(gain_db * 10))
        self.loop = False

    def _recv(self, n):
        b = b''
        while len(b) < n:
            c = self.s.recv(n - len(b))
            if not c:
                raise ConnectionError('rtl_tcp closed')
            b += c
        return b

    def _cmd(self, c, v): self.s.sendall(struct.pack('>BI', c, v & 0xFFFFFFFF))

    def tune(self, hz):
        self.center = hz
        self._cmd(1, int(hz))
        return True

    def read(self, n):
        raw = self._recv(2 * n)
        x = (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 127.5) / 128
        return (x[0::2] + 1j * x[1::2]).astype(np.complex64)

    def duration(self): return float('inf')

    def close(self): self.s.close()


# ---- RTL-SDR on USB (librtlsdr through ctypes) ---------------------------------------------------------------------
def find_rtlsdr_dll():
    """rtlsdr.dll: DJX_RTLSDR_DLL (file or folder), data/rtlsdr/, PATH, or the copy shipped with SDR# / SDR++ / rtl-sdr
    releases in the usual places (it needs libusb-1.0.dll next to it and the WinUSB driver on the dongle, e.g. Zadig)"""
    import glob
    name = 'librtlsdr.dylib' if sys.platform == 'darwin' else 'rtlsdr.dll'
    c = []
    e = os.environ.get('DJX_RTLSDR_DLL')
    if e:
        c.append(os.path.join(e, name) if os.path.isdir(e) else e)
    c.append(os.path.join(ROOT, 'data', 'rtlsdr', name))
    if sys.platform == 'darwin':
        # Homebrew (brew install librtlsdr)
        c += [os.path.join(d, 'lib', name) for d in ('/opt/homebrew', '/usr/local')]
    for d in os.environ.get('PATH', '').split(os.pathsep):
        if d:
            c.append(os.path.join(d, name))
    home = os.path.expanduser('~')
    for base in (os.path.join(home, 'Downloads'), os.path.join(home, 'Desktop'), home, 'C:/Program Files',
                 'C:/Program Files (x86)', 'C:/tools', 'C:/SDR'):
        for depth in ('*', '*/*', '*/*/*'):
            c += sorted(glob.glob(os.path.join(base, depth, name)))
    for f in c:
        if f and os.path.isfile(f):
            return os.path.abspath(f)
    return None


class RtlUsbSource:
    """RTL-SDR dongle on USB through librtlsdr (rtlsdr.dll). A reader thread streams 8-bit IQ into a ring buffer;
    read(n) returns complex64 like the other sources. gain_db None = tuner AGC."""

    def __init__(self, index=0, rate=2400000, center=145.0e6, gain_db=None, ppm=0, dll=None):
        import threading
        dll = dll or find_rtlsdr_dll()
        if not dll:
            raise OSError('rtlsdr library not found (set DJX_RTLSDR_DLL, copy rtlsdr.dll + libusb-1.0.dll to data/rtlsdr/, '
                          'or on macOS: brew install librtlsdr)')
        d = os.path.dirname(dll)
        if hasattr(os, 'add_dll_directory'):
            self._dlldir = os.add_dll_directory(d)
        L = self.L = C.CDLL(dll)
        L.rtlsdr_get_device_count.restype = C.c_uint32
        L.rtlsdr_open.argtypes = [C.POINTER(C.c_void_p), C.c_uint32]
        for f in ('rtlsdr_close', 'rtlsdr_reset_buffer', 'rtlsdr_get_tuner_type'):
            getattr(L, f).argtypes = [C.c_void_p]
        for f in ('rtlsdr_set_sample_rate', 'rtlsdr_set_center_freq'):
            getattr(L, f).argtypes = [C.c_void_p, C.c_uint32]
        for f in ('rtlsdr_set_tuner_gain_mode', 'rtlsdr_set_tuner_gain', 'rtlsdr_set_freq_correction',
                  'rtlsdr_set_agc_mode'):
            getattr(L, f).argtypes = [C.c_void_p, C.c_int]
        L.rtlsdr_get_tuner_gains.argtypes = [C.c_void_p, C.POINTER(C.c_int)]
        L.rtlsdr_read_sync.argtypes = [C.c_void_p, C.c_void_p, C.c_int, C.POINTER(C.c_int)]
        L.rtlsdr_get_device_name.restype = C.c_char_p
        L.rtlsdr_get_device_name.argtypes = [C.c_uint32]
        n = L.rtlsdr_get_device_count()
        if n == 0:
            raise OSError('no RTL-SDR on USB (is the WinUSB driver installed, e.g. with Zadig?)')
        if index >= n:
            raise OSError(f'RTL-SDR #{index} not present ({n} found)')
        self.name = L.rtlsdr_get_device_name(index).decode(errors='replace')
        self.dev = C.c_void_p()
        r = L.rtlsdr_open(C.byref(self.dev), index)
        if r != 0:
            raise OSError(f'rtlsdr_open failed ({r}): in use by another program?')
        self.rate, self.center = rate, center
        L.rtlsdr_set_sample_rate(self.dev, int(rate))
        L.rtlsdr_set_center_freq(self.dev, int(center))
        if ppm:
            L.rtlsdr_set_freq_correction(self.dev, int(ppm))
        self.set_gain(gain_db)
        L.rtlsdr_reset_buffer(self.dev)
        self.loop = False
        self.ring = bytearray()
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        self.err = None
        self.running = True
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()

    def set_gain(self, gain_db):
        L = self.L
        self.gain_db = gain_db
        if gain_db is None:
            L.rtlsdr_set_tuner_gain_mode(self.dev, 0)
        else:
            k = L.rtlsdr_get_tuner_gains(self.dev, None)
            g = (C.c_int * max(1, k))()
            L.rtlsdr_get_tuner_gains(self.dev, g)
            best = min(list(g)[:k] or [int(gain_db * 10)], key=lambda v: abs(v - gain_db * 10))
            L.rtlsdr_set_tuner_gain_mode(self.dev, 1)
            L.rtlsdr_set_tuner_gain(self.dev, int(best))

    def _reader(self):
        buf = (C.c_uint8 * 65536)()
        got = C.c_int()
        while self.running:
            r = self.L.rtlsdr_read_sync(self.dev, buf, len(buf), C.byref(got))
            if r != 0 or got.value <= 0:
                self.err = f'rtlsdr_read_sync failed ({r})'
                with self.cv:
                    self.cv.notify_all()
                break
            with self.cv:
                self.ring += bytes(buf)[:got.value]
                if len(self.ring) > 8 * self.rate:            # > 4 s behind: drop the oldest
                    del self.ring[:len(self.ring) - 2 * self.rate]
                self.cv.notify_all()

    def read(self, n):
        with self.cv:
            while len(self.ring) < 2 * n:
                if self.err or not self.running:
                    raise OSError(self.err or 'closed')
                self.cv.wait(1.0)
            raw = bytes(self.ring[:2 * n])
            del self.ring[:2 * n]
        x = (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 127.4) / 128
        return (x[0::2] + 1j * x[1::2]).astype(np.complex64)

    def tune(self, hz):
        self.center = hz
        self.L.rtlsdr_set_center_freq(self.dev, int(hz))
        with self.cv:
            self.ring.clear()                    # samples of the old frequency
        return True

    def duration(self): return float('inf')

    def close(self):
        self.running = False
        self.th.join(timeout=2)
        try:
            self.L.rtlsdr_close(self.dev)
        except Exception:
            pass


class PcLinkServer:
    """TCP bridge for the radio's PC link (USB serial): connect a terminal / tool to host:port, or a virtual COM
    port pair (e.g. com0com + com2tcp / hub4com) to use Windows software that expects the radio's COM port."""

    def __init__(self, core, port=7373, host='127.0.0.1', log=None):
        import threading
        self.core, self.log = core, log or (lambda s: None)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.conn = None
        self.running = True
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._pump, daemon=True).start()

    def _accept(self):
        while self.running:
            try:
                c, a = self.srv.accept()
            except OSError:
                return
            if self.conn:
                try:
                    self.conn.close()
                except OSError:
                    pass
            self.conn = c
            self.log(f'PC link: {a[0]}:{a[1]} connected')
            try:
                while self.running:
                    d = c.recv(4096)
                    if not d:
                        break
                    self.core.usb_write(d)
            except OSError:
                pass
            self.log('PC link: disconnected')
            if self.conn is c:
                self.conn = None

    def _pump(self):
        while self.running:
            d = self.core.usb_read()
            if d and self.conn:
                try:
                    self.conn.sendall(d)
                except OSError:
                    pass
            time.sleep(0.01)

    def close(self):
        self.running = False
        try:
            self.srv.close()
        except OSError:
            pass
