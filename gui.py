"""DJ-X100 emulator - GUI.

  python gui.py [iq.wav] [--rtl host:port | --usb [index] [--usb-gain dB]] [--radio-freq MHz]

The radio's own CPU firmware (cpu_flash_FFE00000-FFFFFFFF.bin, RX65N) runs on the RX simulator (rx/) and its own DSP
firmware (DigRxx107.bin, C5517) on the C55x simulator (c55/). The left panel shows the radio's real LCD (GLCDC frame
buffer); the buttons / keyboard press the real keys (key matrix, dial and volume encoders). The CPU firmware boots,
commands the DSP over the simulated I2C link and tunes the receiver; the SDR follows the frequency it tunes to.
Settings live in data/radio/eeprom.bin (saved on close).
  keyboard: digits . ; Left/Right or wheel on the LCD = dial; Up/Down or Shift+wheel = volume;
            F=FUNC V=V/M S=SCAN M=MODE T=STEP Enter=SET Esc=MONI/SQL
Right side: IQ source, IF filter model, channel spectrum before / after the IF filter, DSP audio, CPU <-> DSP log.
"""
import argparse
import os
import queue
import sys
import threading
import time
import tkinter as tk
import wave
from tkinter import filedialog, ttk

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import djxcore as D  # noqa: E402

try:
    import sounddevice as sd
except Exception:      # audio is optional
    sd = None

AUDIO_FS = 24000
AUDIO_MAX = AUDIO_FS * 15 // 100       # output buffer above 150 ms: trimmed to AUDIO_KEEP (latency bound)
AUDIO_KEEP = AUDIO_FS * 6 // 100
BODY = '#2b2d30'

class Engine:
    """Source -> core -> audio. Runs in a worker thread."""

    def __init__(self):
        self.core = None
        self.src = None
        self.freq = 145.0e6
        self.running = False
        self.realtime = True
        self.audio_q = queue.Queue(maxsize=200)
        self.last_iq = None
        self.rec = None
        self.lock = threading.Lock()
        self.rt_factor = 0.0
        self.last_audio = np.zeros(0, np.int16)
        self.err = ''

    def open(self, src):
        with self.lock:
            if self.src:
                self.src.close()
            self.src = src
            self.core.set_input_rate(src.rate, self.freq - src.center)
            self.retune(self.freq)

    def retune(self, hz):
        self.freq = hz
        if not self.src:
            return
        off = hz - self.src.center
        lim = self.src.rate * 0.45
        if abs(off) > lim and self.src.tune(hz - self.src.rate / 4):   # live SDR: move the centre
            off = hz - self.src.center
        self.core.set_offset(off)

    def offset_ok(self):
        return self.src is not None and abs(self.freq - self.src.center) < self.src.rate * 0.48

    def loop(self):
        blk_t = 0.02
        t_next = None                          # WAV real-time pacing: absolute deadline of the next block
        while self.running:
            if not self.src:
                if self.core is not None:            # the radio runs without SDR input
                    a = self.core.read_audio()
                    if len(a):
                        self.last_audio = a
                        if self.rec:
                            self.rec.writeframes(a.tobytes())
                        try:
                            self.audio_q.put_nowait(a)
                        except queue.Full:
                            pass
                time.sleep(0.02)
                continue
            t0 = time.perf_counter()
            with self.lock:
                n = int(self.src.rate * blk_t)
                try:
                    x = self.src.read(n)
                except Exception as e:
                    self.err = str(e)
                    x = None
                if x is None:
                    self.src = None
                    continue
                # keep the latest 8192 IQ samples for the band display (any input rate)
                self.last_iq = x if self.last_iq is None or len(x) >= 8192 else                     np.concatenate([self.last_iq, x])[-8192:]
                self.core.process(x)
                a = self.core.read_audio()
                if len(a):
                    self.last_audio = a
                    if self.rec:
                        self.rec.writeframes(a.tobytes())
            if len(a):
                try:
                    self.audio_q.put_nowait(a)
                except queue.Full:
                    pass
            dt = time.perf_counter() - t0
            self.rt_factor = blk_t / dt if dt > 0 else 0
            if self.realtime and isinstance(self.src, D.WavSource):
                # pace by an absolute clock: sleeping "block - processing time - 1 ms" ran the file ~5 % fast, and
                # the surplus audio piled up in the output buffer (up to 1 s late)
                now = time.perf_counter()
                t_next = now + blk_t if t_next is None or t_next < now - 0.2 else t_next + blk_t
                time.sleep(max(0.0, t_next - time.perf_counter()))
            else:
                t_next = None

    def start(self):
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()


class Audio:
    def __init__(self, engine):
        self.e = engine
        self.buf = np.zeros(0, np.int16)
        self.stream = None
        if sd is None:
            return
        try:
            self.stream = sd.OutputStream(samplerate=AUDIO_FS, channels=1, dtype='int16', blocksize=480,
                                          callback=self.cb, latency=0.08)
            self.stream.start()
        except Exception as ex:
            print('audio disabled:', ex)
            self.stream = None

    def cb(self, out, frames, t, status):
        while True:                           # take everything queued (so the latency bound sees it all)
            try:
                self.buf = np.concatenate([self.buf, self.e.audio_q.get_nowait()])
            except queue.Empty:
                break
        if len(self.buf) > AUDIO_MAX:         # too much latency (source / sound card clock drift): drop
            self.buf = self.buf[-AUDIO_KEEP:]
        n = min(frames, len(self.buf))
        out[:n, 0] = self.buf[:n]
        out[n:, 0] = 0
        self.buf = self.buf[n:]



class App:
    def __init__(self, root, args):
        self.root = root
        self.e = Engine()
        self.radio_freq = args.radio_freq          # typed on the keypad once the radio has booted
        self.pc_port = args.pc_port
        self.key_script = []
        self.lcd_scale = args.lcd_scale
        self.kmask = 0
        self.tuned_n = -1
        root.title('DJ-X100 emulator')
        root.configure(bg=BODY)
        self.build_radio_panel()
        self.build_analysis()
        self.start_radio()
        self.e.start()
        self.audio = Audio(self.e)
        if args.wav:
            self.open_wav(args.wav)
        elif args.rtl:
            self.connect_rtl(args.rtl)
        elif args.usb is not None:
            if args.usb_gain is not None:
                self.usb_gain.set(str(args.usb_gain))
            self.connect_usb(args.usb)
        root.bind('<Key>', self.on_key)
        root.bind('<KeyRelease>', self.on_key_release)
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.tick()

    # ---------------------------------------------------------------- the radio (CPU + DSP firmware)
    RADIO_KEYMAP = {'F': 'FUNC', 'V': 'V/M', 'S': 'SCAN', 'M': 'MODE', 'T': 'STEP', 'Return': 'SET',
                    'Escape': 'MONI/SQL', 'period': '.'}

    def start_radio(self):
        self.e.core = D.Core(2.4e6, 0.0)
        err = self.e.core.load_radio()
        if err:
            self.log('cannot start the radio: ' + err)
            self.log('put cpu_flash_FFE00000-FFFFFFFF.bin and DigRxx107.bin from the official updater in firmware/ '
                     '(or set DJX_FW_DIR / DJX_CPU_FLASH / DJX_STOCK_BIN) - see README.md')
            return
        self.e.core.set_mode(D.MODE_FM)
        self.e.core.set_volume(1.0)
        self.log(f'radio: {self.e.core.fw_name} - the CPU firmware boots, releases the DSP reset and sets it up; '
                 f'settings: {os.path.relpath(D.EEPROM_IMAGE)}')
        self.pc_link = None
        if self.pc_port:
            try:
                self.pc_link = D.PcLinkServer(self.e.core, self.pc_port, log=lambda m: self.root.after(0, self.log, m))
                self.log(f'PC link (the radio' + "'" + f's USB serial port): tcp 127.0.0.1:{self.pc_link.port} - e.g. send '
                         f'"AL~WHO" + CR LF')
            except OSError as ex:
                self.log(f'PC link: port {self.pc_port}: {ex}')
        self.lcd_loop()

    # the DJ-X100 keyboard (manual p.26): 4 x 4 below the LCD, main label / FUNC label; this is also the wiring of the
    # key matrix (each column of the keyboard is one drive line: 1 4 7 0 / 2 5 8 . / 3 6 9 SET / MODE STEP V/M SCAN)
    RADIO_KEYPAD = [[('1', ''), ('2', ''), ('3', ''), ('MODE', 'CONFIG')],
                    [('4', ''), ('5', ''), ('6', ''), ('STEP', 'MHz')],
                    [('7', ''), ('8', ''), ('9', ''), ('V/M', 'MW')],
                    [('0', ''), ('.', 'SKIP'), ('SET', '\u2713'), ('SCAN', 'PROG')]]

    def build_radio_panel(self):
        f = tk.Frame(self.root, bg=BODY, padx=14, pady=14)
        f.grid(row=0, column=0, sticky='ns')
        # top: volume ring and dial (encoders)
        top = tk.Frame(f, bg=BODY)
        top.grid(row=0, column=0, columnspan=2, sticky='e', pady=(0, 6))
        eb = dict(bg='#2f4f6b', fg='white', activebackground='#41698d', relief='raised', bd=2, width=4,
                  font=('Segoe UI', 10, 'bold'))
        tk.Label(top, text='VOL ring', bg=BODY, fg='#aaa', font=('Segoe UI', 8)).pack(side='left', padx=(0, 2))
        tk.Button(top, text='\u2212', command=lambda: self.radio_encoder(1, -1), **eb).pack(side='left')
        tk.Button(top, text='+', command=lambda: self.radio_encoder(1, 1), **eb).pack(side='left', padx=(2, 12))
        tk.Label(top, text='DIAL', bg=BODY, fg='#aaa', font=('Segoe UI', 8)).pack(side='left', padx=(0, 2))
        tk.Button(top, text='\u25bc', command=lambda: self.radio_encoder(0, -1), **eb).pack(side='left')
        tk.Button(top, text='\u25b2', command=lambda: self.radio_encoder(0, 1), **eb).pack(side='left', padx=(2, 0))
        # side keys (left side of the radio): MONI (SQL), FUNC (LOCK), POWER
        side = tk.Frame(f, bg=BODY)
        side.grid(row=1, column=0, sticky='n', padx=(0, 8))
        sb = dict(width=6, bg='#3a3d42', fg='white', activebackground='#5b6067', relief='raised', bd=2,
                  font=('Segoe UI', 9, 'bold'))
        for name, sub in (('MONI/SQL', 'MONI\n(SQL)'), ('FUNC', 'FUNC\n(LOCK)')):
            b = tk.Button(side, text=sub, height=2, **sb)
            b.bind('<ButtonPress-1>', lambda ev, n=name: self.radio_key(n, True))
            b.bind('<ButtonRelease-1>', lambda ev, n=name: self.radio_key(n, False))
            b.pack(pady=(26, 0) if name == 'MONI/SQL' else (10, 0))
        tk.Label(side, text='POWER\n(always on)', bg=BODY, fg='#777', font=('Segoe UI', 7)).pack(pady=(14, 0))
        body = tk.Frame(f, bg='#1b1c1e', padx=10, pady=10, highlightthickness=1, highlightbackground='#111')
        body.grid(row=1, column=1)
        w, h = int(320 * self.lcd_scale), int(240 * self.lcd_scale)
        self.rlcd_blank = tk.PhotoImage(width=w, height=h)      # pixel size until the firmware draws the LCD
        self.rlcd = tk.Label(body, bg='black', image=self.rlcd_blank, width=w, height=h, bd=0, highlightthickness=3,
                             highlightbackground='#111')
        self.rlcd.grid(row=0, column=0, pady=(0, 10))
        self.rlcd.bind('<MouseWheel>', self.on_radio_wheel)
        self.rlcd_img = None
        kp = tk.Frame(body, bg='#c9ccd1', padx=6, pady=6)
        kp.grid(row=1, column=0)
        for r, row in enumerate(self.RADIO_KEYPAD):
            for c, (name, sub) in enumerate(row):
                cell = tk.Frame(kp, bg='#c9ccd1', width=62, height=50)
                cell.grid(row=r, column=c, padx=3, pady=3)
                cell.pack_propagate(False)
                fn = c == 3 or sub
                txt = (sub + '\n' if sub else '') + ('\u00b7' if name == '.' else name)
                b = tk.Button(cell, text=txt,
                              bg='#f4f4f4', fg='#111', activebackground='#dcdcdc', relief='raised', bd=2,
                              font=('Segoe UI', 8 if fn else 13, 'bold'))
                b.bind('<ButtonPress-1>', lambda ev, n=name: self.radio_key(n, True))
                b.bind('<ButtonRelease-1>', lambda ev, n=name: self.radio_key(n, False))
                b.pack(fill='both', expand=True)
        tk.Label(f, text='real CPU firmware: hold buttons like keys. keyboard: 0-9 . F=FUNC V=V/M S=SCAN M=MODE '
                         'T=STEP Enter=SET Esc=MONI\n\u2190/\u2192 or wheel on the LCD = dial, \u2191/\u2193 or '
                         'Shift+wheel = volume ring.  SQL: FUNC, MONI, dial', bg=BODY, fg='#888',
                 font=('Segoe UI', 8), justify='left').grid(row=2, column=0, columnspan=2, pady=(10, 0), sticky='w')
        rf = tk.Frame(f, bg=BODY)
        rf.grid(row=3, column=0, columnspan=2, sticky='w', pady=(6, 0))
        tk.Label(rf, text='antenna level offset dB', bg=BODY, fg='#bbb', font=('Segoe UI', 8)).pack(side='left')
        self.rfgain_var = tk.DoubleVar(value=0.0)
        sp = tk.Spinbox(rf, from_=-40, to=60, increment=5, width=5, textvariable=self.rfgain_var,
                        command=self.on_rf_gain)
        sp.pack(side='left', padx=4)
        sp.bind('<Return>', lambda ev: self.on_rf_gain())
        self.radio_info = tk.Label(f, text='', bg=BODY, fg='#bbb', font=('Consolas', 9), justify='left')
        self.radio_info.grid(row=4, column=0, columnspan=2, sticky='w', pady=(6, 0))

    def on_rf_gain(self):
        try:
            v = float(self.rfgain_var.get())
        except (tk.TclError, ValueError):
            return
        if self.e.core and self.e.core.radio_active():
            self.e.core.set_rf_gain(v)

    def radio_key(self, name, down):
        code = D.KEY_CODES[name]
        self.kmask = (self.kmask | (1 << code)) if down else (self.kmask & ~(1 << code))
        if self.e.core and self.e.core.radio_active():
            self.e.core.keys(self.kmask)

    def radio_encoder(self, which, steps):
        if self.e.core and self.e.core.radio_active():
            self.e.core.encoder(which, steps)

    def on_radio_wheel(self, ev):
        shift = bool(ev.state & 0x0001)
        self.radio_encoder(1 if shift else 0, 1 if ev.delta > 0 else -1)

    def radio_keysym(self, ev):
        k = ev.keysym
        if ev.char and ev.char in '0123456789':
            return ev.char
        if k in self.RADIO_KEYMAP:
            return self.RADIO_KEYMAP[k]
        if len(k) == 1 and k.upper() in self.RADIO_KEYMAP:
            return self.RADIO_KEYMAP[k.upper()]
        return None

    def on_key_release(self, ev):
        if isinstance(ev.widget, (tk.Entry, ttk.Entry, ttk.Combobox)):
            return
        name = self.radio_keysym(ev)
        if name:
            self.radio_key(name, False)

    def lcd_loop(self):
        c = self.e.core
        if c and c.radio_active():
            self.key_script_step()
            try:
                from PIL import Image, ImageTk
                n, img = c.lcd()
                if n != getattr(self, 'lcd_n', -1):
                    self.lcd_n = n
                    im = Image.fromarray(img[:, :, :3].copy())
                    if self.lcd_scale != 1:
                        im = im.resize((int(320 * self.lcd_scale), int(240 * self.lcd_scale)),
                                       Image.NEAREST if float(self.lcd_scale).is_integer() else Image.BILINEAR)
                    self.rlcd_img = ImageTk.PhotoImage(im)
                    self.rlcd.configure(image=self.rlcd_img, width=im.width, height=im.height)
            except ImportError:
                self.log('the radio LCD needs Pillow (pip install pillow)')
                return
        self.root.after(40, self.lcd_loop)

    def type_keys(self, names):
        """press and release keys one after another, timed in radio time (0.15 s down, 0.25 s up)"""
        t = self.e.core.radio_time() + 0.1
        for n in names:
            self.key_script += [(t, n, True), (t + 0.15, n, False)]
            t += 0.4

    def key_script_step(self):
        if not self.key_script:
            return
        rt = self.e.core.radio_time()
        while self.key_script and self.key_script[0][0] <= rt:
            _, n, down = self.key_script.pop(0)
            self.radio_key(n, down)

    def radio_tick(self):
        """the CPU firmware tunes the receiver: move the SDR to its frequency"""
        c = self.e.core
        if self.radio_freq and c.radio_time() > 8:
            f, self.radio_freq = self.radio_freq, None
            self.log(f'typing {f} MHz + SET on the keypad')
            self.type_keys([ch for ch in f] + ['SET'])
        hz, n = c.tuned()
        if n != self.tuned_n and hz:
            self.tuned_n = n
            if hz != getattr(self, 'tuned_hz', None):
                self.tuned_hz = hz
                self.log(f'radio tunes {hz / 1e6:.6f} MHz')
            self.e.retune(float(hz))
            src = self.e.src
            if src is not None and not isinstance(src, D.WavSource):
                t = self.src_lbl.cget('text')
                i = t.find('centre ')
                if i >= 0:
                    j = t.find(' MHz', i)
                    self.src_lbl.config(text=t[:i] + f'centre {src.center / 1e6:.4f}' + t[j:])
        err = c.radio_error()
        dbm, snr, af, vol = c.radio_rf()
        s = max(0.0, (dbm + 127) / 6)
        sm = f'S{min(9, int(s))}' + (f'+{int((dbm + 73) // 10 * 10)}' if dbm > -73 else '')
        src = 'no antenna signal' if self.e.src is None else f'{dbm:6.1f} dBm ({sm})  SNR {snr:5.1f} dB'
        rx = 'BK1068 broadcast FM (WFM)' if c.radio_wfm() else 'main receiver'
        self.radio_info.config(text=f'radio time {c.radio_time():8.1f} s   tuned {hz / 1e6:11.6f} MHz  {rx}\n'
                                    f'RF {src}   AF amp {"ON " if af else "off"}'
                                    + (f'  vol DAC {vol >> 8},{vol & 255}' if vol >= 0 else '')
                                    + (f'\nCPU STOPPED: {err}' if err else ''))

    def on_close(self):
        if self.e.core and self.e.core.radio_active():
            self.e.core.radio_save()           # the radio's settings (EEPROM image)
        self.e.running = False
        if self.e.rec:
            self.e.rec.close()
        if getattr(self, 'pc_link', None):
            self.pc_link.close()
        if self.e.src is not None:
            try:
                with self.e.lock:
                    self.e.src.close()         # releases a USB dongle
            except Exception:
                pass
        self.root.destroy()

    def k_rec(self):
        if self.e.rec:
            self.e.rec.close()
            self.e.rec = None
            self.log(f'recording stopped')
        else:
            fn = time.strftime('djx_%Y%m%d_%H%M%S.wav')
            w = wave.open(fn, 'wb')
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(AUDIO_FS)
            self.e.rec = w
            self.log(f'recording to {os.path.abspath(fn)}')

    def set_freq(self, hz):
        self.log('the CPU firmware sets the frequency - use the keypad / dial')

    def on_key(self, ev):
        if isinstance(ev.widget, (tk.Entry, ttk.Entry, ttk.Combobox)):
            return
        k = ev.keysym
        name = self.radio_keysym(ev)
        if name:
            self.radio_key(name, True)
        elif k in ('Left', 'Right'):
            self.radio_encoder(0, 1 if k == 'Right' else -1)
        elif k in ('Up', 'Down'):
            self.radio_encoder(1, 1 if k == 'Up' else -1)

    # ---------------------------------------------------------------- analysis side
    def build_analysis(self):
        f = tk.Frame(self.root, padx=10, pady=10)
        f.grid(row=0, column=1, sticky='nsew')
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        src = ttk.LabelFrame(f, text='IQ source')
        src.grid(row=0, column=0, columnspan=2, sticky='ew')
        ttk.Button(src, text='Open IQ WAV…', command=self.open_wav_dialog).grid(row=0, column=0, padx=4, pady=4)
        self.rtl_var = tk.StringVar(value='127.0.0.1:1234')
        ttk.Entry(src, textvariable=self.rtl_var, width=18).grid(row=0, column=1)
        ttk.Button(src, text='rtl_tcp connect', command=lambda: self.connect_rtl(self.rtl_var.get())).grid(row=0, column=2, padx=4)
        usb = ttk.Frame(src)
        usb.grid(row=0, column=4, padx=(10, 4))
        ttk.Label(usb, text='USB RTL-SDR #').pack(side='left')
        self.usb_idx = tk.StringVar(value='0')
        ttk.Entry(usb, textvariable=self.usb_idx, width=2).pack(side='left')
        ttk.Label(usb, text='gain dB').pack(side='left', padx=(4, 0))
        self.usb_gain = tk.StringVar(value='auto')
        cg = ttk.Combobox(usb, textvariable=self.usb_gain, width=5,
                          values=['auto', '0', '10', '20', '30', '40', '49.6'])
        cg.pack(side='left')
        cg.bind('<<ComboboxSelected>>', lambda ev: self.usb_set_gain())
        cg.bind('<Return>', lambda ev: self.usb_set_gain())
        ttk.Button(usb, text='connect', command=lambda: self.connect_usb(int(self.usb_idx.get() or 0))).pack(
            side='left', padx=(4, 0))
        self.rt_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(src, text='real time', variable=self.rt_var,
                        command=lambda: setattr(self.e, 'realtime', self.rt_var.get())).grid(row=0, column=3, padx=6)
        self.src_lbl = ttk.Label(src, text='no source')
        self.src_lbl.grid(row=1, column=0, columnspan=4, sticky='w', padx=4)
        self.pos = ttk.Scale(src, from_=0, to=1, orient='horizontal', command=self.on_seek)
        self.pos.grid(row=2, column=0, columnspan=4, sticky='ew', padx=4, pady=2)
        src.columnconfigure(3, weight=1)

        ctl = ttk.Frame(f)
        ctl.grid(row=1, column=0, columnspan=2, sticky='ew', pady=6)
        ttk.Label(ctl, text='IF filter model').grid(row=0, column=0, padx=(0, 2))
        self.if_var = tk.StringVar(value=list(D.IF_MODELS)[0])
        cb = ttk.Combobox(ctl, textvariable=self.if_var, values=list(D.IF_MODELS), width=26, state='readonly')
        cb.grid(row=0, column=1)
        cb.bind('<<ComboboxSelected>>', lambda ev: self.e.core and self.e.core.set_if_model(self.if_var.get()))
        self.inv_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctl, text='IF spectrum inverted (as in the radio)', variable=self.inv_var,
                        command=lambda: self.e.core and self.e.core.set_if_invert(self.inv_var.get())).grid(
            row=0, column=2, padx=(12, 0))
        self.rec_btn = ttk.Button(ctl, text='REC audio', command=self.k_rec)
        self.rec_btn.grid(row=0, column=3, padx=(12, 0))

        sf = tk.Frame(f)
        sf.grid(row=2, column=0, sticky='nsew')
        self.spec = tk.Canvas(sf, width=620, height=160, bg='#0d1117', highlightthickness=0)
        self.spec.pack(fill='x')
        self.wf = tk.Canvas(sf, width=620, height=60, bg='#000', highlightthickness=0)
        self.wf.pack(fill='x')
        self.spec.bind('<Button-1>', self.on_spec_click)
        self.scope = tk.Canvas(f, width=220, height=220, bg='#0d1117', highlightthickness=0)
        self.scope.grid(row=2, column=1, padx=(8, 0))
        self.band = tk.Canvas(f, width=850, height=90, bg='#0d1117', highlightthickness=0)
        self.band.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(6, 0))
        self.band.bind('<Button-1>', self.on_band_click)

        self.logw = tk.Text(f, height=12, width=110, bg='#111', fg='#d0d0d0', font=('Consolas', 9), wrap='none')
        self.logw.grid(row=4, column=0, columnspan=2, sticky='nsew', pady=(8, 0))
        f.rowconfigure(4, weight=1)
        f.columnconfigure(0, weight=1)
        self.stat = ttk.Label(f, text='', font=('Consolas', 9))
        self.stat.grid(row=5, column=0, columnspan=2, sticky='w')


    def log(self, s):
        self.logw.insert('end', s + '\n')
        if os.environ.get('DJX_LOGSTDOUT'):        # testing: the log on stdout too
            print(s, flush=True)
        if int(self.logw.index('end-1c').split('.')[0]) > 600:
            self.logw.delete('1.0', '100.0')
        self.logw.see('end')

    def open_wav_dialog(self):
        fn = filedialog.askopenfilename(filetypes=[('IQ WAV', '*.wav'), ('all', '*.*')])
        if fn:
            self.open_wav(fn)

    def open_wav(self, fn):
        try:
            s = D.WavSource(fn)
        except Exception as ex:
            self.log(f'open failed: {ex}')
            return
        self.e.open(s)
        self.e.core.set_if_model(self.if_var.get())
        self.src_lbl.config(text=f'{os.path.basename(fn)}  {s.rate / 1e6:.3f} MS/s  centre {s.center / 1e6:.4f} MHz  '
                                 f'{s.duration():.0f} s  (channel rate {self.e.core.fs1:.0f} S/s)')
        self.log(f'opened {fn}')

    def connect_rtl(self, hp):
        host, _, port = hp.partition(':')
        try:
            s = D.RtlTcpSource(host, int(port or 1234), rate=2400000, center=self.e.freq - 600000)
        except Exception as ex:
            self.log(f'rtl_tcp: {ex}')
            return
        self.e.open(s)
        self.e.core.set_if_model(self.if_var.get())
        self.src_lbl.config(text=f'rtl_tcp {hp}  2.4 MS/s  centre {s.center / 1e6:.4f} MHz')
        self.log(f'connected rtl_tcp {hp}')

    def usb_gain_db(self):
        g = self.usb_gain.get().strip().lower()
        return None if g in ('', 'auto', 'agc') else float(g)

    def connect_usb(self, index=0):
        """RTL-SDR on USB (librtlsdr): the emulated radio's antenna"""
        try:
            s = D.RtlUsbSource(index, rate=2400000, center=self.e.freq - 600000, gain_db=self.usb_gain_db())
        except Exception as ex:
            self.log(f'USB RTL-SDR: {ex}')
            return
        self.e.open(s)
        self.e.core.set_if_model(self.if_var.get())
        self.src_lbl.config(text=f'USB RTL-SDR #{index} {s.name}  2.4 MS/s  centre {s.center / 1e6:.4f} MHz  '
                                 f'gain {self.usb_gain.get()}')
        self.log(f'connected USB RTL-SDR #{index} ({s.name})')

    def usb_set_gain(self):
        s = self.e.src
        if isinstance(s, D.RtlUsbSource):
            try:
                s.set_gain(self.usb_gain_db())
                self.log(f'RTL-SDR gain {self.usb_gain.get()}')
            except Exception as ex:
                self.log(f'RTL-SDR gain: {ex}')

    def on_seek(self, v):
        s = self.e.src
        if isinstance(s, D.WavSource) and abs(float(v) * s.duration() - s.pos / s.block_align / s.rate) > 1:
            with self.e.lock:
                s.seek(float(v) * s.duration())

    def on_spec_click(self, ev):
        w = int(self.spec.winfo_width())
        fs1 = self.e.core.fs1 if self.e.core else 96000
        df = (ev.x / w - 0.5) * fs1
        self.set_freq(round((self.e.freq + df) / 6250) * 6250)

    def on_band_click(self, ev):
        s = self.e.src
        if not s:
            return
        w = int(self.band.winfo_width())
        hz = s.center + (ev.x / w - 0.5) * s.rate
        self.set_freq(round(hz / 6250) * 6250)

    def draw_spectrum(self):
        c = self.spec
        c.delete('all')
        w, h = int(c.winfo_width()), int(c.winfo_height())
        if not self.e.core:
            return
        pre, post = self.e.core.channel_iq(4096)
        fs1 = self.e.core.fs1
        win = np.hanning(len(pre)).astype(np.float32)
        specs = []
        for data in (pre, post):
            sp = np.fft.fftshift(np.abs(np.fft.fft(data * win)) ** 2)
            sp = np.convolve(10 * np.log10(sp + 1e-20), np.ones(9) / 9, mode='same')
            specs.append(sp)
        top = max(specs[0].max(), specs[1].max()) + 3
        self.spec_top = top if not hasattr(self, 'spec_top') else 0.8 * self.spec_top + 0.2 * top
        rng = 50.0
        for sp, col in zip(specs[::-1], ('#ffd166', '#6fa8ff')):
            x = np.linspace(0, w, len(sp))
            y = np.clip((self.spec_top - sp) / rng * h, 0, h)
            c.create_line(*np.column_stack([x[::4], y[::4]]).ravel().tolist(), fill=col, width=1)
        # waterfall line of the filtered channel (kept as an image strip under the spectrum)
        self.wf_rows = getattr(self, 'wf_rows', [])
        row = np.clip((specs[0][::8] - (self.spec_top - rng)) / rng, 0, 1)
        self.wf_rows.insert(0, row)
        del self.wf_rows[60:]
        for khz in (-12.5, -7.5, 7.5, 12.5):     # channel edges and nominal crystal filter edges
            xx = w / 2 + khz * 1000 / fs1 * w
            c.create_line(xx, 0, xx, h, fill='#444' if abs(khz) > 10 else '#665', dash=(2, 3))
        c.create_line(w / 2, 0, w / 2, h, fill='#aa3333')
        c.create_text(6, 4, anchor='nw', fill='#aaa', font=('Consolas', 8),
                      text=f'channel {self.e.freq / 1e6:.4f} MHz  ±{fs1 / 2e3:.0f} kHz   blue: antenna (before IF filter)'
                           f'   yellow: after crystal filter   50 dB range')
        self.draw_waterfall()

    def draw_waterfall(self):
        c = self.wf
        c.delete('all')
        rows = getattr(self, 'wf_rows', [])
        if not rows:
            return
        from PIL import Image, ImageTk
        a = np.array(rows)
        rgb = np.stack([np.clip(a * 3 - 1.2, 0, 1), np.clip(a * 2.2 - 0.5, 0, 1), np.clip(a * 1.5, 0, 1) * 0.9], -1)
        img = Image.fromarray((rgb * 255).astype(np.uint8)).resize((int(c.winfo_width()), int(c.winfo_height())))
        self.wf_img = ImageTk.PhotoImage(img)
        c.create_image(0, 0, anchor='nw', image=self.wf_img)

    def draw_band(self):
        c = self.band
        c.delete('all')
        x = self.e.last_iq
        s = self.e.src
        if x is None or s is None or len(x) < 4096:
            return
        w, h = int(c.winfo_width()), int(c.winfo_height())
        n = 8192 if len(x) >= 8192 else 4096
        x = x[-n:]
        sp = np.fft.fftshift(np.abs(np.fft.fft(x[:n] * np.hanning(n))) ** 2)
        sp = 10 * np.log10(sp.reshape(-1, n // 512).mean(axis=1) + 1e-20)
        xs = np.linspace(0, w, len(sp))
        ys = np.clip(h - (sp - sp.min()) / (sp.max() - sp.min() + 1e-9) * (h - 12), 0, h)
        c.create_line(*np.column_stack([xs, ys]).ravel().tolist(), fill='#58a6ff')
        mx = w / 2 + (self.e.freq - s.center) / s.rate * w
        c.create_line(mx, 0, mx, h, fill='#ff5555')
        c.create_text(6, 4, anchor='nw', fill='#aaa', font=('Consolas', 8),
                      text=f'IQ band {s.center / 1e6:.4f} MHz ±{s.rate / 2e6:.2f} MHz  (click to tune)')

    def draw_scope(self):
        c = self.scope
        c.delete('all')
        w = int(c.winfo_width())
        c.create_line(0, w / 2, w, w / 2, fill='#222')
        a = self.e.last_audio.astype(np.float32)
        if len(a) > 8:
            a = a[-480:]
            xs = np.linspace(0, w, len(a))
            ys = w / 2 - np.clip(a / 8000.0, -1, 1) * (w * 0.45)
            c.create_line(*np.column_stack([xs, ys]).ravel().tolist(), fill='#7ee787')
        c.create_text(4, 4, anchor='nw', fill='#aaa', font=('Consolas', 8), text='audio out (20 ms)')

    def tick(self):
        c = self.e.core
        st = c.status() if c else None
        if c and c.radio_active():
            self.radio_tick()
        self.draw_spectrum()
        self.draw_band()
        self.draw_scope()
        if c:
            for ev in c.events():                    # CPU <-> DSP link lines, DSP boots
                self.log(ev)
            for line in c.dev_lines():
                self.log(f'DSP -> codec (I2C {line})')
        s = self.e.src
        if isinstance(s, D.WavSource):
            self.pos.set(s.pos / s.block_align / s.rate / max(1, s.duration()))
        self.rec_btn.config(text='STOP rec' if self.e.rec else 'REC audio')
        if st and st.backend == 1:
            dsp = (f'{c.fw_name}   DSP load {st.dsp_load:4.1f}% (peak {st.dsp_load_peak:5.1f}%)  '
                   f'{st.mips:5.1f} MIPS emulated  lag {st.fw_queue * 10:3d} ms  UART {st.uart_tx}/{st.uart_rx} B')
            if st.halted:
                dsp += '  HALTED: ' + c.lib.djx_fw_error().decode()
            self.stat.config(text=f'{dsp}\nrt x{self.e.rt_factor:4.1f}  ADC rms {st.adc_rms:5.0f} LSB  '
                                  f'AGC {st.agc_db:5.1f} dB  clip {st.adc_clip}  AMBE packets {st.frames_to_ambe} '
                                  f'(decoded {st.speech_packets})  IF: {self.if_var.get()}  '
                                  f'audio: {"on" if self.audio.stream else "none"}  {self.e.err}')
        self.root.after(100, self.tick)


def main():
    ap = argparse.ArgumentParser(description='DJ-X100 emulator: the radio\'s CPU + DSP firmware on simulators')
    ap.add_argument('wav', nargs='?', help='SDR# / SDR++ IQ WAV (centre frequency in the file name)')
    ap.add_argument('--rtl', help='rtl_tcp server host:port')
    ap.add_argument('--usb', type=int, nargs='?', const=0, help='RTL-SDR on USB (device index, default 0)')
    ap.add_argument('--usb-gain', type=float, help='RTL-SDR tuner gain dB (default: tuner AGC)')
    ap.add_argument('--pc-port', type=int, default=7373,
                    help='TCP port of the radio PC link (its USB serial port); 0 = off')
    ap.add_argument('--seek', type=float, default=0)
    ap.add_argument('--lcd-scale', type=float, default=1.5, help='LCD zoom (320 x 240 pixels)')
    ap.add_argument('--radio-freq', help='type this frequency in MHz (e.g. 145.1) + SET on the keypad after boot '
                                         '(must lie on the radio\'s STEP grid)')
    ap.add_argument('--fullband', action='store_true',
                    help='boot with the CPU input PJ0 high')
    ap.add_argument('--shot', help='save a screenshot after --shot-after seconds and quit (testing)')
    ap.add_argument('--shot-after', type=float, default=8)
    a = ap.parse_args()
    if a.fullband:
        os.environ['DJX_FULLBAND'] = '1'       # DLL の CPU 基板モデルが起動時に読む
    root = tk.Tk()
    app = App(root, a)
    if a.seek and isinstance(app.e.src, D.WavSource):
        app.e.src.seek(a.seek)
    if a.shot:
        def shot():
            # capture only this window's own pixels (PrintWindow), never other windows on the screen
            import ctypes
            from ctypes import wintypes
            from PIL import Image
            root.update()
            hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
            r = wintypes.RECT()
            ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(r))
            w, h = r.right - r.left, r.bottom - r.top
            gdi, u32 = ctypes.windll.gdi32, ctypes.windll.user32
            hdc = u32.GetWindowDC(hwnd)
            mdc = gdi.CreateCompatibleDC(hdc)
            bmp = gdi.CreateCompatibleBitmap(hdc, w, h)
            gdi.SelectObject(mdc, bmp)
            u32.PrintWindow(hwnd, mdc, 2)                       # PW_RENDERFULLCONTENT
            class BIH(ctypes.Structure):
                _fields_ = [('biSize', ctypes.c_uint32), ('biWidth', ctypes.c_int32), ('biHeight', ctypes.c_int32),
                            ('biPlanes', ctypes.c_uint16), ('biBitCount', ctypes.c_uint16), ('biCompression', ctypes.c_uint32),
                            ('biSizeImage', ctypes.c_uint32), ('a', ctypes.c_int32), ('b', ctypes.c_int32),
                            ('c', ctypes.c_uint32), ('d', ctypes.c_uint32)]
            bi = BIH(ctypes.sizeof(BIH), w, -h, 1, 32, 0, 0, 0, 0, 0, 0)
            buf = ctypes.create_string_buffer(w * h * 4)
            gdi.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0)
            Image.frombuffer('RGBA', (w, h), buf, 'raw', 'BGRA', 0, 1).convert('RGB').save(a.shot)
            gdi.DeleteObject(bmp); gdi.DeleteDC(mdc); u32.ReleaseDC(hwnd, hdc)
            root.destroy()
        root.after(int(a.shot_after * 1000), shot)
    root.mainloop()


if __name__ == '__main__':
    main()
