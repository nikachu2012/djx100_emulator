# djx100_emulator

> [!WARNING]
> このリポジトリのソースコード、ドキュメントは人間が読むことを想定して記述されていません。LLMに読ませてください。

[English](README.md)

アルインコの受信機 DJ-X100 のエミュレータです。**無線機自身のファームウェアを 1 命令ずつ実行します。**

* CPU ファーム（Renesas RX65N）を RX シミュレータで動かします。周りには CPU 基板のモデル（LCD、キーマトリクス、
  ダイヤル・音量エンコーダ、EEPROM、DAC、ポートエキスパンダ、FM 放送用受信 IC BK1068、PC リンク）があります。
* DSP ファーム（TI TMS320C5517）を C55x シミュレータで動かします。周りには DSP 基板のモデル（IF の ADC、オーディオ
  コーデック、CPU との I2C リンク、ボコーダ IC AMBE-3000R）があります。AMBE-3000R は
  [ambe2-c](https://github.com/tallcat4/ambe2-c) で再現しています（D-STAR の音声は
  [mbelib](https://github.com/szechyjs/mbelib)）。
* アンテナは SDR です。SDR# / SDR++ の IQ 録音、rtl_tcp サーバ、USB 接続の RTL-SDR ドングルが使えます。

CPU ファームが起動して EEPROM から設定を読み、DSP のリセットを解除して DSP を設定し、本物の LCD を描画し、
本物のキーを読み、受信周波数を合わせます。SDR はファームが合わせた周波数に追従します。DSP ファームは DJ-X100 の
全モード（FM NFM AM NAM、T98、T102/B54、DMR、NXDN(N)、T61、ICDU、dPMR、D-STAR、C4FM、AIS、ACARS、POCSAG、12 kHz IF）を
無線機自身のフィルタとデコーダで復調します。

**ファームウェアは同梱していません。** アルインコ公式の DJ-X100 ファームウェアアップデータに含まれるファイルが必要です。

## 必要なもの

* Windows（x86-64）
* [zig](https://ziglang.org)（C/C++ コンパイラ。0.17 で確認）と、`build.sh` を動かす bash（Git Bash）
* Python 3（3.14 で確認）と `pip install numpy scipy pillow sounddevice`
* TI の **C5500 Code Generation Tools**（TI から無償で入手できます。`C5500_CGT`、既定は `~/tools/ti-cgt-c5500`）。
  DSP シミュレータは TI 自身の逆アセンブラの出力から命令を解読するので、DSP ファームの逆アセンブルに 1 回だけ使います。
* ファームウェアのファイル（下記）
* 任意: RTL-SDR ドングル（SDR# / SDR++ / rtl-sdr の配布物にある `rtlsdr.dll` と `libusb-1.0.dll`、Zadig で入れる
  WinUSB ドライバ）

## 準備

1. 公式アップデータのファームウェアを `firmware/` に置きます。

   | ファイル | 内容 |
   |---|---|
   | `cpu_flash_FFE00000-FFFFFFFF.bin` | CPU ファーム（コードフラッシュ、2 MiB） |
   | `cpu_option_FE7F5D00.bin` | CPU のオプション設定メモリ（なくても可） |
   | `DigRxx107.bin` | DSP ファーム 1.07（C5517 のブートイメージ） |

2. ビルドします。

   ```
   bash build.sh
   ```

   `build/djxemu.dll`（GUI が使うエミュレータ本体）とコマンドラインツール（`radio_test.exe`、`cpu_test.exe`、
   `stock_test.exe`）ができ、TI のツールで DSP の逆アセンブルリスト `build/stock/DigRxx107_r.dis` が作られます
   （手動なら `python stock/mkcoff.py firmware/DigRxx107.bin build/stock`）。リストから解読した命令は、すべて
   ファームのイメージとバイト単位で照合します。

## 使い方

```
python tools/make_test_iq.py analog                          # テスト信号 data/test_145000000Hz.wav（NFM 145.100、AM 144.800）
python gui.py data/test_145000000Hz.wav --radio-freq 145.1   # 起動後にキーパッドで 145.1 + SET を入力
python gui.py --usb                                          # アンテナ: USB の RTL-SDR（--usb-gain dB、既定は AGC）
python gui.py --rtl 127.0.0.1:1234                           # アンテナ: rtl_tcp サーバ
python gui.py --usb --fullband                               # CPU の入力 PJ0 を High にして起動
```

ウィンドウの左側が無線機です。ボタンは本物のキーと同じく押している間だけ押下になります（マウスを押す = キーを押す）。

| キー | キーボード / マウス |
|---|---|
| 0-9、`.`、SET | 0-9、`.`、Enter |
| MODE、STEP、V/M、SCAN | M、T、V、S |
| FUNC、MONI/SQL | F、Esc |
| ダイヤル | ← / →、LCD 上でホイール |
| 音量リング | ↑ / ↓、Shift + ホイール |

新しい EEPROM では音量が 0 から始まるので、最初に音量を上げてください。STEP の刻みに合わない周波数は無線機が
受け付けません。設定は `data/radio/eeprom.bin` に保存されます（終了時）。

右側には IQ ソース、IF フィルタのモデル、IF クリスタルフィルタ前後のチャンネルスペクトル、音声出力、CPU と DSP の
間の通信ログ、シミュレータの負荷が出ます。**REC audio** で音声を `djx_*.wav` に録音します。無線機の USB シリアル
（PC リンク）は TCP 127.0.0.1:7373 で使えます（`--pc-port`、0 で無効）。

IQ の WAV ファイルは SDR# / SDR++ の録音（RIFF/RF64、uint8 / int16 / float32）です。中心周波数はファイル名
（`..._145000000Hz...`）から読みます。

### 環境変数

| 変数 | 意味 |
|---|---|
| `DJX_FW_DIR` | ファームウェアのフォルダ（既定 `firmware/`） |
| `DJX_CPU_FLASH`、`DJX_CPU_OPTION`、`DJX_STOCK_BIN` | ファームのファイルを個別に指定。`DJX_STOCK_BIN` には別の DSP イメージも指定できます（[docs/internals.md](docs/internals.md)） |
| `DJX_EEPROM` | EEPROM イメージ（既定 `data/radio/eeprom.bin`） |
| `C5500_CGT` | TI C5500 Code Generation Tools |
| `DJX_RTLSDR_DLL` | `rtlsdr.dll`（ファイルかフォルダ） |
| `DJX_FULLBAND` | `1` で CPU の入力 PJ0 を High にして起動（`gui.py --fullband` と同じ） |

## テスト

| コマンド | 確認する内容 |
|---|---|
| `python radio_headless.py outdir [秒]` | 起動、キー、ダイヤル。LCD を PNG に保存 |
| `python rf_test.py [iq.wav] [outdir]` | S メータ、スケルチ、AF のオン / オフ、音量 |
| `python wfm_test.py station.wav MHz [outdir]` | FM 放送（BK1068） |
| `python stock/tone_test.py [workdir]` | DSP ファームの CTCSS / DCS トーンスケルチ |
| `python stock/digital_test.py [workdir]` | 合成信号による D-STAR / AIS / POCSAG / DMR |
| `python stock/stock_offline.py iq.wav Hz MODE 秒 [開始] [out.wav]` | 録音を DSP ファーム単体に通す |

テストスクリプトは PASS / FAIL を表示します（`radio_headless` と `stock_offline` はログを表示）。

## ファイル構成

| パス | 内容 |
|---|---|
| `gui.py`、`djxcore.py` | GUI と、DLL への Python インターフェース（IQ ソース、RTL-SDR、PC リンク） |
| `djxemu.cpp` | エミュレータ本体の DLL。SDR のチャンネル、IF フィルタ / ADC モデル、RF レベルモデル、スレッド、音声経路 |
| `rx/` | CPU 基板（`djxcpu.cpp`）と RX シミュレータのコア（`rx/gnu`、GNU binutils-gdb 由来） |
| `c55/` | C55x シミュレータ（`c55sim.cpp`）と DSP 基板（`djxboard.cpp`） |
| `stock/` | DSP の逆アセンブルリスト作成（`mkcoff.py`）、DSP のテスト、信号発生器 |
| `tools/` | テスト信号の作成、ADC モデル、ブートイメージツール、LCD ダンプの PNG 変換 |
| `third_party/ambe2-c` | AMBE+ / AMBE+2 デコーダ（MIT） |
| `third_party/mbelib` | D-STAR の音声デコーダ（ISC） |
| `docs/internals.md` | 無線機のモデル化の詳細、検証結果、デバッグ用オプション（英語） |

## 制限

* RF フロントエンドはレベルのモデルです（SDR のチャンネルのレベルと SNR から RSSI とノイズ検出を作ります）。
  チューナ、PLL、IF の IC は再現していません。IF フィルタと ADC のモデルは推定です。
* タイミングは近似です（RX は 1 命令あたり固定サイクル、C55x は推定サイクル）。
* PC リンクはファームのシリアル処理のレベルで再現しています。USB デバイスコントローラは再現していません。

詳しくは [docs/internals.md](docs/internals.md) を参照してください。

## ライセンス

**GPL-3.0-or-later**（[LICENSE](LICENSE)）です。RX シミュレータのコアが GPL のコードで、すべてのプログラムが
これをリンクしているためです。

| コード | 出典 | ライセンス |
|---|---|---|
| `rx/gnu/*.c`、`*.h`、`rx/gnu/opcode/rx.h` | GNU binutils-gdb（`sim/rx`、`opcodes/rx-decode.c`、`include/opcode/rx.h`、Free Software Foundation）。`mem.c`、`reg.c`、`rx.c` を改変（[rx/gnu/README.md](rx/gnu/README.md)） | GPL-3.0-or-later |
| `third_party/ambe2-c` | ambe2-c / blip25-vocoder。mbelib 互換 API は mbelib（ISC）のインターフェースを再現 | MIT |
| `third_party/mbelib` | mbelib（D-STAR の 3600x2400 のみ。[third_party/mbelib/VENDORED.md](third_party/mbelib/VENDORED.md)）。`dstar.c`、`dstar.h` はこのプロジェクトで、DSD の D-STAR インターリーブ表（ISC）を使用 | ISC（`dstar.c`、`dstar.h` は GPL-3.0-or-later） |
| 上記以外 | このプロジェクト | GPL-3.0-or-later |

## 免責事項

DJ-X100 および Alinco はアルインコ株式会社の商標です。このプロジェクトはアルインコとは関係がなく、承認も受けて
いません。アルインコのファームウェアやその他の著作物は配布していません。ソース中に記したファームのアドレスや
動作は、相互運用のためにファームを解析して調べたものです。電波法は、自分宛てでない通信の内容を漏らしたり
窃用したりすることを禁じています。その範囲で使ってください。
