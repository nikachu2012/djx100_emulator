#!/bin/bash
# Linux 版 TI C5500 CGT（i386）を linux/386 コンテナで実行するラッパー。Apple シリコンの macOS 用
# （Mac 版 CGT は i386 で、Rosetta 2 では動かない）。$C5500_CGT/bin/cl55 と dis55 としてリンクして使う。
# CGT_LINUX: Linux 版 CGT のインストール先
CGT=${CGT_LINUX:-$HOME/tools/ti-cgt-c5500-linux/c5500_4.4.1}
exec docker run --rm --platform linux/386 -v "$CGT:$CGT:ro" -v "$PWD:$PWD" -w "$PWD" -u "$(id -u):$(id -g)" \
  i386/debian:bookworm-slim "$CGT/bin/$(basename "$0")" "$@"
