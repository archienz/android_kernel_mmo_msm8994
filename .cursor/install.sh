#!/usr/bin/env bash
#
# Cloud Agent install for the talkman (msm8994) kernel tree.
# Prepares everything needed to cross-compile the arm64 kernel:
#   make ARCH=arm64 mmo_defconfig
#   make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE" -j"$(nproc)"
#
# Idempotent and non-interactive: safe to run repeatedly.
set -euo pipefail

TOOLCHAIN_ROOT="${TOOLCHAIN_ROOT:-$HOME/toolchains}"
GCC49_DIR="$TOOLCHAIN_ROOT/aarch64-linux-android-4.9"
GCC49_REPO="https://github.com/kindle4jerry/aarch64-linux-android-4.9-bakup"

echo "==> Installing host build dependencies"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
# bc/bison/flex/libssl/ncurses: kernel build; zip/unzip: AnyKernel3 packaging;
# ccache: faster rebuilds; python-is-python3: the GCC 4.9 wrapper needs /usr/bin/python.
sudo apt-get install -y --no-install-recommends \
    build-essential \
    bc bison flex \
    libssl-dev libncurses-dev \
    zip unzip \
    ccache \
    git curl ca-certificates \
    python-is-python3

echo "==> Ensuring aarch64-linux-android-4.9 cross toolchain"
mkdir -p "$TOOLCHAIN_ROOT"
if [ ! -x "$GCC49_DIR/bin/real-aarch64-linux-android-gcc" ]; then
    rm -rf "$GCC49_DIR"
    git clone --depth=1 "$GCC49_REPO" "$GCC49_DIR"
else
    echo "    toolchain already present at $GCC49_DIR"
fi

echo "==> Sanity checking cross compiler"
"$GCC49_DIR/bin/aarch64-linux-android-gcc" --version | head -1

echo "==> Generating kernel config (mmo_defconfig)"
make ARCH=arm64 mmo_defconfig >/dev/null

cat <<EOF

Install complete.

Toolchain: $GCC49_DIR
To build the kernel:
  export CROSS_COMPILE="$GCC49_DIR/bin/aarch64-linux-android-"
  make ARCH=arm64 mmo_defconfig
  make ARCH=arm64 CROSS_COMPILE="\$CROSS_COMPILE" -j"\$(nproc)"

Output: arch/arm64/boot/Image.gz-dtb
EOF
