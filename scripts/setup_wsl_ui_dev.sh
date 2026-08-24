#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install WSL development packages." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  clang \
  cmake \
  capnproto \
  libbz2-dev \
  libcurl4-openssl-dev \
  libssl-dev \
  libzmq3-dev \
  libavcodec-dev \
  libavformat-dev \
  libavutil-dev \
  libncurses-dev \
  ocl-icd-opencl-dev \
  opencl-headers \
  libyuv-dev \
  patchelf \
  pkg-config \
  python3-dev \
  x11vnc \
  xvfb \
  novnc \
  websockify \
  fluxbox \
  dbus-x11

if [[ ! -x .venv/bin/python3 ]]; then
  ./tools/install_python_dependencies.sh
fi

if [[ -f third_party/acados/x86_64/lib/libblasfeo.so ]] &&
   readelf -W -l third_party/acados/x86_64/lib/libblasfeo.so 2>/dev/null | grep -q "GNU_STACK.*RWE"; then
  patchelf --clear-execstack third_party/acados/x86_64/lib/libblasfeo.so
fi

echo "WSL UI development setup complete."
echo "Run: ./scripts/start_raylib_ui_browser.sh"
echo "Then: ./scripts/run_onroad_browser.sh --demo"
