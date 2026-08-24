#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

export DISPLAY="${DISPLAY:-:99}"
export NOVNC_PORT="${NOVNC_PORT:-6080}"
export BIG=1
export FPS="${FPS:-30}"
export SCALE="${SCALE:-1.0}"
export SHOW_FPS="${SHOW_FPS:-1}"
export SP_SKIP_REPLAY_BUILD_IF_PRESENT="${SP_SKIP_REPLAY_BUILD_IF_PRESENT:-1}"
export SP_ALLOW_DESKTOP_FAKE_WIFI="${SP_ALLOW_DESKTOP_FAKE_WIFI:-1}"
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11

if ! ss -ltn | grep -q ":${NOVNC_PORT} "; then
  "${ROOT_DIR}/scripts/start_raylib_ui_browser.sh"
fi

exec "${ROOT_DIR}/onroad" --c3 "$@"
