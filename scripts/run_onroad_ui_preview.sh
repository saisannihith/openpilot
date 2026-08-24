#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="${SNITH_UI_SESSION:-snithpilot-ui-onroad}"
DISPLAY_NUM="${DISPLAY_NUM:-99}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5901}"
LOG_PATH="${LOG_PATH:-/tmp/snithpilot-ui-onroad.log}"
URL="http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"

kill_tmux_sessions() {
  if ! command -v tmux >/dev/null 2>&1; then
    return
  fi

  tmux kill-session -t "${SESSION_NAME}" >/dev/null 2>&1 || true
  while IFS= read -r session; do
    tmux kill-session -t "${session}" >/dev/null 2>&1 || true
  done < <(tmux list-sessions -F '#S' 2>/dev/null | grep -E '^snithpilot-ui($|-)' || true)
}

kill_matching_processes() {
  local display=":${DISPLAY_NUM}"
  pkill -f "Xvfb ${display}" >/dev/null 2>&1 || true
  pkill -f "x11vnc .*${display}" >/dev/null 2>&1 || true
  pkill -f "websockify .*${NOVNC_PORT}" >/dev/null 2>&1 || true
  pkill -f "${ROOT_DIR}/onroad --c3" >/dev/null 2>&1 || true
  rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
}

kill_tmux_sessions
kill_matching_processes

tmux new-session -d -s "${SESSION_NAME}" \
  "cd '${ROOT_DIR}' && DISPLAY_NUM='${DISPLAY_NUM}' NOVNC_PORT='${NOVNC_PORT}' VNC_PORT='${VNC_PORT}' FPS='${FPS:-15}' ./scripts/run_onroad_browser.sh --nav --demo >'${LOG_PATH}' 2>&1"

sleep 2

echo "Started onroad UI preview."
echo "tmux: ${SESSION_NAME}"
echo "log: ${LOG_PATH}"
echo "url: ${URL}"
