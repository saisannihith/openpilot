#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${DISPLAY_NUM:-99}"
GEOMETRY="${GEOMETRY:-2160x1080x24}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5901}"
DISPLAY=":${DISPLAY_NUM}"
LOG_DIR="${LOG_DIR:-/tmp/snithpilot-ui-browser}"

mkdir -p "${LOG_DIR}"
export DISPLAY
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11

stop_matching() {
  local pattern="$1"
  local self="$$"
  (pgrep -f "${pattern}" || true) | while read -r pid; do
    [[ -z "${pid}" || "${pid}" == "${self}" ]] && continue
    if [[ "$(ps -o ppid= -p "${pid}" 2>/dev/null | tr -d ' ')" == "${self}" ]]; then
      continue
    fi
    kill "${pid}" 2>/dev/null || true
  done
}

stop_matching "Xvfb ${DISPLAY}"
stop_matching "x11vnc .*${DISPLAY}"
stop_matching "websockify .*${NOVNC_PORT}"
stop_matching "fluxbox"

rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"

Xvfb "${DISPLAY}" -screen 0 "${GEOMETRY}" -ac +extension GLX +render -noreset \
  >"${LOG_DIR}/xvfb.log" 2>&1 &
sleep 0.5

fluxbox >"${LOG_DIR}/fluxbox.log" 2>&1 &
sleep 0.5

env -u WAYLAND_DISPLAY -u XDG_SESSION_TYPE \
  x11vnc -display "${DISPLAY}" -localhost -forever -shared -nopw -rfbport "${VNC_PORT}" \
  >"${LOG_DIR}/x11vnc.log" 2>&1 &
sleep 0.5

websockify --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:${VNC_PORT}" \
  >"${LOG_DIR}/novnc.log" 2>&1 &
sleep 1

xsetroot -solid "#101820" >"${LOG_DIR}/xsetroot.log" 2>&1 || true

echo "noVNC: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"
echo "DISPLAY=${DISPLAY}"
echo "logs=${LOG_DIR}"
echo
ps -ef | grep -E "Xvfb ${DISPLAY}|x11vnc|websockify|fluxbox" | grep -v grep || true
ss -ltnp | grep -E ":${NOVNC_PORT}|:${VNC_PORT}" || true
