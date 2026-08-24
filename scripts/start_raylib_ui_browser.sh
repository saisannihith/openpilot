#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${DISPLAY_NUM:-99}"
GEOMETRY="${GEOMETRY:-2160x1080x24}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5901}"
DISPLAY=":${DISPLAY_NUM}"
LOG_DIR="${LOG_DIR:-/tmp/snithpilot-ui-browser}"
USE_FLUXBOX="${USE_FLUXBOX:-0}"

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

start_detached() {
  local log_path="$1"
  shift
  setsid "$@" >"${log_path}" 2>&1 < /dev/null &
}

stop_matching "[X]vfb ${DISPLAY}"
stop_matching "[x]11vnc .*${DISPLAY}"
stop_matching "[w]ebsockify .*${NOVNC_PORT}"
if [[ "${USE_FLUXBOX}" =~ ^(1|true|yes|on)$ ]]; then
  stop_matching "[f]luxbox"
fi

rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"

start_detached "${LOG_DIR}/xvfb.log" \
  Xvfb "${DISPLAY}" -screen 0 "${GEOMETRY}" -ac +extension GLX +render -noreset
sleep 0.5

if [[ "${USE_FLUXBOX}" =~ ^(1|true|yes|on)$ ]]; then
  start_detached "${LOG_DIR}/fluxbox.log" fluxbox
  sleep 0.5
fi

start_detached "${LOG_DIR}/x11vnc.log" \
  env -u WAYLAND_DISPLAY -u XDG_SESSION_TYPE \
  x11vnc -display "${DISPLAY}" -localhost -forever -shared -nopw -rfbport "${VNC_PORT}" \
  -noxdamage -ncache 0 -wait 10 -defer 10 -cursor arrow -repeat
sleep 0.5

start_detached "${LOG_DIR}/novnc.log" \
  websockify --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:${VNC_PORT}"
sleep 1

if command -v xsetroot >/dev/null 2>&1; then
  xsetroot -display "${DISPLAY}" -solid "#101820" >"${LOG_DIR}/xsetroot.log" 2>&1 || true
fi

echo "noVNC: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"
if command -v hostname >/dev/null 2>&1; then
  wsl_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  if [[ -n "${wsl_ip}" ]]; then
    echo "noVNC WSL IP: http://${wsl_ip}:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"
  fi
fi
echo "DISPLAY=${DISPLAY}"
echo "logs=${LOG_DIR}"
echo
ps -ef | grep -E "Xvfb ${DISPLAY}|x11vnc|websockify|fluxbox" | grep -v grep || true
ss -ltnp | grep -E ":${NOVNC_PORT}|:${VNC_PORT}" || true
