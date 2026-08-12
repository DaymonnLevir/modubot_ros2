#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

FACE_APP_ROOT="${FACE_APP_ROOT:-/opt/modubot/face}"
if [[ -f "${FACE_APP_ROOT}/robot_face.html" ]]; then
  python3 -m http.server "${FACE_APP_PORT:-8080}" \
    --bind 127.0.0.1 \
    --directory "${FACE_APP_ROOT}" &
fi

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:="${ROSBRIDGE_PORT:-19090}"
