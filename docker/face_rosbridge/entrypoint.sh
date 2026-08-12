#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:="${ROSBRIDGE_PORT:-9090}"
