#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f "/colcon_ws/install/setup.bash" ]; then
  source "/colcon_ws/install/setup.bash"
else
  echo "WARNING: /colcon_ws/install/setup.bash not found!"
fi

exec "$@"