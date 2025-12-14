#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
source /app/project-tasr/install/setup.bash

exec "$@"