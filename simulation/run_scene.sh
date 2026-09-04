#!/bin/bash
# Launch the Isaac Sim kitchen scene on the project's ROS 2 environment.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/ros_env.sh"
cd "$(dirname "${BASH_SOURCE[0]}")/.."
exec "$KK_PYTHON" simulation/kitchen_scene.py "$@"
