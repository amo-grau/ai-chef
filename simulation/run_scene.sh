#!/bin/bash
# Launch the Isaac Sim kitchen scene.
#
# The scene runs on Isaac Sim's bundled Python, which is 3.12. A system ROS 2
# whose Python differs (Lyrical on Ubuntu 26.04 is 3.14) cannot be used here:
# rclpy ships a compiled extension built for one ABI only. Isaac Sim bundles
# its own Jazzy rclpy for 3.12, so that one is used instead, together with the
# custom messages built for the same distro (see `make interfaces-sim`).
#
# Deliberately does NOT source the system ROS 2 setup: putting its rclpy on the
# path would shadow the bundled one and fail to import.
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM="${ISAAC_SIM:-$HOME/isaacsim}"
SIM_ROS="$ISAAC_SIM/exts/isaacsim.ros2.core/${SIM_ROS_DISTRO:-jazzy}"
SIM_MSGS="$REPO/${SIM_INSTALL:-install_jazzy}/kinematic_kitchen_interfaces"

if [ ! -x "$ISAAC_SIM/python.sh" ]; then
    echo "Isaac Sim not found at $ISAAC_SIM. Set ISAAC_SIM=/path/to/isaacsim" >&2; exit 1
fi
if [ ! -d "$SIM_MSGS" ]; then
    echo "Custom messages not built for the simulator. Run: make interfaces-sim" >&2; exit 1
fi

export ROS_DISTRO="${SIM_ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$SIM_ROS/rclpy:$SIM_MSGS/lib/python3.12/site-packages:$REPO:$PYTHONPATH"
export LD_LIBRARY_PATH="$SIM_ROS/lib:$SIM_MSGS/lib:$LD_LIBRARY_PATH"

cd "$REPO"
exec "$ISAAC_SIM/python.sh" simulation/kitchen_scene.py "$@"
