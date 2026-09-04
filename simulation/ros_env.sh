# Sourced, not executed.
#
# Defines the single ROS 2 environment this project runs on: the Jazzy build
# that Isaac Sim bundles, on Isaac Sim's own Python 3.12.
#
# Isaac Sim 6.0 embeds CPython 3.12 and every one of its extensions is a cp312
# C extension, so the scene cannot run on any other Python. rclpy is likewise
# built for a single Python ABI. Rather than run two ROS distros -- one for the
# host and one for the simulator -- everything here speaks Jazzy on 3.12:
# the scene, the CLI, and the ROS-dependent tests. A host ROS 2 install is not
# used, and does not need to exist.
#
# Exports KK_PYTHON, the interpreter every ROS-speaking process must use.

# Idempotent: run_scene.sh sources this too, and the Makefile sources it before
# launching the scene, so guard against stacking the paths twice.
if [ -n "$KK_ROS_ENV_LOADED" ]; then
    return 0 2>/dev/null || exit 0
fi

_KK_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM="${ISAAC_SIM:-$HOME/isaacsim}"
KK_ROS_DISTRO="${KK_ROS_DISTRO:-jazzy}"

_KK_SIM_ROS="$ISAAC_SIM/exts/isaacsim.ros2.core/$KK_ROS_DISTRO"
_KK_MSGS="$_KK_REPO/${INTERFACES_INSTALL:-install_jazzy}/kinematic_kitchen_interfaces"

if [ ! -x "$ISAAC_SIM/python.sh" ]; then
    echo "Isaac Sim not found at $ISAAC_SIM. Set ISAAC_SIM=/path/to/isaacsim" >&2
    return 1 2>/dev/null || exit 1
fi
if [ ! -d "$_KK_SIM_ROS/rclpy" ]; then
    echo "Isaac Sim has no bundled rclpy for '$KK_ROS_DISTRO' at $_KK_SIM_ROS" >&2
    return 1 2>/dev/null || exit 1
fi
if [ ! -d "$_KK_MSGS" ]; then
    echo "Custom messages not built. Run: make interfaces" >&2
    return 1 2>/dev/null || exit 1
fi

export ROS_DISTRO="$KK_ROS_DISTRO"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$_KK_SIM_ROS/rclpy:$_KK_MSGS/lib/python3.12/site-packages:$_KK_REPO:$PYTHONPATH"
export LD_LIBRARY_PATH="$_KK_SIM_ROS/lib:$_KK_MSGS/lib:$LD_LIBRARY_PATH"

KK_PYTHON="$ISAAC_SIM/python.sh"
KK_ROS_ENV_LOADED=1
