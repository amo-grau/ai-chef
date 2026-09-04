"""Block until a freshly created publisher can see the scene's subscriber.

`ros2 topic info` only proves the subscription exists in the DDS graph; it says
nothing about whether a brand-new participant can complete the discovery
handshake with it. Right after the scene loads it is busy enough that this can
take longer than the CLI's own discovery timeout, and the order is then
published into the void and silently dropped. Gating on the condition the CLI
actually needs -- a publisher of this exact type matching the subscriber --
removes that race.

Exits 0 once matched, 1 on timeout.
"""

import sys
import time

import rclpy
from kinematic_kitchen_interfaces.msg import PrepareOrder

TOPIC = "/kinematic_kitchen/prepare_order"


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0

    rclpy.init()
    node = rclpy.create_node("kinematic_kitchen_readiness_probe")
    publisher = node.create_publisher(PrepareOrder, TOPIC, 10)

    deadline = time.time() + timeout
    while time.time() < deadline:
        if publisher.get_subscription_count() > 0:
            node.destroy_node()
            rclpy.shutdown()
            return 0
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
