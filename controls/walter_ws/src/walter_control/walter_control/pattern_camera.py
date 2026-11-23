"""Publishes a moving test pattern on the camera topics so the bridge can run on mock hardware.

    ros2 run walter_control test_pattern_camera
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PatternCamera(Node):
    def __init__(self):
        super().__init__("test_pattern_camera")
        topics = self.declare_parameter("topics", ["/scene_camera/image_raw", "/wrist_camera/image_raw"]).value
        self.w = self.declare_parameter("width", 640).value
        self.h = self.declare_parameter("height", 480).value
        self.pubs = [self.create_publisher(Image, t, 2) for t in topics]
        self.k = 0
        self.create_timer(1.0 / self.declare_parameter("fps", 15.0).value, self._tick)

    def _tick(self):
        self.k += 1
        x = (np.arange(self.w)[None, :] + 4 * self.k) % 256
        y = np.arange(self.h)[:, None] % 256
        img = np.stack([np.broadcast_to(x, (self.h, self.w)), np.broadcast_to(y, (self.h, self.w)),
                        np.full((self.h, self.w), 128)], axis=-1).astype(np.uint8)
        msg = Image(height=self.h, width=self.w, encoding="rgb8", step=3 * self.w, data=img.tobytes())
        msg.header.stamp = self.get_clock().now().to_msg()
        for pub in self.pubs:
            pub.publish(msg)


def main():
    rclpy.init()
    node = PatternCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
