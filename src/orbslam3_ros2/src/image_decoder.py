#!/usr/bin/env python3
"""
Visualiza /racecar/camera/image_raw cuyo encoding es 'mjpeg'.
Los bytes en data[] son un JPEG completo — se decodifica con OpenCV.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MjpegViewer(Node):
    def __init__(self):
        super().__init__("mjpeg_viewer")
        self.bridge = CvBridge()
        self._last_stamp = None
        self.sub = self.create_subscription(
            Image,
            "/racecar/camera/image_raw",
            self.callback,
            10,
        )
        self.pub = self.create_publisher(Image, "/image", 10)
        self.get_logger().info("Suscrito a /racecar/camera/image_raw (mjpeg)")

    def callback(self, msg: Image):
        # data es una lista/array de bytes que forman un JPEG completo
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("No se pudo decodificar el frame JPEG")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)
        decoded = self.bridge.cv2_to_imgmsg(resized, encoding="mono8")

        current_stamp = msg.header.stamp
        if self._last_stamp is not None:
            if (current_stamp.sec < self._last_stamp.sec) or (
                current_stamp.sec == self._last_stamp.sec
                and current_stamp.nanosec <= self._last_stamp.nanosec
            ):
                self.get_logger().warn("Dropping stale image frame with non-increasing timestamp")
                current_stamp = self.get_clock().now().to_msg()
                return

        decoded.header.stamp = current_stamp
        decoded.header.frame_id = msg.header.frame_id
        self._last_stamp = current_stamp
        self.pub.publish(decoded)
        #cv2.imshow("racecar camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

            
def main():
    rclpy.init()
    node = MjpegViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()