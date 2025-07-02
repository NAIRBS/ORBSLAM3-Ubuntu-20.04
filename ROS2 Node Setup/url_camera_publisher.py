#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()
        self.video_url = 'http://192.168.121.123:81/stream' # UPDATE URL HERE
        self.running = True
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        self.get_logger().info(f"Connecting to {self.video_url}")
        cap = cv2.VideoCapture(self.video_url)
        if not cap.isOpened():
            self.get_logger().error("Failed to open stream.")
            return
        while self.running:
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("Failed to read frame.")
        cap.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
