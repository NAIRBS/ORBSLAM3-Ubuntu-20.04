#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class StereoCameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')
        self.left_pub = self.create_publisher(Image, '/camera/left', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right', 10)
        self.bridge = CvBridge()
        self.left_url = 'http://192.168.121.123:81/stream'
        self.right_url = 'http://192.168.121.153:81/stream'
        self.running = True
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        cap_left = cv2.VideoCapture(self.left_url)
        cap_right = cv2.VideoCapture(self.right_url)
        if not cap_left.isOpened() or not cap_right.isOpened():
            self.get_logger().error("Failed to open one or both streams.")
            return
        while self.running:
            ret_l, frame_l = cap_left.read()
            ret_r, frame_r = cap_right.read()
            if ret_l and ret_r:
                now = self.get_clock().now().to_msg()
                msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
                msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')
                msg_l.header.stamp = now
                msg_r.header.stamp = now
                self.left_pub.publish(msg_l)
                self.right_pub.publish(msg_r)
            else:
                self.get_logger().warn("Failed to read one or both frames.")
        cap_left.release()
        cap_right.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
