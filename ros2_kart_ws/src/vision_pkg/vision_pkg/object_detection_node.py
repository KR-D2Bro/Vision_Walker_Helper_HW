import rclpy
from rclpy.node import Node
from interfaces.msg import Vision
from .yolo_detector import YoloDetector
import cv2
from picamera2 import Picamera2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.detector = YoloDetector()
        self.publisher = self.create_publisher(Vision, '/object_detection/alert', 10)
        self.picam2 = Picamera2()
        self.picam2.start()

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        frame = self.picam2.capture_array()
        if frame is None:
            print('fail to read frame')

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        detections = self.detector.detect(frame)
        for det in detections:
            if det['name'] in ['pothole']:
                self.get_logger().info('capture!!')
                msg = Vision()
                msg.capture = True
                self.publisher.publish(msg)
                break

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
