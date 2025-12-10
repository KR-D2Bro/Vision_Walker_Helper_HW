import rclpy
from rclpy.node import Node
from interfaces.msg import Vision
from .yolo_detector import YoloDetector
import cv2
from picamera2 import Picamera2
import time
import threading

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.detector = YoloDetector()
        self.publisher = self.create_publisher(Vision, '/object_detection/alert', 10)
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={"size": (1920, 1080), "format": "RGB888"})
        self.picam2.configure(config)
        self.picam2.start()

        self.frame_buffer = None
        self.lock = threading.Lock()

        # Simple IOU-based persistence tracker
        self.target_labels = {'pothole', 'edge'}
        self.required_consecutive = 3
        self.iou_match_threshold = 0.5
        self.track = None
        self.alerted = False
        
        self.detector_thread = threading.Thread(target=self.detection_loop)
        self.detector_thread.daemon = True
        self.detector_thread.start()

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        frame = self.picam2.capture_array()
        with self.lock:
            self.frame_buffer = frame

    def detection_loop(self):
         while rclpy.ok():
            with self.lock:
                if self.frame_buffer is None:
                    continue
                frame = self.frame_buffer.copy()
                self.frame_buffer = None
                if frame is None:
                    print('fail to read frame')

                # start_time = time.time()
                detections = self.detector.detect(frame)

                # Filter target classes
                targets = [d for d in detections if d['name'] in self.target_labels]
                if not targets:
                    # reset if nothing of interest this frame
                    self.track = None
                    self.alerted = False
                else:
                    self._update_track(targets)

                # end_time = time.time()
                # elapsed_time = end_time - start_time
                # self.get_logger().info(f"object detection time: {elapsed_time:.4f} seconds")

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

    def _update_track(self, detections):
        """
        Keep track of the most plausible same object using IOU and trigger
        alert after it persists for required_consecutive frames.
        """
        best_det = detections[0]
        if self.track is None:
            # start new track with highest confidence detection
            best_det = max(detections, key=lambda d: d['conf'])
            self.track = {
                'name': best_det['name'],
                'bbox': best_det['bbox'],
                'count': 1
            }
            self.alerted = False
            return

        # Find matching detection for current track
        match = None
        best_iou = 0.0
        for det in detections:
            if det['name'] != self.track['name']:
                continue
            iou = self._compute_iou(self.track['bbox'], det['bbox'])
            if iou > best_iou:
                best_iou = iou
                match = det

        if match and best_iou >= self.iou_match_threshold:
            self.track['bbox'] = match['bbox']
            self.track['count'] += 1
        else:
            # restart track with highest confidence current detection
            best_det = max(detections, key=lambda d: d['conf'])
            self.track = {
                'name': best_det['name'],
                'bbox': best_det['bbox'],
                'count': 1
            }
            self.alerted = False

        if self.track['count'] >= self.required_consecutive and not self.alerted:
            label = self.track['name']
            self.get_logger().info(f"{label} detected {self.track['count']} frames in a row -> alert")
            msg = Vision()
            msg.capture = True
            self.publisher.publish(msg)
            self.alerted = True

    @staticmethod
    def _compute_iou(box1, box2):
        x1, y1, x2, y2 = box1
        a1, b1, a2, b2 = box2
        inter_x1 = max(x1, a1)
        inter_y1 = max(y1, b1)
        inter_x2 = min(x2, a2)
        inter_y2 = min(y2, b2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area1 = max(0, x2 - x1) * max(0, y2 - y1)
        area2 = max(0, a2 - a1) * max(0, b2 - b1)
        union_area = area1 + area2 - inter_area
        if union_area == 0:
            return 0.0
        return inter_area / union_area

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
