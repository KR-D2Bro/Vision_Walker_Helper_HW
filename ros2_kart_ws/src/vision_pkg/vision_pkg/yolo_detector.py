#YOLOv11 code
import os, time, cv2

import ncnn

try:
    ncnn.create_gpu_instance()
except Exception as e:
    print("ncnn Vulkan init failed:", e)

from ultralytics import YOLO

HOME_DIR = '/home/dongjae/Vision_Walker_Helper_HW/'
VISION_PKG_DIR = 'ros2_kart_ws/src/vision_pkg/vision_pkg/'
PKG_ROOT = os.path.join(HOME_DIR, VISION_PKG_DIR)

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(THIS_DIR, 'model')

# �� NCNN export�� ���� ���� �̸�(���ο� .param/.bin ����)
MODEL_FOLDER = os.path.join(MODEL_DIR, 'best_openvino_model')

IMGSZ = 640
CONF_THR = 0.25
IOU_THR  = 0.45


class YoloDetector:
    """
    Ultralytics YOLO(NCNN) �߷� ����
    ��ȯ: [{'name': str, 'conf': float, 'bbox': [x1,y1,x2,y2]}, ...]
    """
    def __init__(self,
                 model_dir: str = MODEL_FOLDER,
                 imgsz: int = IMGSZ,
                 conf_thres: float = CONF_THR,
                 iou_thres: float = IOU_THR):
        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

        # NCNN ������ ���� �ε�, task ����
        self.model = YOLO(model_dir, task='detect')

    def detect(self, frame):
        """
        frame: BGR np.ndarray(H,W,3)
        """
        # Ultralytics�� ���ο��� NCNN ���� + NMS���� ó��
        r = self.model(frame,
                       imgsz=self.imgsz,
                       conf=self.conf_thres,
                       iou=self.iou_thres,
                       verbose=False)[0]

        results = []
        if r.boxes is not None and len(r.boxes) > 0:
            names = r.names if isinstance(r.names, dict) else {i: n for i, n in enumerate(r.names)}
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                cls = int(b.cls[0])
                conf = float(b.conf[0])
                results.append({
                    'name': names.get(cls, str(cls)),
                    'conf': conf,
                    'bbox': [x1, y1, x2, y2]
                })
        #         name = r.names.get(cls, str(cls))
        #         cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        #         cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(0,y1-6)),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # cv2.imshow("Openvino YOLO", frame)
        # cv2.waitKey(1)
        return results