import sys
import torch
import os

# YOLOv5 코드 경로를 Python 모듈 경로에 추가
YOLOV5_PATH = os.path.join(os.path.dirname(__file__), 'yolov5')
sys.path.append(YOLOV5_PATH)

from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes 
import cv2
import numpy as np
model_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),'model', 'best.pt')
headless = os.environ.get('DISPLAY','') == ''

class YoloDetector:
    def __init__(self, model_path= model_PATH, device='cpu'):
        self.device = device
        self.model = DetectMultiBackend(model_path, device=self.device)
        self.model.eval()

    def detect(self, frame):
        img = cv2.resize(frame, (640, 640))
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #img = np.stack([gray, gray, gray], axis=-1)
        img = img.transpose(2, 0, 1)
        img = torch.from_numpy(img).float() / 255.0
        img = img.unsqueeze(0).to(self.device)
        pred = None
        try:
            pred = self.model(img, augment=False)
            pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.45)[0]  # 첫 번째 이미지 결과
            #시각화#
            if not headless:
                #frame = gray.copy()
                if pred is not None and len(pred):
                    pred[:, :4] = scale_boxes(img.shape[2:], pred[:, :4], frame.shape).round()

                    for *xyxy, conf, cls in pred:
                        label = f'{self.model.names[int(cls)]} {conf:.2f}'
                        xyxy = [int(x) for x in xyxy]
                        cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                        cv2.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
           
                print(type(frame), frame.shape, frame.dtype)
                cv2.imshow('Detection', frame)
                cv2.waitKey(1)
            #시각화 끝#
        except Exception as e:
            print("에러발생:",e)
        results = []
        if pred is not None and len(pred):
            for *xyxy, conf, cls in pred:
                class_name = self.model.names[int(cls)]
                results.append({'name': class_name, 'conf': float(conf), 'bbox': xyxy})
        return results
