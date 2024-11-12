import torch
import numpy as np
import cv2

class ObjectDetector:
    def __init__(self, model_path='yolov5s.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.eval()

    def detect_objects(self, image):
        """檢測影像中的人和車"""
        results = self.model(image)
        labels, coords = results.xyxyn[0][:, -1].numpy(), results.xyxyn[0][:, :-1].numpy()
        return labels, coords