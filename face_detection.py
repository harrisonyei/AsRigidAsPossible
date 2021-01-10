# system
import os
import sys

# python lib
import face_alignment
import numpy as np
import cv2

class LM_detector():
    def __init__(self, use_cnn_face_detector=True ,lm_type=2, device='cpu', face_detector='sfd'):
        if lm_type == 2:
            self.fa = face_alignment.FaceAlignment(
                face_alignment.LandmarksType._2D, device=device, flip_input=False, face_detector=face_detector)
        else:
            self.fa = face_alignment.FaceAlignment(
                face_alignment.LandmarksType._3D, device=device, flip_input=False, face_detector=face_detector)

    def detect(self, image):
        # filter very large image
        scale = 1.0
        h, w, c = image.shape
        if max(h, w) > 1000:
            scale = max(h, w) / (1000.0)
            image = cv2.resize(image, (int(w / scale), int(h / scale)))

        h, w, c = image.shape
        norm_scale = 1 / max(h, w)

        # torch
        detected_faces = self.fa.face_detector.detect_from_image(image[..., ::-1].copy())

        if detected_faces != None and len(detected_faces) != 0:
            landmarks = self.fa.get_landmarks(image, detected_faces=[detected_faces[0],])

            # normalize to 0 ~ 1
            for face in landmarks:
                face *= norm_scale

            return landmarks

        return None