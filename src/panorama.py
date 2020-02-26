#!/usr/bin/env python2

import cv2
import numpy as np


class Stitcher:
    def __init__(self, imgs, transformations):
        self.dict_imgs = imgs
        self.dict_transformations = transformations
        self.result_size = (1920, 480)
        self.imgs_dummy = []
        self.tx = 600.0

    def stitch(self):
        T = np.array([[1.0,0.0,600.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        warped_left = cv2.warpPerspective(self.dict_imgs["left_camera"],np.dot(T,  self.dict_transformations["left_camera"]), self.result_size)
        warped_center = cv2.warpPerspective(self.dict_imgs["reference_camera"], T, self.result_size)
        warped_right = cv2.warpPerspective(self.dict_imgs["right_camera"], np.dot(T ,self.dict_transformations["right_camera"]), self.result_size)
        final_result = np.zeros((self.result_size[1], self.result_size[0], 3), dtype=np.uint8)
        for w in [warped_left, warped_center, warped_right]:
            final_result[w != 0] = w[w != 0]
        return final_result

    def process_image(self, result):
        return result
        
