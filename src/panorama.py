#!/usr/bin/env python2

import cv2
import numpy as np


class Stitcher:
    def __init__(self, imgs, transformations):
        self.dict_imgs = imgs
        self.dict_transformations = transformations
        self.result_size = (1800, 480)
        self.imgs_dummy = []

    def stitch(self):
        self.preprocess_dicts()
        
        reference_image = cv2.warpPerspective(self.reference_image, self.dict_transformations["reference_camera"], self.result_size)

        for i in range(len(self.imgs_dummy)):
            result_warp = cv2.warpPerspective(self.imgs_dummy[i], self.dict_transformations[self.cameras_dummy[i]], self.result_size)
            result = cv2.addWeighted(result_warp, 1, reference_image, 1, 0)

        final_result = self.process_image(result) 
        return final_result

    def preprocess_dicts(self):
        self.cameras_dummy = []
        for key,values in self.dict_imgs.items():
            if key == 'reference_camera':
                self.reference_image = values
            else:
                self.imgs_dummy.append(values)
                self.cameras_dummy.append(key)
        return self 

    def process_image(self, result):
        return result
        
