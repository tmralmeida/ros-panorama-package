#!/usr/bin/env python2

import roslib
import numpy as np
import sys
import json
import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# Setup variables
IMAGE = 'right'
TOTAL_NUM_SAMPLES = 100
CHESS_DIMENSION = (9,6)
#Counter
NUM_SAMPLE = 0


class image_converter():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub_center = rospy.Subscriber("/center_camera/image_rect_color",Image,self.callback_center_image)
        self.image_sub_left = rospy.Subscriber("/left_camera/image_rect_color",Image,self.callback_left_image)
        self.image_sub_right = rospy.Subscriber("/right_camera/image_rect_color",Image,self.callback_right_image)
        self.left_image = None
        self.center_image = None
        self.right_image = None
    
    def callback_center_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.center_image = cv_image
        except CvBridgeError as e:
            print(e)
       
    
    def callback_left_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.left_image = cv_image
        except CvBridgeError as e:
            print(e)
       
    
    def callback_right_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.right_image = cv_image
        except CvBridgeError as e:
            print(e)
       
    
    def clean_images(self):
        self.left_image = None
        self.center_image = None
        self.right_image = None


class get_h_matrix():
    def __init__(self, patternsize):
        self.patternsize = patternsize
        self.dict_sample = {}
        
           
    
    def get_h_matrix_fn(self, imgs_2_transform):
        H_transforms = []
        lateral_corners = []
        center_corners = []
        dict_info = {}
        global NUM_SAMPLE
        self.center_image = imgs_2_transform[0]
        self.lateral_image = imgs_2_transform[1]
        ret1, corners1 = cv2.findChessboardCorners(self.lateral_image,  self.patternsize)
        ret2, corners2 = cv2.findChessboardCorners(self.center_image,  self.patternsize)
        if ret1 == True and ret2 == True:
            H,_ = cv2.findHomography(corners1,corners2)
            H = np.float32(H)
            cv2.drawChessboardCorners(self.lateral_image, self.patternsize, corners1,ret1)
            cv2.drawChessboardCorners(self.center_image, self.patternsize, corners2,ret2)
            dict_info ["Transformation"] = H
            dict_info ["lateral corners"] = corners1
            dict_info ["center corners"] = corners2
            self.dict_sample["sample "+str(NUM_SAMPLE)] = dict_info 
            NUM_SAMPLE+=1
        img_final = cv2.hconcat([self.lateral_image,self.center_image])
        cv2.imshow('Chessboard Detection', img_final)
        time.sleep(0.5)


def write_json_file(transformations):

    
    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return json.JSONEncoder.default(self, obj)
    
    json_dump = json.dumps(transformations, cls=NumpyEncoder)
    if IMAGE == 'left':
        filename = 'transforms_left'
    else:
        filename = 'transforms_right'
    with open('../catkin_ws/src/data_matrix_detection/cameras_transforms/'+filename+'.json', 'w') as outfile:
        json.dump(json_dump, outfile)
        
def main(args):
    ic = image_converter()
    h_tranformations = get_h_matrix(patternsize=CHESS_DIMENSION)
    
    rospy.init_node('get_transforms_node', anonymous=True)

    while True:
        try:
            # rospy.spin()
            if ic.left_image is not None and ic.center_image is not None and ic.right_image is not None:
                if IMAGE == 'left':
                    h_tranformations.get_h_matrix_fn([ic.center_image, ic.left_image])
                else:
                    h_tranformations.get_h_matrix_fn([ic.center_image, ic.right_image])

                if cv2.waitKey(1) & NUM_SAMPLE==TOTAL_NUM_SAMPLES:
                    write_json_file(h_tranformations.dict_sample)
                    break
      
            else:
                continue
        except KeyboardInterrupt:
            print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)