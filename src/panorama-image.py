#!/usr/bin/env python2

from __future__ import print_function
import numpy as np
import roslib
from panorama import Stitcher
#roslib.load_manifest('my_package')
import sys
import json
import os
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
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



def pub_panorama(panorama):
    panorama_pub = rospy.Publisher("panorama",Image,queue_size=1)
    bridge = CvBridge()
    try:
      panorama_pub.publish(bridge.cv2_to_imgmsg(panorama, "bgr8"))
    except CvBridgeError as e:
      print(e)


        

def main(args):
    ic = image_converter()
    stitcher = Stitcher()
    rospy.init_node('panorama_creation_node', anonymous=True)

    while True:
        try:
            if ic.left_image is not None and ic.center_image is not None and ic.right_image is not None:
                result = stitcher.stitch([ic.left_image,ic.center_image, ic.right_image])
                if result is None:
                    print("There was an error in the stitching procedure")
                else:
                    pub_panorama(result)
                    ic.clean_images()
            else:
                continue
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)