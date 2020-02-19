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


def load_json_file():
    # Load cameras transformation
    cameras_ids = ["left_camera", "reference_camera", "right_camera"]
    transformation_dict = {}
    print("\n Loading camera transformations file...\n")
    fileDir = os.path.dirname(os.path.realpath('__file__'))
    filename = os.path.join(fileDir, '../catkin_ws/src/data_matrix_detection/cameras_params/transforms.json')
    with open(filename, 'r') as f:
        data = json.load(f)
    for i in range(len(data)):
        transform = np.asarray(data[cameras_ids[i]])
        transform = transform.reshape((3,3)).astype(np.float32)
        transformation_dict[cameras_ids[i]]=(transform)
    print("\n Camera transformations loaded successfully!\n")
    return transformation_dict


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub_center = rospy.Subscriber("/center_camera/image_raw",Image,self.callback_center_image)
        self.image_sub_left = rospy.Subscriber("/left_camera/image_raw",Image,self.callback_left_image)
        self.image_sub_right = rospy.Subscriber("/right_camera/image_raw",Image,self.callback_right_image)
        self.left_image = None
        self.center_image = None
        self.right_image = None
    
    def callback_center_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.center_image = cv_image
        except CvBridgeError as e:
            print(e)
        return self
    
    def callback_left_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.left_image = cv_image
        except CvBridgeError as e:
            print(e)
        return self
    
    def callback_right_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.right_image = cv_image
        except CvBridgeError as e:
            print(e)
        return self
    
    def clean_images(self):
        self.left_image = None
        self.center_image = None
        self.right_image = None


#TODO: pass this function to the while main()
def create_panorama(imgs, transform_dict):
    stitcher = Stitcher(imgs,transform_dict)
    result = stitcher.stitch()
    if result is not None:
        return result
    else:
        print("[ERROR]")

def pub_panorama(panorama):
    panorama_pub = rospy.Publisher("panorama",Image,queue_size=1)
    bridge = CvBridge()
    try:
      panorama_pub.publish(bridge.cv2_to_imgmsg(panorama, "bgr8"))
    except CvBridgeError as e:
      print(e)


        

def main(args):
    cameras_ids = ["left_camera", "reference_camera", "right_camera"]
    transform_dict = load_json_file()
    
    ic = image_converter()
    rospy.init_node('panorama_creation_node', anonymous=True)

    while True:
        try:
            # rospy.spin()
            if ic.left_image is not None and ic.center_image is not None and ic.right_image is not None:
                # pano_initial = create_panorama([ic.left_image,ic.center_image])
                # pano_final = create_panorama([pano_initial,ic.right_image])
                # pub_panorama(pano_final)
                imgs = {
                    'left_camera': ic.left_image,
                    'reference_camera': ic.center_image,
                    'right_camera': ic.right_image
                }
                pano_final = create_panorama(imgs,transform_dict)
                pub_panorama(pano_final)
                ic.clean_images()
            else:
                continue
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)