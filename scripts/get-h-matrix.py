#!/usr/bin/env python2
import cv2
import numpy as np
import json

LEFT_CAMERA = 3
RIGHT_CAMERA = 0
CENTER_CAMERA = 2
PANORAMA_WIDTH = 1920 
SRC_IMAGES_WIDTH = 640


class get_matrix_h():
    def __init__(self, device1, device2, patternsize):
        self.device1 = device1
        self.device2 = device2
        self.patternsize = patternsize

    def image_capture(self):
        self.cap1 = cv2.VideoCapture(self.device1)
        self.cap2 = cv2.VideoCapture(self.device2)
        ret1, self.frame1 = self.cap1.read()
        ret2, self.frame2 = self.cap2.read()
    
    def get_matrix(self):
        self.image_capture()
        ret1, corners1 = cv2.findChessboardCorners(self.frame1,  self.patternsize)
        ret2, corners2 = cv2.findChessboardCorners(self.frame2,  self.patternsize)
        H,_ = cv2.findHomography(corners1,corners2)
        H = np.float32(H)
        cv2.drawChessboardCorners(self.frame1, self.patternsize, corners1,ret1)
        cv2.drawChessboardCorners(self.frame2, self.patternsize, corners2,ret2)
        img_final = cv2.hconcat([self.frame1,self.frame2])
        cv2.imshow('Detections',img_final)

        if cv2.waitKey(-1) & 0xFF == ord('q'):
            self.caps_release()    
        
         
        return H


    def caps_release(self):
        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()

def write_json_file(h_lc,h_rc):
    transformations = {}
    center_init = PANORAMA_WIDTH/2 - (SRC_IMAGES_WIDTH/2)
    h_lc[0,2] = center_init - np.abs(h_lc[0,2]) 
    h_rc[0,2] = center_init + h_rc[0,2]
    print(center_init)
    #h_lc[0,2] = h_lc[0,2] 
    #h_rc[0,2] = h_rc[0,2]
    transforms_lst = [h_lc,np.array([[1,0,center_init],[0,1,0],[0,0,1]], dtype = np.float32), h_rc]
    cameras_ids = ["left_camera", "reference_camera", "right_camera"]

    for i in range(len(cameras_ids)):
        transformations[cameras_ids[i]] = transforms_lst[i]
    
    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return json.JSONEncoder.default(self, obj)
    
    json_dump = json.dumps(transformations, cls=NumpyEncoder)
    with open('../cameras_transforms/transforms.json', 'w') as outfile:
        json.dump(json_dump, outfile)
 


    

    
    
    
    
def main():
    get_h = get_matrix_h(CENTER_CAMERA, LEFT_CAMERA, patternsize = (9,7))
    h_lc = get_h.get_matrix()
    get_h2 = get_matrix_h(RIGHT_CAMERA,CENTER_CAMERA, patternsize = (9,7))
    h_rc = get_h2.get_matrix()
    write_json_file(h_lc,h_rc)
    
if __name__ == "__main__":
    main()


    