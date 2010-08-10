#!/usr/bin/env python

import roslib
import sys
roslib.load_manifest("snapshotter")
import rospy
import math
import tf
from tf.msg import tfMessage
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from snapshotter.srv import *
from snapshotter.msg import *
from snapshot_filter import SnapshotFilter

class CropFilter(SnapshotFilter):
    
    def get_extended_params(self):
        self.x_min = rospy.get_param("~x_min",0)
        self.x_max = rospy.get_param("~x_max",100)
        self.y_min = rospy.get_param("~y_min",0)
        self.y_max = rospy.get_param("~y_max",100)
    
    def image_filter(self,cv_image,info,copy=None):
        x_min = self.x_min * cv_image.width / 100.0
        x_max = self.x_max * cv_image.width / 100.0
        y_min = self.y_min * cv_image.height / 100.0
        y_max = self.y_max * cv_image.height / 100.0
        x = x_min
        y = y_min
        width = x_max - x_min
        height = y_max - y_min
        #cropped_image = cv.CreateImage((width,height),8,3)
        cropped_image = cv.GetSubRect(cv_image,(int(x),int(y),int(width),int(height)))
        return cropped_image
        
        
    
## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("crop_filter")
    filt = CropFilter()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
