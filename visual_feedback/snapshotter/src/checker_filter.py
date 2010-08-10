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

class CheckerFilter(SnapshotFilter):
    
    def get_extended_params(self):
        self.cols = rospy.get_param("~cols",5)
        self.rows = rospy.get_param("~rows",4)
    
    def image_filter(self,cv_image,info,copy=None):
        board_sz = (self.cols,self.rows)
        (found,corners) = cv.FindChessboardCorners(cv_image,board_sz, (cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_FILTER_QUADS))
        cv.DrawChessboardCorners(cv_image,(self.cols,self.rows),corners,1)
        return cv_image
        
        
    
## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("birdseye_filter")
    filt = CheckerFilter()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
