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

class BirdseyeFilter(SnapshotFilter):
    
    def get_extended_params(self):
        self.filled = rospy.get_param("~filled",False)
    
    def image_filter(self,cv_image,info):
        if self.filled:
            thickness = -1
            color = (0,0,255)
        else:
            thickness = 1
            color = (255,0,0)
        cv.Circle(cv_image,(200,200),5,color,thickness)
        return cv_image

## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("birdseye_filter")
    filt = BirdseyeFilter()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
