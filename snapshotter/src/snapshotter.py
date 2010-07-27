#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("snapshotter")
import rospy
import math
import tf
from tf.msg import tfMessage
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import thread
from snapshotter.srv import *
import time

## Snapshotter documentation
#
#  A node which, when instantiated, watches a camera feed and stores "snapshots" which can be pulled from a service
class Snapshotter:

    ##   The constructor
    #    @param self The object pointer
    #    @param cameraName The name of the camera to watch
    def __init__(self,cameraName):
        self.name = rospy.get_name()
        self.bridge = CvBridge()
        self.image_lock = thread.allocate_lock()
        self.latest_image = None
        self.cameraName = cameraName
        self.cameraTopic = "%s/image_rect_color"%cameraName
        self.camera_sub = rospy.Subscriber(self.cameraTopic,Image,self.update_image)
        
        self.snapshot_serv = rospy.Service("%s/get_snapshot"%self.name,GetSnapshot,self.get_snapshot)
        self.saver_serv = rospy.Service("%s/save_snapshot"%self.name,SaveSnapshot,self.save_snapshot)
    ##   Updates the iomage, given a new packet of camera data
    #    @param data The camera data (in Image format)
    def update_image(self,data):
        self.set_image(data)
        
    def set_image(self,image):
        self.image_lock.acquire()
        self.latest_image = image
        self.image_lock.release()
    
    def get_image(self):
        image = None
        self.image_lock.acquire()
        image = self.latest_image
        self.image_lock.release()
        return image
        
    def get_snapshot(self,req):
        image = self.get_image()
        return GetSnapshotResponse(image)
        
    def save_snapshot(self,req):
        filepath = req.filepath
        image = self.get_image()
        try:
            cv_image = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError, e:
            print e
        now = time.localtime()
        name = "%s_%04d-%02d-%02d__%02d-%02d-%02d.png"%(self.name,now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
        filename = "%s/%s"%(filepath,name)
        cv.SaveImage(filename,cv_image)
        return SaveSnapshotResponse()
        

## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("snapshotter")
    cameraName = rospy.get_param("~camera","defaultSnapshotterCamera")
    snap = Snapshotter(cameraName=cameraName)
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
