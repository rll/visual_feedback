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
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import thread
from snapshotter.srv import *
from snapshotter.msg import *
import time
import os.path
from visual_feedback_utils.pickle_utils import *
from visual_feedback_utils import TopicUtils

## Snapshotter documentation
#
#  A node which, when instantiated, watches a camera feed and stores "snapshots" which can be pulled from a service
class Snapshotter:

    ##   The constructor
    #    @param self The object pointer
    #    @param cameraName The name of the camera to watch
    def __init__(self,cameraName,output_topic):
        self.name = rospy.get_name()
        self.bridge = CvBridge()
        self.image_lock = thread.allocate_lock()
        self.info_lock = thread.allocate_lock()
        self.latest_image = None
        self.latest_info = None
        self.cameraName = cameraName
        self.output_topic = output_topic
        self.output_pub = rospy.Publisher(self.output_topic,Snapshot)
        self.cameraTopic = "%s/image_rect_color"%cameraName
        self.cameraInfoTopic = "%s/camera_info"%cameraName
        self.get_snapshot_serv = rospy.Service("%s/get_snapshot"%self.name,GetSnapshot,self.get_snapshot)
        self.take_snapshot_serv = rospy.Service("%s/take_snapshot"%self.name,TakeSnapshot,self.take_snapshot)
        self.load_snapshot_serv = rospy.Service("%s/load_snapshot"%self.name,LoadSnapshot,self.load_snapshot)
        self.saver_serv = rospy.Service("%s/take_and_save_snapshot"%self.name,TakeAndSaveSnapshot,self.save_snapshot)
    ##   Updates the image, given a new packet of camera data
    #    @param data The camera data (in Image format)
    def update_image(self,image):
        self.set_image(image)
        
    def update_camera_info(self,info):
        self.set_info(info)
        
    def set_image(self,image):
        self.image_lock.acquire()
        self.latest_image = image
        self.image_lock.release()
    
    def get_image(self):
        image = TopicUtils.get_next_message(self.cameraTopic,Image)
        return image
        
    def set_info(self,info):
        self.info_lock.acquire()
        self.latest_info = info
        self.info_lock.release()
    
    def get_info(self):
        info = TopicUtils.get_next_message(self.cameraInfoTopic,CameraInfo)
        return info
        
    def get_snapshot(self,req):
        image = self.get_image()
        info = self.get_info()
        return GetSnapshotResponse(Snapshot(image=image,info=info))
        
    def load_snapshot(self,req):
        cv_image = cv.LoadImage(os.path.expanduser(req.filepath))
        image = self.bridge.cv_to_imgmsg(cv_image,"bgr8")
        if req.infofile:
            info = load_info(os.path.expanduser(req.infofile))
        else:
            info = self.get_info()
        self.output_pub.publish(Snapshot(image=image,info=info))
        return LoadSnapshotResponse()
        
    def take_snapshot(self,req):
        image = self.get_image()
        info = self.get_info()
        self.output_pub.publish(Snapshot(image=image,info=info))
        return TakeSnapshotResponse()
        
    def save_snapshot(self,req):
        try:
            srv = rospy.ServiceProxy("saver/save_snapshot",SaveSnapshot)
            resp = srv(SaveSnapshotRequest(req.filepath,Snapshot(self.get_image(),self.get_info(),[])))
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        return TakeAndSaveSnapshotResponse()


## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("snapshotter")
    cameraName = rospy.get_param("~camera","defaultSnapshotterCamera")
    output_topic = rospy.get_param("~output","%s/snapshot_output"%rospy.get_name())
    print output_topic
    snap = Snapshotter(cameraName=cameraName,output_topic=output_topic)
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
