#!/usr/bin/env python

##    @package snapshotter

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
import time
import re
import os.path

class SnapshotSaver:

    ##   The constructor
    #    @param self The object pointer
    def __init__(self):
        self.name = rospy.get_name()
        self.streaming = rospy.get_param("~streaming",False)
        self.input_topic = rospy.get_param("~input","%s/input"%self.name)
        self.default_filepath = rospy.get_param("~default_filepath","~/snapshots")
        self.bridge = CvBridge()
        if self.streaming:
            self.input_sub = rospy.Subscriber(self.input_topic,Snapshot,self.handle_input)
        self.save_serv = rospy.Service("%s/save_snapshot"%self.name,SaveSnapshot,self.save_snapshot)
        
    #Takes a snapshot as input, saves it to the default location
    def handle_input(self,snapshot):
        rospy.loginfo("RECEIVED INPUT!")
        try:
            cv_image = self.bridge.imgmsg_to_cv(snapshot.image, "bgr8")
        except CvBridgeError, e:
            print "CVERROR!!!"
        self.save(cv_image=cv_image,filepath=self.default_filepath,filename=self.generate_name(snapshot))
        
    def save_snapshot(self,req):
        snapshot = req.snapshot
        filepath = req.filepath
        try:
            cv_image = self.bridge.imgmsg_to_cv(snapshot.image, "bgr8")
        except CvBridgeError, e:
            print e
        self.save(cv_image=cv_image,filepath=filepath,filename=self.generate_name(snapshot))
        return SaveSnapshotResponse()
        
    def generate_name(self,snapshot):
        cam_prefix = re.split("_cam_",snapshot.info.header.frame_id)[0]
        filters = ""
        for i,f in enumerate(snapshot.filters):
            filters += f
            if i < len(snapshot.filters)-1:
                filters += "_"
        now = time.localtime()
        timestamp = "%04d-%02d-%02d-%02d-%02d-%02d"%(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
        if len(filters) > 0:
            return "%s_%s_%s.png"%(cam_prefix,filters,timestamp)
        else:
            return "%s_%s.png"%(cam_prefix,timestamp)
        
    
    def save(self,cv_image,filepath,filename):
        corrected_filepath = os.path.expanduser(filepath)
        fullname = "%s/%s"%(corrected_filepath,filename)
        print fullname
        print cv_image
        cv.SaveImage(fullname,cv_image)
        
    
## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("shapshot_saver")
    saver = SnapshotSaver()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
