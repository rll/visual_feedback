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
import pickle
from pickle_utils import *

class SnapshotSaver:

    ##   The constructor
    #    @param self The object pointer
    def __init__(self):
        self.name = rospy.get_name()
        self.streaming = rospy.get_param("~streaming",False)
        self.save_camera_info = rospy.get_param("~save_camera_info",False)
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
        self.save(cv_image=cv_image,filepath=self.default_filepath,filename=self.generate_name(snapshot),camera_info = snapshot.info)
        
    def save_snapshot(self,req):
        snapshot = req.snapshot
        filepath = req.filepath
        camera_info = snapshot.info
        try:
            cv_image = self.bridge.imgmsg_to_cv(snapshot.image, "bgr8")
        except CvBridgeError, e:
            print e
        self.save(cv_image=cv_image,filepath=filepath,filename=self.generate_name(snapshot),camera_info = camera_info)
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
        
    
    def save(self,cv_image,filepath,filename,camera_info):
        corrected_filepath = os.path.expanduser(filepath)
        fullname = "%s/%s"%(corrected_filepath,filename)
        cv.SaveImage(fullname,cv_image)
        print "Saved image to %s"%fullname
        if self.save_camera_info:
            infofullname = fullname.replace(".png",".pickle")
            infofile = open(infofullname,'w')
            info_pickle = InfoPickle(camera_info)
            pickle.dump(info_pickle,infofile)
            print "Saved camera info to %s"%infofullname
            intrinsicsfile = fullname.replace(".png","_intrinsics.xml")
            cv.Save(intrinsicsfile,self.intrinsic_matrix_from_info(camera_info),"intrinsics","Intrinsics for our camera")
            print "Saved intrinsics to %s"%intrinsicsfile
            distfile = fullname.replace(".png","_distortion.xml")
            cv.Save(distfile,self.dist_coeff_from_info(camera_info),"dist_coeff","Distortion coefficients for our camera")
            print "Saved distortion to %s"%distfile
        
    def intrinsic_matrix_from_info(self, cam_info):
       intrinsic_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)

       #Because we only want the upper 3x3 (normal) portion of the rectified intrinsic matrix
       for i in range(0, 3):
         for j in range(0, 3):
           intrinsic_matrix[i, j] = cam_info.P[4*i+j]
       return intrinsic_matrix
       
    def dist_coeff_from_info(self,cam_info):
        dist_coeff = cv.CreateMat(1, len(cam_info.D), cv.CV_32FC1)
        for i in range(0,len(cam_info.D)-1):
            dist_coeff[0,i] = cam_info.D[i]
        return dist_coeff
        
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
