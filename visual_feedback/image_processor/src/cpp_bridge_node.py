#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
from numpy import *
import math
import cv
from cv_bridge import CvBridge, CvBridgeError
from image_processor.srv import *
from image_processor_node import ImageProcessor
from sensor_msgs.msg import Image
from dynamic_reconfigure.client import Client as ReconfigureClient
from visual_feedback_utils import TopicUtils
from time import strftime

class CPPBridgeNode(ImageProcessor):
    
    def init_extended(self):
        self.cpp_process_service = rospy.get_param("~cpp_service","cpp_process")
        self.multi_exposure = rospy.get_param("~multi_exposure",False)
        self.ignore_first_picture = True
        if self.multi_exposure:
            self.client = ReconfigureClient("prosilica_driver")
            self.client.update_configuration({'exposure':0.4})
        
    def get_second_image(self,image_topic):
        # Set exposure
        print "Changing the exposure of camera to 0.2"
        self.client.update_configuration({'exposure':0.2})
        rospy.sleep(0.1)
        image1 = TopicUtils.get_next_message(image_topic,Image)
        print "Changing the exposure of camera to 0.4"
        self.client.update_configuration({'exposure':0.4})
        rospy.sleep(0.1)
        image2 = TopicUtils.get_next_message(image_topic,Image)
        

        
        return (image1,image2)
    
    def process(self,cv_image,info,cv_image2=None):
        #ignore and get my own pictures
        if self.multi_exposure:
            (image,image2) = self.get_second_image("prosilica/image_rect_color")
        else:
            image = self.bridge.cv_to_imgmsg(cv_image,"bgr8")
            image2 = self.bridge.cv_to_imgmsg(cv_image,"bgr8")
        
        # save and display image in order to document the current experiment
        cvImage1= self.bridge.imgmsg_to_cv(image, "bgr8")
        cvImage2= self.bridge.imgmsg_to_cv(image2, "bgr8")
        timetag= strftime("%Y_%m_%d__%H_%M_%S")
        imageName1= "/tmp/" + timetag + "_exposure_image1.png"
        imageName2= "/tmp/" + timetag + "_exposure_image2.png"      
        print "======= Storing images of 2 exposures ======="
        print "Saving image1: " + imageName1
        cv.SaveImage(imageName1, cvImage1);
        print "Saving image2: " + imageName2
        cv.SaveImage(imageName2, cvImage2);
            
        try:
            cpp_process = rospy.ServiceProxy(self.cpp_process_service,ProcessBridge)
            resp = cpp_process(image,info,image2)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
            return ([],{},cv_image)
        pts_y = resp.pts_y
        pts_x = resp.pts_x
        pts2d = []
        for i in range(len(pts_y)):
            x = pts_x[i]
            y = pts_y[i]
            pts2d.append((x,y))
        params_list = resp.params
        params = {}
        names = ("l","r")
        for i,val in enumerate(params_list):
            params[names[i]] = val
        image_annotated = resp.image_annotated
        try:
            cv_image_annotated = self.bridge.imgmsg_to_cv(image_annotated, "bgr8")
        except CvBridgeError, e:
            "CVERROR converting from cv IplImage to ImageMessage"
        return (pts2d,params,cv_image_annotated)
        
        


def main(args):

    rospy.init_node("cpp_node")
    cn = CPPBridgeNode()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
