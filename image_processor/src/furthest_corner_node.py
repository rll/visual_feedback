#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
from numpy import *
import math
import cv
import os.path
import pickle
import Geometry2D
import Vector2D
import tf
from geometry_msgs.msg import PointStamped
from image_processor_node import ImageProcessor
from shape_fitting_utils import *
import image_geometry

SHOW_CONTOURS = True
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

class FurthestCornerFinder(ImageProcessor):
    
    def init_extended(self):
        self.threshold = rospy.get_param("~threshold",95)
        self.left_to_right = rospy.get_param("~left_to_right",True)
        self.listener = tf.TransformListener()
        
    def process(self,cv_image,info,image2=None):
        image_raw = cv_image
        image_gray = cv.CreateImage(cv.GetSize(image_raw),8,1)        
        cv.CvtColor(image_raw,image_gray,cv.CV_RGB2GRAY)
        self.image_gray = image_gray
        self.image_raw = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.Copy(image_raw,self.image_raw)
        image_hsv = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
        self.dist_fxn = l2_norm
        hue = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        sat = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        val = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        trash = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        cv.Split(image_hsv,hue,sat,val,None)
        r = cv.CreateImage(cv.GetSize(image_raw),8,1)
        g = cv.CreateImage(cv.GetSize(image_raw),8,1)
        b = cv.CreateImage(cv.GetSize(image_raw),8,1)
        cv.Split(image_raw,r,g,b,None)
        """
        cv.NamedWindow("Raw")
        cv.NamedWindow("Red")
        cv.NamedWindow("Green")
        cv.NamedWindow("Blue")
        cv.ShowImage("Raw",image_raw)
        cv.ShowImage("Red",r)
        cv.ShowImage("Green",g)
        cv.ShowImage("Blue",b)
        
        cv.WaitKey()
        
        rorb = cv.CreateImage(cv.GetSize(image_raw),8,1)
        """
        self.image = hue
        self.image_sat = sat
        self.image_val = val
        self.image_g = g
        #Do the actual computation
        storage = cv.CreateMemStorage(0)
        
        self.image1 = cv.CloneImage( self.image )
        self.image3 = cv.CloneImage( self.image )
        self.image4 = cv.CloneImage( self.image_gray)
        self.image2 = cv.CloneImage( self.image_raw )
        cv.Threshold( self.image, self.image1, 75, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image, self.image3, 140, 255, cv.CV_THRESH_BINARY_INV )  
        cv.Not(self.image3,self.image3)
        cv.Or(self.image1,self.image3,self.image3)
        
        #and_img = cv.CloneImage( self.image_gray)
        #nand_img = cv.CloneImage( self.image_gray)
        #cv.And(self.image3,self.image4,and_img)
        #cv.Not(and_img,nand_img)

         #Filter out grippers
        cam_frame = info.header.frame_id
        now = rospy.Time.now()
        for link in ("l_gripper_l_finger_tip_link","r_gripper_l_finger_tip_link"):
            self.listener.waitForTransform(cam_frame,link,now,rospy.Duration(10.0))
            l_grip_origin = PointStamped()
            l_grip_origin.header.frame_id = link
            l_grip_in_camera = self.listener.transformPoint(cam_frame,l_grip_origin)
            camera_model = image_geometry.PinholeCameraModel()
            camera_model.fromCameraInfo(info)
            (u,v) = camera_model.project3dToPixel((l_grip_in_camera.point.x,l_grip_in_camera.point.y,l_grip_in_camera.point.z))
            if link[0] == "l":
                x_range = range(0,u)
            else:
                x_range = range(u,self.image3.width)
            if 0 < u < self.image3.width and 0 < v < self.image3.height:
                for x in x_range:
                    for y in range(0,self.image3.height):
                        self.image3[y,x] = 0.0
        #Filter out edges
        
        for i in range(15):
            for j in range(self.image3.height):
                self.image3[j,i] = 0.0
                self.image3[j,self.image3.width-i-1] = 0.0
        """
        cv.NamedWindow("Result")
        cv.ShowImage("Result",self.image3)
        cv.WaitKey()      
        """
        contour_reg = cv.FindContours   ( self.image1, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_inv = cv.FindContours   ( self.image3, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_gray = cv.FindContours   ( self.image4, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        
       
        
        max_length = 0
        max_contour = None
        if INV_CONTOUR:
            contours = [contour_inv]
        else:
            contours = [contour_gray]
        for contour in contours:
            while contour != None:
                length = area(contour)   
                if length > max_length and not self.image_edge(contour):
                    max_length = length
                    max_contour = contour
                    print "Replaced with %f"%length
                contour = contour.h_next()
        if max_contour == None:
            print "Couldn't find any contours"
            return ([],{},raw_image)
        else:
            print area(max_contour)
        shape_contour = max_contour
        if SHOW_CONTOURS:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
        multiplier = 1
        if not self.left_to_right:
            multiplier = -1
        pt = max(shape_contour,key=lambda pt: pt[0]*multiplier)
        pt_opp = min(shape_contour,key=lambda pt: pt[0]*multiplier)
        self.highlight_pt(pt,cv.CV_RGB(255,255,255))
        self.highlight_pt(pt_opp,cv.CV_RGB(0,0,0))
        pts = [pt,pt_opp]
        return (pts,{"tilt":multiplier*-pi/2},self.image2)

    def image_edge(self,contour):
        width = self.image.width
        height = self.image.height
        for (x,y) in contour:
            if x < NEAREST_EDGE:
                return True
            if x > width - NEAREST_EDGE:
                return True
            if y < NEAREST_EDGE:
                return True
            if y > height - NEAREST_EDGE:
                return True
        return False
        
    def highlight_pt(self,pt,color=None):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(self.image2,pt,5,color,-1)

def main(args):
    rospy.init_node("furthest_corner_node")
    fcf = FurthestCornerFinder()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
