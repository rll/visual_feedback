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
import clothing_models.Vector2D
import tf
from geometry_msgs.msg import PointStamped
from image_processor_node import ImageProcessor
from shape_fitting_utils import *
import image_geometry
import thresholding

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
        self.image2 = cv.CloneImage( cv_image )
        
        shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.GREEN_BG,filter_pr2=True,crop_rect=(51,135,560,300),cam_info=info,listener=self.listener)

        multiplier = 1
        if not self.left_to_right:
            multiplier = -1
        pt = max(shape_contour,key=lambda pt: pt[0]*multiplier)
        params = {}
        # Make sure it isn't super close: homogeneity is proportional to the number of pts within 5 pixels of this pt
        params["homogeneity"] = len([p for p in shape_contour if abs(p[0]-pt[0])<=15])/float(len(shape_contour))

        pt_opp = min(shape_contour,key=lambda pt: pt[0]*multiplier)
        self.highlight_pt(pt,cv.CV_RGB(255,255,255))
        self.highlight_pt(pt_opp,cv.CV_RGB(0,0,0))
        pts = [pt,pt_opp]
        params["tilt"] = multiplier*-pi/2
        return (pts,params,self.image2)

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
