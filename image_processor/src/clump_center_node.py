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
import thresholding

SHOW_CONTOURS = True
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

class ClumpCenterFinder(ImageProcessor):
    
    def init_extended(self):
        self.threshold = rospy.get_param("~threshold",95)
        self.left_to_right = rospy.get_param("~left_to_right",True)
        self.listener = tf.TransformListener()
        
    def process(self,cv_image,info,image2=None):
        self.image2 = cv.CloneImage( cv_image )
        
        shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.GREEN_BG,filter_pr2=True,crop_rect=(120,170,390,267),cam_info=info,listener=self.listener)

        moments = cv.Moments(shape_contour,0)
        pt = get_center(moments)
        self.highlight_pt(pt,cv.CV_RGB(255,255,255))
        pts = [pt]
        return (pts,{},self.image2)

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
    rospy.init_node("clump_center_node")
    fcf = ClumpCenterFinder()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
