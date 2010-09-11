#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
from numpy import *
import pyflann
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
import shape_fitting

SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = False
SHOW_FINAL_MODEL = False
SHOW_POINTS = False
SHOW_ITER = True

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

class ShapeFitter(ImageProcessor):
    
    def init_extended(self):
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        modelname = rospy.get_param("~model","model")
        self.mode = rospy.get_param("~mode","default")
        corrected_modelpath = "%s/%s_model.pickle"%(self.config_dir,modelname)
        self.load_model(corrected_modelpath)
        
    def process(self,cv_image,info,image2=None):
    
        #Use the thresholding module to get the contour out
        shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.WHITE_BG,filter_pr2=True,crop_rect=None)
        #Use the shape_fitting module to fit the model to the contour
        fitter = shape_fitting.ShapeFitter()
        image_anno = cv.CloneImage(cv_image)
        (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_anno)  
        
        pts = nearest_pts
        
        params = {}
        if self.mode == "triangles":
            return_pts = [pts[1],pts[4],pts[2],pts[3]]
            self.highlight_pt(pts[1],cv.CV_RGB(255,0,0),image_anno)
            font = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX,1.0,1.0)
            cv.PutText(self.image2,"(l)eft",(pts[1][0]-20,pts[1][1]-15),font,cv.CV_RGB(255,0,0))
            self.highlight_pt(pts[4],cv.CV_RGB(0,0,255),image_anno)
            cv.PutText(self.image2,"(r)ight",(pts[4][0]-20,pts[4][1]-15),font,cv.CV_RGB(0,0,255))
            params = {"tilt":0.0}
        elif self.mode == "towel":
            return_pts = pts
        else:
            return_pts = pts
            params = {}
        if self.mode != "triangles":
            for pt in return_pts:
                self.highlight_pt(pt,cv.CV_RGB(255,255,255),image_anno)
        return (return_pts,params,image_anno)
        

    def load_model(self,filepath):
        self.model = pickle.load(open(filepath))
        
    def highlight_pt(self,pt,color=None,image):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(self.image,pt,5,color,3)

def main(args):
    rospy.init_node("shape_fitter_node")
    sfn = ShapeFitter()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
