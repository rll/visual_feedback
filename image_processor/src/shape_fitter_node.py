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
import pickle
import Models

SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = False
SHOW_FINAL_MODEL = False
SHOW_POINTS = False
SHOW_ITER = True

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

class ShapeFitterNode(ImageProcessor):
    
    def init_extended(self):
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        self.load_dir = rospy.get_param("~load_dir",self.config_dir)
        self.save_dir = rospy.get_param("~save_dir",self.config_dir)
        modelname = rospy.get_param("~model","model")
        self.mode = rospy.get_param("~mode","default")
        self.transform = rospy.get_param("~transform",False)
        self.matrix_location = "%s/H.yaml"%self.config_dir
        self.modelpath = "%s/%s.pickle"%(self.load_dir,modelname)
        self.filter_pr2 = True
        if self.filter_pr2:
            self.listener = tf.TransformListener()
        
        
    def process(self,cv_image,info,image2=None):
        self.load_model(self.modelpath)
        if self.transform:
            H = cv.Load(self.matrix_location)
            input_image = cv.CloneImage(cv_image)
            cv.WarpPerspective(input_image,cv_image,H,
                    cv.CV_INTER_LINEAR+cv.CV_WARP_INVERSE_MAP+cv.CV_WARP_FILL_OUTLIERS)
        #Use the thresholding module to get the contour out
        if self.filter_pr2:
            shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.GREEN_BG,filter_pr2=True
                                                    ,crop_rect=(133,58,386,355),cam_info=info,listener=self.listener)
        else:
            shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.GREEN_BG,filter_pr2=False
                                                    ,crop_rect=(133,58,386,355),cam_info=info,listener=None)
        #Use the shape_fitting module to fit the model to the contour
        if self.mode=="tee":
            #fitter = shape_fitting.ShapeFitter(SYMM_OPT=True,ORIENT_OPT=False,FINE_TUNE=False)
            fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=True,FINE_TUNE=False)
        elif self.mode=="sweater":
            fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False)
        elif self.mode=="folded":
            fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,INITIALIZE=False)
        else:
            fitter = shape_fitting.ShapeFitter(SYMM_OPT=True,ORIENT_OPT=False,FINE_TUNE=False)
        image_anno = cv.CloneImage(cv_image)
        """
        if self.mode == "folded":
            self.model.draw_to_image(image_anno,cv.CV_RGB(255,0,0))
            return ([self.model.fold_bottom(),self.model.fold_top()],{},image_anno)
        """
        (nearest_pts, final_model, fitted_model) = fitter.fit(self.model,shape_contour,image_anno)
        pts = nearest_pts
        
        params = {}
        
        if self.mode == "triangles":
            return_pts = [pts[1],pts[4],pts[2],pts[3]]
            self.highlight_pt(pts[1],cv.CV_RGB(255,0,0),image_anno)
            font = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX,1.0,1.0)
            cv.PutText(image_anno,"(l)eft",(pts[1][0]-20,pts[1][1]-15),font,cv.CV_RGB(255,0,0))
            self.highlight_pt(pts[4],cv.CV_RGB(0,0,255),image_anno)
            cv.PutText(image_anno,"(r)ight",(pts[4][0]-20,pts[4][1]-15),font,cv.CV_RGB(0,0,255))
            params = {"tilt":0.0}
        elif self.mode == "towel":
            return_pts = pts
        elif self.mode == "tee" or self.mode == "sweater":
            return_pts = pts[0:5]+pts[8:]
            #return_pts[0] = (return_pts[0][0],return_pts[0][1]-10)
            #return_pts[9] = (return_pts[9][0],return_pts[9][1]-10)
            params = {}
        elif self.mode == "folded":
            return_pts = [final_model.fold_bottom(),final_model.fold_top()]
        else:
            return_pts = pts
            params = {}
        """    
        if self.mode != "triangles":
            for pt in return_pts:
                self.highlight_pt(pt,cv.CV_RGB(255,255,255),image_anno)
        """
        if self.transform:
            H_inv = cv.CloneMat(H)
            cv.Invert(H,H_inv)
            anno_unchanged = cv.CloneImage(image_anno)
            cv.WarpPerspective(anno_unchanged,image_anno,H_inv,
                    cv.CV_INTER_LINEAR+cv.CV_WARP_INVERSE_MAP+cv.CV_WARP_FILL_OUTLIERS)
            new_return_pts = self.transform_pts(return_pts,H_inv)
            for i,pt in enumerate(new_return_pts):
                return_pts[i] = pt
        if self.mode != "triangles":
            for pt in return_pts:
                self.highlight_pt(pt,cv.CV_RGB(255,0,0),image_anno)
        if self.mode != "folded":
            fitted_model.set_image(None)
            pickle.dump(fitted_model,open("%s/last_model.pickle"%self.save_dir,'w'))
        else:
            model_pts = final_model.vertices_full()
            new_model = Models.Point_Model_Contour_Only_Asymm(*model_pts)
            pickle.dump(new_model,open("%s/last_model.pickle"%self.save_dir,'w'))
        #Ignore nearest
        #if self.mode == "tee" or self.mode == "sweater":
        #    return_pts = final_model.vertices_full()[0:5] + final_model.vertices_full()[8:]
        return (return_pts,params,image_anno)
        
    def transform_pts(self,pts,M): 
        ptMat = cv.CreateMat(len(pts),1,cv.CV_32FC2)
        for i,pt in enumerate(pts):
            ptMat[i,0] = (pt[0],pt[1])
        returnPtMat = cv.CloneMat(ptMat)
        cv.PerspectiveTransform(ptMat,returnPtMat,M)
        return_pts = []
        for i in range(len(pts)):
            return_pts.append(returnPtMat[i,0][:2])
        return return_pts
    def load_model(self,filepath):
        self.model = pickle.load(open(filepath))
        
    def highlight_pt(self,pt,color,image):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(image,pt,5,color,3)

def main(args):
    rospy.init_node("shape_fitter_node")
    sfn = ShapeFitterNode()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
