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
import visual_feedback_utils.Vector2D
import tf
from geometry_msgs.msg import PointStamped
from image_processor_node import ImageProcessor
from shape_fitting_utils import *
import image_geometry
import thresholding
import shape_fitting
import pickle
import clothing_models.Models

import cvgreyc.features.lbp as lbp
import cvgreyc.features.sift as sift
from scipy.misc import imread

SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = False
SHOW_FINAL_MODEL = False
SHOW_POINTS = False
SHOW_ITER = True

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

(LEFT,RIGHT,UNDETERMINED) = range(3)

RED = cv.CV_RGB(255,0,0)
BLUE = cv.CV_RGB(0,0,255)
GREEN = cv.CV_RGB(0,255,0)

class TriangleFitterNode(ImageProcessor):
    
    def init_extended(self):
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        self.load_dir = rospy.get_param("~load_dir",self.config_dir)
        self.save_dir = rospy.get_param("~save_dir",self.config_dir)
        modelname = rospy.get_param("~model","model")
        self.transform = rospy.get_param("~transform",False)
        self.matrix_location = "%s/H.yaml"%self.config_dir
        self.modelpath = "%s/%s.pickle"%(self.load_dir,modelname)
        self.filter_pr2 = True
        self.num_iters = rospy.get_param("~num_iters",None)
        if self.filter_pr2:
            self.listener = tf.TransformListener()
    

        
    def process(self,cv_image,info,image2=None):
        self.load_model(self.modelpath)
       
        #Use the thresholding module to get the contour out
        shape_contour = thresholding.get_contour(cv_image,bg_mode=thresholding.GREEN_BG,filter_pr2=True
                                                    ,crop_rect=(116,121,431,347),cam_info=info,listener=self.listener)
        #Use shapefitting module to fit a triangles model to the data
        fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,num_iters=self.num_iters)
        image_anno = cv.CloneImage(cv_image)
        (nearest_pts, final_model, fitted_model) = fitter.fit(self.model,shape_contour,image_anno)
        #Get 3 samples from the 3 different regions of interest
        (l_line,r_line) = self.extract_samples(nearest_pts,cv_image,shape_contour)
        lbp_classification = self.lbp_classify()
        if lbp_classification == LEFT:
            left = True
        elif lbp_classification == RIGHT:
            left = False
        else:
            edge_classification = self.edge_classify(cv_image,l_line,r_line)
            if edge_classification == LEFT:
                left = True
            else:
                left = False
        pts = nearest_pts        
        return_pts = [pts[1],pts[4],pts[2],pts[3]]
        params = {"tilt":0.0}
        if left:
            params["left"] = 1.0
            self.draw_seg(image_anno,r_line,RED)
        else:
            params["left"] = 0.0
            self.draw_seg(image_anno,l_line,BLUE)
        
        return (return_pts,params,image_anno)


    def extract_samples(self,nearest_pts,cv_image,contour):
        [center,b_l,t_l,t_r,b_r] = nearest_pts
        l_line = Vector2D.make_seg(center,t_l)
        r_line = Vector2D.make_seg(center,t_r)
        
        l_side = Vector2D.make_seg(b_l,t_l)
        bl_side = Vector2D.make_seg(b_l,center)
        br_side = Vector2D.make_seg(b_r,center)
        r_side = Vector2D.make_seg(b_r,t_r)
        t_side = Vector2D.make_seg(t_l,t_r)
        l_crop_br = Vector2D.extrapolate_pct(bl_side,0.5)
        l_crop_bl = Vector2D.intercept(l_side,Vector2D.horiz_ln(l_crop_br[1]))
        l_crop_tr = Vector2D.intercept(Vector2D.vert_ln(l_crop_br[0]),l_line)
        l_crop_tl = Vector2D.pt_sum(l_crop_bl,Vector2D.pt_diff(l_crop_tr,l_crop_br))
        l_rect = (l_crop_bl,l_crop_br,l_crop_tr,l_crop_tl)
        r_crop_bl = Vector2D.extrapolate_pct(br_side,0.5)
        r_crop_br = Vector2D.intercept(r_side,Vector2D.horiz_ln(r_crop_bl[1]))
        r_crop_tl = Vector2D.intercept(Vector2D.vert_ln(r_crop_bl[0]),r_line)
        r_crop_tr = Vector2D.pt_sum(r_crop_br,Vector2D.pt_diff(r_crop_tl,r_crop_bl))
        r_rect = (r_crop_bl,r_crop_br,r_crop_tr,r_crop_tl)
        t_crop_bl = Vector2D.extrapolate_pct(l_line,0.5)
        t_crop_br = Vector2D.intercept(Vector2D.horiz_ln(t_crop_bl[1]),r_line)
        if t_l[1] > t_r[1]:
            t_crop_tl = Vector2D.intercept(Vector2D.vert_ln(t_crop_bl[0]),t_side)
            t_crop_tr = Vector2D.pt_sum(t_crop_br,Vector2D.pt_diff(t_crop_tl,t_crop_bl))
        else:
            t_crop_tr = Vector2D.intercept(Vector2D.vert_ln(t_crop_br[0]),t_side)
            t_crop_tl = Vector2D.pt_sum(t_crop_bl,Vector2D.pt_diff(t_crop_tr,t_crop_br))
        
        """
        t_rect_old = (t_crop_bl,t_crop_br,t_crop_tr,t_crop_tl)
        
        (orig_t_width,orig_t_height) = Vector2D.rect_size(t_rect_old)
        while cv.PointPolygonTest(contour,Vector2D.pt_scale(Vector2D.pt_sum(t_crop_tl,t_crop_tr),0.5),0) < 0:
            t_crop_tl = (t_crop_tl[0],t_crop_tl[1]+0.05*orig_t_height)
            t_crop_tr = (t_crop_tr[0],t_crop_tr[1]+0.05*orig_t_height)
            print "shrinking t_height"
        """
        t_rect = (t_crop_bl,t_crop_br,t_crop_tr,t_crop_tl)
        
        (l_width,l_height) = Vector2D.rect_size(l_rect)
        (r_width,r_height) = Vector2D.rect_size(r_rect)
        (t_width,t_height) = Vector2D.rect_size(t_rect)
        #print "Height difference:%f"%(t_height - orig_t_height)
        
        width = min(l_width,r_width,t_width) * 0.9
        height = min(l_height,r_height,t_height) * 0.9
        if width < 5:
            width = 5
            print "Hit min"
        if height < 5:
            height = 5
            print "Hit min"
        l_rect_scaled = Vector2D.scale_rect(l_rect,width,height)
        r_rect_scaled = Vector2D.scale_rect(r_rect,width,height)
        t_rect_scaled = Vector2D.scale_rect(t_rect,width,height)
        filenames = ("l_part.png","r_part.png","t_part.png")
        for i,r in enumerate((l_rect_scaled,r_rect_scaled,t_rect_scaled)):
            
            image_temp = cv.CloneImage(cv_image)
            cv.SetImageROI(image_temp,Vector2D.rect_to_cv(r))
            cv.SaveImage("%s/%s"%(self.save_dir,filenames[i]),image_temp)
        return (l_line,r_line)
            
    def lbp_classify(self,P=5,R=5):
        L_img = imread(self.save_dir + '/l_part.png')
        T_img = imread(self.save_dir + '/t_part.png')
        R_img = imread(self.save_dir + '/r_part.png')
        lbp_left = lbp.compute_hist(L_img,P,R,'raw')
        lbp_top = lbp.compute_hist(T_img,P,R,'raw')
        lbp_right = lbp.compute_hist(R_img,P,R,'raw')
        
        print "Norm of L: %f\nNorm of R: %f\nNorm of T: %f\n"%(linalg.norm(lbp_left),linalg.norm(lbp_right),linalg.norm(lbp_top))
        scale = (linalg.norm(lbp_left)+linalg.norm(lbp_right)+linalg.norm(lbp_top))/3.0
        
        l_r_dist = linalg.norm(lbp_left - lbp_right)
        l_t_dist = linalg.norm(lbp_left - lbp_top)
        r_t_dist = linalg.norm(lbp_right - lbp_top)
        
        print "L-to-R: %f\nL-to-T: %f\nR-to-T: %f\n"%(l_r_dist,l_t_dist,r_t_dist)
        print "As fraction of scale:\nL-to-R: %f\nL-to-T: %f\nR-to-T: %f\n"%(l_r_dist/scale,l_t_dist/scale,r_t_dist/scale)
        
        #if abs(l_t_dist/scale - r_t_dist/scale) < 0.05: #Was 0.02
        if l_r_dist/float(scale) < .3:
            return UNDETERMINED
        
        if l_t_dist < r_t_dist:
            return LEFT
        else:
            return RIGHT
            
    def edge_classify(self,cv_image,l_line,r_line):
        image_hsv = cv.CloneImage(cv_image)
        cv.CvtColor(cv_image,image_hsv,cv.CV_RGB2HSV)
        image_hue = cv.CreateImage(cv.GetSize(image_hsv),cv.IPL_DEPTH_8U,1)
        cv.Split(image_hsv,image_hue,None,None,None)
        image_grad = cv.CreateImage(cv.GetSize(image_hsv),cv.IPL_DEPTH_16S,1)
        cv.Sobel(image_hue,image_grad,0,1)
        scores = []
        for line in (l_line,r_line):
            score = self.score_of(line,image_hue)
            print "Score: "
            print score
            scores.append(score)
    
        if scores[0] > scores[1]:
            return RIGHT
        else:
            return LEFT
            
    def score_of(self,line,image):
        image_grad_x = cv.CreateImage(cv.GetSize(image),cv.IPL_DEPTH_16S,1)
        image_grad_y = cv.CreateImage(cv.GetSize(image),cv.IPL_DEPTH_16S,1)
        cv.Sobel(image,image_grad_x,0,1)
        cv.Sobel(image,image_grad_y,0,1)
        scores = [self.score_pt(pt,image_grad_y) for pt in sampled(line,50)]
        return norm(scores)
        
    def score_pt(self,pt,image_grad):
        (pt_x,pt_y) = pt
        kernel_size = 3
        values = []
        for x in range(-1 * kernel_size, kernel_size+1):
            for y in range(-1 * kernel_size,kernel_size+1):
                values.append(abs(image_grad[pt_y+y,pt_x+x]))
        return lst_avg(values)
    
    def load_model(self,filepath):
        self.model = pickle.load(open(filepath))
        
    def highlight_pt(self,pt,color,image):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(image,pt,5,color,3)
        
    def draw_seg(self,img,seg,color):
        (start,end) = Vector2D.end_points(seg)
        cv.Line(img,start,end,color,3)

def lst_avg(lst):
    return sum(lst) / float(len(lst))
    
def lst_var(lst):
    mean = lst_avg(lst)
    total = 0
    for el in lst:
        total += (el - mean)**2
    return total / float(len(lst))
    
def sampled(line,NUM_SAMPLES):
    pts = []
    (start,end) = Vector2D.end_points(line)
    dist = Vector2D.pt_distance(start,end)
    for i in range(NUM_SAMPLES):
        pt = Vector2D.extrapolate(line, dist * i / float(NUM_SAMPLES))
        pts.append(pt)
    return pts
    
def norm(lst):
    total = 0
    for v in lst:
        total += abs(v)
    return total

def main(args):
    rospy.init_node("triangle_fitter_node")
    tfn = TriangleFitterNode()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
