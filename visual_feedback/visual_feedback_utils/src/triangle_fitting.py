#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("clothing_models")
import rospy
from numpy import *
import math
import cv
import os.path
import pickle
import thresholding
import clothing_models.Vector2D
import shape_fitting
import random

import cvgreyc.features.lbp as lbp
import cvgreyc.features.sift as sift
from scipy.misc import imread

RED = cv.CV_RGB(255,0,0)
BLUE = cv.CV_RGB(0,0,255)
GREEN = cv.CV_RGB(0,255,0)

(LEFT,RIGHT,UNDETERMINED) = range(3)

class TriangleFitter:
    def __init__(self):
        pass
    
    def lbp_classify(self,imgpath,P=5,R=5):
        L_img = imread(imgpath + '/l_part.png')
        T_img = imread(imgpath + '/t_part.png')
        R_img = imread(imgpath + '/r_part.png')
        lbp_left = lbp.compute_hist(L_img,P,R,'raw')
        lbp_top = lbp.compute_hist(T_img,P,R,'raw')
        lbp_right = lbp.compute_hist(R_img,P,R,'raw')
        
        print "Norm of L: %f\nNorm of R: %f\nNorm of T: %f\n"%(linalg.norm(lbp_left),linalg.norm(lbp_right),linalg.norm(lbp_top))
        #scale = (chi_square(lbp_left)+linalg.norm(lbp_right)+linalg.norm(lbp_top))/3.0
        
        #l_r_dist = linalg.norm(lbp_left - lbp_right)
        #l_t_dist = linalg.norm(lbp_left - lbp_top)
        #r_t_dist = linalg.norm(lbp_right - lbp_top)
        dist_fxn = chi_square
        scale = (dist_fxn(lbp_left,lbp_left)+dist_fxn(lbp_right,lbp_right)_dist_fxn(lbp_top,lbp_top))/3.0
        l_r_dist = dist_fxn(lbp_left, lbp_right)
        l_t_dist = dist_fxn(lbp_left, lbp_top)
        r_t_dist = dist_fxn(lbp_right, lbp_top)
        
        print "L-to-R: %f\nL-to-T: %f\nR-to-T: %f\n"%(l_r_dist,l_t_dist,r_t_dist)
        print "As fraction of scale:\nL-to-R: %f\nL-to-T: %f\nR-to-T: %f\n"%(l_r_dist/scale,l_t_dist/scale,r_t_dist/scale)
        
        if abs(l_t_dist/scale - r_t_dist/scale) < 0.02:
            return UNDETERMINED
        
        if l_t_dist < r_t_dist:
            return LEFT
        else:
            return RIGHT
            
    def sift_classify(self,imgpath):
        L_img = imread(imgpath + '/l_part.png')
        T_img = imread(imgpath + '/t_part.png')
        R_img = imread(imgpath + '/r_part.png')
        sift_left = sift.sift(L_img)[0]
        sift_top = sift.sift(T_img)[0]
        sift_right = sift.sift(R_img)[0]
        
        l_r_dist = linalg.norm(sift_left - sift_right)
        l_t_dist = linalg.norm(sift_left - sift_top)
        r_t_dist = linalg.norm(sift_right - sift_top)
        
        print "L-to-R: %f\nL-to-T: %f\nR-to-T: %f\n"%(l_r_dist,l_t_dist,r_t_dist)
        if l_t_dist < r_t_dist:
            return LEFT
        else:
            return RIGHT
        
    def locate(self,imgpath):
        image_raw = cv.LoadImage(imgpath)
        modelpath = "/home/stephen/triangles_snapshots/triangles.pickle"
        model = pickle.load(open(modelpath))
        #Create an image to output
        image_out = cv.CloneImage(image_raw)
        #Use the thresholding module to get the contour out
        shape_contour = thresholding.get_contour(image_raw,bg_mode=thresholding.GREEN_BG,filter_pr2=False,crop_rect=None)
        #Use the shape_fitting module to fit the model to the contour
        fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,INITIALIZE=True,num_iters=30)
        (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_out)    
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
        
        t_rect = (t_crop_bl,t_crop_br,t_crop_tr,t_crop_tl)
        
        (l_width,l_height) = Vector2D.rect_size(l_rect)
        (r_width,r_height) = Vector2D.rect_size(r_rect)
        (t_width,t_height) = Vector2D.rect_size(t_rect)
        
        width = min(l_width,r_width,t_width) * 0.9
        height = min(l_height,r_height,t_height) * 0.9
        
        l_rect_scaled = Vector2D.scale_rect(l_rect,width,height)
        r_rect_scaled = Vector2D.scale_rect(r_rect,width,height)
        t_rect_scaled = Vector2D.scale_rect(t_rect,width,height)
        

        
        filenames = ("l_part.png","r_part.png","t_part.png")
        for i,r in enumerate((l_rect_scaled,r_rect_scaled,t_rect_scaled)):
            
            image_temp = cv.CloneImage(image_raw)
            #cv.CvtColor(image_raw,image_temp,cv.CV_RGB2HSV)
            #image_hue = cv.CreateImage(cv.GetSize(image_temp),cv.IPL_DEPTH_8U,1)
            #cv.Split(image_temp,image_hue,None,None,None)
            cv.SetImageROI(image_temp,Vector2D.rect_to_cv(r))
            cv.SaveImage(filenames[i],image_temp)
            
        response = self.lbp_classify('.')
        if response == RIGHT:
            right = True
        elif response == LEFT:
            right = False
        else:

            image_hsv = cv.CloneImage(image_raw)
            cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
            image_hue = cv.CreateImage(cv.GetSize(image_hsv),cv.IPL_DEPTH_8U,1)
            cv.Split(image_hsv,image_hue,None,None,None)
            image_grad = cv.CreateImage(cv.GetSize(image_hsv),cv.IPL_DEPTH_16S,1)
            cv.Sobel(image_hue,image_grad,0,1)
            cv.SaveImage("grad_"+imgpath,image_grad)
            scores = []
            for line in (l_line,r_line):
                score = self.score_of(line,image_hue)
                print "Score: "
                print score
                scores.append(score)
        
            if scores[0] > scores[1]:
                right = True
            else:
                right = False
    
        
        if right:
            print "RIGHT"
            draw_seg(image_out,l_line,RED)
        else:
            print "LEFT"
            draw_seg(image_out,r_line,BLUE)
        
        cv.SaveImage("labeled_"+imgpath,image_out)

    def convolve(self,image):
        ker = cv.CreateMat((3,3),8,1)
        ker[0,0] = 5
        
    def score_of(self,line,image):
        image_grad_x = cv.CreateImage(cv.GetSize(image),cv.IPL_DEPTH_16S,1)
        image_grad_y = cv.CreateImage(cv.GetSize(image),cv.IPL_DEPTH_16S,1)
        cv.Sobel(image,image_grad_x,0,1)
        cv.Sobel(image,image_grad_y,0,1)
        scores = [self.score_pt(pt,image_grad_y) for pt in sampled(line,50)]
        #print scores
        #return norm(scores)
        """
        print "Variance:"
        var = lst_var(scores)
        if var > 500:
            return -1*var
        """
        return norm(scores)

        
    def score_pt(self,pt,image_grad):
        (pt_x,pt_y) = pt
        kernel_size = 3
        values = []
        for x in range(-1 * kernel_size, kernel_size+1):
            for y in range(-1 * kernel_size,kernel_size+1):
                values.append(abs(image_grad[pt_y+y,pt_x+x]))
        return lst_avg(values)
        
def draw_seg(img,seg,color):
    (start,end) = Vector2D.end_points(seg)
    cv.Line(img,start,end,color,3)

def highlight_pt(img,pt,color=RED):
    cv.Circle(img,pt,10,color,-1)

def lst_avg(lst):
    return sum(lst) / float(len(lst))
    
def lst_var(lst):
    mean = lst_avg(lst)
    total = 0
    for el in lst:
        total += (el - mean)**2
    return total / float(len(lst))
    
def chi_square(v1,v2):
    total = 0
    for i in range(len(v1)):
        total += (v1[i] - v2[i])**2 / float(v1[i] + v2[i]+
    return total
    
def l2(v1,v2):
    return linalg.norm(v1 - v2)
    
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
    imgpath = args[0]
    trf = TriangleFitter()
    trf.locate(imgpath)
    #trf.lbp_classify(".")
    #trf.sift_classify(".")
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    

