
#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("visual_feedback_utils")
import rospy
from numpy import *
import pyflann
import math
import cv
import os.path
import pickle
import Geometry2D
import Vector2D
import Models
import annotator
import thresholding
import shape_fitting_utils
import random

SHOW_CONTOURS = True
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = False
SHOW_ITER = False
SHOW_SYMM_MODEL = False
SHOW_OPT = True
SAVE_ITERS = False
SHOW_FITTED = False
SHOW_INIT_PTS = False 

(BOUNDING, AXIS) = range(2)
SCALE_METHOD = BOUNDING


class ShapeFitter:
    def __init__(self,DIST_FXN="l2",SYMM_OPT=False,FINE_TUNE=False,ORIENT_OPT=True,INITIALIZE=True,HIGH_EXPLORATION=False,ROTATE=True, SHOW=True,SILENT=False,num_iters=100,INIT_APPEARANCE=True):
        if DIST_FXN == "l1":
            self.dist_fxn = l1_norm
        elif DIST_FXN == "l2":
            self.dist_fxn = l2_norm
        else:
            self.dist_fxn = l2_norm
        self.num_iters = num_iters
        self.SYMM_OPT = SYMM_OPT
        self.ORIENT_OPT = ORIENT_OPT
        self.FINE_TUNE = FINE_TUNE   
        self.INITIALIZE=INITIALIZE
        self.HIGH_EXPLORATION = HIGH_EXPLORATION
        self.ROTATE=ROTATE
        self.SHOW=SHOW
        self.SILENT = SILENT
        self.INIT_APPEARANCE = INIT_APPEARANCE
        self.flann = pyflann.FLANN()
        
    

    #Returns the nearest points, the model, and the fitted model
    def fit(self,model,contour,img_annotated=None,img=None):
        #infuriating fix
        x = 0
        cv.NamedWindow("Result",1)
        cv.CreateTrackbar( "Threshold", "Result2", x, 255, lambda n: n )
        
        assert not model.illegal()
        if not img_annotated:
            xs = [x for (x,y) in contour]
            ys = [y for (x,y) in contour]
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
            cv.Set(img_annotated,cv.CV_RGB(255,255,255))
        #model.set_image(cv.CloneImage(img_annotated))
        model.set_image_size((img_annotated.width,img_annotated.height))
        
        shape_contour = contour
        
        if SHOW_CONTOURS:
            cv.DrawContours(img_annotated,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
        if self.INITIALIZE:
            self.printout("INITIALIZING")
            (real_center,real_top,real_theta,real_scale) = get_principle_info(shape_contour)
            if SHOW_UNSCALED_MODEL:
                model.draw_to_image(img_annotated,cv.CV_RGB(0,0,255))
            model_contour = model.vertices_dense(constant_length=False,density=30)
            (model_center,model_top,model_theta,model_scale) = get_principle_info(model_contour)
            displ = displacement(model_center,real_center)
            
            #Drawing
            if SHOW_INIT_PTS:
                top_img = cv.CloneImage(img_annotated)
                cv.DrawContours(top_img,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
                model.draw_contour(top_img,cv.CV_RGB(0,0,255),2)
                draw_pt(top_img,real_top,cv.CV_RGB(255,0,0))
                draw_pt(top_img,real_center,cv.CV_RGB(255,0,0))
                draw_pt(top_img,model_top,cv.CV_RGB(0,0,255))
                draw_pt(top_img,model_center,cv.CV_RGB(0,0,255))
                cv.NamedWindow("Top")
                cv.ShowImage("Top",top_img)
                cv.WaitKey()
            
            self.printout(model_theta)
            self.printout(real_theta)
            angle = model_theta - real_theta
            self.printout(angle)
            #angle = pi/8.0 #FIXME
            if self.ORIENT_OPT:
                angle = 0
            scale = real_scale/float(model_scale)
            if scale < 0.25:
                scale = 1
            print "%f = %f/%f"%(scale,real_scale,model_scale) 
            model_trans = translate_poly(model.polygon_vertices(),displ)
            model_rot = rotate_poly(model_trans,-1*angle,real_center)
            model_scaled = scale_poly(model_rot,scale,real_center)
               
            #(model_center,model_top,model_theta,model_scale) = get_principle_info(model_scaled)
        
                
                #Do the same to the actual model

            model.translate(displ)
            if self.ROTATE:
                model.rotate(-1*angle,real_center)
            model.scale(scale,real_center)
        
            if SHOW_SCALED_MODEL:
                model.draw_to_image(img_annotated,cv.CV_RGB(0,0,255))
    
            #Energy calculation
        self.printout("Energy is: %f"%model.score())
        self.printout("Shape contour has %d points"%(len(shape_contour)))
        sparse_shape_contour = make_sparse(shape_contour,1000)
            
        #Optimize
        if self.ORIENT_OPT:
            init_model = Models.Orient_Model(model,pi/2)
            orient_model_finished = self.black_box_opt(model=init_model,contour=shape_contour,energy_fxn=self.energy_fxn,num_iters = self.num_iters,delta=init_model.preferred_delta(),epsilon = 0.01,mode="orient",image=img) 
            model_oriented = orient_model_finished.transformed_model()
        else:
            model_oriented = model
       
        #FIXME
        if self.INIT_APPEARANCE:
            if model_oriented.beta() < 1:
                model_oriented.initialize_appearance(img)

        if self.SYMM_OPT:
           self.printout("SYMMETRIC OPTIMIZATION")
           new_model_symm = self.black_box_opt(model=model_oriented,contour=shape_contour,energy_fxn=self.energy_fxn,num_iters = self.num_iters,delta=model.preferred_delta(),epsilon = 0.01,mode="symm",image=img)
        else:
            new_model_symm = model_oriented    
        if SHOW_SYMM_MODEL:
           new_model_symm.draw_to_image(img=img_annotated,color=cv.CV_RGB(0,255,0))
        model=new_model_symm.make_asymm()
        if self.HIGH_EXPLORATION:
            exp_factor = 3.0
        else:
            exp_factor = 1.5
        new_model_contour_only = self.black_box_opt(model=model,contour=shape_contour,energy_fxn=self.energy_fxn,num_iters=self.num_iters,delta=model.preferred_delta(),exploration_factor=exp_factor,fine_tune=False,mode="asymm",image=img,contourOnly=True)
        new_model_appearance = self.black_box_opt(model=new_model_contour_only,contour=shape_contour,energy_fxn=self.energy_fxn,num_iters=self.num_iters,delta=model.preferred_delta(),exploration_factor=exp_factor,fine_tune=False,mode="asymm",image=img,contourOnly=False)
        phase1_model = new_model_contour_only
        final_model = new_model_appearance
        final_model.draw_to_image(img=img_annotated,color=cv.CV_RGB(255,0,255))
        nearest_pts = []
        for vert in final_model.save_pts():
            nearest_pt = min(shape_contour,key=lambda pt: Vector2D.pt_distance(pt,vert))
            cv.Circle(img_annotated,nearest_pt,5,cv.CV_RGB(255,255,255),3)
            nearest_pts.append(nearest_pt)
                
        return (nearest_pts,final_model,phase1_model)
    

    def energy_fxn(self,model,contour):
        model_dist_param = 0.5
        contour_dist_param = 0.5
        sparse_contour = make_sparse(contour,1000)
        num_model_pts = 30*len(model.sides())
        
        nn=self.nearest_neighbors_fast
        extra_sparse_contour = make_sparse(contour,num_model_pts)
        model_contour = model.vertices_dense(constant_length=False,density=30)#was 30
        
        nn_model = nn(model_contour,sparse_contour)
        model_dist_energy = sum([self.dist_fxn(dist) for dist in nn_model]) / float(len(nn_model))
        #Normalize
        model_dist_energy /= float(self.dist_fxn(max(model.image.width,model.image.height)))
    
        nn_contour = nn(extra_sparse_contour,model_contour)
        contour_dist_energy = sum([self.dist_fxn(dist) for dist in nn_contour]) / float(len(nn_contour))
        #Normalize
        contour_dist_energy /= float(self.dist_fxn(max(model.image.width,model.image.height)))
        
        energy = model_dist_param * model_dist_energy + contour_dist_param * contour_dist_energy
        penalty = model.structural_penalty()
        energy += penalty
        return energy
        
    def nearest_neighbors_fast(self,model_contour,sparse_contour):
        model_arr = array(model_contour)
        contour_arr = array(sparse_contour)
        result,dists = self.flann.nn(sparse_contour,model_contour, num_neighbors=1,algorithm="kmeans",branching=32, iterations=3, checks=16);
        return [sqrt(dist) for dist in dists]
            
    def nearest_neighbors_local(self,model_contour,sparse_contour):
        num_subdivisions = 10
        overlap = 2
        model_len = len(model_contour)
        match_len = len(sparse_contour)
        #Compute one matching point
        sparse_index = min(range(len(sparse_contour)),key=lambda i: Vector2D.pt_distance(sparse_contour[i],model_contour[0]))
        shifted_contour = list(sparse_contour[sparse_index:])
        shifted_contour.extend(list(sparse_contour[:sparse_index]))
        if Vector2D.pt_distance(model_contour[1],shifted_contour[1]) > Vector2D.pt_distance(model_contour[1],shifted_contour[-1]):
            shifted_contour.reverse()
        
        distances = []
        for subdiv in range(num_subdivisions):
            model_center = model_len*(subdiv)/float(num_subdivisions)
            match_center = match_len*(subdiv)/float(num_subdivisions)
            #double overlap
            model_window_size = int(overlap * model_len / float(num_subdivisions))
            match_window_size = int(overlap * match_len / float(num_subdivisions))
            model_start = int(model_center-model_window_size/2.0)
            model_end = int(model_center+model_window_size/2.0)
            match_start = int(match_center-match_window_size/2.0)
            match_end = int(match_center+match_window_size/2.0)
            if match_start < 0:
                match_sub = shifted_contour[match_len + match_start:]
                match_sub += shifted_contour[:match_end]
            elif match_end > match_len:
                match_sub = shifted_contour[match_start:]
                match_sub += shifted_contour[:match_end%match_len]
            else:
                match_sub = shifted_contour[match_start:match_end]
            if model_start < 0:
                model_sub = model_contour[model_len + model_start:]
                model_sub += model_contour[:model_end]
            elif model_end > model_len:
                model_sub = model_contour[model_start:]
                model_sub += model_contour[:model_end%model_len]
            else:
                model_sub = model_contour[model_start:model_end]
            sub_distances = self.nearest_neighbors_fast(model_sub,match_sub)
            distances.extend(sub_distances[int(0.25*model_window_size):int(0.75*model_window_size)])
        return distances
        
    def nearest_neighbors_dtw(self,model_contour,sparse_contour):
        model_x = array([x for (x,y) in model_contour])
        model_y = array([y for (x,y) in model_contour])
        sparse_x = array([x for (x,y) in sparse_contour])
        sparse_y = array([y for (x,y) in sparse_contour])
        dtw = mlpy.Dtw(onlydist=False)
        score_x = dtw.compute(model_x,sparse_x)
        score_y = dtw.compute(model_y,sparse_y)
        
    def black_box_opt(self,model,contour, energy_fxn,delta = 0.1, num_iters = 100, epsilon = 0.001,exploration_factor=1.5,fine_tune=False,num_fine_tunes=0,mode="asymm",image=None,contourOnly=False):
    
        epsilon = 0.001
        score = -1 * model.score(contourOnly=contourOnly)
        self.printout("Initial score was %f"%score)
        params = model.params()
        deltas = [delta for p in params]
        if(self.SHOW):
            cv.NamedWindow("Optimizing")
            img = cv.CloneImage(image)
            model.from_params(params).draw_to_image(img,cv.CV_RGB(255,0,0))
            cv.ShowImage("Optimizing",img)
            cv.WaitKey(50)
        for it in range(num_iters):
            self.printout("Starting iteration number %d"%it)
            for i in range(len(params)):
                new_params = list(params)
                new_params[i] += deltas[i]
                new_score = -1 * model.from_params(new_params).score(contourOnly=contourOnly)
                if new_score > score:
                    params = new_params
                    score = new_score
                    deltas[i] *= exploration_factor
                else:
                    deltas[i] *= -1
                    new_params = list(params)
                    new_params[i] += deltas[i]
                    new_score = -1 * model.from_params(new_params).score(contourOnly=contourOnly)
                    if new_score > score:
                        params = new_params
                        score = new_score
                        deltas[i] *= exploration_factor  
                    else:
                        deltas[i] *= 0.5
            self.printout("Current best score is %f"%score)
            if(self.SHOW):
                img = cv.CloneImage(image)
                model.from_params(params).draw_to_image(img,cv.CV_RGB(255,0,0))
                if SAVE_ITERS:
                    cv.SaveImage("%s_iter_%d.png"%(mode,it),img)
                cv.ShowImage("Optimizing",img)
                cv.WaitKey(50)
            if max([abs(d) for d in deltas]) < epsilon:
                self.printout("BREAKING")
                break
        return model.from_params(params)
        
    def printout(self,str):
        if not self.SILENT:
            print str
        
        
def draw_line(img,pt1,pt2):
    line = Vector2D.make_ln_from_pts(pt1,pt2)
    ln_start = Vector2D.intercept(line,Vector2D.horiz_ln(y=0))
    ln_end = Vector2D.intercept(line,Vector2D.horiz_ln(y=img.height))
    cv.Line(img,ln_start,ln_end,cv.CV_RGB(255,0,0),2)
        
def draw_pt(img,pt, color=None):
    if not color:
        color = cv.CV_RGB(255,0,0)
    cv.Circle(img,pt,5,color,-1)

def make_sparse(contour,num_pts = 1000):
        sparsity = int(math.ceil(len(contour) / float(num_pts)))
        sparse_contour = []
        for i,pt in enumerate(contour):
            if i%sparsity == 0:
                sparse_contour.append(pt)
        return sparse_contour

def get_principle_info(shape):

        moments = cv.Moments(shape,0)
        center = get_center(moments)
        
        theta = get_angle(moments)
        (top_pt,scale) = get_top(shape,center,theta)
        return(center,top_pt,theta,scale)

def get_top(shape,center,theta):
        pt = center
        EPSILON = 1.0
        angle = theta
        scale = 0
        (r_x,r_y,r_w,r_h) = cv.BoundingRect(shape)
       # #If initially outside, go twice
       # if(cv.PointPolygonTest(shape,pt,0) <= 0):
       #     while(cv.PointPolygonTest(shape,pt,0) <= 0):
       #         (x,y) = pt
       #         new_x = x + EPSILON*sin(angle)
       #         new_y = y - EPSILON*cos(angle)
       #         pt = (new_x,new_y)
       #         scale += EPSILON
       #     while(cv.PointPolygonTest(shape,pt,0) > 0):
       #         (x,y) = pt
       #         new_x = x + EPSILON*sin(angle)
       #         new_y = y - EPSILON*cos(angle)
       #         pt = (new_x,new_y)
       #         scale += EPSILON
        if SCALE_METHOD == AXIS:
            top_pt = center
            best_scale = 1
            while(pt[0] > r_x and pt[0] < r_x+r_w and pt[1] > r_y and pt[1] < r_y+r_w):
                print 
                (x,y) = pt
                new_x = x + EPSILON*sin(angle)
                new_y = y - EPSILON*cos(angle)
                pt = (new_x,new_y)
                scale += EPSILON
                if cv.PointPolygonTest(shape,pt,0) > 0:
                    top_pt = pt
                    best_scale = scale
            return (top_pt,best_scale)
        else:
            return ((r_x + r_w/2, r_y),max(r_w,r_h))
    
        
def l2_norm(val):
    return val**2
    
def l1_norm(val):
    return abs(val)
    
    
def drop_off(fxn,limit):
    return lambda val: fxn(min(val,limit))   

def slack(fxn,limit):
    return lambda val: fxn(max(val,limit)-limit)

def avg(lst):
    return float(sum(lst))/len(lst)
    
def displacement(pt1,pt2):
    (x_1,y_1) = pt1
    (x_2,y_2) = pt2
    return (x_2-x_1,y_2-y_1)
    
def translate_pt(pt,trans):
    (x,y) = pt
    (x_displ,y_displ) = trans
    (x_t,y_t) = (x+x_displ,y+y_displ)
    return (x_t,y_t)

def translate_poly(poly,trans):
    return [translate_pt(pt,trans) for pt in poly]

def rotate_pt(pt,angle,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    off_rot_x = x_n*cos(angle) - y_n*sin(angle)
    off_rot_y = y_n*cos(angle) + x_n*sin(angle)
    rot_x = off_rot_x + x_o
    rot_y = off_rot_y + y_o
    return (rot_x,rot_y)

def rotate_poly(poly,angle,origin=(0,0)):
    return [rotate_pt(pt,angle,origin) for pt in poly]

def scale_pt(pt,amt,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    (x_ns,y_ns) = (amt*x_n,amt*y_n)
    (x_s,y_s) = (x_ns+x_o,y_ns+y_o)
    return (x_s,y_s)

def scale_poly(poly,amt,origin=(0,0)):
    return [scale_pt(pt,amt,origin) for pt in poly]

    
def get_angle(moments):
    mu11 = cv.GetCentralMoment(moments,1,1)
    mu20 = cv.GetCentralMoment(moments,2,0)
    mu02 = cv.GetCentralMoment(moments,0,2)
    return 1/2.0 * arctan( (2 * mu11 / float(mu20 - mu02)))

def get_center(moments):
    m00 = cv.GetSpatialMoment(moments,0,0)
    m10 = cv.GetSpatialMoment(moments,1,0)
    m01 = cv.GetSpatialMoment(moments,0,1)
    x = float(m10) / m00
    y = float(m01) / m00
    return (x,y)
    

