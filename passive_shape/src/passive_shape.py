#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("passive_shape")
import rospy
from numpy import *
import math
import cv
import os.path
import pickle
import Geometry2D

SHOW_CONTOURS = True
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False
SHOW_ITER = True

class PassiveShapeMaker:
    def __init__(self,corrected_filepath,corrected_modelpath):
        self.slider_pos = 90
        self.load_model(corrected_modelpath)
        
        image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
        image_white = cv.CloneImage(image_raw)
        for i in range(image_white.width):
            for j in range(image_white.height):
                image_white[j][i] = cv.CV_RGB(255,255,255)
        self.model.draw_to_image(image_white,cv.CV_RGB(0,0,255))
        cv.SaveImage("white_image",image_white)
        return
        self.image_gray = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image_raw = image_raw
        image_hsv = cv.CloneImage(image_raw)
        cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
        print image_hsv
        hue = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        sat = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        val = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        trash = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        cv.Split(image_hsv,hue,None,None,None)
        self.image = hue
        cv.NamedWindow("Source",1)
        cv.NamedWindow("Result",1)
        cv.ShowImage("Source",image_raw)
        cv.CreateTrackbar( "Threshold", "Result", self.slider_pos, 255, self.process_image )
        self.process_image(self.slider_pos)
        cv.WaitKey(0)
    
        cv.DestroyWindow("Source")
        cv.DestroyWindow("Result")   
        
    def load_model(self,filepath):
        modelPoly = pickle.load(open(filepath))
        self.model = [vert.toTuple() for vert in modelPoly.vertices()]

        
        
    def process_image(self,thresh):
        storage = cv.CreateMemStorage(0)
        
        self.image1 = cv.CloneImage( self.image )
        self.image3 = cv.CloneImage( self.image )
        self.image4 = cv.CloneImage( self.image_gray)
        self.image2 = cv.CloneImage( self.image_raw )
        cv.Threshold( self.image, self.image1, thresh, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image, self.image3, thresh, 255, cv.CV_THRESH_BINARY_INV )
        cv.Threshold( self.image_gray, self.image4, thresh, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image_gray, self.image4, thresh, 255, cv.CV_THRESH_BINARY_INV )
        #self.image2 = cv.CloneImage( self.image1 )
        #cv.Canny(self.image,self.image1,thresh*0.1,thresh*1.5)
        contour_reg = cv.FindContours   ( self.image1, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_inv = cv.FindContours   ( self.image3, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_gray = cv.FindContours   ( self.image4, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        #contour_hue = min([contour_reg,contour_inv],key=lambda c: len(c))
        #contour = min([contour_hue,contour_gray],key=lambda c: len(c))
        
        max_length = 0
        max_contour = None
        for contour in (contour_reg,contour_gray):
            while contour != None:
                length = area(contour)   
                if length > max_length and not self.image_edge(contour):
                    max_length = length
                    max_contour = contour
                    print "Replaced with %f"%length
                contour = contour.h_next()
        if max_contour == None:
            return
        else:
            print area(max_contour)
        shape_contour = max_contour
        if SHOW_CONTOURS:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
        #cv.ShowImage("Result",self.image2)
        #return
        (real_center,real_top,real_theta,real_scale) = self.get_principle_info(shape_contour)
        #self.model = translate_poly(rotate_poly(shape_contour,-0.2,real_center),(500,500)) ##FIXME
        if SHOW_UNSCALED_MODEL:
            cv.PolyLine(self.image2,[self.model],1,cv.CV_RGB(0,0,0),2)
        (model_center,model_top,model_theta,model_scale) = self.get_principle_info(self.model)
        displ = displacement(model_center,real_center)
        if SHOW_POINTS:
            self.highlight_pt(real_center,cv.CV_RGB(200,200,200))
            self.highlight_pt(real_top,cv.CV_RGB(200,200,200))
        if SHOW_POINTS:
            self.highlight_pt(model_center,cv.CV_RGB(0,0,0))
            self.highlight_pt(model_top,cv.CV_RGB(0,0,0))
        print model_theta
        print real_theta
        angle = model_theta - real_theta
        print angle
        scale = real_scale/float(model_scale)
        model_trans = translate_poly(self.model,displ)
        model_rot = rotate_poly(model_trans,-1*angle,real_center)
        #scale = 1 #FIXME
        model_scaled = scale_poly(model_rot,scale,real_center)
        
        (model_center,model_top,model_theta,model_scale) = self.get_principle_info(model_scaled)
        if SHOW_POINTS:
            self.highlight_pt(model_center,cv.CV_RGB(128,128,128))
            self.highlight_pt(model_top,cv.CV_RGB(128,128,128))
        

            
        ALPHA = 1.0
        ALPHA_DECAY = 1.0
        BETA = 0.9
        BETA_DECAY = 1.0
        ITERS = 30
        SUB_ITERS = 2
        cv.ShowImage("Result",self.image2)
        
        for k in range(ITERS):
            model_scaled.reverse()
            print "Starthing iteration number: %d"%k
            lengths = [distance(model_scaled[i-1],model_scaled[i]) for i in range(len(model_scaled))]    
            iter_color = cv.CV_RGB(0,0,k*(255.0/ITERS))
            if SHOW_ITER:
                cv.PolyLine(self.image2,[model_scaled],1,iter_color,2)  
            
            print "Lengths: %s"%lengths
            for i in range(len(model_scaled)):
                vert = model_scaled[i]
                nearest_pt = min(shape_contour,key=lambda pt: distance(pt,vert))
                if SHOW_ITER:
                    self.highlight_pt(nearest_pt,iter_color)
                    cv.ShowImage("Result",self.image2)
                    cv.WaitKey(10)
                (dx,dy) = displacement(vert,nearest_pt)
                dx *= ALPHA*(ALPHA_DECAY**k)
                dy *= ALPHA*(ALPHA_DECAY**k)
                model_scaled[i] = translate_pt(vert,(dx,dy))
                
            for j in range(SUB_ITERS):
                #model_scaled.reverse()
                #lengths.reverse()
                #new_lengths = []
                #for i in range(len(lengths)):
                #    new_lengths.append(lengths[i-1])
                #lengths = new_lengths
                
                for i in range(len(model_scaled)):
                    old_length = lengths[i]
                    new_length = distance(model_scaled[i-1],model_scaled[i])
                    side = displacement(model_scaled[i-1],model_scaled[i])
                    move_amt = (old_length - new_length) / float(new_length) * BETA*(BETA_DECAY**k) / SUB_ITERS
                    
                    move_displ = scale_pt(side,move_amt)
                    print "Old_length: %f, New_length: %f, Move_amount: %f,Length of Move Displ:%f"%(old_length,new_length,move_amt,distance(move_displ,(0,0)))
                    model_scaled[i] = translate_pt(model_scaled[i],scale_pt(move_displ,1.0))
                    model_scaled[i-1] = translate_pt(model_scaled[i-1],scale_pt(move_displ,-0.0))
                """
                for i in range(len(model_scaled)-2,-2,-1):
                    old_length = lengths[i+1]
                    new_length = distance(model_scaled[i+1],model_scaled[i])
                    side = displacement(model_scaled[i+1],model_scaled[i])
                    move_amt = (old_length - new_length) / float(new_length) * BETA
                    
                    move_displ = scale_pt(side,move_amt)
                    print "Old_length: %f, New_length: %f, Move_amount: %f,Length of Move Displ:%f"%(old_length,new_length,move_amt,distance(move_displ,(0,0)))
                    model_scaled[i] = translate_pt(model_scaled[i],scale_pt(move_displ,1.0))
                    #model_scaled[i-1] = translate_pt(model_scaled[i-1],scale_pt(move_displ,-0.5))
                 """
        if SHOW_SCALED_MODEL:
            cv.PolyLine(self.image2,[model_scaled],1,cv.CV_RGB(0,0,255),2)
              
        
        for vert in model_scaled:
            nearest_pt = min(shape_contour,key=lambda pt: distance(pt,vert))
            self.highlight_pt(nearest_pt,cv.CV_RGB(255,255,255))
            
        """
        for pt in shape_contour:
            rot_pt = rotate(pt,1*theta,center)
            cv.Circle(self.image2,rot_pt,2,cv.CV_RGB(0,0,0),-1)
        """
        cv.ShowImage("Result",self.image2)
        return
        
    def image_edge(self,contour):
        width = self.image.width
        height = self.image.height
        for (x,y) in contour:
            if x < 5:
                return True
            if x > width - 5:
                return True
            if y < 5:
                return True
            if y > height - 5:
                return True
        return False
        
    def highlight_pt(self,pt,color=None):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(self.image2,pt,5,color,-1)
    
    
    def get_principle_info(self,shape):
        """
        storage2 = cv.CreateMemStorage(0)
        bounding = cv.MinAreaRect2(shape,storage2)
        #(x, y, width, height) = bounding
        #center_x = x + width/2
        #center_y = y + width/2
        #center_x = avg([x for (x,y) in shape_contour])
        #center_y = avg([y for (x,y) in shape_contour])
        center = (bounding[0])
        """
        moments = cv.Moments(shape,0)
        center = get_center(moments)
        self.moments = moments
        
        theta = get_angle(moments)
        (top_pt,scale) = self.get_top(shape,center,theta)
        #scale = distance(center,top_pt)
        print "Scale = %s"%scale
        return(center,top_pt,theta,scale)
        
    def get_top(self,shape,center,theta):
        pt = center
        EPSILON = 1.0
        angle = theta
        scale = 0
        print "ANGLE = %s"%angle
        while(cv.PointPolygonTest(shape,pt,0) > 0):
            (x,y) = pt
            new_x = x + EPSILON*sin(angle)
            new_y = y - EPSILON*cos(angle)
            pt = (new_x,new_y)
            scale += EPSILON
        return (pt,scale)
        
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

def distance(pt1,pt2):
    (x_1,y_1) = pt1
    (x_2,y_2) = pt2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
    
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
    
def area(contour):
    return abs(cv.ContourArea(contour))
    
    
    
    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpath = args[1]
    #corrected_filepath = os.path.expanduser(filepath)
    print corrected_filepath
    psm = PassiveShapeMaker(corrected_filepath,corrected_modelpath)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
