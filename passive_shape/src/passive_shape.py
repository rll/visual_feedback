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

SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False

class PassiveShapeMaker:
    def __init__(self,corrected_filepath,corrected_modelpath):
        self.slider_pos = 100
        self.load_model(corrected_modelpath)
        self.image = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image1 = cv.CloneImage( self.image )
        self.image2 = cv.CloneImage( self.image )
        cv.NamedWindow("Source",1)
        cv.NamedWindow("Result",1)
        cv.ShowImage("Source",self.image)
        cv.CreateTrackbar( "Threshold", "Result", self.slider_pos, 255, self.process_image )
        self.process_image(self.slider_pos)
        cv.WaitKey(0)
    
        cv.DestroyWindow("Source")
        cv.DestroyWindow("Result")   
        
    def load_model(self,filepath):
        modelPoly = pickle.load(open(filepath))
        self.model = [vert.toTuple() for vert in modelPoly.vertices()]
        self.model = rotate_poly(self.model,pi/4)
        
        
    def process_image(self,thresh):
        storage = cv.CreateMemStorage(0)
        self.image1 = cv.CloneImage( self.image )
        self.image2 = cv.CloneImage( self.image )
        cv.Threshold( self.image, self.image1, thresh, 255, cv.CV_THRESH_BINARY )
        #cv.Canny(self.image,self.image1,thresh*0.1,thresh*1.5)
        contour = cv.FindContours   ( self.image1, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        max_length = 0
        max_contour = None
        while contour != None:
            length = len(contour)
            if length > max_length:
                max_length = length
                max_contour = contour
            contour = contour.h_next()
        if max_contour == None:
            return
        else:
            print len(max_contour)
        shape_contour = max_contour
        if SHOW_CONTOURS:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,255,255),cv.CV_RGB(255,255,255),0,1,8,(0,0))
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
        if SHOW_SCALED_MODEL:
            cv.PolyLine(self.image2,[model_scaled],1,cv.CV_RGB(100,100,100),2)
        (model_center,model_top,model_theta,model_scale) = self.get_principle_info(model_scaled)
        if SHOW_POINTS:
            self.highlight_pt(model_center,cv.CV_RGB(128,128,128))
            self.highlight_pt(model_top,cv.CV_RGB(128,128,128))
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
