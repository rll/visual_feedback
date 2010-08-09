#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("passive_shape")
import rospy
from numpy import *
import pyflann
import math
import cv
import os.path
import pickle
import Geometry2D
import Vector2D
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from stereo_click.msg import ClickPoint
from snapshotter.msg import Snapshot,ImageResult

SHOW_CONTOURS = True
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False
SHOW_ITER = True

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 5.0


class PassiveShapeNode:
    def __init__(self):
        self.name = rospy.get_name()
        self.modelpath = rospy.get_param("~model","model.pickle")
        corrected_modelpath = os.path.expanduser(self.modelpath)
        self.input_topic = rospy.get_param("~input","%s/input"%self.name)
        self.output_topic = rospy.get_param("~output","%s/output"%self.name)
        self.slider_pos = rospy.get_param("~threshold",95)
        self.left_to_right = rospy.get_param("~left_to_right",True)
        self.load_model(corrected_modelpath)
        
        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher(self.output_topic,ImageResult)
        self.image_pub = rospy.Publisher(self.output_topic+"_images",Image)
        self.sub = rospy.Subscriber(self.input_topic,Snapshot,self.handle_input)
        
        
    def handle_input(self,snapshot):
        img = snapshot.image
        try:
            image_raw = self.bridge.imgmsg_to_cv(img, "bgr8")
        except CvBridgeError, e:
            print "CVERROR!!!"
        image_gray = cv.CreateImage(cv.GetSize(image_raw),8,1)
        
        cv.CvtColor(image_raw,image_gray,cv.CV_RGB2GRAY)
        self.image_gray = image_gray
        self.image_raw = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.Copy(image_raw,self.image_raw)
        image_hsv = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
        self.flann = pyflann.FLANN()
        self.dist_fxn = l2_norm
        hue = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        sat = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        val = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        trash = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        cv.Split(image_hsv,hue,None,None,None)
        self.image = hue

        (pts,annotated_image) = self.process_image(self.slider_pos)
        try:
            
            self.publish_result(snapshot,pts,annotated_image)
        except CvBridgeError, e:
            print "CVERRORAHHHH!"
            
    def publish_result(self,snapshot,pts,annotated_image):
        result = ImageResult()
        img = self.bridge.cv_to_imgmsg(annotated_image,"bgr8")
        result.image = img
        for (x,y) in pts:
            result.points.append(ClickPoint(x=x,y=y,camera_info = snapshot.info))
        self.result_pub.publish(result)
        self.image_pub.publish(img)
            
        
            
    def load_model(self,filepath):
        self.model = pickle.load(open(filepath))
        #self.model = modelClass.vertices_full()
        
    def get_model_contour(self):
        return self.model.vertices_full()
        
    def get_dense_model_contour(self):
        return self.model.vertices_dense(constant_length=False,density=20)
        
    def process_image(self,thresh):
        storage = cv.CreateMemStorage(0)
        
        self.image1 = cv.CloneImage( self.image )
        self.image3 = cv.CloneImage( self.image )
        self.image4 = cv.CloneImage( self.image_gray)
        self.image2 = cv.CloneImage( self.image_raw )
        cv.Threshold( self.image, self.image1, thresh, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image_gray, self.image3, thresh, 255, cv.CV_THRESH_BINARY_INV )
        cv.Threshold( self.image_gray, self.image4, thresh, 255, cv.CV_THRESH_BINARY )
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
        if INV_CONTOUR:
            contours = [contour_inv]
        else:
            contours = [contour_reg,contour_gray]
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
            return
        else:
            print area(max_contour)
        shape_contour = max_contour
        if SHOW_CONTOURS:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
        multiplier = 1
        if not self.left_to_right:
            multiplier = -1
        pt = max(shape_contour,key=lambda pt: pt[0]*multiplier)
        self.highlight_pt(pt,cv.CV_RGB(255,255,255))
        pts = [pt]
        return pts,self.image2
        
        
    def image_edge(self,contour):
        width = self.image.width
        height = self.image.height
        for (x,y) in contour:
            #if x < NEAREST_EDGE:
            #    return True
            #if x > width - NEAREST_EDGE:
            #    return True
            if y < NEAREST_EDGE:
                return True
            if y > height - NEAREST_EDGE:
                return True
        return False
        
    def highlight_pt(self,pt,color=None):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(self.image2,pt,5,color,-1)
    

        
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
    if contour == None:
        return 0.0
    ar = abs(cv.ContourArea(contour))
    return ar
    

    
def main(args):
    rospy.init_node("passive_shape_node")
    psn = PassiveShapeNode()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
