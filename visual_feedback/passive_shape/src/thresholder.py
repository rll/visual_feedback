#!/usr/bin/env python
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
import Vector2D
import tf


SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = True
SHOW_POINTS = False

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0
TUNE = False


LIGHT_THRESH = 75
DARK_THRESH = 84
NEW_THRESH = 74
INIT_THRESH = DARK_THRESH

class Thresholder:
    
    def __init__(self,filepath):
        self.savepath = filepath[:len(filepath)-4]+"_thresh.png"
        img = cv.LoadImage(filepath)
        thresh = INIT_THRESH
        if TUNE:
            cv.NamedWindow("Result")
            cv.ShowImage("Result",img)
            cv.CreateTrackbar( "Threshold", "Result", thresh, 255, lambda t: self.process(img,t) )
        self.process(img,thresh)
        if TUNE:
            cv.WaitKey()
        
    def process(self,cv_image,thresh):
        image_raw = cv_image
        image_gray = cv.CreateImage(cv.GetSize(image_raw),8,1)        
        cv.CvtColor(image_raw,image_gray,cv.CV_RGB2GRAY)
        self.image_gray = image_gray
        self.image_raw = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.Copy(image_raw,self.image_raw)
        image_hsv = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
        hue = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        sat = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        val = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        trash = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        cv.Split(image_hsv,hue,sat,val,None)
        r = cv.CreateImage(cv.GetSize(image_raw),8,1)
        g = cv.CreateImage(cv.GetSize(image_raw),8,1)
        b = cv.CreateImage(cv.GetSize(image_raw),8,1)
        cv.Split(image_raw,r,g,b,None)
        
        self.image = hue
        """
        cv.NamedWindow("Hue")
        cv.ShowImage("Hue",hue)
        cv.NamedWindow("Sat")
        cv.ShowImage("Sat",sat)
        cv.NamedWindow("Val")
        cv.ShowImage("Val",val)
        """
        self.image_sat = sat
        self.image_val = val
        self.image_g = g
        #Do the actual computation
        storage = cv.CreateMemStorage(0)
        
        self.image1 = cv.CloneImage( self.image )
        self.image3 = cv.CloneImage( self.image )
        self.image4 = cv.CloneImage( self.image_gray)
        self.image2 = cv.CloneImage( self.image_raw )
        self.image5 = cv.CloneImage( self.image )
        cv.Threshold( self.image, self.image1, thresh, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image, self.image3, 30, 255, cv.CV_THRESH_BINARY_INV )
        cv.Threshold( self.image, self.image5, 1, 255, cv.CV_THRESH_BINARY )
        """
        cv.NamedWindow("image1")
        cv.ShowImage("image1",self.image1)
        """
        #cv.Not(self.image3,self.image3)
        """
        cv.NamedWindow("image3")
        cv.ShowImage("image3",self.image3)
        cv.WaitKey(10)
        return
        """
        
        cv.Or(self.image1,self.image3,self.image3)
        cv.And(self.image3,self.image5,self.image3)
        
        #and_img = cv.CloneImage( self.image_gray)
        #nand_img = cv.CloneImage( self.image_gray)
        #cv.And(self.image3,self.image4,and_img)
        #cv.Not(and_img,nand_img)

        #Filter out edges
        
        #Were 196,775,217,402
        for j in range(self.image3.height):
            for i in range(4):
                self.image3[j,i] = 0
            for i in range(4+629,self.image3.width):
                self.image3[j,i] = 0
        for i in range(self.image3.width):
            for j in range(144):
                self.image3[j,i] = 0
            for j in range(144+329,self.image3.height):
                self.image3[j,i] = 0
        
        
             
        
        
        cv.Not(self.image3,self.image3)
        
        #Take black out of the equation
        #cv.Threshold( self.image3, self.image3, 1,255, cv.CV_THRESH_BINARY_INV)
        
        #contour_reg = cv.FindContours   ( self.image1, storage,
        #                            cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_inv = cv.FindContours   ( self.image3, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        #contour_gray = cv.FindContours   ( self.image4, storage,
        #                            cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        
       
        
        max_length = 0
        max_contour = None
        if INV_CONTOUR:
            contours = [contour_inv]
        else:
            contours = [contour_gray]
        for contour in contours:
            while contour != None:
                length = cv.ContourArea(contour)   
                if length > max_length:
                    max_length = length
                    max_contour = contour
                    print "Replaced with %f"%length
                contour = contour.h_next()
        if max_contour == None:
            print "Couldn't find any contours"
            return ([],{},raw_image)
        else:
            print cv.ContourArea(max_contour)
        shape_contour = max_contour
        
        
        
        if TUNE:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
            cv.NamedWindow("Contours")
            cv.ShowImage("Contours",self.image2)
            cv.WaitKey(10)
            
        #Make a mask out of the contour
        for i in range(self.image_raw.width):
            for j in range(self.image_raw.height):
                if cv.PointPolygonTest(shape_contour,(i,j),0) < 0:
                    self.image_raw[j,i] = (255,255,255)
        cv.NamedWindow("Filtered")
        cv.ShowImage("Filtered",self.image_raw)
        cv.WaitKey(100)
        if TUNE:
            return
        cv.SaveImage(self.savepath,self.image_raw)
        return



def main(args):
    filepath = args[0]
    t = Thresholder(filepath)
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
