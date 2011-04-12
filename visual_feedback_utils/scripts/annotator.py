#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("visual_feedback_utils")
import rospy
from numpy import *
import math
import cv
import os.path
import visual_feedback_utils.thresholding

SNAP = False

class Annotator:
    
    def __init__(self,filepath,num_pts):
        self.image_path = filepath
        self.anno_path = filepath.replace(".png",".anno")
        self.num_pts = num_pts
        self.pts = []
        self.t = []
        self.open = True
        img = cv.LoadImage(self.image_path)
        self.background = self.normalize_image(img)
        self.clearImage()
        cv.NamedWindow("Annotator",1)
        self.contour = thresholding.get_contour(self.img,bg_mode=thresholding.GREEN_BG,filter_pr2=False,crop_rect=None,cam_info=None,listener=None)
        self.showImage()
        
        self.img_gray = cv.LoadImage(self.image_path,cv.CV_LOAD_IMAGE_GRAYSCALE)
        cv.SetMouseCallback("Annotator",self.handleEvents,0)
        
        
    def normalize_image(self,img):
        return img #FIXME: Do scaling/rotation here

        
    def contour_size(self,contour):
        #bounding = cv.BoundingRect(contour)
        #(x,y,width,height) = bounding
        #return width*height
        return abs(cv.ContourArea(contour))
    
    def handleEvents(self,event,x,y,flags,param):
        (x,y) = self.snap((x,y))
        if event==cv.CV_EVENT_LBUTTONUP:
            self.pts.append((x,y))
            self.t.append(True)
            self.highlight((x,y),True)
            self.showImage()
            if len(self.pts) >= self.num_pts:
                self.writeAnno()
                cv.DestroyWindow("Annotator")
                self.open = False
                
        elif event==cv.CV_EVENT_RBUTTONUP:
            if len(self.pts) > 0:
                self.pts = self.pts[0:len(self.pts)-1]
                self.t = self.t[0:len(self.t)-1]
                self.clearImage()
                for i,pt in enumerate(self.pts):
                    self.highlight(pt,self.t[i])
                self.showImage()
            
        elif event==cv.CV_EVENT_MBUTTONUP:
            self.pts.append((x,y))
            self.t.append(False)
            self.highlight((x,y),False)
            self.showImage()
            if len(self.pts) >= self.num_pts:
                self.writeAnno()
                cv.DestroyWindow("Annotator")
                self.open = False
        else:
            self.temp_highlight((x,y),False)
    
    def snap(self,pt):
        if SNAP:
            return min(self.contour, key = lambda c_pt: self.distance(pt,c_pt))
        else:
            return pt
        
    def distance(self,pt1,pt2):
        (x1,y1) = pt1
        (x2,y2) = pt2
        return sqrt((x1-x2)**2 + (y1-y2)**2)
                    
    def highlight(self,pt,landmark=True):
        if landmark:
            color = cv.CV_RGB(255,0,0)
        else:
            color = cv.CV_RGB(0,255,255)
        cv.Circle(self.img,pt,5,color,-1)
        
    def temp_highlight(self,pt,landmark=True):
        color = cv.CV_RGB(0,255,255)
        newimg = cv.CloneImage(self.img)
        cv.Circle(newimg,pt,5,color,-1)
        cv.ShowImage("Annotator",newimg)
            
    def showImage(self):
        cv.ShowImage("Annotator",self.img)
        
    def clearImage(self):
        self.img = cv.CloneImage(self.background)
        
    def writeAnno(self):
        write_anno(self.pts,self.anno_path)
        
def write_anno(pts,filename):
    output = open(filename,'w')
    xs = [x for (x,y) in pts]
    ys = [y for (x,y) in pts]
    output.write("%d\n"%len(pts))
    for i in range(len(pts)):
        output.write("%f\n"%xs[i])
        output.write("%f\n"%ys[i])
    output.close()
    
def read_anno(filename):
    anno_input = open(filename,'r')
    num_pts = int(anno_input.readline())
    pts = []
    for i in range(num_pts):
        x = float(anno_input.readline())
        y = float(anno_input.readline())
        pts.append((x,y))
    anno_input.close()
    return pts
        
    
def main(args):
    filepath = args[0]
    num_pts = int(args[1])
    mm = Annotator(os.path.expanduser(filepath),num_pts)
    cv.WaitKey(10)
    while(mm.open):
        cv.WaitKey(0)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
