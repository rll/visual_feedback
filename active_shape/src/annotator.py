#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("active_shape")
import rospy
from numpy import *
import math
import cv
import os.path


class Annotator:
    
    def __init__(self,filepath,num_pts):
        self.image_path = filepath
        self.anno_path = filepath.replace(".gif",".anno")
        self.gray_path = filepath.replace(".gif","_gray.png")
        self.num_pts = num_pts
        self.pts = []
        self.t = []
        self.open = True
        img = cv.LoadImage(self.image_path)
        self.background = self.normalize_image(img)
        self.clearImage()
        cv.NamedWindow("Annotator",1)
        self.showImage()
        cv.SetMouseCallback("Annotator",self.handleEvents,0)
        
        
    def normalize_image(self,img):
        return img #FIXME: Do scaling/rotation here
    
    def handleEvents(self,event,x,y,flags,param):
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
                    
    def highlight(self,pt,landmark=True):
        if landmark:
            color = cv.CV_RGB(255,0,0)
        else:
            color = cv.CV_RGB(0,255,255)
        cv.Circle(self.img,pt,2,color,-1)
            
    def showImage(self):
        cv.ShowImage("Annotator",self.img)
        
    def clearImage(self):
        self.img = cv.CloneImage(self.background)
        
    def writeAnno(self):
        output = open(self.anno_path,'w')
        xs = [x for (x,y) in self.pts]
        ys = [y for (x,y) in self.pts]
        vect = xs+ys
        output.write("%d\n"%len(self.pts))
        for i in vect:
            output.write("%d\n"%i)
        for is_landmark in self.t:
            if is_landmark:
                writeval = 0
            else:
                writeval = 2
            output.write("%d\n"%writeval)
        gray = cv.LoadImage(self.image_path,cv.CV_LOAD_IMAGE_GRAYSCALE)
        print self.gray_path
        cv.SaveImage(self.gray_path,gray)
        output.close()
    
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
