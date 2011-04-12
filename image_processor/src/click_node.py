#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
from numpy import *
import math
import cv


from image_processor_node import ImageProcessor


class ClickNode(ImageProcessor):
    
    def init_extended(self):
        self.clicked = False
        self.point = (0.0,0.0)
        return
        

        
    def process(self,cv_image,info,image2=None):
        self.clicked = False
        self.name = "ClickNode"
        cv.NamedWindow(self.name)
        cv.ShowImage(self.name,cv_image)
        cv.SetMouseCallback(self.name,self.onMouse,0)
        while not self.clicked:
            cv.WaitKey(5)
        cv.WaitKey(10)
        cv.DestroyWindow(self.name)
        for i in range(10):
			cv.WaitKey(1)
        return ([self.point],{},cv_image)
        
    def onMouse(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONUP:
            self.point = (x,y)
            self.clicked = True
        
        


def main(args):
    rospy.init_node("click_node")
    cn = ClickNode()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
