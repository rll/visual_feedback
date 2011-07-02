#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.utils.zoom_window import ZoomWindow, keycommand, cvkeymappings
from patch_vision.slicing.pt_io import PointSet
from patch_vision.utils.formulas import l2_dist

points = []
deleted = []

save_flag = False

MODES = [SET, DELETE, LINE_TILE, BLOCK_TILE] = range(4) 

class GradientViewer( ZoomWindow ):
    def __init__(self, image, zoom_out):
        self.image = image
        self.view_image = cv.CreateImage( (image.width, image.height), cv.IPL_DEPTH_32F, 1)
        self.mode = 0
        self.smooth_param = 3
        ZoomWindow.__init__(self,"GradientViewer", 100, zoom_out)

    def image_to_show( self ):
        if self.mode == 0:
            return self.image
        gray_image = cv.CreateImage( (self.image.width, self.image.height), self.image.depth, 1 )
        cv.CvtColor(self.image, gray_image, cv.CV_BGR2GRAY)
        smooth_image = cv.CreateImage( (gray_image.width, gray_image.height), gray_image.depth, 1 )
        cv.Smooth(gray_image, smooth_image, param1=self.smooth_param, param2=self.smooth_param)
        if self.mode == 1:
            return smooth_image
        dx = cv.CreateImage( (smooth_image.width, smooth_image.height), cv.IPL_DEPTH_32F, 1 )
        cv.Sobel(smooth_image,dx,1,0)
        dy = cv.CreateImage( (smooth_image.width, smooth_image.height), cv.IPL_DEPTH_32F, 1 )
        cv.Sobel(smooth_image,dy,0,1)
        if self.mode == 2:
            return dx
        if self.mode == 3:
            return dy
        else:
            sum = cv.CreateImage( (dx.width,dx.height),dx.depth,1 )
            cv.Add(dx,dy,sum)
            cv.ConvertScale(sum,sum,0.5)
            return sum
            
        return self.view_image

    @keycommand('n','Increment view mode')
    def increment_mode(self):
        self.mode = (self.mode + 1) % 5

    @keycommand('p','Decrement view mode')
    def decrement_mode(self):
        self.mode = (self.mode - 1) % 5

    @keycommand('=','Increment smooth param')
    def inc_smooth(self):
        self.smooth_param += 2
        self.print_smooth()

    @keycommand('-','Decrement smooth param')
    def dec_smooth(self):
        if self.smooth_param > 1:
            self.smooth_param -= 2
        self.print_smooth()

    def print_smooth(self):
        print "Gaussian smoothing with a %d x %d kernel"%(self.smooth_param, self.smooth_param)




def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--input-image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to label' )
                            
    return parser.parse_args()

def main(args):
    image = cv.LoadImage( args.input_image)
    window = GradientViewer( image, 1)
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
