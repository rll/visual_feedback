#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.extraction.feature_io import FeatureMap, draw_patch
from patch_vision.utils.zoom_window import ZoomWindow, keycommand, update_all_windows

VIEW_MODES = [PLAIN, PATCH] = range(2);

class ReferenceWindow( ZoomWindow ):
    def __init__(self, image, zoom_out, feature_map):
        self.image = image
        self.feature_map = feature_map
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.view_mode = PLAIN
        ZoomWindow.__init__(self,"Reference",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        for i,pt in enumerate(self.feature_map.get_feature_points()):
            shape = self.feature_map.get_shape(pt)
            size = self.feature_map.get_size(pt)
            color = cv.RGB(0,0,0)
            draw_patch( self.view_image, pt, shape, size, color, False, 5 )
        return self.view_image


    def set_shape_map( self, shape_map):
        self.shape_map = shape_map
    
    def set_size_map( self, size_map):
        self.size_map = size_map

    @keycommand('m', "Switch to the next view mode")
    def toggle_mode(self):
        self.view_mode = (self.view_mode + 1) % len(VIEW_MODES);
        print "switched to mode %d" % self.view_mode

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--image',   dest='image', type=str,   
                            required=True,
                            help='the image to compare WITH' )
    parser.add_argument(    '-f','--features',   dest='features', type=str,   
                            required=True,
                            help='features of the reference image' )
    parser.add_argument(    '-z','--zoom-out',   dest='zoom_out', type=int,   
                            default=1,
                            help='Amount to zoom by' )
                            
    return parser.parse_args()

def main(args):
    image = cv.LoadImage( args.image )
    featuremap = FeatureMap()
    featuremap.read_from_file( args.features )
    reference_window = ReferenceWindow( image, args.zoom_out, featuremap)
    while(True):
        cont = update_all_windows()
        if not cont:
            break
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

