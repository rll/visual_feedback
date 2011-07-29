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

import gtk, pygtk
window = gtk.Window()
screen = window.get_screen()
SCREEN_WIDTH = screen.get_width()
SCREEN_HEIGHT = screen.get_height()

VIEW_MODES = [PLAIN, PATCH] = range(2);

class ViewWindow( ZoomWindow ):
    def __init__(self, image, zoom_out, feature_map, id):
        self.image = image
        self.feature_map = feature_map
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.view_mode = PLAIN
        ZoomWindow.__init__(self,"Viewer %d"%id,-1,zoom_out)

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
    parser.add_argument(    '-i','--images',   dest='images', type=str, nargs="+",  
                            required=True,
                            help='the image to compare WITH' )
    parser.add_argument(    '-f','--features',   dest='features', type=str, nargs="+",
                            required=True,
                            help='features of the reference image' )
    parser.add_argument(    '-z','--zoom-outs',   dest='zoom_outs', type=int, nargs="+",
                            default=1,
                            help='Amount to zoom by' )
                            
    return parser.parse_args()

def main(args):
    n_views = len(args.images)
    assert len(args.features) == len(args.zoom_outs) == n_views
    view_windows = []
    for i in range(n_views):
        image = cv.LoadImage( args.images[i] )
        featuremap = FeatureMap()
        featuremap.read_from_file( args.features[i] )
        zoom_out = args.zoom_outs[i]
        view_window = ViewWindow( image, zoom_out, featuremap, i)
        view_windows.append(view_window)
    #Tile
    x = 0
    y = 0
    for win in view_windows:
        if x + win.size()[0] >= SCREEN_WIDTH:
            x = 0
            y += win.size()[1]
        win.move(x,y)
        x += win.size()[0]
    while(True):
        cont = update_all_windows()
        if not cont:
            break
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

