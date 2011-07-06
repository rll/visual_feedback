#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.utils.zoom_window import ZoomWindow, keycommand, globalkeycommand, update_all_windows
from patch_vision.matching.match_io import MatchSet
from patch_vision.slicing.pt_io import PointSet
from patch_vision.utils.formulas import l2_dist

import gtk, pygtk
window = gtk.Window()
screen = window.get_screen()
SCREEN_WIDTH = screen.get_width()
SCREEN_HEIGHT = screen.get_height()

# Map from points in reference image to points in comparison image
MODES = [SET, SELECT] = range(2)
global mode
mode = SET
IMAGE_TYPES = [COMPARE, REFERENCE] = range(2)
strength = 1
reference_pts = {}
compared_pts = []
strengths = []

global selected_index
selected_index = -1

save_flag = False

class ClickWindow( ZoomWindow ):
    def __init__(self, image, zoom_out, id):
        self.image = image
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.click_pt = None
        self.id = id
        ZoomWindow.__init__(self,"Window %d"%id,-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        highlighted = []
        if self.id != -1 or mode == SET:
            if self.click_pt:
                cv.Circle( self.view_image, self.click_pt, 7*self.zoom_out, cv.RGB(0,255,0), -1 )
        pts = self.get_pts()
        pts_color = cv.RGB(100,100,100)
        if selected_index in pts.keys():
            selected_pt = pts[selected_index]
            highlighted.append(selected_pt)
            if self.id < 0:
                cv.Circle( self.view_image, selected_pt, 30, cv.RGB(255,0,0), -1 )
                pts_color = cv.RGB(75,75,150)
            else:
                cv.Circle( self.view_image, selected_pt, 30, cv.RGB(0,0,255), -1 )
        for pt in self.get_pts().values():
            if pt in highlighted:
                continue
            cv.Circle( self.view_image, pt, 20, pts_color, -1 )
        return self.view_image

    def handleEventsUnzoomed(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            if mode == SET or self.id >= 0:
                self.click_pt = (x,y)
            elif mode == SELECT:
                pts = self.get_pts()
                global selected_index
                selected_index = min( pts.keys(), 
                                      key = lambda i: l2_dist(pts[i], (x,y) ) ) 

    def clear_click_pt(self):
        self.click_pt = None


    def get_pts( self ):
        if self.id == -1:
            return reference_pts
        else:
            return compared_pts[self.id]

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-c','--compared-images',             dest='compared_images', type=str,   
                            required=True, nargs='+',
                            help='the image to compare' )
    parser.add_argument(    '-r','--reference-image',   dest='reference_image', type=str,   
                            required=True,
                            help='the image to compare WITH' )
    parser.add_argument(    '-cz','--compared-zoom-out',   dest='compared_zoom_out', type=int,   
                            default=1,
                            help='Amount to zoom by' )
    parser.add_argument(    '-rz','--reference-zoom-out',   dest='reference_zoom_out', type=int,   
                            default=1,
                            help='Amount to zoom by' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filenames)' )
    parser.add_argument(    '-p','--input-points-file',     dest='input_points_file',    type=str,
                            default=None,
                            help='Reference pts file to start with' )
                            
    return parser.parse_args()

def main(args):
    output_files = []
    compared_images = []
    compared_windows = []
    for i,compared_image_file in enumerate(args.compared_images):
        if args.output_directory:
            directory = args.output_directory
        else:
            directory = os.path.dirname(args.compared_images[0])
            print "No output directory specified. Defaulting to %s"%directory
        if not os.path.exists(directory):
            os.makedirs(directory)
        if args.output_prefix:
            prefix = args.output_prefix
        else:
            prefix_c = os.path.splitext(os.path.basename(args.compared_images[i]))[0]
            prefix_r = os.path.splitext(os.path.basename(args.reference_image))[0]
            prefix = "%s_TO_%s"%(prefix_c, prefix_r) 
            print "No output prefix selected. Defaulting to %s"%prefix
        output_file = "%s/%s.matches"%(directory, prefix)
        output_files.append(output_file)
        compared_image = cv.LoadImage( args.compared_images[i])
        compared_images.append(compared_image)
        compared_window = ClickWindow( compared_image, args.compared_zoom_out, i)
        compared_windows.append(compared_window)
        compared_pts.append({})

    if args.input_points_file:
        point_set = PointSet()
        point_set.read_from_file( args.input_points_file )
        for i,pt in enumerate(point_set.get_points()):
            reference_pts[i] = pt
    reference_image = cv.LoadImage( args.reference_image )
    reference_window = ClickWindow( reference_image, args.reference_zoom_out, -1)
    update_all_windows()
    # Tile
    y = reference_window.size()[1]
    x = 0
    for win in compared_windows:
        if x + win.size()[0] >= SCREEN_WIDTH:
            x = 0
            y += win.size()[1]
        print "Moving window %d to %f,%f"%(win.id,x,y)
        win.move(x,y)
        x += win.size()[0]

    global mode
    global selected_index
    while(True):
        cont = update_all_windows()
        if not cont:
            break
        if reference_window.click_pt:
            print "Updating reference_window click_pt"
            key = safe_max(reference_pts.keys())+1
            reference_pts[key] = reference_window.click_pt
            selected_index = key
            reference_window.clear_click_pt()
        for i,compared_window in enumerate(compared_windows):
            if compared_window.click_pt and selected_index > -1:
                print selected_index
                compared_pts[i][selected_index] = compared_window.click_pt
                compared_window.clear_click_pt()
    if save_flag:
        for i in range(len(compared_windows)):
            match_set = MatchSet(args.compared_images[i], args.reference_image)
            for k in compared_pts[i].keys():
                match_set.add_match( compared_pts[i][k], reference_pts[k], 1 )
            match_set.save_to_file( output_files[i] )
            print "Saved to %s"%output_files[i]

def safe_max( lst ):
    if len(lst) == 0:
        return -1
    else:
        return max( lst )

@globalkeycommand( 'm', "Go to the next mode")
def increment_mode():
    global mode
    mode = (mode + 1) % len(MODES)


@globalkeycommand( 's', "Save the match to file", exits=True )
def save():
    global save_flag
    save_flag = True

@globalkeycommand( 'd', "Delete pt" )
def undo():
    if selected_index == -1:
        return
    reference_pts.pop(selected_index)
    for i in range(len(compared_pts)):
        if selected_index in compared_pts[i].keys():
            compared_pts[i].pop(selected_index)


@globalkeycommand( 'LEFT', "Select previous match")
def select_previous():
    global mode
    mode = SELECT
    global selected_index
    keys = reference_pts.keys()
    idx = keys.index(selected_index)
    idx = idx-1 % len( reference_pts.values())
    selected_index = keys[idx]

@globalkeycommand( 'RIGHT', "Select next match")
def select_next():
    global mode
    mode = SELECT
    global selected_index
    keys = reference_pts.keys()
    idx = keys.index(selected_index)
    idx = idx+1 % len( reference_pts.values())
    selected_index = keys[idx]

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

