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

class PointWindow( ZoomWindow ):
    def __init__(self, image, zoom_out):
        self.image = image
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.mode = SET
        self.tile_params = []
        ZoomWindow.__init__(self,"PointWindow", 100, zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        pts_color = cv.RGB(0,0,255)
        for pt in points:
            cv.Circle( self.view_image, pt, 5*self.zoom_out, pts_color, -1 )
        if self.mode == BLOCK_TILE or self.mode == LINE_TILE:
            if len(self.tile_params) >= 1:
                cv.Circle( self.view_image, self.tile_params[0], 5*self.zoom_out, cv.RGB(255,0,0),-1)
            if len(self.tile_params) >= 2:
                cv.Circle( self.view_image, self.tile_params[1], 5*self.zoom_out, cv.RGB(150,0,150),-1)
            if len(self.tile_params) >= 3:
                cv.Circle( self.view_image, self.tile_params[2], 5*self.zoom_out, cv.RGB(255,0,255),-1)
            if len(self.tile_params) >= 4:
                cv.Circle( self.view_image, self.tile_params[3], 5*self.zoom_out, cv.RGB(0,255,255),-1)
            if len(self.tile_params) >= 5:
                cv.Circle( self.view_image, self.tile_params[4], 5*self.zoom_out, cv.RGB(255,255,255),-1)
        return self.view_image

    def handleEventsUnzoomed(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            if self.mode == SET:
                points.append( (x,y) )
            elif self.mode == DELETE:
                i = min( range(len(points)), 
                         key = lambda i: l2_dist(points[i], (x,y) ) )
                deleted.append(points.pop(i))
            elif self.mode == LINE_TILE:
                if len(self.tile_params) < 3:
                    self.tile_params.append( (x,y) )
            elif self.mode  == BLOCK_TILE:
                if len(self.tile_params) < 5:
                    self.tile_params.append( (x,y) )

    @keycommand('d', "Go to DELETE Mode" )
    def delete_mode(self):
        self.mode = DELETE

    @keycommand('a', "Go to SET Mode" )
    def set_mode(self):
        self.mode = SET

    @keycommand('l', "Go to LINE_TILE Mode" )
    def line_tile_mode(self):
        self.mode = LINE_TILE

    @keycommand('b', "Go to BLOCK Mode" )
    def block_mode(self):
        self.mode = BLOCK_TILE

    @keycommand('ENTER', "In TILE Mode: create the tile from the selected parameters")
    def commit_params(self):
        if self.mode == BLOCK_TILE:
            if len(self.tile_params) >= 5:
                self.block_tile()
                self.tile_params = []
        elif self.mode == LINE_TILE:
            if len(self.tile_params) >= 3:
                self.line_tile()
                self.tile_params = []

    @keycommand('u', "Undo the last change made in the current mode")
    def undo(self):
        if self.mode == SET:
            if len(points) > 0:
                points.pop()
        elif self.mode == DELETE:
            if len(deleted) > 0:
                points.append(deleted.pop())
        elif self.mode == BLOCK_TILE or self.mode == LINE_TILE:
            if len(self.tile_params) > 0:
                self.tile_params.pop()
            elif len(points) > 0:
                points.pop()

    @keycommand('s', "Save the points and quit", exits=True)
    def save(self):
        global save_flag
        save_flag = True

    def line_tile(self):
        [start, first, end] = self.tile_params
        # Get the line
        disp = np.array(end) - np.array(start)
        direction = disp / ( np.sqrt(np.dot(disp, disp) ) )
        disp_start = np.array(first) - np.array(start)
        
        amt = np.dot( disp_start, direction )
        step = amt * direction
        pt = np.array( start )
        while np.dot(pt,direction) < np.dot(end,direction):
            points.append( tuple(pt) )
            pt += step

    def block_tile(self):
        [start, first_u, end_u, first_v, end_v] = self.tile_params
        # Get the line
        disp_u = np.array(end_u) - np.array(start)
        dir_u = disp_u / ( np.sqrt(np.dot(disp_u, disp_u) ) )
        amt_u = np.dot (np.array(first_u) - np.array(start), dir_u )
        step_u = amt_u * dir_u
        disp_v = np.array(end_v) - np.array(start)
        dir_v = disp_v / ( np.sqrt(np.dot(disp_v, disp_v) ) )
        amt_v = np.dot (np.array(first_v) - np.array(start), dir_v )
        step_v = amt_v * dir_v

        step_amt_u = np.array((0,0))
        while np.dot(step_amt_u,step_amt_u) <= np.dot(disp_u,disp_u):
            step_amt_v = np.array((0,0))
            while np.dot(step_amt_v,step_amt_v) <= np.dot(disp_v,disp_v):
                points.append( tuple(start + step_amt_u + step_amt_v) )
                step_amt_v += step_v
            step_amt_u += step_u

    def get_pts( self ):
        if self.id == COMPARE:
            return compare_pts
        else:
            return reference_pts

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--input-image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to label' )
    parser.add_argument(    '-z','--zoom-out',                dest='zoom_out', type=int,   
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
                            help='PointSet file to start with' )
    parser.add_argument(    '-f','--force-overwrite',      dest='force_overwrite',
                            action='store_true',           default=False,
                            help='Force to overwrite existing PointSet without looking at contents' )
    
                            
    return parser.parse_args()

def main(args):
    if args.output_directory:
        directory = args.output_directory
    else:
        directory = os.path.dirname(args.input_image)
        print "No output directory specified. Defaulting to %s"%directory
    if not os.path.exists(directory):
        os.makedirs(directory)
    if args.output_prefix:
        prefix = args.output_prefix
    else:
        prefix = os.path.splitext(os.path.basename(args.input_image))[0]
        print "No output prefix selected. Defaulting to %s"%prefix
    output_file = "%s/%s.pts"%(directory, prefix)
    if not args.input_points_file:
        args.input_points_file = output_file
    if not args.force_overwrite and os.path.exists( args.input_points_file ):
        point_set_old = PointSet( )
        point_set_old.read_from_file( args.input_points_file )
        for pt in point_set_old.get_points():
            points.append( pt )
    image = cv.LoadImage( args.input_image)
    window = PointWindow( image, args.zoom_out)

    if save_flag:
        point_set = PointSet()
        for pt in points:
            point_set.add_point( pt )
        point_set.save_to_file( output_file )
        print "Saved to %s"%output_file
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

