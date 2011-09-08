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
from patch_vision.utils.formulas import l2_dist

# Map from points in reference image to points in comparison image
MODES = [SET, SELECT] = range(2)
global mode
mode = SET
IMAGE_TYPES = [COMPARE, REFERENCE] = range(2)
strength = 1
reference_pts = []
compare_pts = []
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
        if mode == SET:
            if self.click_pt:
                cv.Circle( self.view_image, self.click_pt, 7*self.zoom_out, cv.RGB(0,255,0), -1 )
            pts_color = cv.RGB(100,100,100)
        elif mode == SELECT:
            if selected_index != -1:
                selected_pt = self.get_pts()[selected_index]
                highlighted.append(selected_pt)
                cv.Circle( self.view_image, selected_pt, 7*self.zoom_out, cv.RGB(0,0,255), -1 )
            pts_color = cv.RGB(75,75,150)
        for pt in self.get_pts():
            if pt in highlighted:
                continue
            cv.Circle( self.view_image, pt, 5*self.zoom_out, pts_color, -1 )
        return self.view_image

    def handleEventsUnzoomed(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            if mode == SET:
                self.click_pt = (x,y)
            elif mode == SELECT:
                pts = self.get_pts()
                global selected_index
                selected_index = min( range(len(pts)), 
                                      key = lambda i: l2_dist(pts[i], (x,y) ) ) 
                print_selected_info()

    def clear_click_pt(self):
        self.click_pt = None


    def get_pts( self ):
        if self.id == COMPARE:
            return compare_pts
        else:
            return reference_pts

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-c','--compared-image',             dest='compared_image', type=str,   
                            required=True,
                            help='the image to compare' )
    parser.add_argument(    '-r','--reference-image',   dest='reference_image', type=str,   
                            required=True,
                            help='the image to compare WITH' )
    parser.add_argument(    '-cz','--compare-zoom-out',   dest='compare_zoom_out', type=int,   
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
    parser.add_argument(    '-m','--input-match-file',     dest='input_match_file',    type=str,
                            default=None,
                            help='Match file to start with' )
    parser.add_argument(    '-f','--force-overwrite',      dest='force_overwrite',
                            action='store_true',           default=False,
                            help='Force to overwrite existing MatchSet without looking at contents' )
    
                            
    return parser.parse_args()

def main(args):
    if args.output_directory:
        directory = args.output_directory
    else:
        directory = os.path.dirname(args.compared_image)
        print "No output directory specified. Defaulting to %s"%directory
    if not os.path.exists(directory):
        os.makedirs(directory)
    if args.output_prefix:
        prefix = args.output_prefix
    else:
        prefix_c = os.path.splitext(os.path.basename(args.compared_image))[0]
        prefix_r = os.path.splitext(os.path.basename(args.reference_image))[0]
        prefix = "%s_TO_%s"%(prefix_c, prefix_r) 
        print "No output prefix selected. Defaulting to %s"%prefix
    output_file = "%s/%s.matches"%(directory, prefix)
    if not args.input_match_file:
        args.input_match_file = output_file
    if not args.force_overwrite and os.path.exists( args.input_match_file ):
        match_set_old = MatchSet( args.compared_image, args.reference_image )
        match_set_old.read_from_file( args.input_match_file )
        for match in match_set_old.get_matches():
            compare_pts.append(match.compare_pt)
            reference_pts.append(match.reference_pt)
            strengths.append(match.strength)
    compared_image = cv.LoadImage( args.compared_image)
    reference_image = cv.LoadImage( args.reference_image )
    compare_window = ClickWindow( compared_image, args.compare_zoom_out, COMPARE)
    reference_window = ClickWindow( reference_image, args.reference_zoom_out, REFERENCE)

    global mode
    global strength
    while(True):
        cont = update_all_windows()
        if not cont:
            break
        if compare_window.click_pt and reference_window.click_pt:
            compare_pts.append(compare_window.click_pt)
            reference_pts.append(reference_window.click_pt)
            strengths.append(strength)
            compare_window.clear_click_pt()
            reference_window.clear_click_pt()
    if save_flag:
        match_set = MatchSet(args.compared_image, args.reference_image)
        for i in range( len(compare_pts) ):
            match_set.add_match( compare_pts[i], reference_pts[i], strengths[i] )
        match_set.save_to_file( output_file )
        print "Saved to %s"%output_file

@globalkeycommand( 'm', "Go to the next mode")
def increment_mode():
    global mode
    mode = (mode + 1) % len(MODES)

@globalkeycommand( 'LEFT', "Select previous match")
def select_previous():
    global mode
    mode = SELECT
    global selected_index
    if selected_index == -1:
        selected_index = len(compare_pts) - 1
    else:
        selected_index = (selected_index - 1) % len(compare_pts)
    print_selected_info()

@globalkeycommand( 'RIGHT', "Select next match")
def select_next():
    global mode
    mode = SELECT
    global selected_index
    selected_index = (selected_index + 1) % len(compare_pts)
    print_selected_info()

@globalkeycommand( 's', "Save the match to file", exits=True )
def save():
    global save_flag
    save_flag = True

@globalkeycommand( 'u', "Undo the last action" )
def undo():
    if mode == SET:
        compare_pts.pop()
        reference_pts.pop()
        strengths.pop()

@globalkeycommand( '=', 
                    "Increase the strength"+
                    "\n\t\t-- in SET mode, changes what will be set to"+
                    "\n\t\t-- in SELECT mode, changes the strength of current match")
def increase_strength( ):
    global selected_index
    if mode == SET:
        global strength
        strength = min(strength + 0.05, 1)
        print "Strength: %f"%strength
    elif mode == SELECT and selected_index != -1:
        strengths[selected_index] = min(strengths[selected_index] + 0.05, 1)
        print "Match strength %d set to %f"%(selected_index, strengths[selected_index])

@globalkeycommand( '-', 
                    "Decrease the strength"+
                    "\n\t\t-- in SET mode, changes what will be set to"+
                    "\n\t\t-- in SELECT mode, changes the strength of current match")
def decrease_strength( ):
    global selected_index
    if mode == SET:
        global strength
        strength = max(strength - 0.05, 0)
        print "Strength: %f"%strength
    elif mode == SELECT and selected_index != -1:
        strengths[selected_index] = max(strengths[selected_index] - 0.05, 0)
        print "Match strength %d set to %f"%(selected_index, strengths[selected_index])

@globalkeycommand( 'd', "Delete the selected match" )
def delete_selected( ):
    global selected_index
    if mode == SELECT and selected_index != -1:
        del compare_pts[selected_index]
        del reference_pts[selected_index]
        del strengths[selected_index]
        selected_index = (selected_index - 1) % len(compare_pts)

def print_selected_info():
    if selected_index == -1:
        return
    print "Selected match: %d (%d,%d) -> (%d,%d)" %( selected_index, compare_pts[selected_index][0], compare_pts[selected_index][1],
                                            reference_pts[selected_index][0], reference_pts[selected_index][1])
    print "Strength: %f" % strengths[selected_index]
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        
