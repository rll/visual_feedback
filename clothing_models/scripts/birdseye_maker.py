#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it
import roslib
roslib.load_manifest("clothing_models")
import annotator
import os
import re
from visual_feedback_utils import Vector2D
import sys
import cv
import os
        

def main(args):
    if len(args) != 2:
        print "birdseye_maker.py [H_location.yaml] [birdseye_directory]"
        return
    H_loc = args[0]
    directory = args[1]
    test_files_all = os.listdir(".")
    test_files = sorted([f for f in test_files_all if re.match(".*png",f)])
    H = cv.Load(H_loc)
    for f in test_files:
        img = cv.LoadImage(f)
        birds_image = cv.CloneImage(img)
        cv.WarpPerspective(img,birds_image,H, cv.CV_INTER_LINEAR+cv.CV_WARP_INVERSE_MAP+cv.CV_WARP_FILL_OUTLIERS )
        newname =f.replace("_frame_","_frame_birdseye_")
        print newname
        cv.SaveImage("%s/%s"%(directory,newname),birds_image)
    
if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
