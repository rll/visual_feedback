#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it
import roslib
roslib.load_manifest("passive_shape")
import annotator
import os
import re
import Vector2D
import sys
import cv
import os

def score(testfile,correctfile):
    test_pts = annotator.read_anno(testfile)
    correct_pts = annotator.read_anno(correctfile)
    net_error = 0.0
    max_error = 0.0
    for i in (1,4,8,11):#range(len(test_pts)):
        test_pt = test_pts[i]
        correct_pt = correct_pts[i]
        error = Vector2D.pt_distance(test_pt,correct_pt)
        if error < 5:
            error = 0
        net_error += error
        if max_error < error:
            max_error = error
    net_error /= float(len(test_pts))
    #print "File %s had a net error of %f and max_error of %f"%(testfile,net_error,max_error)
    return net_error
        

def main(args):
    good_directory = args[0]
    bad_directory = args[1]
    test_files_all = os.listdir(".")
    test_files = sorted([f for f in test_files_all if re.match(".*_thresh.*png",f)])
    for f in test_files:
        img = cv.LoadImage(f)
        cv.NamedWindow("Image")
        cv.ShowImage("Image",img)
        print "GOOD?"
        resp = cv.WaitKey()
        print "Received response: %s"%resp
        pluskey = 1048637
        minuskey = 1048621
        if resp==pluskey:
            direct = good_directory
        elif resp==minuskey:
            direct = bad_directory
        else:
            return
        os.system("cp %s %s/%s"%(f,direct,f))
        orig_f = f.replace("_thresh","")
        os.system("cp %s %s/%s"%(orig_f,direct,orig_f))
    
if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
