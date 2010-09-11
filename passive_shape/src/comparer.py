#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import annotator
import os
import re
import Vector2D
import sys

TOWEL,PANTS,TEE,SWEATER = range(4)
MODE = TEE

def score(testfile,correctfile):
    test_pts = annotator.read_anno(testfile)
    correct_pts = annotator.read_anno(correctfile)
    net_error = 0.0
    max_error = 0.0
    if MODE == SWEATER or MODE == TEE:
        check_points = (0,1,2,3,4,8,9,10,11,12)
    else:
        check_points = range(len(test_pts))
    for i in check_points:#(1,4,8,11):#range(len(test_pts)):
        test_pt = test_pts[i]
        correct_pt = correct_pts[i]
        error = Vector2D.pt_distance(test_pt,correct_pt)
        #if error < 5:
        #    error = 0
        net_error += error
        if max_error < error:
            max_error = error
    net_error /= float(len(check_points))
    #print "File %s had a net error of %f and max_error of %f"%(testfile,net_error,max_error)
    return net_error
        

def main(args):
    test_directory = args[0]
    correct_directory = args[1]
    test_files_all = os.listdir(test_directory)
    test_files = sorted([f for f in test_files_all if re.match(".*\.anno",f)])
    correct_files_all = os.listdir(correct_directory)
    correct_files = sorted([f for f in correct_files_all if re.match(".*\.anno",f)])
    if len(correct_files) < len(test_files):
        print "Had %d test files but only %d annotations"%(len(test_files),len(correct_files))
        for f in [f for f in correct_files if not f in [g.replace("_classified","") for g in test_files]]:
            print "%s is not in the test files"%f
        return 1
    net_score = 0.0
    for test_file in test_files:
        if not test_file.replace("_classified","") in correct_files:
            print "Error: could not find the correct annotations for file %s"%test_file
            return 1
        else:
            net_score += score("%s/%s"%(test_directory,test_file),"%s/%s"%(correct_directory,test_file.replace("_classified","")))
    net_score /= float(len(test_files))
    print "Average distance of %f pixels"%net_score
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
