#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import annotator
import os
import re
import Vector2D
import sys
from numpy import *

TOWEL,PANTS,TEE,SWEATER = range(4)
MODE = TOWEL

def score(testfile,correctfile):
    test_pts = annotator.read_anno(testfile)
    correct_pts = annotator.read_anno(correctfile)
    net_error = 0.0
    max_error = 0.0
    if MODE == SWEATER or MODE == TEE:
        check_points = (0,1,2,3,4,8,9,10,11,12)
    else:
        check_points = range(len(test_pts))
    errors = []
    for i in check_points:#(1,4,8,11):#range(len(test_pts)):
        test_pt = test_pts[i]
        correct_pt = correct_pts[i]
        error = Vector2D.pt_distance(test_pt,correct_pt)
        errors.append(error)
    #return lst_avg(errors)
    return [lst_avg(errors)]
        
def lst_avg(lst):
    return sum(lst) / float(len(lst))
    
def lst_std(lst):
    std = 0
    avg = lst_avg(lst)
    print avg
    for el in lst:
        std += abs(el - avg)**2
    return sqrt(std / float(len(lst)))

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
    scores = []
    for test_file in test_files:
        if not test_file.replace("_classified","") in correct_files:
            print "Error: could not find the correct annotations for file %s"%test_file
            return 1
        else:
            scores.extend(score("%s/%s"%(test_directory,test_file),"%s/%s"%(correct_directory,test_file.replace("_classified",""))))
    net_score = lst_avg(scores)
    std = lst_std(scores)
    print "%s: %d files, average distance of %f +- %f pixels"%(test_directory,len(test_files),net_score,std)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
