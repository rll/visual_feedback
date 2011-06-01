#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import sys
        
def write_anno(pts,filename):
    output = open(filename,'w')
    xs = [x for (x,y) in pts]
    ys = [y for (x,y) in pts]
    output.write("%d\n"%len(pts))
    for i in range(len(pts)):
        output.write("%f\n"%xs[i])
        output.write("%f\n"%ys[i])
    output.close()
    
def read_anno(filename):
    anno_input = open(filename,'r')
    num_pts = int(anno_input.readline())
    pts = []
    for i in range(num_pts):
        x = float(anno_input.readline())
        y = float(anno_input.readline())
        pts.append((x,y))
    anno_input.close()
    return pts
