#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import os.path
import sys
import rospy
import numpy as np
from patch_vision.matching.match_io import MatchSet
from patch_vision.slicing.pt_io import PointSet

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-r,--ref-file',     dest='ref_file',    type=str,
                            required=True, default=None,
                            help='The reference pts file' )
    parser.add_argument(    '-c','--comp-file',     dest='comp_file',    type=str,
                            required=True, default=None,
                            help='The matching comp file' )
    
                            
    return parser.parse_args()

def main(args):
    match_set = MatchSet( "foo", "bar" )
    match_set.read_from_file( args.comp_file )
    ref_set = PointSet( )
    ref_set.read_from_file( args.ref_file )
    
    ref_pts = ref_set.get_points()
    print ref_pts
    output_set = MatchSet( "foo", "bar" )
    for match in match_set.get_matches():
        idx = min( [i for i in range(len(ref_pts))],
                key = lambda i: dist(ref_pts[i],match.reference_pt) )
        print match.reference_pt
        print dist(ref_pts[idx],match.reference_pt)
        match.ref_idx = idx
        output_set.add_match( match )

    output_set.save_to_file( args.comp_file )

def dist(pt1,pt2):
    return np.sqrt( (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2 )
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
