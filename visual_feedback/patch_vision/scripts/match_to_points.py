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
    parser.add_argument(    '-m','--match',             dest='match', type=str,   
                            required=True,
                            help='the match file' )
    parser.add_argument(    '-rf','--ref-file',     dest='ref_file',    type=str,
                            required=False, default=None,
                            help='Prefix of the ref output (defaults to original filenames)' )
    parser.add_argument(    '-cf','--comp-file',     dest='comp_file',    type=str,
                            required=False, default=None,
                            help='Prefix of the ref output (defaults to original filenames)' )
    
                            
    return parser.parse_args()

def main(args):
    match_set = MatchSet( "foo", "bar" )
    print "Reading from " + args.match
    match_set.read_from_file( args.match )
    compare_set = PointSet( )
    ref_set = PointSet( )
    if not args.ref_file:
        args.ref_file = os.path.basename(args.match).split('_TO_')[1].split('.matches')[0]+".pts"
    if not args.comp_file:
        args.comp_file = os.path.basename(args.match).split('_TO_')[0]+".pts"
    for match in match_set.get_matches():
            compare_set.add_point(match.compare_pt)
            ref_set.add_point(match.reference_pt)
    ##if not os.path.exists(os.path.dirname( args.ref_file ) ):
    ##    os.makedirs(os.path.dirname( args.ref_file ) )
    ##if not os.path.exists(os.path.dirname( args.comp_file ) ):
    ##    os.makedirs(os.path.dirname( args.comp_file ) )
    ref_set.save_to_file( args.ref_file )
    compare_set.save_to_file( args.comp_file )

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        
