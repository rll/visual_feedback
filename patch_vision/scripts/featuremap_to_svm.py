#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np

from patch_vision.extraction.feature_io import FeatureMap

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-f','--input-features',             dest='input_features', type=str,   
                            required=True,
                            help='the input features' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filename)' )
    parser.add_argument(    '-u','--include-unlabeled', dest='include_unlabeled',
                            action='store_true', default=False,
                            help='Store a value of 0 for unlabelled portions' )
    return parser.parse_args()

def main(args):
    if args.output_directory:
        directory = args.output_directory
    else:
        directory = os.path.dirname(args.input_features)
        if directory == "":
            directory = "."
        print "No output directory specified. Defaulting to %s"%directory
    if not os.path.exists(directory):
        os.makedirs(directory)
    if args.output_prefix:
        prefix = args.output_prefix
    else:
        prefix, extension = os.path.splitext(os.path.basename(args.input_features))
        print "No output prefix selected. Defaulting to %s"%prefix
    output_file = "%s/%s.libsvm"%(directory,prefix)

    fm = FeatureMap()
    fm.read_from_file( args.input_features )
    
    labeled_features = []
    for pt in fm.get_feature_points():
        feature = fm.get_feature(pt)
        label = fm.get_label(pt)
        if label < 0:
            continue
        if (not args.include_unlabeled) and label == 0:
            continue
        labeled_features.append( (label, feature) )
    f = open(output_file,'w')
    print len(labeled_features)
    for (label, feature) in labeled_features:
        f.write("%d\t"%label)
        for i,val in enumerate(feature):
            if val == 0:
                continue
            f.write("%d:%f "%(i,val))
        f.write("\n")
    f.close()

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

