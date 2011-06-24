#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np

from patch_vision.extraction.feature_io import FeatureMap
from patch_vision.classification.classifiers_common import load_classifier_from_file, instantiate_classifier_by_name, get_classifier_name

CPP_CLASSIFIERS = []
PYTHON_CLASSIFIERS = ['NNClassifier']

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Train a classifier on a labeled featuremap and save it to file')
    parser.add_argument(    '-f','--input-features',             dest='input_features', type=str,   
                            required=True,
                            help='the input labeled features' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filename)' )
    parser.add_argument(    '-c','--input_classifier',  dest='input_classifier', type=str,
                            default=None,
                            help='Optionally take as input an untrained classifier' )
    parser.add_argument(    '-t','--classifier_type', dest="classifier_type", type=str,
                            default=None, choices=CPP_CLASSIFIERS + PYTHON_CLASSIFIERS,
                            help='The type of classifier to use' )
    parser.add_argument(    '-u','--include-unlabeled', dest='include_unlabeled',
                            action='store_true', default=False,
                            help='Store a value of 0 for unlabelled portions' )
    parser.add_argument(    '--no-train', dest='train',
                            action='store_false', default=True,
                            help='Save the untrained classifier' )

    return parser.parse_args()

def main(args):
    
    if not ( args.input_classifier or args.classifier_type ):
        print "Must specify input classifier or type!"
        return 1

    if args.input_classifier:
        args.classifier_type = get_classifier_name( args.input_classifier )
        output_file = args.input_classifier
    else:
        if args.output_directory:
            directory = args.output_directory
        else:
            directory = os.path.dirname(args.input_features)
            print "No output directory specified. Defaulting to %s"%directory
        if not os.path.exists(directory):
            os.makedirs(directory)
        if args.output_prefix:
            prefix = args.output_prefix
        else:
            prefix, extension = os.path.splitext(os.path.basename(args.input_features))
            print "No output prefix selected. Defaulting to %s"%prefix
        output_file = "%s/%s.cls"%(directory,prefix)

    if args.classifier_type in CPP_CLASSIFIERS:
        call_cpp(args)
        return
    #Now onto just the python implementation
    if args.input_classifier:
        classifier = load_classifier_from_file( args.input_classifier )
    else:
        classifier = instantiate_classifier_by_name( args.classifier_type, include_unlabeled=args.include_unlabeled )

    fm = FeatureMap()
    fm.read_from_file( args.input_features )

    classifier.add_featuremap( fm )
    if args.train:
        classifier.train()

    classifier.save_to_file( output_file )
    print "Saved classifier to %s"%output_file
    

def call_cpp(args):
    pass
    
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        


