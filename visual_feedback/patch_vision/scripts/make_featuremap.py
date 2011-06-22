#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os
import os.path
import sys
from subprocess import call,STDOUT
from patch_vision.extraction.descriptors_common import RawBWDescriptor, RawColorDescriptor, LBPDescriptor
from patch_vision.extraction.feature_io import FeatureMap
from patch_vision.slicing.patch_makers_common import SlidingWindowPatchMaker
import rospy


PACKAGE_NAME = "patch_vision"
CPP_DESCRIPTORS = [ 'LBP', 'HSV_LBP', 'RGB_LBP', 'LUV_LBP', 'SIFT', 'HOG', 'HUE_HISTOGRAM', 
                    'LBP+HUE_HISTOGRAM', 'LBP+SIFT', 'LBP+SIFT+HUE_HISTOGRAM', 
                    'ROTATED_LBP+HUE_HISTOGRAM', 'ROTATED_LBP+SIFT', 'ROTATED_LBP+SIFT+HUE_HISTOGRAM', 
                    'ROTATED_LBP']
PYTHON_DESCRIPTORS = ['RAW_BW','RAW_COLOR']
CPP_DETECTORS = ["DENSE_SQUARE","DENSE_CIRCLE","SIFT","POINTS_SQUARE","POINTS_CIRCLE"]

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--input-image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to make a featuremap of' )
    parser.add_argument(    '-f','--feature_type',dest='feature_type', type=str,   
                            required=True, choices=CPP_DESCRIPTORS + PYTHON_DESCRIPTORS,
                            help='What descriptor to use' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filename)' )
    parser.add_argument(    '-p','--patch-size',   dest='patch_size', type=int,   
                            default = 9,
                            help='size of patches to extract' )
    parser.add_argument(    '-s','--patch-step',   dest='patch_step', type=int,   
                            default = None,
                            help='Amount to step from one patch to the next' )
    parser.add_argument(    '-D','--keypoint-detector', dest='detector', type=str,
                            default = "DENSE_SQUARE", choices=CPP_DETECTORS,
                            help="Keypoint detector to use" );
    parser.add_argument(    '-P','--points-file', dest='points_file', type=str,
                            default = None,
                            help="Points file (only necessary if using a Point Detector)" );
    parser.add_argument(    '-m','--mask-file', dest='mask_file', type=str,
                            default = None,
                            help="Mask file" );
    parser.add_argument(    '-l','--label-file', dest='label_file', type=str,
                            default = None,
                            help="label file" );
    parser.add_argument(    '-v','--verbose',   dest='verbose', action='store_true',
                            default=False,
                            help='Print debugging information' )
                            
    return parser.parse_args()

def main(args):
    
    if args.output_directory:
        directory = args.output_directory
    else:
        directory = os.path.dirname(args.input_image)
        print "No output directory specified. Defaulting to %s"%directory
    if not os.path.exists(directory):
        os.makedirs(directory)
    if args.output_prefix:
        prefix = args.output_prefix
    else:
        prefix = os.path.splitext(os.path.basename(args.input_image))[0]
        print "No output prefix selected. Defaulting to %s"%prefix
    output_file = "%s/%s.fm"%(directory,prefix)
    if not args.patch_step:
        args.patch_step = args.patch_size
    
    if args.feature_type in CPP_DESCRIPTORS:
        cmd = "rosrun %s make_featuremap -i %s -o %s -f %s -p %d -s %d -D %s %s %s %s %s"%(
                PACKAGE_NAME, args.input_image, output_file, args.feature_type, 
                args.patch_size, args.patch_step, args.detector,
                "-P %s"%args.points_file if args.points_file else "",
                "-m %s"%args.mask_file if args.mask_file else "",
                "-l %s"%args.label_file if args.label_file else "",
                "-v" if args.verbose else "")
        print "Calling %s"%cmd
        return call( cmd, shell=True);
    
    if args.feature_type == 'LBP':
        descriptor = LBPDescriptor(args.patch_size)
    elif args.feature_type == 'RAW_BW':
        descriptor = RawBWDescriptor(args.patch_size)
    elif args.feature_type == "RAW_COLOR":
        descriptor = RawColorDescriptor(args.patch_size)
    else:
        raise Exception("Invalid feature!")
    input_image = cv.LoadImage( args.input_image )
    patch_maker = SlidingWindowPatchMaker( args.patch_size, args.patch_step)
    features, patch_centers = descriptor.process_image( input_image, patch_maker, args.verbose )

    feature_map = FeatureMap()
    feature_map.set_patch_size( (args.patch_size,args.patch_size) )
    for i, ctr in enumerate(patch_centers):
        feature_map.add_feature( ctr, features[i] )
    feature_map.save_to_file("%s/%s.fm"%(directory,prefix))



        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

