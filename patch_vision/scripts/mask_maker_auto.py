#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.labelling.zoom_window import ZoomWindow

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Mask out a hue band and save it to a file')
    parser.add_argument(    '-i','--input-image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to mask' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filename)' )
    parser.add_argument(    '-min','--hue-min',   dest='hue_min', type=int,   
                            default = 30,
                            help='min hue band to threshold out' )
    parser.add_argument(    '-max','--hue-max',   dest='hue_max', type=int,   
                            default = 80,
                            help='max hue band to threshold out' )
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
        extension = os.path.splitext(os.path.basename(args.input_image))[1]
    else:
        prefix, extension = os.path.splitext(os.path.basename(args.input_image))
        print "No output prefix selected. Defaulting to %s"%prefix
    output_file = "%s/%s%s"%(directory,prefix,extension)

    image = cv.LoadImage( args.input_image)
    input_image = cv.LoadImage( args.input_image )
    mask = cv.CreateImage(cv.GetSize(input_image),8,1)
    image_hue = cv.CreateImage(cv.GetSize(input_image),8,1)

    image_hsv = cv.CreateImage(cv.GetSize(input_image),8,3)
    
    cv.CvtColor(input_image,image_hsv,cv.CV_BGR2HSV)
    cv.Split(image_hsv,image_hue,None,None,None)
    upper_thresh = cv.CreateImage(cv.GetSize(input_image),8,1)
    lower_thresh = cv.CreateImage(cv.GetSize(input_image),8,1)
    
    cv.Threshold( image_hue, upper_thresh, args.hue_max, 255, cv.CV_THRESH_BINARY )
    cv.Threshold( image_hue, lower_thresh, args.hue_min, 255, cv.CV_THRESH_BINARY_INV )
    cv.Or(upper_thresh, lower_thresh, mask)
    cv.SaveImage( output_file, mask )

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

