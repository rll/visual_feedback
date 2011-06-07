#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os
import os.path
import sys
from patch_vision.labelling.label_window import LabelWindow, LabelWindowParams
import rospy

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to label' )
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output files (defaults to original filename)' )
    parser.add_argument(    '-l','--label_set',  dest='label_set', type=str,
                            default=None,
                            help='The location of the input label set. If none is given, you will'+
                                  'be asked to construct your own on startup' )
    parser.add_argument(    '-si','--save-image',       dest='save_image',  action='store_true',
                            default = False,
                            help='Save the label-annotated image')
    parser.add_argument(    '-b','--brush-size',     dest='brush_size',    type=int,
                            default=5,
                            help='Default brush size to begin with' )

                            
    return parser.parse_args()

def construct_label_set(label_set, directory):
    label = 0
    while(True):
        if label > 0:
            add_label = safe_read_bool("Would you like to add another label? [y/n]")
        else:
            add_label = True
        if not add_label:
            break
        label += 1
        print "Making Label %d."%label
        label_name = raw_input("What name would you like to give this label? [no spaces]\n")
        red = safe_read_int("How much RED should this label be colored with?",0,255)
        green = safe_read_int("How much GREEN should this label be colored with?",0,255)
        blue = safe_read_int("How much BLUE should this label be colored with?",0,255)
        try:
            label_set.add_label(label, label_name, (red,green,blue))
        except Exception,e:
            print "%s"%e
    if safe_read_bool("Would you like to save the label set for later use? [y/n]"):
        label_location = directory + "/default.labels"
        while True:
            print "Will save label to %s"%label_location
            if not safe_read_bool("Would you like to change the label path? [y/n]"):
                break
            label_location = safe_read_filepath("Where would you like to save it to?",dir_only=True)
        label_set.save_to_file(label_location)
        print "Saved to %s"%label_location


def safe_read_int(prompt, lower_bound = None, upper_bound = None):
    print prompt
    while(True):
        try:
            val = int(raw_input())
            if lower_bound and val < lower_bound:
                print "Value must be >= %d"%lower_bound
            elif upper_bound and val > upper_bound:
                print "Value must be <= %d"%upper_bound
            else:
                return val
        except ValueError:
            print "Must input an integer"

def safe_read_bool(prompt):
    print prompt
    while(True):
        resp = raw_input()
        if len(resp) > 0:
            if resp[0] in ['y','Y']:
                return True
            elif resp[0] in ['n','N']:
                return False
        print "Please answer [y]es or [n]o."

def safe_read_filepath(prompt,dir_only=False):
    print prompt
    while(True):
        resp = raw_input()
        filepath = os.path.expanduser(resp)
        if dir_only:
            to_check = os.path.dirname(filepath)
        else:
            to_check = filepath
        if not os.path.exists(to_check):
            print "Path does not exists"
            continue
        return filepath


def main(args):
    image = cv.LoadImage( args.input_image )

    params = LabelWindowParams()
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
    if args.label_set:
        params.label_set.read_from_file(args.label_set)
    else:
        print "No input label set was provided. Constructing one."
        construct_label_set(params.label_set, directory)
    params.default_brush_size = args.brush_size
    params.save_mat_path = directory + "/" + prefix + ".labelmat"
    if args.save_image:
        params.save_image_path = params.save_mat_path + ".png"

    lw = LabelWindow(image, params)
    

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        
