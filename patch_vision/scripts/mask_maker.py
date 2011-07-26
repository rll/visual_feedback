#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.utils.zoom_window import ZoomWindow, keycommand


class MaskWindow (ZoomWindow):
    def __init__(self, image, mask, output_path, default_brush_size=5, zoom_out=1):
        self.image = image
        self.output_path = output_path
        self.visible_image = cv.CreateImage((image.width,image.height),image.depth,3)
        self.mask = mask
        self.erase = False
        self.brush_size = default_brush_size
        ZoomWindow.__init__(self, "MaskLabeler", 100, zoom_out=zoom_out)

    def handleEventsUnzoomed(self,event,x,y,flags,param):
        if flags:
            color = 0 if not self.erase else 255
            cv.Circle(self.mask,  (x,y), self.brush_size, color, -1)

    def image_to_show( self ):
        cv.Set(self.visible_image,cv.RGB(0,0,0))
        cv.Copy(self.image,self.visible_image, self.mask)
        return self.visible_image

    @keycommand('=', "Increment brush size")
    def increment_brush_size(self):
        self.brush_size += 1
        print "Brush size: %d"%self.brush_size

    @keycommand('-', "Decrement brush size")
    def decrement_brush_size(self):
        if self.brush_size > 1:
            self.brush_size -= 1
            print "Brush size: %d"%self.brush_size

    @keycommand('e', "Erase mask")
    def erase_mask(self):
        self.erase = True

    @keycommand('m', "Draw mask")
    def draw_mask(self):
        self.erase = False

    @keycommand('c', "Clear mask")
    def clear(self):
        cv.Set(self.mask,(255,255,255))

    @keycommand('s', "Save mask", exits=True) 
    def save(self):
        cv.SaveImage(self.output_path, self.mask)

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Mask out a hue band and save it to a file')
    parser.add_argument(    '-i','--input-image',             dest='image', type=str,   
                            required=True,
                            help='the image to mask' )
    parser.add_argument(    '-b','--brush-size',              dest='brush_size', type=int,
                            default=5,
                            help='Default brush size to begin with' )
    parser.add_argument(    '-z','--zoom_out',                dest='zoom_out', type=int,
                            default=1,
                            help='Default zoom to begin with' )
    parser.add_argument(    '-t','--threshold',   dest='threshold', action='store_true',
                            default=False,
                            help="Optionally apply a thresholding step first" )
    parser.add_argument(    '-min','--hue-min',   dest='hue_min', type=int,   
                            default = 30,
                            help='min hue band to threshold out' )
    parser.add_argument(    '-max','--hue-max',   dest='hue_max', type=int,   
                            default = 80,
                            help='max hue band to threshold out' )
    return parser.parse_args()

def main(args):
    image = cv.LoadImage( args.image)
    prefix, extension = os.path.splitext(args.image)
    output_path = "%s.mask.png"%(prefix)
    if os.path.exists(output_path):
        mask = cv.LoadImage(output_path, cv.CV_LOAD_IMAGE_GRAYSCALE)
    else:
        mask = cv.CreateImage((image.width,image.height),8,1)
        cv.Set(mask,255)
    if args.threshold:
        print "Thresholding"
        image_hue = cv.CreateImage(cv.GetSize(image),8,1)
        image_sat = cv.CreateImage(cv.GetSize(image),8,1)
        image_val = cv.CreateImage(cv.GetSize(image),8,1)

        image_hsv = cv.CreateImage(cv.GetSize(image),8,3)
        
        mask_hue = cv.CreateImage((image.width,image.height),8,1)
        mask_val = cv.CreateImage((image.width,image.height),8,1)
        mask_new = cv.CreateImage((image.width,image.height),8,1)
        cv.CvtColor(image,image_hsv,cv.CV_BGR2HSV)
        cv.Split(image_hsv,image_hue,image_sat,image_val,None)
        upper_thresh_hue = cv.CreateImage(cv.GetSize(image),8,1)
        lower_thresh_hue = cv.CreateImage(cv.GetSize(image),8,1)
        upper_thresh_val = cv.CreateImage(cv.GetSize(image),8,1)
        lower_thresh_val = cv.CreateImage(cv.GetSize(image),8,1)
        cv.Threshold( image_hue, upper_thresh_hue, args.hue_max, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( image_hue, lower_thresh_hue, args.hue_min, 255, cv.CV_THRESH_BINARY_INV )
        cv.Threshold( image_val, upper_thresh_val, 235, 255, cv.CV_THRESH_BINARY)
        cv.Threshold( image_val, lower_thresh_val, 30,  255, cv.CV_THRESH_BINARY_INV)
        cv.Or(upper_thresh_hue, lower_thresh_hue, mask_hue)
        cv.Or(upper_thresh_val, lower_thresh_val, mask_val)
        cv.Or(mask_hue, mask_val, mask_new)
        cv.And(mask, mask_new, mask)
    
    MaskWindow( image, mask, output_path, args.brush_size, args.zoom_out )
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

