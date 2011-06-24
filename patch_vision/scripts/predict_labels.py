#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.extraction.feature_io import FeatureMap, draw_patch
from patch_vision.labelling.zoom_window import ZoomWindow
from patch_vision.labelling.label_set import LabelSet
from patch_vision.classification.classifiers_common import load_classifier_from_file

class ClickWindow( ZoomWindow ):
    def __init__(self, image, zoom_out):
        self.image = image
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.click_pt = None
        self.update_label = False
        self.patch = None
        ZoomWindow.__init__(self,"Predicted",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        if self.patch:
            draw_patch( self.view_image, *self.patch, filled=True )
        if self.click_pt:
            cv.Circle( self.view_image, self.click_pt, 5*self.zoom_out, cv.RGB(0,0,255), -1 )
        return self.view_image

    def set_patch( self, ctr, shape, size, color ):
        self.patch = (ctr, shape, size, color)

    def handleEvents(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.click_pt = (x,y)
            self.update_label = True


def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--input-image',             dest='input_image', type=str,   
                            required=True,
                            help='the image to predict on' )
    parser.add_argument(    '-f','--input-features',             dest='input_features', type=str,   
                            required=True,
                            help='the featuremap to predict with' )
    parser.add_argument(    '-c','--classifier',   dest='classifier', type=str,   
                            required=True,
                            help='the classifier to use' )
    parser.add_argument(    '-l','--label-set',    dest='label_set', type=str,
                            required=True,
                            help='The label set to display with' )
    parser.add_argument(    '-z','--zoom-out', dest='zoom_out', type=int,
                            default=1,
                            help='the amount to zoom out by' )
    return parser.parse_args()

def main(args):
    image = cv.LoadImage( args.input_image)
    featuremap = FeatureMap()
    featuremap.read_from_file( args.input_features )
    classifier = load_classifier_from_file( args.classifier )
    if not classifier.is_trained():
        print "Warning: using an untrained classifier. This will probably break."
    label_set = LabelSet()
    label_set.read_from_file( args.label_set )

    window = ClickWindow( image, args.zoom_out)

    while(True):
        keycode = cv.WaitKey(100)
        cont = window.update(keycode)
        if not cont:
            break
        if window.update_label:
            click_pt = window.click_pt
            closest_pt = min( featuremap.get_feature_points(),
                              key = lambda pt: l2_dist(pt,click_pt) )
            feature = featuremap.get_feature( closest_pt )
            shape = featuremap.get_shape( closest_pt )
            size = featuremap.get_size( closest_pt )

            label = classifier.predict_label( feature )
            label_name = "None" if label==0 else label_set.get_label_name(label)
            label_color = cv.RGB(0,0,0) if label==0 else label_set.get_label_color(label)
            print "Predicted label: %d (%s)"%(label, label_name);
            window.set_patch( closest_pt, shape, size, label_color )
            window.update_label = False


def l2_dist(v1, v2):
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)
    diff = v1_arr - v2_arr
    return np.dot(diff,diff)
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

