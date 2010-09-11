#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("passive_shape")
import rospy
from numpy import *
import pyflann
import math
import cv
import os.path
import pickle
import Geometry2D
import Vector2D
import Models
import annotator
import thresholding
import shape_fitting


SAVE_IMAGE = True
SAVE_MODEL = True
ANNOTATE = True
    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpath = args[1]
    #Load model and image
    model = pickle.load(open(corrected_modelpath))
    image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
    
    #Create an image to output
    image_out = cv.CloneImage(image_raw)
    #Use the thresholding module to get the contour out
    shape_contour = thresholding.get_contour(image_raw,bg_mode=thresholding.WHITE_BG,filter_pr2=False,crop_rect=None)
    #Use the shape_fitting module to fit the model to the contour
    fitter = shape_fitting.ShapeFitter(SYMM_OPT=True,ORIENT_OPT=True,FINE_TUNE=True)
    (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_out)    
    
    #Optionally save the nearest points in a .anno file, for comparison with my own annotations
    if ANNOTATE:
        anno_path = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.anno"
        annotator.write_anno(nearest_pts,anno_path)
    #Optionally save the model, for reuse later
    if SAVE_MODEL:
        #Remove the image to make pickling possible
        final_model.image = None
        save_model_path = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.pickle"
        model_dest = open(save_model_path,'w')
        pickle.dump(final_model,model_dest)
        model_dest.close()
    #Optionally save the image      
    if SAVE_IMAGE:
        savepath = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.png"
        cv.SaveImage(savepath,image_out)
        return
    else:
        cv.NamedWindow("Result")
        cv.ShowImage("Result",image_out)
        cv.WaitKey()
        return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
