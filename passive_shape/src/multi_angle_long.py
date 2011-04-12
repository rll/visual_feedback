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
    corrected_modelpaths = args[1:]
    #Load model and image
    models = [pickle.load(open(corrected_modelpath)) for corrected_modelpath in corrected_modelpaths]
    image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
    best_model = None
    best_nearest_pts = None
    best_fitted_model = None
    best_image_out = None
    best_score = 100
    for model_orig in models:
        model_orig.set_image(None)
        for theta in range(0,360,18):
            print "On angle %f\n"%theta
            model = model_orig.from_params(model_orig.params())
            model.rotate(theta*pi/180)
            #Create an image to output
            image_out = cv.CloneImage(image_raw)
            #Use the thresholding module to get the contour out
            shape_contour = thresholding.get_contour(image_raw,bg_mode=thresholding.GREEN_BG,filter_pr2=False,crop_rect=None)
            #Use the shape_fitting module to fit the model to the contour
            fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False, ROTATE=False,SHOW=False, SILENT=True)
            
            (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_out)   
            final_model.set_image(cv.CloneImage(image_raw))
            score = fitter.energy_fxn(final_model,shape_contour)
            final_model.set_image(None)
            if not best_model or score <= best_score:
                best_score = score
                best_model = model
                best_nearest_pts = nearest_pts
                best_fitted_model = fitted_model
                best_image_out = image_out
            
    final_model = best_model
    nearest_pts = best_nearest_pts
    fitted_model = best_fitted_model
    image_out = best_image_out
    
    
    #Optionally save the nearest points in a .anno file, for comparison with my own annotations
    if ANNOTATE:
        anno_path = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.anno"
        annotator.write_anno(nearest_pts,anno_path)
    #Optionally save the model, for reuse later
    if SAVE_MODEL:
        #Remove the image to make pickling possible
        final_model.set_image(None)
        save_model_path = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.pickle"
        model_dest = open(save_model_path,'w')
        pickle.dump(fitted_model,model_dest)
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
