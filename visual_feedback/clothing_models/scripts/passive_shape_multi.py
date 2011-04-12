#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("clothing_models")
import rospy
from numpy import *
import pyflann
import math
import cv
import os.path
import pickle
import Geometry2D
import visual_feedback_utils.Vector2D
import clothing_models.Models
import annotator
import visual_feedback_utils.thresholding
import visual_feedback_utils.shape_fitting


SAVE_IMAGE = True
SAVE_MODEL = True
SAVE_SCORES = True
ANNOTATE = True
    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpaths = args[1:]
    #Load model and image
    models = [pickle.load(open(corrected_modelpath)) for corrected_modelpath in corrected_modelpaths]
    return
    image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
    best_model = None
    best_nearest_pts = None
    best_fitted_model = None
    best_image_out = None
    best_score = 100
    scores = []
    for i,model in enumerate(models):
        print "On model %d of %d"%(i+1,len(models))
        #Create an image to output
        image_out = cv.CloneImage(image_raw)
        #Use the thresholding module to get the contour out
        shape_contour = thresholding.get_contour(image_raw,bg_mode=thresholding.GREEN_BG,filter_pr2=False,crop_rect=None)
        #image_cont = cv.CloneImage(image_raw)
        #cv.DrawContours(image_cont,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,3)
        #cv.NamedWindow("Contours")
        #cv.ShowImage("Contours",image_cont)
        #cv.WaitKey()
        #return #FIXME
        #Use the shape_fitting module to fit the model to the contour
        fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,SILENT=True, SHOW=False,num_iters=10,INIT_APPEARANCE=(i==0))
        
        (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_out,image_raw)   
        final_model.set_image(cv.CloneImage(image_raw))
        score = final_model.score(shape_contour,image_raw)
        final_model.set_image(None)
        scores.append(score)
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
        pickle.dump(final_model,model_dest)
        model_dest.close()
    #Optionally save scores
    if SAVE_SCORES:
        savepath = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.log"
        savefile = open(savepath,'w')
        for i in range(len(scores)):
            savefile.write("%s\t%f\n"%(corrected_modelpaths[i],scores[i]))
        savefile.close()
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
