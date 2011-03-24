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
import RosUtils
from appearance_utils.srv import LoadImage

SAVE_IMAGE = True
SAVE_MODEL = True
SAVE_SCORES = True
ANNOTATE = True
    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpaths = args[1:]
    #Load model and image
    models = [pickle.load(open(modelpath)) for modelpath in corrected_modelpaths]
    #Special for socks
    image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
    image_mask = models[0].initialize_appearance_cached(corrected_filepath)
    shape_contour = thresholding.get_contour_from_thresh(image_mask)
    models[0].initialize_contour(shape_contour)
    best_model = None
    best_nearest_pts = None
    best_fitted_model = None
    best_image_out = None
    best_score = 100
    scores = []
    appearance_scores = []
    contour_scores = []
    appearance_responses = []
    best_appearance_responses =None

    for i,model in enumerate(models):
        print "On model %d of %d"%(i+1,len(models))
        #Create an image to output
        image_out = cv.CloneImage(image_raw)
        #Use the thresholding module to get the contour out
        #image_cont = cv.CloneImage(image_raw)
        #cv.DrawContours(image_cont,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,3)
        #cv.NamedWindow("Contours")
        #cv.ShowImage("Contours",image_cont)
        #cv.WaitKey()
        #return #FIXME
        #Use the shape_fitting module to fit the model to the contour
        fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,SILENT=True, 
                SHOW=False,num_iters=25,INIT_APPEARANCE=False)

        
        (nearest_pts, final_model, fitted_model) = fitter.fit(model,shape_contour,image_out,image_raw)
        model_cache_name = final_model.get_cache_filename(corrected_filepath,corrected_modelpaths[i])+".shapepickle"
        pickle.dump(final_model, open(model_cache_name,'w'))
        score = final_model.score()
        scores.append(score)
        appearance_scores.append(final_model.appearance_score(None))
        contour_scores.append(final_model.contour_score(shape_contour))
        #appearance_responses.append(final_model.appearance_responses())
        if not best_model or score <= best_score:
            best_score = score
            best_appearance_responses = appearance_responses
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
        savepath2 = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.appearancelog"
        """
        savefile2 = open(savepath2,'w')
        for appearance_response in appearance_responses:
            for val in appearance_response:
                savefile2.write("%f\t"%val)
            savefile2.write("\n")
        savefile2.close()
        """
        savepath3 = corrected_filepath[0:len(corrected_filepath)-4]+"_classified.scorebreakdownlog"
        savefile3 = open(savepath3,'w')
        for i in range(len(appearance_scores)):
            savefile3.write("%f\t%f\n"%(appearance_scores[i],contour_scores[i]))
        savefile3.close()
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
