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
import shape_fitting_twophase
import RosUtils
from appearance_utils.srv import LoadImage

SAVE_IMAGE = True
SAVE_MODEL = True
SAVE_SCORES = True
ANNOTATE = True
    
def main(args):
    corrected_filepath = args[0]
    set = int(args[1])
    corrected_modelpaths = args[2:]
    #Load model and image
    models = [pickle.load(open(modelpath)) for modelpath in corrected_modelpaths]
    root_path = models[0].get_cache_experiment_directory(corrected_filepath,set)
    save_model_path = root_path + "/final_model.pickle"
    if os.path.exists(save_model_path):
        print "Already exists!"
        exit()
    #Special for socks
    image_raw = cv.LoadImage(corrected_filepath,cv.CV_LOAD_IMAGE_COLOR)
    best_model = None
    best_nearest_pts = None
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
        image_mask = model.initialize_appearance_cached(corrected_filepath,set=set)
        image_out = cv.CloneImage(image_raw)
        shape_contour = thresholding.get_contour_from_thresh(image_mask)
        model.initialize_contour(shape_contour)
        fitter = shape_fitting_twophase.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,SILENT=True, 
                SHOW=False,num_iters=25,INIT_APPEARANCE=False)

        
        (nearest_pts, final_model, phase1_model) = fitter.fit(model,shape_contour,image_out,image_raw)
        #Get root path
        root_path = final_model.get_cache_model_directory(corrected_filepath,set,corrected_modelpaths[i])
        #Save the phase1 model
        model_cache_name_phase1 = root_path+"/phase1.pickle"
        pickle.dump(final_model, open(model_cache_name_phase1,'w'))
        #Save the phase2 model
        model_cache_name_phase2 = root_path+"/phase2.pickle"
        pickle.dump(final_model, open(model_cache_name_phase2,'w'))
        #Save the annotated points
        anno_path = root_path + "/points.anno"
        annotator.write_anno(nearest_pts,anno_path)
        #Save the score breakdown
        score = final_model.score()
        appearance_score = final_model.appearance_score(None)
        contour_score = final_model.contour_score(shape_contour)
        savepath = root_path + "/scorebreakdown.log"
        savefile = open(savepath,'w')
        savefile.write("%f\t%f\t%f\n"%(appearance_score,contour_score,score))
        savefile.close()
        scores.append(score)
        #Save the annotated image at 1024
        img_path = root_path + "/classified.png"
        scale = 640.0 / image_out.width
        if scale < 1:
            new_width = 640
            new_height = int(scale * image_out.height)
            resized_img = cv.CreateImage((new_width,new_height),8,3)
            cv.Resize(image_out,resized_img)
        else:
            resized_img = image_out
        cv.SaveImage(img_path,resized_img)
        appearance_scores.append(appearance_score)
        contour_scores.append(contour_score)
        #appearance_responses.append(final_model.appearance_responses())
        if not best_model or score <= best_score:
            best_score = score
            best_appearance_responses = appearance_responses
            best_model = model
            best_nearest_pts = nearest_pts
            best_image_out = image_out
    final_model = best_model
    nearest_pts = best_nearest_pts
    image_out = best_image_out
    
    #New root is the experiment directory
    root_path = final_model.get_cache_experiment_directory(corrected_filepath,set) 
    #Optionally save the nearest points in a .anno file, for comparison with my own annotations
    if ANNOTATE:
        anno_path = root_path + "/points.anno"
        annotator.write_anno(nearest_pts,anno_path)
    #Optionally save the model, for reuse later
    if SAVE_MODEL:
        #Remove the image to make pickling possible
        final_model.set_image(None)
        save_model_path = root_path + "/final_model.pickle"
        model_dest = open(save_model_path,'w')
        pickle.dump(final_model,model_dest)
        model_dest.close()
    #Optionally save the image      
    if SAVE_IMAGE:
        #Save the annotated image
        img_path = root_path + "/classified.png"
        scale = 1024.0 / image_out.width
        if scale < 1:
            new_width = 1024
            new_height = int(scale * image_out.height)
            resized_img = cv.CreateImage((new_width,new_height),8,3)
            cv.Resize(image_out,resized_img)
        else:
            resized_img = image_out
        cv.SaveImage(img_path,resized_img)
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
