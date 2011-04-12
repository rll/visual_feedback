#!/usr/bin/env python

##    @package clothing_models
# An example script which runs our shape fitting code on an image with a green background

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
from shape_window import Geometry2D
from visual_feedback_utils import Vector2D
from clothing_models import Models
import annotator
from visual_feedback_utils import thresholding
from visual_feedback_utils import shape_fitting


SAVE_IMAGE = True  		#Save an annotated image?
SAVE_MODEL = True 		#Save (pickle) the resulting model?
SAVE_SCORES = True 		#Save the scores to a .scores file?
ANNOTATE = True 		#Save the fit polygon points to a .anno file?
    
def main(args):
    imagepath = args[0]
    modelpaths = args[1:]
    #Load model and image
    models = [pickle.load(open(modelpath)) for modelpath in modelpaths]
    image_raw = cv.LoadImage(imagepath,cv.CV_LOAD_IMAGE_COLOR)
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
        fitter = shape_fitting.ShapeFitter(SYMM_OPT=False,ORIENT_OPT=False,FINE_TUNE=False,SILENT=True, SHOW=False,num_iters=10)
        
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
        anno_path = imagepath[0:len(imagepath)-4]+"_classified.anno"
        annotator.write_anno(nearest_pts,anno_path)
    #Optionally save the model, for reuse later
    if SAVE_MODEL:
        #Remove the image to make pickling possible
        final_model.set_image(None)
        save_model_path = imagepath[0:len(imagepath)-4]+"_classified.pickle"
        model_dest = open(save_model_path,'w')
        pickle.dump(final_model,model_dest)
        model_dest.close()
    #Optionally save scores
    if SAVE_SCORES:
        savepath = imagepath[0:len(imagepath)-4]+"_classified.scores"
        savefile = open(savepath,'w')
        for i in range(len(scores)):
            savefile.write("%s\t%f\n"%(modelpaths[i],scores[i]))
        savefile.close()
    #Optionally save the image      
    if SAVE_IMAGE:
        savepath = imagepath[0:len(imagepath)-4]+"_classified.png"
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
