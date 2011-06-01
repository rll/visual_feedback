#!/usr/bin/env python

##    @package clothing_models
# An example script which runs our shape fitting code on an image with a green background


import roslib
import sys
roslib.load_manifest("clothing_models")
import rospy
import cv
import os.path
import pickle
from clothing_models import Models

from visual_feedback_utils import Vector2D, Annotating, thresholding, shape_fitting

def boolstr(string):
    if string[0] == 'T' or string[0] == 't':
        return True
    elif string[0] == 'F' or string[0] == 'f':
        return False
    else:
        msg = '%s is not understood to mean True or False'
        raise argparse.ArgumentTypeError(msg)

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-i','--image',             dest='input_image', type=str,   
                            required=True,
                            help='the image fit a model to' )
    parser.add_argument(    '-m','--models',            dest='models',      type=str, 
                            metavar='M',                 nargs='+',          required=True,
                            help='the models to fit')
    parser.add_argument(    '-d','--output-directory',  dest='output_directory', type=str,
                            default=None,
                            help='The directory to save all output files in' )
    parser.add_argument(    '-o','--output-prefix',     dest='output_prefix',    type=str,
                            default=None,
                            help='Prefix of all output image, annotation, models, etc' )
    parser.add_argument(    '-si','--save-image',       dest='save_image',  type=boolstr,
                            choices=[True,False],   default=True, 
                            help='Save the annotated image')
    parser.add_argument(    '-sm','--save-model',       dest='save_model',  type=boolstr, 
                            choices=[True,False],   default=True, 
                            help='Save the resulting model')
    parser.add_argument(    '-ss','--save-scores',      dest='save_scores', type=boolstr, 
                            choices=[True,False],   default=False, 
                            help='Save the scores to a .scores file')
    parser.add_argument(    '-sa','--save-anno',        dest='save_anno',   type=boolstr, 
                            choices=[True,False],   default=True, 
                            help='Save the annotations to an anno file')
    parser.add_argument(    '-sc','--save-classification', dest='save_classification',   type=boolstr, 
                            choices=[True,False],   default=False, 
                            help='Save the classification to a classif file')
    parser.add_argument(    '-p','--phases',            dest='phases',      type=str, 
                            metavar='P',                 nargs='+',
                            choices=['O','S','A','F'],  default=['O','S','A'], 
                            help='Optimization phases to run' )
    parser.add_argument(    '-g','--show-graphics',     dest='show_graphics', action='store_true',
                            default=False,              
                            help='Show graphics' )
    parser.add_argument(    '-v','--verbose',           dest='verbose',       action='store_true',
                            default=False, 
                            help='Print output' )
    parser.add_argument(    '-ma','--multi-angle',      dest='multi_angle',   action='store_true',
                            default=False, 
                            help='Multi-angle initialization' )
    parser.add_argument(    '-n','--num-iters',          dest='num_iters',     type=int,
                            default=50,
                            help='Number of iterations to run per phase' )
    parser.add_argument(    '-bg','--background-color',  dest='background_color',
                            choices=['GREEN','WHITE','BLACK'],                  default='GREEN',
                            help='Background color to threshold out' )
                            
    return parser.parse_args()

def main(args):

    #Properly set phases
    orient_opt      = 'O' in args.phases
    symm_opt        = 'S' in args.phases
    asymm_opt       = 'A' in args.phases
    fine_tuning_opt = 'F' in args.phases
    #Properly set background colors
    if args.background_color == 'GREEN':
        background = thresholding.GREEN_BG
    elif args.background_color == 'WHITE':
        background = thresholding.WHITE_BG
    elif args.background_color == 'BLACK':
        background = thresholding.BLACK_BG
    else:
        raise Exception("Invalid background color: %s"%args.background_color)
    #Translate silent/verbose
    silent = not args.verbose
    #Translate output- directory and prefix
    if args.output_directory:
        directory = args.output_directory
    else:
        directory = os.path.dirname(args.input_image)
    if not os.path.exists(directory):
        os.path.makedirs(directory)
    if args.output_prefix:
        prefix = args.output_prefix
    else:
        prefix = os.path.splitext(os.path.basename(args.input_image))[0]+"_fit"

        
    #Load model and image
    models = [pickle.load(open(modelpath)) for modelpath in args.models]
    image_raw = cv.LoadImage(args.input_image,cv.CV_LOAD_IMAGE_COLOR)
    best_model = None
    best_nearest_pts = None
    best_fitted_model = None
    best_image_out = None
    best_index = 0
    best_score = 100
    scores = []
    for i,model in enumerate(models):
        print "On model %d of %d"%(i+1,len(models))
        #Create an image to output
        image_out = cv.CloneImage(image_raw)
        #Use the thresholding module to get the contour out
        shape_contour = thresholding.get_contour(image_raw,bg_mode=background,filter_pr2=False,crop_rect=None)
        fitter = shape_fitting.ShapeFitter(     ORIENT_OPT=orient_opt,  SYMM_OPT=symm_opt,   
                                                ASYMM_OPT=asymm_opt,    FINE_TUNE=fine_tuning_opt,
                                                SILENT=silent,          SHOW=args.show_graphics,
                                                num_iters=args.num_iters )
        
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
            best_index = i
    final_model = best_model
    nearest_pts = best_nearest_pts
    fitted_model = best_fitted_model
    image_out = best_image_out
    
    
    #Optionally save the nearest points in a .anno file, for comparison with my own annotations
    full_prefix = directory + '/' + prefix
    if args.save_anno:
        anno_path = full_prefix+".anno"
        print "Saving anno to " + anno_path
        Annotating.write_anno(nearest_pts,anno_path)
    #Optionally save the model, for reuse later
    if args.save_model:
        #Remove the image to make pickling possible
        final_model.set_image(None)
        save_model_path = full_prefix+".pickle"
        print "Saving model to " + save_model_path
        model_dest = open(save_model_path,'w')
        pickle.dump(final_model,model_dest)
        model_dest.close()
    #Optionally save scores
    if args.save_scores:
        savepath = full_prefix+".scores"
        print "Saving scores to " + savepath
        savefile = open(savepath,'w')
        for i in range(len(scores)):
            savefile.write("%s\t%f\n"%(args.models[i],scores[i]))
        savefile.close()
    #Optionally save the classification
    if args.save_classification:
        savepath = full_prefix+".classif"
        print "Saving classification to " + savepath
        savefile = open(savepath,'w')
        savefile.write(models[best_index].name()+"\n")
        savefile.close()

    #Optionally save the image      
    if args.save_image:
        savepath = full_prefix+".png"
        print "Saving annotated image to " + savepath
        cv.SaveImage(savepath,image_out)
        return
    else:
        cv.NamedWindow("Result")
        cv.ShowImage("Result",image_out)
        cv.WaitKey()
        return
    
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
