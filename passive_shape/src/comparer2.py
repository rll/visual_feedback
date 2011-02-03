#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it
import roslib
roslib.load_manifest('passive_shape')
import annotator
import os
import re
import Vector2D
import Models
import sys
from numpy import *
import pickle
import cv
from numpy import *

TOWEL,PANTS,TEE,SWEATER = range(4)
MODE = SWEATER
IGNORE_COLLAR = False 

MODELS = {  TOWEL:'/home/stephen/snapshots/models/towel_model.pickle',
            PANTS:'/home/stephen/snapshots/models/pants_model.pickle',
            TEE:'/home/stephen/snapshots/models/tee_model.pickle',
            SWEATER:'/home/stephen/snapshots/models/sweater_model.pickle'}

def score(testfile,correctfile):
    test_pts = annotator.read_anno(testfile)
    correct_pts = annotator.read_anno(correctfile)
    net_error = 0.0
    max_error = 0.0
    rel_pts = []
    if IGNORE_COLLAR:
        if MODE == SWEATER or MODE == TEE:
            check_points = (0,1,2,3,4,8,9,10,11,12)
        else:
            check_points = range(len(test_pts))
    else:
        check_points = range(len(test_pts))
    errors = []
    (x_axis,y_axis) = get_axes(correct_pts)
    for i in check_points:#(1,4,8,11):#range(len(test_pts)):
        test_pt = test_pts[i]
        correct_pt = correct_pts[i]
        rel_pt = Vector2D.pt_diff(test_pt,correct_pt)
        rel_pts.append((Vector2D.dot_prod(rel_pt,x_axis),Vector2D.dot_prod(rel_pt,y_axis)))
        error = Vector2D.pt_distance(test_pt,correct_pt)
        errors.append(error)
    return (lst_avg(errors),rel_pts)
    
def get_axes(pts):
    if MODE == TOWEL:
        y_axis = Vector2D.normalize(Vector2D.pt_diff(pts[1],pts[0]))
        
    elif MODE == PANTS:
        y_axis = Vector2D.normalize(Vector2D.pt_diff(Vector2D.pt_scale(Vector2D.pt_sum(pts[4],pts[3]),0.5),pts[0]))
    else:
        y_axis = Vector2D.normalize(Vector2D.pt_diff(pts[5],Vector2D.pt_scale(Vector2D.pt_sum(pts[0],pts[-1]),0.5)))
    x_axis = (-1*y_axis[1],y_axis[0])
    print (x_axis,y_axis)
    return (x_axis,y_axis)
    if MODE == SWEATER or MODE == TEE:
        check_points = (0,1,2,3,4,8,9,10,11,12)
    else:
        check_points = range(len(test_pts))
        
def lst_avg(lst):
    return sum(lst) / float(len(lst))
    
def lst_std(lst):
    std = 0
    avg = lst_avg(lst)
    print avg
    for el in lst:
        std += abs(el - avg)**2
    return sqrt(std / float(len(lst)))

def zip_together(deep_lst):
    new_lst = []
    for i in range(len(deep_lst[0])):
        new_lst.append([])
    for lst in deep_lst:
        for i in range(len(lst)):
            new_lst[i].append(lst[i])
    return new_lst
    
def visualize_errors(rel_pts):
    model_file = MODELS[MODE]
    model = pickle.load(open(model_file))
    white_image = cv.CreateImage((750,500),8,3)
    cv.Set(white_image,cv.CV_RGB(255,255,255))
    model.translate((50,0))
    model.draw_to_image(white_image,cv.CV_RGB(0,0,255))
    """
    for i,pt in enumerate(model.vertices_full()):
        ctr = (pt[0] + mean_x[i],pt[1] + mean_y[i])
        y_axis = std_y[i]
        x_axis = std_x[i]
        cv.Ellipse(white_image,ctr,(x_axis,y_axis),0,0,360,cv.CV_RGB(255,0,0))
    """
    
    for i,pt in enumerate(model.vertices_full()):
        print "Drawing model"
        absolute_pts = [Vector2D.pt_sum(pt,rel_pt) for rel_pt in rel_pts[i]]
        #for abs_pt in absolute_pts:
        #    cv.Circle(white_image,abs_pt,2,cv.CV_RGB(0,255,0),-1)
        angle = get_angle(rel_pts[i])
        
        x_axis = (cos(angle),-1*sin(angle))
        y_axis = (sin(angle),cos(angle))
        mean_x = lst_avg([x for (x,y) in rel_pts[i]]) + pt[0]
        mean_y = lst_avg([y for (x,y) in rel_pts[i]]) + pt[1]
        std_dev_x = lst_std([Vector2D.dot_prod(rel_pt,x_axis) for rel_pt in rel_pts[i]])
        std_dev_y = lst_std([Vector2D.dot_prod(rel_pt,y_axis) for rel_pt in rel_pts[i]])
        cv.Ellipse(white_image,(mean_x,mean_y),(std_dev_x,std_dev_y),angle*360/(2*pi),0,360,cv.CV_RGB(255,0,0),2)
        """
        newmat = cv.CreateMat(1,len(absolute_pts),cv.CV_32SC2)
        for i in range(len(absolute_pts)):
            newmat[0,i] = absolute_pts[i]
        fit_ellipse = cv.FitEllipse2(newmat)
        cv.EllipseBox(white_image,fit_ellipse,cv.CV_RGB(255,0,0))
        """
        
    cv.SaveImage("comparison.png",white_image)
    


def get_angle(pts):
    moments = cv.Moments(pts,0)
    mu11 = cv.GetCentralMoment(moments,1,1)
    mu20 = cv.GetCentralMoment(moments,2,0)
    mu02 = cv.GetCentralMoment(moments,0,2)
    print "Got moments"
    return 1/2.0 * arctan( (2 * mu11 / float(mu20 - mu02)))
    
def get_center(moments):
    m00 = cv.GetSpatialMoment(moments,0,0)
    m10 = cv.GetSpatialMoment(moments,1,0)
    m01 = cv.GetSpatialMoment(moments,0,1)
    x = float(m10) / m00
    y = float(m01) / m00
    return (x,y)

def main(args):
    test_directory = args[0]
    correct_directory = args[1]
    test_files_all = os.listdir(test_directory)
    test_files = sorted([f for f in test_files_all if re.match(".*\.anno",f)])
    correct_files_all = os.listdir(correct_directory)
    correct_files = sorted([f for f in correct_files_all if re.match(".*\.anno",f)])
    if len(correct_files) < len(test_files):
        print "Had %d test files but only %d annotations"%(len(test_files),len(correct_files))
        for f in [f for f in correct_files if not f in [g.replace("_classified","") for g in test_files]]:
            print "%s is not in the test files"%f
        return 1
    scores = []
    rel_pts = []
    print "%d test files"%len(test_files)
    for test_file in test_files:
        if not test_file.replace("_classified","") in correct_files:
            print "Error: could not find the correct annotations for file %s"%test_file
            return 1
        else:
            (new_score,new_rel_pts) = score("%s/%s"%(test_directory,test_file),"%s/%s"%(correct_directory,test_file.replace("_classified","")))
            scores.append(new_score)
            rel_pts.append(new_rel_pts)
    net_score = lst_avg(scores)
    std = lst_std(scores)
    
    pointwise_rel_pts = zip_together(rel_pts)
    if not IGNORE_COLLAR:
        visualize_errors(pointwise_rel_pts)
    
    print "%s: %d files, average distance of %f +- %f pixels"%(test_directory,len(test_files),net_score,std)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    main(args)
