import cv
from numpy import *

def black_box_opt(model,contour, energy_fxn,delta = 0.1, num_iters = 100, epsilon = 0.001,exploration_factor=1.5):
    #epsilon = delta / 100.0
    epsilon = 0.001
    score = -1 * energy_fxn(model,contour)
    params = model.params()
    deltas = [delta for p in params]
    for it in range(num_iters):
        print "Starting iteration number %d"%it
        for i in range(len(params)):
            #print "Updating param number: %d"%i
            new_params = list(params)
            new_params[i] += deltas[i]
            new_score = -1 * energy_fxn(model.from_params(new_params),contour)
            if new_score > score:
                params = new_params
                score = new_score
                deltas[i] *= exploration_factor
            else:
                deltas[i] *= -1
                new_params = list(params)
                new_params[i] += deltas[i]
                new_score = -1 * energy_fxn(model.from_params(new_params),contour)
                if new_score > score:
                    params = new_params
                    score = new_score
                    deltas[i] *= exploration_factor  
                else:
                    deltas[i] *= 0.5
        print "Current best score is %f"%score
        if max([abs(d) for d in deltas]) < epsilon:
            print "BREAKING"
            break
    return model.from_params(params)
        
def l2_norm(val):
    return val**2
    
def l1_norm(val):
    return abs(val)
    
def drop_off(fxn,limit):
    return lambda val: fxn(min(val,limit))   

def slack(fxn,limit):
    return lambda val: fxn(max(val,limit)-limit)

def avg(lst):
    return float(sum(lst))/len(lst)
    
def displacement(pt1,pt2):
    (x_1,y_1) = pt1
    (x_2,y_2) = pt2
    return (x_2-x_1,y_2-y_1)
    
def translate_pt(pt,trans):
    (x,y) = pt
    (x_displ,y_displ) = trans
    (x_t,y_t) = (x+x_displ,y+y_displ)
    return (x_t,y_t)

def translate_poly(poly,trans):
    return [translate_pt(pt,trans) for pt in poly]

def rotate_pt(pt,angle,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    off_rot_x = x_n*cos(angle) - y_n*sin(angle)
    off_rot_y = y_n*cos(angle) + x_n*sin(angle)
    rot_x = off_rot_x + x_o
    rot_y = off_rot_y + y_o
    return (rot_x,rot_y)

def rotate_poly(poly,angle,origin=(0,0)):
    return [rotate_pt(pt,angle,origin) for pt in poly]

def scale_pt(pt,amt,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    (x_ns,y_ns) = (amt*x_n,amt*y_n)
    (x_s,y_s) = (x_ns+x_o,y_ns+y_o)
    return (x_s,y_s)

def scale_poly(poly,amt,origin=(0,0)):
    return [scale_pt(pt,amt,origin) for pt in poly]

def distance(pt1,pt2):
    (x_1,y_1) = pt1
    (x_2,y_2) = pt2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
    
def get_angle(moments):
    mu11 = cv.GetCentralMoment(moments,1,1)
    mu20 = cv.GetCentralMoment(moments,2,0)
    mu02 = cv.GetCentralMoment(moments,0,2)
    return 1/2.0 * arctan( (2 * mu11 / float(mu20 - mu02)))
    
def get_center(moments):
    m00 = cv.GetSpatialMoment(moments,0,0)
    m10 = cv.GetSpatialMoment(moments,1,0)
    m01 = cv.GetSpatialMoment(moments,0,1)
    x = float(m10) / m00
    y = float(m01) / m00
    return (x,y)
    
def area(contour):
    if contour == None:
        return 0.0
    ar = abs(cv.ContourArea(contour))
    return ar
    

