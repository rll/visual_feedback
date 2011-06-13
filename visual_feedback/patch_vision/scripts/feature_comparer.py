#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.extraction.feature_io import FeatureMap
from patch_vision.labelling.zoom_window import ZoomWindow

class ClickWindow( ZoomWindow ):
    def __init__(self, image, zoom_out):
        self.image = image
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.click_pt = None
        self.update_nn = False
        ZoomWindow.__init__(self,"Compared",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        if self.click_pt:
            cv.Circle( self.view_image, self.click_pt, 5*self.zoom_out, cv.RGB(0,0,255), -1 )
        return self.view_image

    def handleEvents(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.click_pt = (x,y)
            self.update_nn = True

class ReferenceWindow( ZoomWindow ):
    def __init__(self, image, zoom_out, patch_size):
        self.image = image
        self.distance_layer = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.nn = None
        self.knn = None
        self.show_gradient = False
        self.log_scale = True
        self.gamma = 0.1
        self.distance_map = None
        self.patch_size = patch_size
        ZoomWindow.__init__(self,"Reference",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        if self.distance_map and self.show_gradient:
            pts = self.distance_map.keys()
            if self.log_scale:
                distances = [np.log(dist) for dist in self.distance_map.values()]
            else:
                distances = self.distance_map.values()
            min_distance = min(distances)
            max_distance = max(distances)
            start_from = np.array(cv.RGB(0,0,255))
            end_at = np.array(cv.RGB(255,0,0))
            transparency = 0.8
            for i,pt in enumerate(pts):
                dist = distances[i]
                pct = 1 - (dist - min_distance) /  (max_distance - min_distance)
                color = tuple( transparency * ((1-pct)*start_from + pct*end_at) )
                v1,v2 = get_rect_vertices(pt, self.patch_size[0]/2, self.patch_size[1]/2)
                cv.Rectangle( self.distance_layer, 
                              v1, v2, color, -1 )
            cv.ScaleAdd(self.view_image, 1 - transparency, self.distance_layer, self.view_image)

        if self.nn:
            cv.Circle( self.view_image, self.nn, 5*self.zoom_out, cv.RGB(0,255,0), -1 )
            v1,v2 = get_rect_vertices(self.nn, self.patch_size[0], self.patch_size[1])
            cv.Rectangle( self.view_image, 
                          v1, v2, cv.RGB(0,255,0), 1 )
        if self.knn and not self.show_gradient:
            for i,pt in enumerate(self.knn):
                factor = 1 - i / float(len(self.knn))
                cv.Circle( self.view_image, pt, 5*self.zoom_out, cv.RGB(factor*255,0,0), -1 )
                
        return self.view_image

    def set_nn( self, pt ):
        self.nn = pt

    def set_knn( self, knn ):
        self.knn = knn

    def set_distance_map( self, distance_map):
        self.distance_map = distance_map

    def toggle_mode(self):
        self.show_gradient = not self.show_gradient
    
    def toggle_log_scale(self):
        self.log_scale = not self.log_scale

    def handle_keypress( self, char_str ):
        if char_str == 'm':
            self.toggle_mode()
        elif char_str == 'l':
            self.toggle_log_scale()
        elif char_str == '=':
            self.gamma *= 2
            print "Gamma = %f"%self.gamma
        elif char_str == "-":
            self.gamma *= 0.5
            print "Gamma = %f"%self.gamma
        return ZoomWindow.handle_keypress( self, char_str)

def get_rect_vertices(center, width, height):
    x = center[0] - (width + 1)/2.0
    y = center[1] - (height + 1)/2.0
    return (x,y),(x+width,y+height)

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='run our shape fitting code on an image with a green background')
    parser.add_argument(    '-c','--compared-image',             dest='compared_image', type=str,   
                            required=True,
                            help='the image to compare' )
    parser.add_argument(    '-r','--reference-image',   dest='reference_image', type=str,   
                            required=True,
                            help='the image to compare WITH' )
    parser.add_argument(    '-cf','--compared-features',dest='compared_features', type=str,   
                            required=True,
                            help='features of the comparison image' )
    parser.add_argument(    '-rf','--reference-features',   dest='reference_features', type=str,   
                            required=True,
                            help='features of the reference image' )
    parser.add_argument(    '-cz','--compare-zoom-out',   dest='compare_zoom_out', type=int,   
                            default=1,
                            help='Amount to zoom by' )
    parser.add_argument(    '-rz','--reference-zoom-out',   dest='reference_zoom_out', type=int,   
                            default=1,
                            help='Amount to zoom by' )
                            
    return parser.parse_args()

def main(args):
    compared_image = cv.LoadImage( args.compared_image)
    reference_image = cv.LoadImage( args.reference_image )
    compared_featuremap = FeatureMap()
    compared_featuremap.read_from_file( args.compared_features )
    reference_featuremap = FeatureMap()
    reference_featuremap.read_from_file( args.reference_features )
    compare_window = ClickWindow( compared_image, args.compare_zoom_out)
    reference_window = ReferenceWindow( reference_image, args.reference_zoom_out,
                                        reference_featuremap.get_patch_size())

    #nn_solver = pyflann.FLANN()
    while(True):
        keycode = cv.WaitKey(100)
        cont = compare_window.update(keycode)
        cont &= reference_window.update(keycode)
        if not cont:
            break
        if compare_window.update_nn:
            click_pt = compare_window.click_pt
            closest_pt = min( compared_featuremap.get_feature_points(),
                              key = lambda pt: l2_dist(pt,click_pt) )
            compared_feature = compared_featuremap.get_feature( closest_pt )
            distance_map = {}
            for pt in reference_featuremap.get_feature_points():
                distance_map[pt] = chi2_dist( compared_feature,
                                              reference_featuremap.get_feature(pt) )
            ##nearest_neighbor = min( distance_map.keys(),
            ##                        key = lambda pt: distance_map[pt] )
            ##reference_window.set_nn( nearest_neighbor )
            knn = compute_knn( distance_map.keys(), lambda pt: distance_map[pt], 20 )
            reference_window.set_knn( knn )
            reference_window.set_distance_map( distance_map )
            compare_window.update_nn = False

def l2_dist(v1, v2):
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)
    diff = v1_arr - v2_arr
    return np.dot(diff,diff)

def chi2_dist(v1, v2):
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)
    abs_sum = abs(v1_arr) + abs(v2_arr)
    diff = (v1_arr - v2_arr)**2 / abs_sum
    #Weed out nans
    for i,is_nan in enumerate(np.isnan(diff)):
        if is_nan:
            diff[i] = 0
    dist = np.dot(diff,diff)
    return dist


def compute_knn( comparisons, key, n ):
    knn = []
    for i in range(len(comparisons)):
        dist = key( comparisons[i] )
        if len(knn) < n:
            knn.append( comparisons[i] )
        elif dist < key( knn[n-1] ):
            knn[n-1] = comparisons[i]
        else:
            continue
        knn.sort(key = key)
    return knn    

        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        
