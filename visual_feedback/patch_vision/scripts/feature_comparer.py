#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.extraction.feature_io import FeatureMap, SHAPES
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

VIEW_MODES = [NN, KNN, GRADIENT] = range(3);

class ReferenceWindow( ZoomWindow ):
    def __init__(self, image, zoom_out):
        self.image = image
        self.distance_layer = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.view_image = cv.CreateImage( (image.width, image.height), image.depth, 3)
        self.knn = None
        self.knn_shapes = None
        self.knn_sizes = None
        self.show_patch = None
        self.view_mode = NN
        self.log_scale = True
        self.gamma = 0.1
        self.distance_map = None
        ZoomWindow.__init__(self,"Reference",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        if self.view_mode == GRADIENT and self.distance_map:
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

        if self.view_mode == NN and self.knn:
            cv.Circle( self.view_image, self.knn[0], 5*self.zoom_out, cv.RGB(0,255,0), -1 )
            if self.show_patch:
                draw_patch( self.view_image, self.knn[0], self.knn_shape[0], self.knn_size[0], color)
        if self.view_mode == KNN and self.knn:
            for i,pt in enumerate(self.knn):
                factor = 1 - i / float(len(self.knn))
                cv.Circle( self.view_image, pt, 5*self.zoom_out, cv.RGB(factor*255,0,0), -1 )
                if self.show_patch:
                    draw_patch( self.view_image, pt, self.knn_shape[i], self.knn_size[i], color )
                
                
        return self.view_image


    def set_knn( self, knn, shapes, sizes ):
        self.knn = knn
        self.knn_shapes = shapes
        self.knn_sizes = sizes

    def set_distance_map( self, distance_map):
        self.distance_map = distance_map

    def toggle_mode(self):
        self.view_mode = (self.view_mode + 1) % len(VIEW_MODES);

    def toggle_log_scale(self):
        self.log_scale = not self.log_scale

    def handle_keypress( self, char_str ):
        if char_str == 'm':
            self.toggle_mode()
        elif char_str == 'l':
            self.toggle_log_scale()
        elif char_str == 'p':
            self.show_patch = not self.show_patch
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

def draw_patch( image, ctr, shape, size, color ):
    if shape == SQUARE:
        v1,v2 = get_rect_vertices(ctr, size[0], size[1])
        cv.Rectangle( self.view_image, 
                      v1, v2, color, 1 )
    elif shape == CIRCLE:
        cv.Circle( self.view_image, ctr, size[0], color, 1 )
    

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
    reference_window = ReferenceWindow( reference_image, args.reference_zoom_out)

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
            shapes = [reference_window.get_shape(pt) for pt in knn]
            sizes = [reference_window.get_size(pt) for pt in knn]
            reference_window.set_knn( knn, shapes, sizes )
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
    
        
