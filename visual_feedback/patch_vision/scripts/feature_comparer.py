#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
import cv
import os.path
import sys
import rospy
import numpy as np
from patch_vision.extraction.feature_io import FeatureMap, draw_patch
from patch_vision.utils.zoom_window import ZoomWindow, keycommand, update_all_windows
from patch_vision.utils.formulas import l2_dist, chi2_dist, compute_knn

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

    def handleEventsUnzoomed(self,event,x,y,flags,param):
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
        self.show_patch = False
        self.view_mode = NN
        self.log_scale = True
        self.gamma = 0.1
        self.distance_map = None
        self.shape_map = None
        self.size_map = None
        ZoomWindow.__init__(self,"Reference",-1,zoom_out)

    def image_to_show( self ):
        cv.Copy( self.image, self.view_image )
        if self.view_mode == GRADIENT and self.distance_map:
            pts = self.distance_map.keys()
            if self.log_scale:
                #TODO: fix case of np.log(0) and other small values since this results
                # in pct and color being nan
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
                shape = self.shape_map[pt]
                size = self.size_map[pt]
                pct = 1 - (dist - min_distance) /  (max_distance - min_distance)
                color = tuple( transparency * ((1-pct)*start_from + pct*end_at) )
                draw_patch( self.distance_layer, pt, shape, size, color, True )
            cv.ScaleAdd(self.view_image, 1 - transparency, self.distance_layer, self.view_image)

        if self.view_mode == NN and self.knn:
            color = cv.RGB(0,255,0)
            pt = self.knn[0]
            cv.Circle( self.view_image, (int(pt[0]), int(pt[1])), 5*self.zoom_out, color, -1 )
            if self.show_patch:
                shape = self.shape_map[pt]
                size = self.size_map[pt]
                draw_patch( self.view_image, pt, shape, size, color)
        if self.view_mode == KNN and self.knn:
            for i,pt in enumerate(self.knn):
                factor = 1 - i / float(len(self.knn))
                color = cv.RGB(factor*255,0,0)
                cv.Circle( self.view_image, (int(pt[0]), int(pt[1])), 5*self.zoom_out, color, -1 )
                if self.show_patch:
                    shape = self.shape_map[pt]
                    size = self.size_map[pt]
                    draw_patch( self.view_image, pt, shape, size, color )
                
                
        return self.view_image


    def set_knn( self, knn ):
        self.knn = knn

    def set_distance_map( self, distance_map):
        self.distance_map = distance_map

    def set_shape_map( self, shape_map):
        self.shape_map = shape_map
    
    def set_size_map( self, size_map):
        self.size_map = size_map

    @keycommand('m', "Switch to the next view mode")
    def toggle_mode(self):
        self.view_mode = (self.view_mode + 1) % len(VIEW_MODES);
        print "switched to mode %d" % self.view_mode

    @keycommand('l', "Toggle log scale on and off")
    def toggle_log_scale(self):
        self.log_scale = not self.log_scale
        print 'log_scale turned %s' % ('on' if self.log_scale else 'off')

    @keycommand('p', "Toggle whether or not to display patch outlines")
    def toggle_show_patch(self):
        self.show_patch = not self.show_patch
        print 'show_patch turned %s' % ('on' if self.show_patch else 'off')

    @keycommand('=', "Increase gamma")
    def increase_gamma(self):
        self.gamma *= 2
        print 'gamma: %d' % gamma
    
    @keycommand('-', "Decrease gamma")
    def decrease_gamma(self):
        self.gamma *= 0.5
        print 'gamma: %d' % gamma


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
        cont = update_all_windows()
        if not cont:
            break
        if compare_window.update_nn:
            click_pt = compare_window.click_pt
            closest_pt = min( compared_featuremap.get_feature_points(),
                              key = lambda pt: l2_dist(pt,click_pt) )
            compared_feature = compared_featuremap.get_feature( closest_pt )
            distance_map = {}
            shape_map = {}
            size_map = {}
            for pt in reference_featuremap.get_feature_points():
                distance_map[pt] = l2_dist( compared_feature,
                                              reference_featuremap.get_feature(pt) )
                shape_map[pt] = reference_featuremap.get_shape(pt)
                size_map[pt] = reference_featuremap.get_size(pt)
            
            knn = compute_knn( distance_map.keys(), lambda pt: distance_map[pt], 20 )
            reference_window.set_knn( knn  )
            reference_window.set_distance_map( distance_map )
            reference_window.set_shape_map( shape_map )
            reference_window.set_size_map( size_map )
            compare_window.update_nn = False
        
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        
