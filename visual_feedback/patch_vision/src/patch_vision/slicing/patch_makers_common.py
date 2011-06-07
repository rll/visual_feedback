#!/usr/bin/env python

import roslib
roslib.load_manifest("patch_vision")
import cv
from patch_vision.slicing.patch_maker import PatchMaker

class SlidingWindowPatchMaker(PatchMaker):
    
    def __init__(self, window_size, step_size):
        """ Basic sliding window approach. Make patches of size
            window_size with the center taken at each step_size increment.
        """
        self.window_size = window_size
        self.step_size = step_size

    def get_patches(self, image):
        #Indices of top_left corners
        horizontal_indices = range( 0, 
                                    image.width - self.window_size + 1, 
                                    self.step_size)
        vertical_indices =   range( 0, 
                                    image.height - self.window_size + 1, 
                                    self.step_size)
        patches = []
        centers = []
        for i in horizontal_indices:
            for j in vertical_indices:
                patches.append( image[j:j + self.window_size,
                                      i:i + self.window_size] )
                centers.append( (i + (self.window_size - 1) / 2.,
                                 j + (self.window_size - 1) / 2.) )
        return patches, centers

