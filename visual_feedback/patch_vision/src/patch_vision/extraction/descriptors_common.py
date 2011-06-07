#!/usr/bin/env python

import roslib
roslib.load_manifest("patch_vision")
import cv
from patch_vision.extraction.descriptor import Descriptor

class RawBWDescriptor(Descriptor):
    
    def __init__(self, patch_size):
        self._patch_size = patch_size

    def process_patch(self, patch):
        vals = []
        for i in range(patch.width):
            for j in range(patch.height):
                vals.append(patch[j,i])
        return vals

    def patch_size(self):
        return self._patch_size
    
    def descriptor_size(self):
        return self.patch_size()**2

    def required_channels(self):
        return 1


class RawColorDescriptor(Descriptor):
    
    def __init__(self, patch_size):
        self._patch_size = patch_size

    def process_patch(self, patch):
        vals = []
        for i in range(patch.width):
            for j in range(patch.height):
                for c in range(3):
                    vals.append(patch[j,i][c])
        return vals
    
    def patch_size(self):
        return self._patch_size

    def descriptor_size(self):
        return 3 * self.patch_size()**2

    def required_channels(self):
        return 3

class LBPDescriptor(Descriptor):
    
    def __init__(self, patch_size):
        self._patch_size = patch_size

    def process_patch(self, patch):
        hist = [0 for v in range( self.descriptor_size() )]
        num_elems = (patch.width - 2) * (patch.height - 2)
        for i in range(1, patch.width-1):
            for j in range(1, patch.height-1):
                pattern_index = self.get_lbp_pattern( patch, (i,j) )
                hist[pattern_index] += 1.0 / num_elems
        return hist

    def patch_size(self):
        return self._patch_size

    def descriptor_size(self):
        return  256

    def required_channels(self):
        return 1

    ## Private methods ##
    def get_lbp_pattern( self, patch, center ):
        # Iterate through all neighbors
        index = 0
        factor = 1
        (i,j) = center
        center_val = patch[j,i]
        for i_step in (-1, 0, 1):
            for j_step in (-1, 0, 1):
                if (i_step == j_step == 0):
                    continue
                if patch[j+j_step, i+i_step] > center_val:
                    index += factor
                factor *= 2
        return index

    

