#!/usr/bin/env python

import roslib
roslib.load_manifest("patch_vision")
from patch_vision.slicing.patch_makers_common import SlidingWindowPatchMaker
import cv

class Descriptor:
   
    def process_image(self, image, patch_maker = None, verbose = False):
        """ 
            process_image(image [, patch_maker]) -> features, patch_centers
            image: The input cvImage
            patch_maker: A PatchMaker object to use. If none is given, does sliding
                         window with 0 overlap
            features: The output list of features, as determined by the descriptor
            patch_centers: The center of all patches, in the same order as the features
        """
        if self.required_channels() and image.nChannels != self.required_channels():
            if ( self.auto_grayscale() and 
                 self.required_channels() == 1 and 
                 image.nChannels == 3):
                bwimage = cv.CreateImage( (image.width,image.height), image.depth, 1 )
                cv.CvtColor(image, bwimage, cv.CV_RGB2GRAY)
                image = bwimage
            else:
                raise Exception("The %s descriptor requires a %d channel image." %
                            (self.name(), self.required_channels()) )
        if not patch_maker:
            patch_maker = SlidingWindowPatchMaker(window_size = self.patch_size(), 
                                                  step_size = self.patch_size())
        if verbose:
            print "Slicing patches..."
        patches, patch_indices = patch_maker.get_patches(image)
        features = []
        for i,p in enumerate(patches):
            if verbose:
                print "Computing feature for patch %d of %d"%(i+1, len(patches))
            features.append( self.process_patch( p ) )
        return features, patch_indices

    ###  USER EXTENDED METHODS ###

    ##   REQUIRED   ##
    def process_patch(self, patch):
        """ Input:  A single patch, as a cvImage.
            Output: A vector of length self.descriptor_size()
        """
        abstract

    def descriptor_size(self):
        abstract

    def patch_size(self):
        abstract

        ##   OPTIONAL   ##

    def required_channels(self):
        """ Required channels: if there is no required number of channels,
            leave as 'None' 
        """
        return None

    def auto_grayscale(self):
        """ If you are given a 3 channel image and require 1, automatically
            convert to grayscale rather than complaining """
        return True
    
    def name(self):
        return self.__class__.__name__ 

    

    



