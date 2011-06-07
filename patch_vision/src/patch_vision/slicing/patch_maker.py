#!/usr/bin/env python

class PatchMaker:

    def get_patches(self, image):
        """ get_patches( image ) -> patches, centers
            Input: A cvImage
            Output: patches: A list of patches
                    centers: The indices of the center of each patch.
                             Fractional values given for center of even patches.
        """
        abstract
