#!/usr/bin/env python
import roslib
roslib.load_manifest("patch_vision")
from patch_vision.learning.weight_features import run_opt
import numpy as np

I = 5
P = 30
N = 2
feature_sizes=(128,20)
reference_indices = [1]
alpha = 1 



# Populate feature_lists
feature_lists = []
for i in range(I):
    image_feature_set = []
    for p in range(P):
        stacked_feature = []
        for n in range(N):
            good_val = np.array([p for i in range(feature_sizes[n]) ])
            bad_val  = np.random.random( feature_sizes[n] )
            if p % 3 == 0:
                if n == 0:
                    feature = good_val
                else:
                    feature = bad_val
            else:
                if n == 0:
                    feature = bad_val
                else:
                    feature = good_val
                feature = bad_val
            stacked_feature.append( feature )
        image_feature_set.append( stacked_feature )
    feature_lists.append( image_feature_set )
w = run_opt( feature_lists, reference_indices, alpha )
print list(w)

            
