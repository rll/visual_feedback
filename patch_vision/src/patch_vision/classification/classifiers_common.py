import roslib
roslib.load_manifest("patch_vision")
from patch_vision.classification.classifier import Classifier, LabeledFeature
import numpy as np

class NNClassifier ( Classifier ):

    def predict_label( self, feature ):
        nn = min( self._labeled_features,
                  key = lambda lf: self.dist(feature, lf.feature ) )
        return nn.label

    def train_impl( self ):
        #No need to train a NN Classifier
        return

    def read_trained ( self, input_file ):
        self.read_untrained( input_file )

    def save_trained ( self, output_file ):
        self.save_untrained( output_file )

    def dist( self, v1, v2 ):
        #Hard coded for now, make variable later
        return l2_dist( v1, v2 )

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

