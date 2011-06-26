import roslib
roslib.load_manifest("patch_vision")
from patch_vision.classification.classifier import Classifier, LabeledFeature
import numpy as np
from patch_vision.utils.formulas import l2_dist, chi2_dist

class NNClassifier ( Classifier ):

    def train_impl( self ):
        #No need to train anything with a NN Classifier
        return
    
    def predict_label( self, feature ):
        nn = min( self._labeled_features,
                  key = lambda lf: self.dist(feature, lf.feature ) )
        return nn.label

    def save_trained ( self, output_file ):
        # I need to store the same data an untrained classifier needs: the labeled features
        self.save_untrained( output_file )
    
    def read_trained ( self, input_file ):
        self.read_untrained( input_file )


    def dist( self, v1, v2 ):
        #Hard coded for now, make dist fxn variable later
        return l2_dist( v1, v2 )

def get_classifier_name( input_file ):
    f = open( input_file,'r' )
    name = f.readline().split()[0]
    f.close( )
    return name

def instantiate_classifier_by_name( name, *args, **keywords ):
    return globals()[name](*args, **keywords) 
    

def load_classifier_from_file( input_file ):
    name = get_classifier_name (input_file)
    classif = instantiate_classifier_by_name( name )
    classif.read_from_file( input_file )
    return classif
