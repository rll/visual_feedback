import roslib
roslib.load_manifest("patch_vision")
from patch_vision.classification.classifier import Classifier, LabeledFeature
import numpy as np
import os
from patch_vision.utils.formulas import l2_dist, chi2_dist, compute_knn
from libsvm.svmutil import svm_train, svm_predict, svm_save_model, svm_load_model


class NNClassifier ( Classifier ):

    def train_impl( self ):
        #No need to train anything with a NN Classifier
        return
    
    def predict_label( self, feature ):
        nn = min( self._labeled_features,
                  key = lambda lf: self.dist(feature, lf.feature ) )
        return nn.label

    def save_trained ( self, output_file, filename ):
        # I need to store the same data an untrained classifier needs: the labeled features
        self.save_untrained( output_file, filename )
    
    def read_trained ( self, input_file, filename ):
        self.read_untrained( input_file, filename )


    def dist( self, v1, v2 ):
        #Hard coded for now, make dist fxn variable later
        return l2_dist( v1, v2 )

class KNNClassifier ( Classifier ):
    def __init__(self, k=10, *args, **keys):
        self._k = k
        Classifier.__init__(self, *args, **keys)

    def train_impl( self):
        return

    def predict_label( self, feature ):
        knn = compute_knn( self._labeled_features,
                           lambda lf: self.dist(feature, lf.feature ),
                           self._k )
        label_counts = {}
        for lf in knn:
            if not lf.label in label_counts.keys():
                label_counts[lf.label] = 0
            label_counts[lf.label] += 1
        return max(label_counts.keys(), key = lambda k: label_counts[k])

    def save_trained ( self, output_file, filename ):
        self.save_untrained( output_file, filename )

    def read_trained ( self, input_file, filename ):
        self.read_untrained ( input_file, filename )

    def save_untrained( self, output_file, filename ):
        output_file.write("%d\n"%self._k)
        Classifier.save_untrained( self, output_file, filename )

    def read_untrained( self, input_file, filename ):
        self._k = int ( input_file.readline().split()[0] )
        Classifier.read_untrained( self, input_file, filename )

    def dist( self, v1, v2 ):
        return l2_dist( v1, v2 )

class SVMClassifier ( Classifier ):
    def train_impl( self ):
        labels = []
        features = []
        for labeled_feature in self.get_labeled_features( ):
            labels.append(labeled_feature.label)
            features.append(labeled_feature.feature)
        options = self.get_train_options()
        self._model = svm_train( labels, features, options )
    
    def predict_label( self, feature ):
        options = self.get_predict_options()
        y = [-1]
        (p_labels, p_acc, p_vals) = svm_predict( y, [feature], self._model, options )
        return p_labels[0]

    def save_trained ( self, output_file, filename ):
        model_filename = self.get_model_filename(filename)
        svm_save_model(model_filename, self._model)
        f = open(model_filename, 'r')
        lines = f.readlines()
        output_file.write("%d\n"%len(lines))
        for ln in lines:
            output_file.write(ln)
        f.close()
        os.remove(model_filename)

    def read_trained ( self, input_file, filename ):
        model_filename = self.get_model_filename(filename)
        num_lines = int(input_file.readline().split()[0])
        f = open(model_filename,'w')
        for i in range(num_lines):
            f.write(input_file.readline())
        f.close()
        self._model = svm_load_model(model_filename)
        os.remove(model_filename)

    def get_predict_options( self ):
        return ''

    def get_train_options( self ):
        return ''

    def get_model_filename( self, filename ):
        return filename + ".libsvm_model"
    


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
