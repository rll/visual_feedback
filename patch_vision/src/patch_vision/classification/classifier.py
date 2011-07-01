#!/usr/bin/env python

import roslib
roslib.load_manifest("patch_vision")
from patch_vision.extraction.feature_io import FeatureMap

class LabeledFeature:
    def __init__(self, feature, label):
        self.feature = feature
        self.label = label

class Classifier:

    def __init__(self, feature_type="N/A", description="N/A", include_unlabeled=False ):
        self._feature_type = feature_type
        self._description = description
        self._include_unlabeled = include_unlabeled
        self.clear( )

    def name(self):
        """ Output: A string which signifies the type of classifier"""
        return self.__class__.__name__

    def feature_type(self):
        return self._feature_type
    
    def description(self):
        return self._description
    
    def include_unlabeled(self):
        return self._include_unlabeled

    def is_trained(self):
        return self._is_trained

    def add_featuremap ( self, labeled_featuremap ):
        """ 
            add_featuremap ( self, labeled_featuremap )
            Adds all features from a labeled featuremap to the
            training set
        """
        for pt in labeled_featuremap.get_feature_points( ):
            feature = labeled_featuremap.get_feature( pt )
            label = labeled_featuremap.get_label( pt )
            if label < 0:
                continue
            if label == 0 and not self.include_unlabeled():
                continue
            self.add_label_and_feature( feature, label )

    def add_label_and_feature( self, feature, label ):
        self._labeled_features.append( LabeledFeature( feature, label ) )

    def add_labeled_feature( self, labeled_feature ):
        self._labeled_features.append( labeled_feature )

    def train( self ):
        self.train_impl()
        self._is_trained = True

    def predict( self, unlabeled_featuremap ):
        """
            Given an unlabeled featuremap, return a dictionary
            of patch centers to predicted labels
        """
        labels = {}
        for pt in unlabeled_featuremap.get_feature_points( ):
            feature = unlabeled_featuremap.get_feature( pt )
            labels[pt] = self.predict_label( feature )
        return labels

    def read_from_file( self, filename ):
        """
            Populate the classifier from a .cls file
        """
        self.clear()
        f = open(filename,'r')
        name = f.readline().split()[0]
        (featuretype, description, include_unlabeled) = f.readline().split()
        self._feature_type = featuretype
        self._description = description
        self._include_unlabeled = bool( int( include_unlabeled) )
        self._is_trained = bool( int( f.readline().split()[0] ) )
        if not self._is_trained:
            self.read_untrained( f, filename )
        else:
            self.read_trained( f, filename )
        f.close()

    def save_to_file( self, filename ):
        """
            Save the classifier to a .cls file
            Note: Required to save this way, so that all classifiers, regardless of type,
            have the same header. Extended classes only modify save_trained and (optionally) save_untrained
        """
        f = open(filename,'w')
        f.write( "%s\n"%self.name() )
        f.write( "%s\t%s\t%d\n"%(self.feature_type(), self.description(), self.include_unlabeled()) )
        f.write( "%d\n"%self.is_trained() )
        if not self.is_trained():
            self.save_untrained( f, filename )
        else:
            self.save_trained( f, filename )
        f.close()



    
    def get_labeled_features( self ):
        return list(self._labeled_features)

    def clear( self ):
        self._labeled_features = []
        self._is_trained = False
    
    def name(self):
        return self.__class__.__name__ 

    ###     USER EXTENDED METHODS ###

    ##      REQUIRED    ##

    def predict_label( self, feature ):
        """ 
            predict_label( self, feature ) -> label
            Given an input feature, output a label
            By convention, valid labels are nonnegative integers; a label
            of -1 indicates that prediction was somehow impossible on the 
            input data
            You may assume that train_impl() has been run prior to this.
        """
        abstract

    def train_impl( self ):
        """ 
            train_impl( self ) -> None
            Train your classifier on all features, accessible through
            get_labeled_features(). Once trained, you must assume that
            you will no longer have access to the labeled features unless
            you explicitly save and read them via save_ & read_trained()
        """
        abstract

    def save_trained( self, output_file, filename ):
        """
            save_trained( self, output_file ) -> None
            Given an open file with write access,
            write all data you will need to repopulate yourself
        """
        abstract

    def read_trained( self, input_file, filename ):
        """
            read_trained( self, input_file ) -> None
            Given an open file with read access, populate all members of this class
        """
        abstract


    
    ##      OPTIONAL    ##
    def save_untrained( self, output_file, filename ):
        """
            Save _labeled_features to an already-open file with write only access.
            By default, untrained classifiers all store the exact same information:
            the labeled data. If you want other things, such as classifier parameters,
            to be persist even when you have not trained yet, extend this method and
            read_untrained accordingly
        """
        output_file.write( "%d\n" % len( self._labeled_features ) )
        for labeled_feature in self._labeled_features:
            output_file.write( "%d\t" % labeled_feature.label )
            output_file.write( "%d\t" % len(labeled_feature.feature) )
            for val in labeled_feature.feature:
                output_file.write( "%f " % val )
            output_file.write( "\n" )

    def read_untrained( self, input_file, filename ):
        """
            Populate _labeled_features from an already-open file with read only access.
        """
        num_features = int( input_file.readline().split()[0] )
        for i in range( num_features ):
            tokens = input_file.readline().split()
            label = int(tokens[0])
            feature_size = int(tokens[1])
            feature = [float(val) for val in tokens[2:] ]
            self.add_label_and_feature( feature, label )




    



