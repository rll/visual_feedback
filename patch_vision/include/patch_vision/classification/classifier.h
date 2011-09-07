#ifndef __CLASSIFIER_H__
#define __CLASSIFIER_H__
/*
 * =====================================================================================
 *
 *       Filename:  classifier.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/22/2011 05:38:07 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <string>
#include <vector>

#include <patch_vision/extraction/feature_io.h>
#include <patch_vision/slicing/patch_maker.h>

namespace patch_vision{

using std::vector;
using std::ifstream;
using std::ofstream;

struct LabeledFeature{
    vector<float> feature;
    int label;
};

class Classifier{

    /*  Note: pure virtual functions are:
     *  string name()
     *  void train_impl( )
     *  void read_trained( const string filename )
     *  void save_trained( const string filename )
     *  int predict_label( )*/
    public:

        Classifier(  );
        Classifier( const string feature_type="N/A", const string description="N/A", bool include_unlabeled=false );
        virtual ~Classifier( );
     
        virtual string name( ) const = 0;

        string feature_type( ) const; 
        string description( ) const; 
        bool    is_trained( ) const;
        
        void    add_featuremap( const FeatureMap &labeled_featuremap );
        void    add_labeled_feature( const vector<float> &feature, int label );
        void    add_labeled_feature( LabeledFeature &labeled_feature );

        void    train( );
        
        void predict( const FeatureMap &unlabeled_featuremap, vector<int> &labels) const;
        virtual int predict_label( const vector<float> &feature ) const = 0;
        

        void    read_from_file( string filename );
        void    save_to_file ( string filename ) const;


    protected:
        
        virtual void train_impl( ) = 0;
        
        void read_untrained ( ifstream &input_file );
        void save_untrained ( ofstream &output_file ) const;

        virtual void read_trained( ifstream &input_file ) = 0;
        virtual void save_trained( ofstream &output_file ) const = 0;

        bool get_labeled_features (vector<LabeledFeature> &labeled_features) const;
        
        vector<LabeledFeature> _labeled_features;

    private:
        void clear();
        string _feature_type, _description;

        bool    _is_trained;
        bool    _include_unlabeled;

};

};

#endif
