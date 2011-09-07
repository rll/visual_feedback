#ifndef __CLASSIFIERS_COMMON_H__
#define __CLASSIFIERS_COMMON_H__
/*
 * =====================================================================================
 *
 *       Filename:  classifiers_common.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/23/2011 02:39:17 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/classification/classifier.h>

namespace patch_vision{
    
    class NNClassifier : public Classifier{
        public:
            NNClassifier(const string feature_type="N/A", const string description="N/A", bool include_unlabeled=false) :
                Classifier( feature_type, description, include_unlabeled ){ };
    
            string name( ) const{ return "NNClassifier"; }
            int predict_label( const vector<float> &feature ) const;
    
        protected:
            void train_impl( );
            void read_trained ( ifstream &input_file );
            void save_trained ( ofstream &output_file ) const;
            
    };
    
    float l2_distance( const vector<float> v1, const vector<float> v2 );
    
    void get_candidate_classifiers( vector<Classifier*> &candidates );
    
    Classifier* instantiate_classifier_by_name( string name );
    
    Classifier* load_classifier_from_file( string filename );
    
};

#endif
