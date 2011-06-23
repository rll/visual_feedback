/*
 * =====================================================================================
 *
 *       Filename:  classifiers_common.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/23/2011 03:16:05 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/classification/classifiers_common.h>
#include <math.h>
#include <limits>

int NNClassifier :: predict_label( const vector<float> &feature ) const{
    int best_label = -1;
    float best_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < _labeled_features.size(); i++){
        float dist = l2_distance( feature, _labeled_features[i].feature );
        if (dist < best_distance){
            best_distance = dist;
            best_label = _labeled_features[i].label;
        }
    }
    return best_label;
}

void NNClassifier :: train_impl( ){
   // No training needed! 
}

void NNClassifier :: read_trained ( ifstream &input_file ){
    read_untrained ( input_file );
}

void NNClassifier :: save_trained ( ofstream &output_file ) const{
    save_untrained ( output_file );
}

float l2_distance( const vector<float> v1, const vector<float> v2 ){
    float val = 0;
    assert( v1.size() == v2.size() );
    for ( size_t i = 0; i < v1.size(); i++ ){
        val += pow( v1[i] - v2[i], 2 );
    }
    return sqrt(val);
}
