/*
 * =====================================================================================
 *
 *       Filename:  classifier.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/22/2011 06:32:03 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/classification/classifier.h>
#include <iostream>

using std::cout;
using std::endl;
using std::ofstream;
using std::ifstream;

Classifier :: Classifier( ){
    clear();
}

Classifier :: Classifier( const string feature_type, const string description, bool include_unlabeled ):
    _feature_type(feature_type), _description(description), _include_unlabeled(include_unlabeled)
{
    clear();
}

Classifier :: ~Classifier( ){
    //for( size_t i = 0; i < _labeled_features.size(); i++ ){
    //    _labeled_features[i].feature.clear();
    //}
    //_labeled_features.clear();
}

string Classifier :: feature_type( ) const{
    return _feature_type;
}

string Classifier :: description( ) const{
    return _description;
}

bool Classifier :: is_trained( ) const{
    return _is_trained;
}

void Classifier :: add_featuremap( const FeatureMap &labeled_featuremap ){
    vector<FeatureMapItem> items;
    labeled_featuremap.get_items( items );
    for (size_t i = 0; i < items.size(); i++){
        int label = items[i].label;
        if (label < 0)
            continue;
        if (label == 0 && !_include_unlabeled)
            continue;
        add_labeled_feature( items[i].feature, label );
    }
}

void Classifier :: add_labeled_feature( const vector<float> &feature, int label ){
    LabeledFeature lf;
    lf.feature = vector<float>( feature );
    lf.label = label;
    _labeled_features.push_back( lf );
}

void Classifier :: add_labeled_feature( LabeledFeature &labeled_feature ){
    _labeled_features.push_back( labeled_feature );
}

void Classifier :: train( ){
    train_impl( );
    _is_trained = true;
    //_labeled_features.clear();
}

void Classifier :: predict( const FeatureMap &unlabeled_featuremap, vector<int> &labels) const{
    vector< FeatureMapItem > items;
    unlabeled_featuremap.get_items( items );
    for (size_t i = 0; i < items.size(); i++){
        labels.push_back ( predict_label ( items[i].feature ) );
    }
}

void Classifier :: read_from_file( string filename ){
    clear();
    ifstream input_file( filename.c_str() );
    string myname;
    input_file >> myname;
    input_file >> _feature_type >> _description >> _include_unlabeled;
    input_file >> _is_trained;
    if( ! _is_trained ){
        read_untrained( input_file );
    }
    else{
        read_trained( input_file );
    }
    input_file.close();
}

void Classifier :: save_to_file( string filename ) const{
    ofstream output_file( filename.c_str() );
    output_file << name() << endl;
    output_file << feature_type() << "\t" << description() << "\t"
                << _include_unlabeled << endl;
    output_file << _is_trained << endl;
    if( ! _is_trained ){
        save_untrained( output_file );
    }
    else{
        save_trained( output_file );
    }
    output_file.close();
}

void Classifier :: read_untrained ( ifstream &input_file ){
    size_t num_features;
    input_file >> num_features;
    for( size_t i = 0; i < num_features; i++ ){
        LabeledFeature labeled_feature;
        input_file >> labeled_feature.label;
        size_t feature_size;
        input_file >> feature_size;
        labeled_feature.feature.resize( feature_size );
        for ( size_t j = 0; j < feature_size; j++ ){
            input_file >> labeled_feature.feature[j];
        }
        add_labeled_feature( labeled_feature );
    }
}

void Classifier :: save_untrained ( ofstream &output_file ) const{
    size_t num_features = _labeled_features.size();
    output_file << num_features << endl;
    for( size_t i = 0; i < num_features; i++ ){
        LabeledFeature labeled_feature = _labeled_features[i];
        output_file << labeled_feature.label << "\t";
        size_t feature_size = labeled_feature.feature.size();
        output_file << feature_size << "\t";
        for ( size_t j = 0; j < feature_size; j++ ){
            output_file << labeled_feature.feature[j] << " ";
        }
        output_file << endl;
    }
}

bool Classifier :: get_labeled_features (vector<LabeledFeature> &labeled_features) const{
    if ( _is_trained )
        return false;
    for (size_t i = 0; i < _labeled_features.size(); i++){
        labeled_features.push_back(_labeled_features[i]);
    }
    return true;
}

void Classifier :: clear( ){
    _is_trained = false;
    _labeled_features.clear();
}


