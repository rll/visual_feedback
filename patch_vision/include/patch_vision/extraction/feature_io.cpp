/*
 * =====================================================================================
 *
 *       Filename:  feature_io.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 05:56:18 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#include <patch_vision/extraction/feature_io.h>

using std::ofstream;
using std::ifstream;
using std::endl;

FeatureMap :: FeatureMap( ){
    clear();
}

FeatureMap :: ~FeatureMap( ){ }

void FeatureMap :: add_feature( const PatchDefinition* patch_definition, const vector<float> &feature ){
    add_feature( patch_definition->center(), feature );
}

void FeatureMap :: add_feature( const pair< float, float> ctr, const vector<float> &feature ){
    FeatureMapItem item;
    item.ctr = pair<float, float>( ctr );
    item.feature = vector<float> ( feature );
    _items.push_back( item ); 
}

void FeatureMap :: get_feature_points( vector< pair<float, float> > &pts) const{
    for( size_t i = 0; i < _items.size(); i++ ){
        pts.push_back( _items[i].ctr );
    }
}

void FeatureMap :: set_patch_size( int patch_size ){
    _patch_size = patch_size;
}

int FeatureMap :: get_patch_size(  ) const{
    return _patch_size;
}

void FeatureMap :: save_to_file( string filename ) const{
    ofstream output_file( filename.c_str() );
    output_file << _items.size() << endl;
    for( size_t i = 0; i < _items.size(); i++ ){
        FeatureMapItem item = _items[i];
        output_file <<  item.ctr.first  << " " << item.ctr.second
                    <<  "\t" 
                    <<  _patch_size << " " << _patch_size
                    <<  "\t";
        output_file << item.feature.size() << " ";
        for( size_t j = 0; j < item.feature.size(); j++ ){
            output_file << item.feature[j] << " ";
        }
        output_file <<  endl;
    }
    output_file.close();
}

void FeatureMap :: read_from_file( string filename ){
    ifstream input_file( filename.c_str() );
    size_t num_features;
    input_file >> num_features;
    for ( size_t i = 0; i < num_features; i++ ){
        pair<float, float> ctr;
        input_file >> ctr.first >> ctr.second;
        input_file >> _patch_size >> _patch_size;
        size_t feature_size;
        input_file >> feature_size;
        vector<float> feature(feature_size);
        for( size_t j = 0; j < feature_size; j++ ){
            input_file >> feature[j];
        }
        add_feature( ctr, feature );

    }
    
}


void FeatureMap :: clear(){
    _items.clear();
}
