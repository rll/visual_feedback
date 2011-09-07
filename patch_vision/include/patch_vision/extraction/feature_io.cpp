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
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#include <patch_vision/extraction/feature_io.h>

namespace patch_vision{

using std::ofstream;
using std::ifstream;
using std::endl;


FeatureMap :: FeatureMap( ){
    clear();
}

FeatureMap :: ~FeatureMap( ){
    //for (size_t i = 0; i < _items.size(); i++){
    //    _items[i].feature.clear();
    //}
    //_items.clear();  
}

void FeatureMap :: add_feature( const PatchDefinition* patch_definition, const vector<float> &feature, int label ){
    pair<int, int> size;
    size.first = size.second = patch_definition->size();
    add_feature( patch_definition->center(), patch_definition->shape(), size, feature, label );
}

void FeatureMap :: add_feature( const pair< float, float> ctr, PatchShape shape, const pair<int, int> size, 
                                const vector<float> &feature, int label ){
    FeatureMapItem item;
    item.ctr = pair<float, float>( ctr );
    item.shape = shape;
    item.size = pair<int, int>( size );
    item.feature = vector<float> ( feature );
    item.label = label;
    _items.push_back( item ); 
}

void FeatureMap :: get_items( vector<FeatureMapItem> &items ) const{
    for( size_t i = 0; i < _items.size(); i++){
        items.push_back(FeatureMapItem( _items[i] ) );
    }
}

void FeatureMap :: save_to_file( string filename ) const{
    ofstream output_file( filename.c_str() );
    output_file << _items.size() << endl;
    for( size_t i = 0; i < _items.size(); i++ ){
        FeatureMapItem item = _items[i];
        output_file <<  item.ctr.first  << " " << item.ctr.second
                    <<  "\t" 
                    <<  shape_to_name(item.shape)
                    <<  "\t"
                    <<  item.size.first << " " << item.size.second
                    <<  "\t";
        output_file << item.feature.size() << " ";
        for( size_t j = 0; j < item.feature.size(); j++ ){
            output_file << item.feature[j] << " ";
        }
        output_file << "\t";
        output_file << item.label;
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
        string shape_name; 
        input_file >> shape_name;
        pair<int, int> size;
        input_file >> size.first >> size.second;
        size_t feature_size;
        input_file >> feature_size;
        vector<float> feature(feature_size);
        for( size_t j = 0; j < feature_size; j++ ){
            input_file >> feature[j];
        }
        int label;
        input_file >> label;
        add_feature( ctr, name_to_shape(shape_name), size, feature, label );

    }
}

void FeatureMap :: clear(){
    _items.clear();
}

};
