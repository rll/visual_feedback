#ifndef __FEATURE_IO_H__
#define __FEATURE_IO_H__
/*
 * =====================================================================================
 *
 *       Filename:  feature_io.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 05:46:06 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/slicing/patch_maker.h>
#include <iostream>
#include <fstream>

namespace patch_vision{

using std::string;

typedef struct FeatureMapItem{
    pair<float, float> ctr;
    PatchShape shape;
    pair<int, int> size; 
    vector<float> feature;
    int label;
} FeatureMapItem;

class FeatureMap{
    public:
        FeatureMap();
        ~FeatureMap();

        void add_feature( const PatchDefinition *patch_definition, const vector<float> &feature, int label=-1 );
        void add_feature( const pair<float, float> ctr, PatchShape shape, const pair<int, int> size, 
                          const vector<float> &feature, int label=-1 );

        void get_items( vector<FeatureMapItem> &items ) const;

        void save_to_file( string filename ) const;
        void read_from_file( string filename );

    private:
        void clear();

        vector<FeatureMapItem> _items;
        

};
    
};
#endif
