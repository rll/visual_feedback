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
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/slicing/patch_maker.h>
#include <iostream>
#include <fstream>

using std::string;

typedef struct FeatureMapItem{
    pair<double, double> ctr;
    vector<double> feature;
} FeatureMapItem;

class FeatureMap{
    public:
        FeatureMap();
        ~FeatureMap();

        void add_feature( const PatchDefinition *patch_definition, const vector<double> &feature );
        void add_feature( const pair<double, double> ctr, const vector<double> &feature );
        void get_feature_points( vector< pair<double, double> > &pts) const;
        void set_patch_size( int patch_size );
        int get_patch_size( ) const;
        void save_to_file( string filename ) const;
        void read_from_file( string filename );

    private:
        void clear();

        int _patch_size;
        vector<FeatureMapItem> _items;
        

};


#endif
