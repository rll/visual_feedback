/*
 * =====================================================================================
 *
 *       Filename:  descriptors_common.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 03:40:49 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/extraction/descriptors_common.h>

LBPDescriptor :: LBPDescriptor( int patch_size ){
    _patch_size = patch_size;
}

LBPDescriptor :: ~LBPDescriptor( ){ }

void LBPDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
   calculate_points();
   int results[256];
   lbp_histogram((int*)(patch.ptr()),patch.size().height,patch.size().width,results,1);
    //L_1 normalization
    int hist_count = 0;
    for (int i=0; i<256; i++) {
        hist_count += results[i];
    }
    for(int i=0; i<256; i++) {
        if(hist_count != 0)
            feature.push_back(results[i]*1.0/hist_count);
        else
            feature.push_back(0.);
    }
   
}

string LBPDescriptor :: name( ) const{
    return "LBP";
}

int LBPDescriptor :: descriptor_size( ) const{
    return 256;
}

int LBPDescriptor :: patch_size( ) const{
    return _patch_size;
}

int LBPDescriptor :: required_channels() const{
    return 1;
}
