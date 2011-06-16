/*
 * =====================================================================================
 *
 *       Filename:  patch_maker.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 01:15:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/slicing/patch_maker.h>
#include <cstring>

KeyPoint PatchDefinition :: get_keypoint( ) const{
    float ctr_x =  center().first;
    float ctr_y =  center().second;
    float sz    =   size();
    return KeyPoint( ctr_x, ctr_y, sz );
    
}

void PatchMaker::get_patches( const Mat &image, vector<Mat> &patches, vector<Mat> &masks, vector<PatchDefinition* > &patch_definitions ) const{
    get_patch_definitions( image, patch_definitions );
    for( size_t i = 0; i < patch_definitions.size(); i++ ){
        Mat patch;
        Mat mask;
        patch_definitions[i]->extract_from_image(image, patch, mask);
        patches.push_back( patch );
        masks.push_back( mask );
    }
}

