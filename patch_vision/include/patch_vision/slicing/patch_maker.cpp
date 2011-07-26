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
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/slicing/patch_maker.h>
#include <cstring>
#include <iostream>

using cv::sum;
using std::cout;
using std::endl;

string shape_to_name( PatchShape shape ){
    switch (shape){
        
        case SQUARE:
            return "SQUARE";
            break;
        case CIRCLE:
            return "CIRCLE";
            break;
    }
}

PatchShape name_to_shape( const string name ){
    if (!strcmp (name.c_str(), "SQUARE") )
        return SQUARE;
    else if (!strcmp (name.c_str(), "CIRCLE") )
        return CIRCLE;
    throw;
    return INVALID; 
}

KeyPoint PatchDefinition :: get_keypoint( ) const{
    float ctr_x =  center().first;
    float ctr_y =  center().second;
    float sz    =   size();
    return KeyPoint( ctr_x, ctr_y, sz );
    
}

void PatchMaker::get_patches( const Mat &image, const Mat &mask, vector<Mat> &patches, vector<Mat> &masks, vector<PatchDefinition* > &patch_definitions ) const{
    vector< PatchDefinition* > patch_definitions_unpruned;
    get_patch_definitions( image, patch_definitions_unpruned );
    for( size_t i = 0; i < patch_definitions_unpruned.size(); i++ ){
        Mat image_patch, mask_patch;
        Mat image_mask, garbage;
        PatchDefinition* patch_definition = patch_definitions_unpruned[i];
        patch_definition->extract_from_image(image, image_patch, image_mask);
        patch_definition->extract_from_image(mask,  mask_patch,  garbage);
        Mat whole_mask = image_mask & mask_patch;
        if(! is_blank(whole_mask) ){
            patches.push_back( image_patch );
            masks.push_back( whole_mask );
            patch_definitions.push_back(patch_definition);
        }
    }
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

/*  Returns true if the mask has less that 10% nonzero values */
bool is_blank( const Mat &mask ){
    float tot = cv::countNonZero(mask);
    float size = mask.size().width*mask.size().height;
    float frac = tot /  size;
    return frac < 0.2;
}
