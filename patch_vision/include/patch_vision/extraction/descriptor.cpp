/*
 * =====================================================================================
 *
 *       Filename:  descriptor.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/07/2011 04:24:04 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/extraction/descriptor.h>

using cv::cvtColor;

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, 
                                vector< PatchDefinition* > &patch_definitions ){
    PatchMaker* pm = get_default_patch_maker( );
    process_image( image, features, patch_definitions, *pm );
}

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, 
                                vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm ){
    Mat converted_image;
    if (image.channels() == 3 && required_channels( ) == 1 && auto_grayscale()){
      cvtColor(image, converted_image, CV_BGR2GRAY);
    }
    else{
      converted_image = image;
    }
    vector<Mat> patches;
    pm.get_patches ( converted_image, patches, patch_definitions );
    for ( size_t i = 0; i < patches.size(); i++ ){
        vector<double> feature;
        process_patch( patches[i], feature );
        features.push_back( feature );
    }
}

PatchMaker* Descriptor::get_default_patch_maker(  ){
  return new SlidingWindowPatchMaker( patch_size(), patch_size(), patch_size(), patch_size() ); 
}


