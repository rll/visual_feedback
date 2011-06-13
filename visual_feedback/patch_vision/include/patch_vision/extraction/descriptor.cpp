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
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
using cv::cvtColor;

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, 
                                vector< PatchDefinition* > &patch_definitions, bool verbose ){
    PatchMaker* pm = get_default_patch_maker( );
    process_image( image, features, patch_definitions, *pm, verbose );
}

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, 
                                vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm, bool verbose ){
    Mat converted_image;
    get_proper_colors( image, converted_image);
    vector<Mat> patches;
    pm.get_patches ( converted_image, patches, patch_definitions );
    for ( size_t i = 0; i < patches.size(); i++ ){
        if (verbose){
            cout << "On patch " << i+1 << " of " << patches.size() << endl;
        }
        vector<double> feature;
        process_patch( patches[i], feature );
        features.push_back( feature );
    }
}

PatchMaker* Descriptor::get_default_patch_maker(  ){
  return new SlidingWindowPatchMaker( patch_size(), patch_size(), patch_size(), patch_size() ); 
}

void Descriptor::get_proper_colors( const Mat &image, Mat &converted_image ){
    
    if (image.channels() == 1 && required_color_mode() != BW){
        cerr << "Must give a colored image" << endl;
        cout << "BLAH" << endl;
        return;
    }
    switch (required_color_mode()){
        case BW:
            cvtColor(image, converted_image, CV_BGR2GRAY);
            break;
        case RGB:
            cvtColor(image, converted_image, CV_BGR2RGB);
            break;
        case HSV:
            converted_image.create(image.size().height, image.size().width, CV_32FC3);
            cvtColor(image, converted_image, CV_BGR2HSV);
            break;
        default:
            image.copyTo(converted_image);
            break;
    }
}
