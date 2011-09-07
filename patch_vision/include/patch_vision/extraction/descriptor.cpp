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
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/extraction/descriptor.h>
#include <iostream>

namespace patch_vision{

using std::cerr;
using std::cout;
using std::endl;
using cv::cvtColor;

void Descriptor::process_patch( const Mat &patch, vector<float> &feature ){
    Mat mask = Mat::ones ( patch.size().height, patch.size().width, CV_8UC1 );
    process_patch( patch, feature, mask );
}

void Descriptor::process_image( const Mat &image, const Mat &mask, vector<vector<float> > &features, 
                                vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm, 
                                bool verbose ){
    Mat converted_image;
    get_proper_colors( image, converted_image);
    vector<Mat> patches;
    vector<Mat> masks;
    pm.get_patches ( converted_image, mask, patches, masks, patch_definitions );
    for ( size_t i = 0; i < patches.size(); i++ ){
        if (verbose){
            cout << "On patch " << i+1 << " of " << patches.size() << endl;
        }
        vector<float> feature;
        process_patch( patches[i], feature, masks[i] );
        features.push_back( feature );
    }
    cout << "Done!" << endl;
}

void Descriptor::process_image( const Mat &image, vector<vector<float> > &features, 
                                vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm, 
                                bool verbose ){
    Mat converted_image;
    get_proper_colors( image, converted_image);
    vector<Mat> patches;
    vector<Mat> masks;
    pm.get_patches ( converted_image, patches, masks, patch_definitions );
    for ( size_t i = 0; i < patches.size(); i++ ){
        if (verbose){
            cout << "On patch " << i+1 << " of " << patches.size() << endl;
        }
        vector<float> feature;
        process_patch( patches[i], feature, masks[i] );
        features.push_back( feature );
    }
}

void Descriptor::get_proper_colors( const Mat &image, Mat &converted_image ){
    
    if (image.channels() == 1 && required_color_mode() != BW){
        cout << "Must give a colored image" << endl;
        throw;
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
        case LUV:
            cvtColor(image, converted_image, CV_BGR2Luv);
            break;
        default:
            image.copyTo(converted_image);
            break;
    }
}

};
