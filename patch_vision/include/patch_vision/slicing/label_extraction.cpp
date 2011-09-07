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

#include <patch_vision/slicing/label_extraction.h>
#include<iostream>

namespace patch_vision{

using std::string;
using std::map;
using cv::Mat;
using std::cout;
using std::endl;


void compute_labels( const Mat &label_mat, const vector<PatchDefinition*> &patch_definitions, 
        vector<int> &labels ){
    for ( size_t i = 0; i < patch_definitions.size(); i++ ){
        PatchDefinition* patch_definition = patch_definitions[i];
        Mat label_slice, mask_slice;
        patch_definition->extract_from_image( label_mat, label_slice, mask_slice);
        int label = compute_label( label_slice, mask_slice );
        labels.push_back(label);
    }
}

int compute_label( const Mat &label_slice, const Mat &mask ){
    vector<int> counts;
    int total = 0;
    for ( int j = 0; j < label_slice.size().height; j++ ){
        for ( int i = 0; i < label_slice.size().width; i++ ){
            if (! mask.at<uint8_t>(j,i))
                continue;
            int label = label_slice.at<uint8_t>(j,i);
            if (label +1 > counts.size()){
                counts.resize(label+1,0);
            }
            counts[label] += 1;
            total += 1;
        }
    }
    int best_label = -1;
    int best_count = 0;

    for ( size_t i = 0; i < counts.size(); i++ ){
        if( counts[i] > best_count ){
            best_label = i;
            best_count = counts[i];
        }
    }
    if (best_count / float(total) >= 0.5){
        return best_label;
    }
    else{
        return -1;
    }
}

};


