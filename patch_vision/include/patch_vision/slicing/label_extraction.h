#ifndef __LABEL_EXTRACTION_H__
#define __LABEL_EXTRACTION_H__
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
using cv::Mat;

void compute_labels( const Mat &label_mat, const vector<PatchDefinition*> &patch_definitions, 
        vector<int> &labels );

int compute_label( const Mat &label_slice, const Mat &mask );


#endif

