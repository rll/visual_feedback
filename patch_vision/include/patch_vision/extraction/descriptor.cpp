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

Descriptor::Descriptor() {
    //Do nothing
}

Descriptor::~Descriptor() {};

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, vector<pair<double,double> > &centers ){
    //Pass
}

void Descriptor::process_image( const Mat &image, vector<vector<double> > &features, vector<pair<double,double> > &centers, PatchMaker &pm ){
    //Pass
}


