#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__
/*
 * =====================================================================================
 *
 *       Filename:  descriptor.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/07/2011 04:06:27 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <patch_vision/slicing/patch_maker.h>

using std::vector;
using std::pair;
using std::string;
using cv::Mat;


class Descriptor{
    public:
        Descriptor();
        ~Descriptor();

        virtual void process_patch( const Mat &patch, vector<double> &feature ) = 0;
        virtual string get_name( ) = 0;
        
        void process_image( const Mat &image, vector<vector<double> > &features, vector<pair<double,double> > &centers );
        void process_image( const Mat &image, vector<vector<double> > &features, vector<pair<double,double> > &centers, PatchMaker &pm );
};

#endif
