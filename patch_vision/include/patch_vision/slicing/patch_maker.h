#ifndef __PATCH_MAKER_H__
#define __PATCH_MAKER_H__
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

using std::vector;
using std::pair;
using std::string;
using cv::Mat;

class PatchDefinition{
    public:

        virtual pair<double, double> center() const=0;
        virtual int size() const=0;
        virtual void extract_from_image(const Mat &image, Mat &patch) const=0; 
};

class PatchMaker{
    public:

        void get_patches( const Mat &image, vector<Mat> &patches, vector<PatchDefinition* > &patch_definitions ) const;

        virtual void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions) const = 0;
};

#endif
