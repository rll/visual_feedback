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


class PatchMaker{
    public:
        PatchMaker();
        ~PatchMaker();

        virtual void get_patches( const Mat &image, vector<Mat> &patches, vector<pair<double,double> > &centers );
};

#endif
