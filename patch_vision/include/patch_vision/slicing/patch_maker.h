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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace patch_vision{

using std::vector;
using std::pair;
using std::string;
using cv::Mat;
using cv::KeyPoint;


enum PatchShape{
    SQUARE, CIRCLE, INVALID
};

string shape_to_name( PatchShape shape );
PatchShape name_to_shape( const string name);

class PatchDefinition{
    public:

        virtual pair<double, double> center() const=0;
        virtual void shift_by(int dx, int dy) = 0;
        virtual void scale_by(float scale) = 0;
        virtual PatchShape shape() const=0;
        virtual int size() const=0;
        virtual void extract_from_image(const Mat &image, Mat &patch, Mat &mask) const=0; 
        virtual KeyPoint get_keypoint( ) const;
};

class PatchMaker{
    public:

        void get_patches( const Mat &image, const Mat &mask, vector<Mat> &patches, vector<Mat> &masks, vector<PatchDefinition* > &patch_definitions ) const;
        void get_patches( const Mat &image, vector<Mat> &patches, vector<Mat> &masks, vector<PatchDefinition* > &patch_definitions ) const;

        virtual void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions) const = 0;
};

bool is_blank( const Mat &mask );

};

#endif
