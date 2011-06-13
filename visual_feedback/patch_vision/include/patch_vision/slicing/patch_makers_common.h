#ifndef PATCH_MAKERS_COMMON_H
#define PATCH_MAKERS_COMMON_H
/*
 * =====================================================================================
 *
 *       Filename:  patch_makers_common.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 02:58:33 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <iostream>
#include <patch_vision/slicing/patch_maker.h>
#include <opencv2/features2d/features2d.hpp>

using cv::FeatureDetector;
using cv::Ptr;
using cv::KeyPoint;

class RectangularPatch : public PatchDefinition{
    public:
        RectangularPatch( int x, int y, int width, int height );
        RectangularPatch( const KeyPoint &kp );
        ~RectangularPatch( );

        pair<double, double> center() const;
        int size() const;
        void extract_from_image(const Mat &image, Mat &patch) const;
        KeyPoint get_keypoint( ) const;

    private:
        int _x, _y, _width, _height;
        
};

class KeyPointPatch : public PatchDefinition{
    public:
        KeyPointPatch( KeyPoint &kp );
        ~KeyPointPatch( );

        pair<double, double> center() const;
        int size() const;
        void extract_from_image(const Mat &image, Mat &patch) const;
        
        KeyPoint get_keypoint( ) const;

    private:
        KeyPoint _kp;
        
};



class SlidingWindowPatchMaker : public PatchMaker{
    public:
       SlidingWindowPatchMaker( int width, int height, int step_x, int step_y );
       ~SlidingWindowPatchMaker( );

       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
       int _width, _height, _step_x, _step_y;
};

class SIFTPatchMaker : public PatchMaker{
    public:
        SIFTPatchMaker( );
        ~SIFTPatchMaker( );
       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
        Ptr<FeatureDetector> _detector;                
};

#endif
