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
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <iostream>
#include <patch_vision/slicing/patch_maker.h>
#include <opencv2/features2d/features2d.hpp>
#include <patch_vision/slicing/pt_io.h>

using cv::FeatureDetector;
using cv::Ptr;
using cv::KeyPoint;

class RectangularPatch : public PatchDefinition{
    public:
        /* Create a rectangular of size (width, height) whose
         * top left point is at (x,y) */
        RectangularPatch( int x, int y, int width, int height );
        ~RectangularPatch( );

        pair<double, double> center() const;
        PatchShape shape() const;
        int size() const;
        void extract_from_image(const Mat &image, Mat &patch, Mat &mask) const;

    private:
        int _x, _y, _width, _height;
        
};

class CircularPatch : public PatchDefinition{
    public:
        /* Create a circular patch inscribed in a square
           of size (diameter,diameter) whose top left point is at (x,y) */
        CircularPatch( int x, int y, int diameter );
        CircularPatch( const KeyPoint &kp );
        ~CircularPatch( );

        pair<double, double> center() const;
        PatchShape shape() const;
        int size() const;
        void extract_from_image(const Mat &image, Mat &patch, Mat &mask) const;

    private:
        int _x, _y, _diameter;
        
};

class KeyPointPatch : public PatchDefinition{
    public:
        /* Create a patch based on a CV keypoint */
        KeyPointPatch( KeyPoint &kp );
        ~KeyPointPatch( );

        pair<double, double> center() const;
        PatchShape shape() const;
        int size() const;
        void extract_from_image(const Mat &image, Mat &patch, Mat &mask) const;
        
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

class SlidingCirclePatchMaker : public PatchMaker{
    public:
       SlidingCirclePatchMaker( int diameter, int step_x, int step_y );
       ~SlidingCirclePatchMaker( );

       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
       int _diameter, _step_x, _step_y;
};

class SIFTPatchMaker : public PatchMaker{
    public:
        SIFTPatchMaker( );
        ~SIFTPatchMaker( );
       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
        Ptr<FeatureDetector> _detector;                
};

class PointsSquarePatchMaker : public PatchMaker{
    public:
        PointsSquarePatchMaker( string input_points_file, int patch_size);
        ~PointsSquarePatchMaker( );
       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
        PointSet _point_set;
        int _patch_size;             
};

class PointsCirclePatchMaker : public PatchMaker{
    public:
        PointsCirclePatchMaker( string input_points_file, int patch_size);
        ~PointsCirclePatchMaker( );
       void get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const;

    private:
        PointSet _point_set;
        int _patch_size;             
};
#endif
