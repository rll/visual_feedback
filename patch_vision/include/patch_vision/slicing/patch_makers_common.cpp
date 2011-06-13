/*
 * =====================================================================================
 *
 *       Filename:  patch_makers_common.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 02:50:53 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/slicing/patch_makers_common.h>

using std::min;
using cv::Range;
using cv::Size_;

RectangularPatch :: RectangularPatch ( int x, int y, int width, int height ){
    _x = x;
    _y = y;
    _width = width;
    _height = height;
}

RectangularPatch :: ~RectangularPatch ( ){ };

pair<double, double> RectangularPatch :: center( ) const{
    pair<double, double> ctr;
    ctr.first = _x + (_width - 1)/2.;
    ctr.second = _y + (_height - 1)/2.;
    return ctr;
}

int RectangularPatch :: size( ) const{
    //return _width * _height;
    return min(_height, _width);
}

void RectangularPatch :: extract_from_image(const Mat &image, Mat &patch) const{
    patch = image( Range(_y, _y + _height), Range(_x, _x + _height) );
}


SlidingWindowPatchMaker :: SlidingWindowPatchMaker ( int width, int height, int step_x, int step_y ){
    _width = width;
    _height = height;
    _step_x = step_x;
    _step_y = step_y;
}

SlidingWindowPatchMaker :: ~SlidingWindowPatchMaker ( ){ };

void SlidingWindowPatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    int image_width = image.size().width;
    int image_height = image.size().height;
    for( int i = 0; i < image_width - _width + 1; i += _step_x ){
        for( int j = 0; j < image_height - _height + 1; j += _step_y ){
            patch_definitions.push_back( new RectangularPatch(i, j, _width, _height) );
        }
    }
}

