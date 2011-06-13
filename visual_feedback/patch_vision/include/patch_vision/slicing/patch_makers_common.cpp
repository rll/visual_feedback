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
using std::max;
using cv::Range;
using std::cout;


RectangularPatch :: RectangularPatch ( int x, int y, int width, int height ){
    _x = x;
    _y = y;
    _width = width;
    _height = height;
}
 RectangularPatch :: RectangularPatch ( const KeyPoint &kp ){
    double ctr_x = kp.pt.x;
    double ctr_y = kp.pt.y;
    double size = kp.size;
    _x = ctr_x - (size-1)/2.;
    _y = ctr_y - (size-1)/2.;
    _width = size;
    _height = size;
    //cout << "Made patch at " << _x << ", " << _y << " of size " << size << "x" << size << endl;
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
    float start_x = max(_x, 0);
    float start_y = max(_y, 0);
    float end_x = min(_x + _width, image.size().width);
    float end_y = min(_y + _height, image.size().height);
    patch = image( Range(start_y, end_y), Range(start_x, end_x) );
}

KeyPoint RectangularPatch :: get_keypoint( ) const{
    float ctr_x =  center().first;
    float ctr_y =  center().second;
    float sz    =   size();
    return KeyPoint( ctr_x, ctr_y, sz );
    
}

 KeyPointPatch :: KeyPointPatch ( KeyPoint &kp ){
    _kp = kp;
}

KeyPointPatch :: ~KeyPointPatch ( ){ };

pair<double, double> KeyPointPatch :: center( ) const{
    pair<double, double> ctr;
    ctr.first = _kp.pt.x;
    ctr.second = _kp.pt.y;
    return ctr;
}

int KeyPointPatch :: size( ) const{
    //return _width * _height;
    return _kp.size;
}

void KeyPointPatch :: extract_from_image(const Mat &image, Mat &patch) const{
    float start_x = max(_kp.pt.x-(_kp.size-1)/2., 0.);
    float start_y = max(_kp.pt.y-(_kp.size-1)/2., 0.);
    float end_x = min(_kp.pt.x + (_kp.size-1)/2., (double) image.size().width);
    float end_y = min(_kp.pt.y + (_kp.size-1)/2., (double) image.size().height);
    patch = image( Range(start_y, end_y), Range(start_x, end_x) );
}

KeyPoint KeyPointPatch :: get_keypoint( ) const{
    return _kp;
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

SIFTPatchMaker :: SIFTPatchMaker( ){
    _detector = FeatureDetector::create("SIFT");
}

SIFTPatchMaker :: ~SIFTPatchMaker( ){ }

void SIFTPatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const{
    vector<KeyPoint> key_points;
    cout << "Detecting key points" << endl;
    _detector->detect( image, key_points );
    cout << "Detected " << key_points.size() << " keypoints" << endl;
    for ( size_t i = 0; i < key_points.size(); i++){
        patch_definitions.push_back( new KeyPointPatch( key_points[i] ) );
    }
}


