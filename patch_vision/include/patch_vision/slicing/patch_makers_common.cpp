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
using std::cout;

using cv::Range;
using cv::circle;
using cv::Point2f;

////////////////////////////////
//      RectangularPatch      //
////////////////////////////////

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

PatchShape RectangularPatch :: shape( ) const{
    return SQUARE;
}

int RectangularPatch :: size( ) const{
    //return _width * _height;
    return min(_height, _width);
}

void RectangularPatch :: extract_from_image(const Mat &image, Mat &patch, Mat &mask) const{
    int start_x = max(_x, 0);
    int start_y = max(_y, 0);
    int end_x = min(_x + _width, image.size().width);
    int end_y = min(_y + _height, image.size().height);
    patch = image( Range(start_y, end_y), Range(start_x, end_x) );
    mask = Mat::ones( patch.size().height, patch.size().width, CV_8UC1 );
}


////////////////////////////////
//      CircularPatch         //
////////////////////////////////

CircularPatch :: CircularPatch ( int x, int y, int diameter ){
    _x = x;
    _y = y;
    _diameter = diameter;
}

CircularPatch :: ~CircularPatch ( ){ };

pair<double, double> CircularPatch :: center( ) const{
    pair<double, double> ctr;
    ctr.first = _x +  (_diameter - 1)/2.;
    ctr.second = _y + (_diameter - 1)/2.;
    return ctr;
}

PatchShape CircularPatch :: shape( ) const{
    return CIRCLE;
}

int CircularPatch :: size( ) const{
    //return _width * _height;
    return _diameter;
}

void CircularPatch :: extract_from_image(const Mat &image, Mat &patch, Mat &mask) const{
    int start_x = max(_x, 0);
    int start_y = max(_y, 0);
    int end_x = min(_x + _diameter, image.size().width);
    int end_y = min(_y + _diameter, image.size().height);
    patch = image( Range(start_y, end_y), Range(start_x, end_x) );
    mask = Mat::zeros( patch.size().height, patch.size().width, CV_8UC1 );
    Point2f ctr = Point2f( (patch.size().width - 1) / 2., (patch.size().height - 1) / 2. );
    circle( mask, ctr, _diameter/2., 1, -1 );
}


////////////////////////////////
//      KeyPointPatch         //
////////////////////////////////


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


PatchShape KeyPointPatch :: shape( ) const{
    return CIRCLE;
}

int KeyPointPatch :: size( ) const{
    //return _width * _height;
    return _kp.size;
}

void KeyPointPatch :: extract_from_image(const Mat &image, Mat &patch, Mat &mask) const{
    float start_x = max(_kp.pt.x-(_kp.size-1)/2., 0.);
    float start_y = max(_kp.pt.y-(_kp.size-1)/2., 0.);
    float end_x = min(_kp.pt.x + (_kp.size-1)/2., (double) image.size().width);
    float end_y = min(_kp.pt.y + (_kp.size-1)/2., (double) image.size().height);
    patch = image( Range(start_y, end_y), Range(start_x, end_x) );
    mask = Mat::zeros( patch.size().height, patch.size().width, CV_8UC1 );
    Point2f ctr = Point2f( (patch.size().width - 1) / 2., (patch.size().height - 1) / 2. );
    circle( mask, ctr, size()/2., 1, -1 );
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

SlidingCirclePatchMaker :: SlidingCirclePatchMaker ( int diameter, int step_x, int step_y ){
    _diameter = diameter;
    _step_x = step_x;
    _step_y = step_y;
}

SlidingCirclePatchMaker :: ~SlidingCirclePatchMaker ( ){ };

void SlidingCirclePatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    int image_width = image.size().width;
    int image_height = image.size().height;
    for( int i = 0; i < image_width - _diameter + 1; i += _step_x ){
        for( int j = 0; j < image_height - _diameter + 1; j += _step_y ){
            patch_definitions.push_back( new CircularPatch(i, j, _diameter) );
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


PointsSquarePatchMaker :: PointsSquarePatchMaker ( string input_points_file, int patch_size ){
    _point_set.read_from_file(input_points_file);
    _patch_size = patch_size;
}

PointsSquarePatchMaker :: ~PointsSquarePatchMaker ( ){ };

void PointsSquarePatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    for( size_t i = 0; i < _point_set.num_points(); i++ ){
        PointT point = _point_set.get_point(i);
        patch_definitions.push_back( new RectangularPatch(point.first, point.second, _patch_size, _patch_size ) );
    }
}

PointsCirclePatchMaker :: PointsCirclePatchMaker ( string input_points_file, int patch_size ){
    _point_set.read_from_file(input_points_file);
    _patch_size = patch_size;
}

PointsCirclePatchMaker :: ~PointsCirclePatchMaker ( ){ };

void PointsCirclePatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    for( size_t i = 0; i < _point_set.num_points(); i++ ){
        PointT point = _point_set.get_point(i);
        patch_definitions.push_back( new CircularPatch(point.first, point.second, _patch_size ) );
    }
}
