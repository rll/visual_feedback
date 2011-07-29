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
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
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
using cv::Point;
using cv::minMaxLoc;
using cv::GaussianBlur;
using cv::Size;
using cv::absdiff;
using cv::cvtColor;

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

void RectangularPatch :: shift_by(int dx, int dy){
    _x += dx;
    _y += dy;
}

void RectangularPatch :: scale_by(float scale){
    _width  *= scale;
    _height *= scale;
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

void CircularPatch :: shift_by(int dx, int dy){
    _x += dx;
    _y += dy;
}

void CircularPatch :: scale_by(float scale){
    _diameter  *= scale;
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

void KeyPointPatch :: shift_by(int dx, int dy){
    _kp.pt.x += dx;
    _kp.pt.y += dy;
}

void KeyPointPatch :: scale_by(float scale){
    _kp.size  *= scale;
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
    float end_x = min(_kp.pt.x + (_kp.size-1)/2.+1., (double) image.size().width);
    float end_y = min(_kp.pt.y + (_kp.size-1)/2.+1, (double) image.size().height);
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





PointsSquarePatchMaker :: PointsSquarePatchMaker ( string input_points_file, int patch_size ){
    _point_set.read_from_file(input_points_file);
    _patch_size = patch_size;
}

PointsSquarePatchMaker :: ~PointsSquarePatchMaker ( ){ };

void PointsSquarePatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    for( size_t i = 0; i < _point_set.num_points(); i++ ){
        PointT point = _point_set.get_point(i);
        float start_x = point.first - (_patch_size-1)/2.;
        float start_y = point.second -(_patch_size-1)/2.;
        patch_definitions.push_back( new RectangularPatch(start_x, start_y, _patch_size, _patch_size ) );
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
        float start_x = point.first - (_patch_size-1)/2.;
        float start_y = point.second -(_patch_size-1)/2.;
        patch_definitions.push_back( new CircularPatch(start_x, start_y, _patch_size ) );
    }
}

////////////////////////////////
//      CVPatchMaker      //
////////////////////////////////
CVPatchMaker :: CVPatchMaker( string type ) : _min_patch_size( -1 ), _max_patch_size( -1 ) {
    _detector = FeatureDetector::create(type);
}

CVPatchMaker :: ~CVPatchMaker( ){  }

void CVPatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition* > &patch_definitions ) const{
    vector<KeyPoint> key_points;
    cout << "Detecting key points" << endl;
    _detector->detect( image, key_points );
    cout << "Detected " << key_points.size() << " keypoints" << endl;
    for ( size_t i = 0; i < key_points.size(); i++){
        if( is_within_bounds( key_points[i] ) ){
            patch_definitions.push_back( new KeyPointPatch( key_points[i] ) );
        }
    }
}

void CVPatchMaker :: set_bounds ( int min_patch_size, int max_patch_size ) {
    _min_patch_size = min_patch_size;
    _max_patch_size = max_patch_size;
}

bool CVPatchMaker :: is_within_bounds ( const KeyPoint &kp ) const{
    bool valid = true;
    if (_min_patch_size >= 0 && kp.size < _min_patch_size){
        valid = false;
    }
    if (_max_patch_size >= 0 && kp.size > _max_patch_size){
        valid = false;
    }
    return valid;

}

////////////////////////////////
//      SIFTPatchMaker      //
////////////////////////////////
SIFTPatchMaker :: SIFTPatchMaker( ) :
  CVPatchMaker("SIFT")
{}

SIFTPatchMaker :: ~SIFTPatchMaker( ) { }

////////////////////////////////
//      MSERPatchMaker      //
////////////////////////////////
MSERPatchMaker :: MSERPatchMaker( ) :
  CVPatchMaker("MSER")
{}

MSERPatchMaker :: ~MSERPatchMaker( ) { }


////////////////////////////////
//      STARPatchMaker      //
////////////////////////////////
STARPatchMaker :: STARPatchMaker( ) :
  CVPatchMaker("STAR")
{}

STARPatchMaker :: ~STARPatchMaker( ) { }

////////////////////////////////
//      SURFPatchMaker      //
////////////////////////////////
SURFPatchMaker :: SURFPatchMaker( ) :
  CVPatchMaker("SURF")
{}

SURFPatchMaker :: ~SURFPatchMaker( ) { }

////////////////////////////////
//       DIPPatchMaker        //
////////////////////////////////

DIPPatchMaker :: DIPPatchMaker( PatchMaker* dense_patch_maker )
    : _dense_patch_maker( dense_patch_maker ){ };

/*  NOTE: I will kill the dense_patch_maker when I'm done with it */
DIPPatchMaker :: ~DIPPatchMaker( ){ 
    delete _dense_patch_maker;
};

void DIPPatchMaker :: get_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    /* First want to get the original, dense patch definitions */
    _dense_patch_maker->get_patch_definitions( image, patch_definitions );
    /*  Want to adjust to the proper ones */
    refine_patch_definitions( image, patch_definitions );
}

void DIPPatchMaker :: refine_patch_definitions( const Mat &image, vector<PatchDefinition*> &patch_definitions ) const{
    Mat rank_image;
    rank_patch_centers( image, rank_image );
    for ( size_t i = 0; i < patch_definitions.size(); i++ ){
        /*  Only consider the region in this patch */
        Mat rank_patch;
        Mat rank_mask;
        patch_definitions[i]->extract_from_image( rank_image, rank_patch, rank_mask );
        Point patch_ctr = Point( (rank_patch.size().width - 1) / 2, (rank_patch.size().height - 1) / 2 );
        Point max_loc;
        minMaxLoc( rank_patch, NULL, NULL, NULL, &max_loc, rank_mask );
        Point offset = max_loc - patch_ctr;
        patch_definitions[i]->shift_by( offset.x, offset.y );
    }
}

////////////////////////////////
//       DOGPatchMaker        //
////////////////////////////////

DOGPatchMaker :: DOGPatchMaker( PatchMaker* dense_patch_maker, int k1, int k2 )
    : DIPPatchMaker( dense_patch_maker ), _k1(k1), _k2(k2) { };

DOGPatchMaker :: ~DOGPatchMaker( ){ };

//FIXME This assumes any 3 channel image is HSV. Clearly not the case.
void DOGPatchMaker :: rank_patch_centers( const Mat &image, Mat &rank_image ) const{
    Mat gray_image;
    if( image.channels() == 1 ){
        gray_image = image;
    }
    else{
        Mat bgr_image;
        cvtColor(image, bgr_image, CV_HSV2BGR );
        cvtColor(bgr_image, gray_image, CV_BGR2GRAY);
    }
    Mat blur1, blur2;
    GaussianBlur( gray_image, blur1, Size(_k1, _k1), 0 );  
    GaussianBlur( gray_image, blur2, Size(_k2, _k2), 0 );  
    absdiff(blur1, blur2, rank_image);
}
