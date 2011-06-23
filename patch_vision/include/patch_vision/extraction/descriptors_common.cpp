/*
 * ===================================================================================== *
 *       Filename:  descriptors_common.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 03:40:49 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/extraction/descriptors_common.h>


using std::cout;
using std::endl;
using std::max;
using std::min;
using cv::split;
using cv::cvtColor;
using cv::KeyPoint;
using cv::Size;
using cv::Point2f;
using cv::getRotationMatrix2D;
using cv::warpAffine;
//using cv::IplImage;

//////////////////////////////////////////
//        CV DESCRIPTOR               //
//////////////////////////////////////////

CVDescriptor :: CVDescriptor( string name, ColorMode color_mode ){
    _name = name;
    _color_mode = color_mode;
    _descriptor_extractor = DescriptorExtractor::create(_name);
}

CVDescriptor :: ~CVDescriptor( ){ 
    //delete _descriptor_extractor;
}

void CVDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    vector<KeyPoint> keypts;
    float ctr_x =  (patch.size().width -1 ) / 2.;
    float ctr_y = ( patch.size().height-1 ) / 2.;
    float size = min(patch.size().width, patch.size().height);
    KeyPoint ctr( ctr_x, ctr_y, size );
    keypts.push_back(ctr);
    Mat descriptors;
    _descriptor_extractor->compute( patch, keypts, descriptors );
    for( int i = 0; i < descriptors.size().width;i++ ){
        feature.push_back( descriptors.at<float>(0,i) );
    }
}

void CVDescriptor :: process_image(    const Mat &image, vector<vector<float> > &features,
                                         vector< PatchDefinition* > &patch_definitions,
                                         const PatchMaker &pm, bool verbose ){
    Mat converted_image;
    get_proper_colors( image, converted_image);
    pm.get_patch_definitions ( converted_image, patch_definitions );
    vector<KeyPoint> keypts;
    if (verbose){
        cout << "Making keypoints" << endl;
    }
    for ( size_t i = 0; i < patch_definitions.size(); i++ ){
        keypts.push_back(patch_definitions[i]->get_keypoint() ); 
    }
    Mat descriptors;
    _descriptor_extractor->compute( converted_image, keypts, descriptors );
    for( int p = 0; p < descriptors.size().height ;p++ ){
        if (verbose){
            cout << "On patch " << p+1 << " of " << descriptors.size().height << endl;
        }
        vector<float> feature;
        float* feature_vals = descriptors.ptr<float>(p);
        for( int i = 0; i < descriptors.size().width; i++ ){
            feature.push_back( feature_vals[i] );
        }
        features.push_back(feature);
    }
}

string CVDescriptor :: name( ) const{
    return _name;
}

int CVDescriptor :: descriptor_size( ) const{
    return _descriptor_extractor->descriptorSize();
}

ColorMode CVDescriptor :: required_color_mode() const{
    return _color_mode;
}

//////////////////////////////////////////
//          STACKED DESCRIPTOR          //
//////////////////////////////////////////

StackedDescriptor :: StackedDescriptor( vector<Descriptor*> descriptors, vector<float> weights ){
    assert (descriptors.size() == weights.size());
    _descriptors = descriptors;
    _weights = weights;
}

StackedDescriptor :: ~StackedDescriptor( ){
    for (size_t i = 0; i < _descriptors.size(); i++){
        delete _descriptors[i];
    }
}

void StackedDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    //Must make sure the patch is in the proper color
    for (size_t i = 0; i < _descriptors.size(); i++){
        Mat converted_patch;
        _descriptors[i]->get_proper_colors(patch, converted_patch);
        _descriptors[i]->process_patch( converted_patch, feature, mask );
        converted_patch.release();
    }
}

//For speed, also do this at the image level
void StackedDescriptor :: process_image( const Mat &image, vector<vector<float> > &features,
                                         vector< PatchDefinition* > &patch_definitions,
                                         const PatchMaker &pm, bool verbose ){

    vector<vector<vector<float> > > features_unflattened;
    for (size_t d = 0; d < _descriptors.size(); d++){
        vector<vector<float> > desc_features;
        vector< PatchDefinition* >* patch_def_ptr;
        if (d == 0){
            patch_def_ptr = &patch_definitions;
        }
        else{
            patch_def_ptr = new vector< PatchDefinition* >();
        }
        _descriptors[d] -> process_image( image, desc_features, *patch_def_ptr, pm );
        features_unflattened.push_back(desc_features);
    }
    features.clear();
    features.resize( patch_definitions.size() );
    for (size_t p = 0; p < patch_definitions.size(); p++){
        if ( verbose ){
            cout << "On patch " << p+1 << " of " << patch_definitions.size() << endl;
        }
        for (size_t d = 0; d < features_unflattened.size(); d++){
            for (size_t i = 0; i < features_unflattened[d][p].size(); i++){
                features[p].push_back( _weights[d] * features_unflattened[d][p][i]);
            }
        }  
    }
}

string StackedDescriptor :: name( ) const{
    string nm = "";
    for (size_t d = 0; d < _descriptors.size(); d++){
        nm += _descriptors[d]->name();
        if (d + 1 < _descriptors.size() )
            nm += "+";
    }
    return nm;
}

int StackedDescriptor :: descriptor_size( ) const{
    int total = 0;
    for (size_t d = 0; d < _descriptors.size(); d++){
        total += _descriptors[d]->descriptor_size();
    }
    return total;
}

ColorMode StackedDescriptor :: required_color_mode( ) const{
    return ANY_COLOR;
}


//////////////////////////////////////////
//          COLORED DESCRIPTOR          //
//////////////////////////////////////////

ColoredDescriptor :: ColoredDescriptor( Descriptor* bw_descriptor, ColorMode color_mode ){
    _bw_descriptor = bw_descriptor;
    _color_mode = color_mode;
}


ColoredDescriptor :: ~ColoredDescriptor( ){
    delete _bw_descriptor;
}

void ColoredDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    vector<Mat> channels;
    split(patch, channels);
    for (size_t i = 0; i < channels.size(); i++){
        _bw_descriptor->process_patch( channels[i], feature, mask );
        channels[i].release();
    }

}

string ColoredDescriptor :: name( ) const{
    return "Color"+_bw_descriptor->name();
}

int ColoredDescriptor :: descriptor_size( ) const{
    return 3*_bw_descriptor->descriptor_size();
}


ColorMode ColoredDescriptor :: required_color_mode() const{
    return _color_mode;
}

//////////////////////////////////////////
//          ROTATED DESCRIPTOR          //
//////////////////////////////////////////

RotatedDescriptor :: RotatedDescriptor( Descriptor* bw_descriptor ){
    _bw_descriptor = bw_descriptor;
}


RotatedDescriptor :: ~RotatedDescriptor( ){
    delete _bw_descriptor;
}

void RotatedDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    float ctr_x = (patch.size().width-1)  / 2.;
    float ctr_y = (patch.size().height-1) / 2.;
    for (int i = 0; i < 4; i++){
        Mat r = getRotationMatrix2D( Point2f(ctr_x, ctr_y), 90*i, 1 );
        Mat rotated_patch;
        Mat rotated_mask;
        warpAffine( patch, rotated_patch, r, patch.size() );
        warpAffine( mask,  rotated_mask, r,  mask.size() );
        _bw_descriptor->process_patch( rotated_patch, feature, mask );
        rotated_patch.release();
        r.release();
    }

}

string RotatedDescriptor :: name( ) const{
    return "Rotated"+_bw_descriptor->name();
}

int RotatedDescriptor :: descriptor_size( ) const{
    return 4*_bw_descriptor->descriptor_size();
}


ColorMode RotatedDescriptor :: required_color_mode() const{
    return _bw_descriptor->required_color_mode();
}


//////////////////////////////////////////
//          LBP DESCRIPTOR              //
//////////////////////////////////////////

LBPDescriptor :: LBPDescriptor( ){}

LBPDescriptor :: ~LBPDescriptor( ){ }

void LBPDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
   calculate_points();
   int results[256];
   int height = patch.size().height;
   int width = patch.size().width;
   int new_ptr[height*width];
   int count = 0;
   for (int j = 0; j < height; j++){
       for (int i = 0; i < width; i++){
            new_ptr[count] = patch.at<uint8_t>(j,i);
            count++;
       }
   }
   const IplImage mask_ipl = mask;
   lbp_histogram((int*)(new_ptr),height,width,results,1, &mask_ipl);
    //L_1 normalization
    int hist_count = 0;
    for (int i=0; i<256; i++) {
        hist_count += results[i];
    }
    for(int i=0; i<256; i++) {
        if(hist_count != 0)
            feature.push_back(results[i]*1.0/hist_count);
        else
            feature.push_back(0.);
    }
   
}

string LBPDescriptor :: name( ) const{
    return "LBP";
}

int LBPDescriptor :: descriptor_size( ) const{
    return 256;
}

ColorMode LBPDescriptor :: required_color_mode() const{
    return BW;
}


//////////////////////////////////////////
//          HOG DESCRIPTOR              //
//////////////////////////////////////////

HOGDescriptor :: HOGDescriptor( int patch_size ){
    _patch_size = patch_size;
    int nearest8 = (((patch_size/2)/8)*8)*2;
    _descriptor = new HOGDescriptor_cv( Size(nearest8/2, nearest8), Size(16,16), 
                                        Size(8,8), Size(8,8), 9 );
}

HOGDescriptor :: ~HOGDescriptor( ){ 
    delete _descriptor;
}

//TODO Get mask working
void HOGDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    _descriptor->compute(patch, feature);

}

string HOGDescriptor :: name( ) const{
    return "HOG";
}

int HOGDescriptor :: descriptor_size( ) const{
    return _descriptor->getDescriptorSize();
}

ColorMode HOGDescriptor :: required_color_mode() const{
    return BW;
}



//////////////////////////////////////////
//        HUE HISTOGRAM DESCRIPTOR      //
//////////////////////////////////////////

HueHistogramDescriptor :: HueHistogramDescriptor ( int num_bins ){
    _num_bins = num_bins;
}

HueHistogramDescriptor :: ~HueHistogramDescriptor ( ){ };

void HueHistogramDescriptor :: process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ){
    vector<Mat> hsv;
    split( patch, hsv );
    vector<float> hues;
    //cout << "TYPE: " << hsv[0].type() << endl;
    //cout << "SIZE: " << hsv[0].size().width << ", " << hsv[0].size().height << ", " << hsv[0].channels() << endl;
    //cout << "CV_8U :" << CV_8UC3 << endl;
    //cout << "CV_8S :" << CV_8SC3 << endl;
    //cout << "CV_16U:" << CV_16UC3 << endl;
    //cout << "CV_16S:" << CV_16SC3 << endl;
    //cout << "CV_32S:" << CV_32SC3 << endl;
    //cout << "CV_32F:" << CV_32FC3 << endl;
    //cout << "CV_64F:" << CV_64FC3 << endl;
    //return;
    for( int i = 0; i < patch.size().width; i++){
        for( int j = 0; j < patch.size().height; j++){
            if (mask.at<uint8_t>(j,i) != 0){
                float val = hsv[0].at<uint8_t>(j,i);
                hues.push_back( val );
            }
        }
    }
    soft_histogram( hues, feature, _num_bins, 0.0, 180.0, true ); 
}

string HueHistogramDescriptor :: name( ) const{
    return "HueHistogramDescriptor";
}

int HueHistogramDescriptor :: descriptor_size( ) const{
    return _num_bins;
}

ColorMode HueHistogramDescriptor :: required_color_mode( ) const{
    return HSV;
}



/* Makes a normalized histogram of values with soft binning */
void soft_histogram( const vector<float> &values, vector<float> &output, int num_bins, float min_val, float max_val, bool cycle ){
  output = vector<float>(num_bins);
  if(num_bins == 0){
    return;
  }
  if (values.size() == 0){
    for (int i = 0; i < num_bins; i ++){
      output[i] = 0;
      return;
    }
  }
  float step = (max_val - min_val) / num_bins;
  int num_values = (int) values.size();
  for (int i = 0; i < num_values; i++){
    float val = values[i];
    if (val < min_val){
      output[0] += 1;
      continue;
    }
    if (val > max_val){
      output[num_bins - 1] += 1;
      continue;
    }
    float val_normalized = (val - min_val) / step;
    int my_bin = (int) floor(val_normalized);
    float dist_from_center = val_normalized - (my_bin + 0.5);
    int nearest_bin;
    if (dist_from_center < 0)
      nearest_bin = my_bin - 1;
    else
      nearest_bin = my_bin + 1;
    if ( cycle )
      nearest_bin = int_mod(nearest_bin, num_bins);
    else{
      nearest_bin = max(nearest_bin,0);
      nearest_bin = min(nearest_bin, num_bins-1);
    }
    //Assign the values
    output[my_bin] += (1.0 - fabs(dist_from_center));
    output[nearest_bin] += fabs(dist_from_center);
  }
  //Normalize
  float sum = 0;
  for (int i = 0; i < num_bins; i++){
    sum += output[i];
  }
  for (int i = 0; i < num_bins; i++){
    output[i] /= sum;
  }
}

float float_mod(float a, float b){
  float ret_val = fmod(a,b);
  if (ret_val < 0){
    ret_val += b;
  }
  if (ret_val == b)
    ret_val = 0;
  return ret_val;
}

int int_mod(int a, int b){
  int ret_val = a % b;
  if (ret_val < 0){
    ret_val += b;
  }
  return ret_val;
}

