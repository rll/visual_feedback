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
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
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

LBPDescriptor :: LBPDescriptor( int patch_size ){
    _patch_size = patch_size;
}

LBPDescriptor :: ~LBPDescriptor( ){ }

void LBPDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
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
   lbp_histogram((int*)(new_ptr),height,width,results,1);
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

int LBPDescriptor :: patch_size( ) const{
    return _patch_size;
}

ColorMode LBPDescriptor :: required_color_mode() const{
    return BW;
}


StackedDescriptor :: StackedDescriptor( vector<Descriptor*> descriptors, vector<double> weights ){
    assert (descriptors.size() == weights.size());
    _descriptors = descriptors;
    _weights = weights;
}

StackedDescriptor :: ~StackedDescriptor( ){
    for (size_t i = 0; i < _descriptors.size(); i++){
        delete _descriptors[i];
    }
}

void StackedDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
    //Must make sure the patch is in the proper color
    for (size_t i = 0; i < _descriptors.size(); i++){
        Mat converted_patch;
        _descriptors[i]->get_proper_colors(patch, converted_patch);
        _descriptors[i]->process_patch( converted_patch, feature );
        converted_patch.release();
    }
}

//For speed, also do this at the image level
void StackedDescriptor :: process_image( const Mat &image, vector<vector<double> > &features,
                                         vector< PatchDefinition* > &patch_definitions,
                                         const PatchMaker &pm, bool verbose ){

    vector<vector<vector<double> > > features_unflattened;
    for (size_t d = 0; d < _descriptors.size(); d++){
        vector<vector<double> > desc_features;
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

int StackedDescriptor :: patch_size( ) const{
    int max_size = 0;
    for (size_t d = 0; d < _descriptors.size(); d++){
        int new_size = _descriptors[d]->patch_size();
        if (new_size > max_size){
            max_size = new_size;
        }
    }
    return max_size;
}

ColorMode StackedDescriptor :: required_color_mode( ) const{
    return ANY_COLOR;
}


ColoredDescriptor :: ColoredDescriptor( Descriptor* bw_descriptor, ColorMode color_mode ){
    _bw_descriptor = bw_descriptor;
    _color_mode = color_mode;
}


ColoredDescriptor :: ~ColoredDescriptor( ){
    delete _bw_descriptor;
}

void ColoredDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
    vector<Mat> channels;
    split(patch, channels);
    for (size_t i = 0; i < channels.size(); i++){
        _bw_descriptor->process_patch( channels[i], feature );
        channels[i].release();
    }

}

string ColoredDescriptor :: name( ) const{
    return "Color"+_bw_descriptor->name();
}

int ColoredDescriptor :: descriptor_size( ) const{
    return 3*_bw_descriptor->descriptor_size();
}

int ColoredDescriptor :: patch_size( ) const{
    return _bw_descriptor->patch_size();
}

ColorMode ColoredDescriptor :: required_color_mode() const{
    return _color_mode;
}


HueHistogramDescriptor :: HueHistogramDescriptor ( int patch_size, int num_bins ){
    _patch_size = patch_size;
    _num_bins = num_bins;
}

HueHistogramDescriptor :: ~HueHistogramDescriptor ( ){ };

void HueHistogramDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
    vector<Mat> hsv;
    split( patch, hsv );
    vector<double> hues;
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
            float val = hsv[0].at<uint8_t>(j,i);
            hues.push_back( val );
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

int HueHistogramDescriptor :: patch_size( ) const{
    return _patch_size;
}

ColorMode HueHistogramDescriptor :: required_color_mode( ) const{
    return HSV;
}



/* Makes a normalized histogram of values with soft binning */
void soft_histogram( const vector<double> &values, vector<double> &output, int num_bins, double min_val, double max_val, bool cycle ){
  output = vector<double>(num_bins);
  if(num_bins == 0){
    return;
  }
  if (values.size() == 0){
    for (int i = 0; i < num_bins; i ++){
      output[i] = 0;
      return;
    }
  }
  double step = (max_val - min_val) / num_bins;
  int num_values = (int) values.size();
  for (int i = 0; i < num_values; i++){
    double val = values[i];
    if (val < min_val){
      output[0] += 1;
      continue;
    }
    if (val > max_val){
      output[num_bins - 1] += 1;
      continue;
    }
    double val_normalized = (val - min_val) / step;
    int my_bin = (int) floor(val_normalized);
    double dist_from_center = val_normalized - (my_bin + 0.5);
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
  double sum = 0;
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


SIFTDescriptor :: SIFTDescriptor( int patch_size ){
    _patch_size = patch_size;
    _descriptor_extractor = DescriptorExtractor::create("SIFT");
}

SIFTDescriptor :: ~SIFTDescriptor( ){ 
    //delete _descriptor_extractor;
}

void SIFTDescriptor :: process_patch( const Mat &patch, vector<double> &feature ){
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

void SIFTDescriptor :: process_image(    const Mat &image, vector<vector<double> > &features,
                                         vector< PatchDefinition* > &patch_definitions,
                                         const PatchMaker &pm, bool verbose ){
    Mat converted_image;
    get_proper_colors( image, converted_image);
    pm.get_patch_definitions ( converted_image, patch_definitions );
    vector<KeyPoint> keypts;
    for ( size_t i = 0; i < patch_definitions.size(); i++ ){
        float ctr_x = patch_definitions[i]->center().first; 
        float ctr_y = patch_definitions[i]->center().second;
        float size  = patch_definitions[i]->size();
        keypts.push_back(KeyPoint(ctr_x, ctr_y, size) ); 
    }
    Mat descriptors;
    _descriptor_extractor->compute( converted_image, keypts, descriptors );
    for( int p = 0; p < descriptors.size().height ;p++ ){
        if (verbose){
            cout << "On patch " << p+1 << " of " << descriptors.size().height << endl;
        }
        vector<double> feature;
        float* feature_vals = descriptors.ptr<float>(p);
        for( int i = 0; i < descriptors.size().width; i++ ){
            feature.push_back( feature_vals[i] );
        }
        features.push_back(feature);
    }
}

string SIFTDescriptor :: name( ) const{
    return "SIFT";
}

int SIFTDescriptor :: descriptor_size( ) const{
    return _descriptor_extractor->descriptorSize();
}

int SIFTDescriptor :: patch_size( ) const{
    return _patch_size;
}

ColorMode SIFTDescriptor :: required_color_mode() const{
    return BW;
}
