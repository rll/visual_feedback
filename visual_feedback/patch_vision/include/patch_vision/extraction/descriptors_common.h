#ifndef __DESCRIPTORS_COMMON_H__
#define __DESCRIPTORS_COMMON_H__
/*
 * =====================================================================================
 *
 *       Filename:  descriptor.h
 *
 *    Description: 
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

#include <patch_vision/extraction/descriptor.h>
#include <patch_vision/thirdparty/LBP.h>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>

using cv::Ptr;
using cv::DescriptorExtractor;

class LBPDescriptor : public Descriptor{
    public:

        LBPDescriptor( int patch_size );
        ~LBPDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        ColorMode required_color_mode( ) const;
        
    private:
        int _patch_size;
};


class StackedDescriptor : public Descriptor{
    public:
        
        StackedDescriptor( vector<Descriptor*> descriptors, vector<double> weights );
        ~StackedDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        void process_image( const Mat &image, vector<vector<double> > &features, 
                            vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm, 
                            bool verbose=false );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        ColorMode required_color_mode( ) const;

    private:
        vector<Descriptor*> _descriptors;
        vector<double> _weights;
};

class ColoredDescriptor : public Descriptor{
    public:
        
        ColoredDescriptor( Descriptor* bw_descriptor, ColorMode color_mode );
        ~ColoredDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        ColorMode required_color_mode( ) const;

    private:
        Descriptor* _bw_descriptor;
        ColorMode _color_mode;

};

class HueHistogramDescriptor : public Descriptor{
    public:

        HueHistogramDescriptor( int patch_size, int num_bins );
        ~HueHistogramDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        ColorMode required_color_mode( ) const;
        
    private:
        int _patch_size;
        int _num_bins;

};

void soft_histogram( const vector<double> &values, vector<double> &output, int num_bins, double min_val, double max_val, bool cycle );

float float_mod(float a, float b);
int int_mod(int a, int b);

class SIFTDescriptor : public Descriptor{
    public:
        SIFTDescriptor( int patch_size );
        ~SIFTDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        void process_image( const Mat &image, vector<vector<double> > &features, 
                            vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm, 
                            bool verbose=false );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        ColorMode required_color_mode( ) const;

    private:
        int _patch_size;
        Ptr<DescriptorExtractor> _descriptor_extractor;
        
};

#endif
