#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__
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

#include <patch_vision/slicing/patch_maker.h>
#include <patch_vision/slicing/patch_makers_common.h>

using std::vector;
using std::pair;
using std::string;
using cv::Mat;

enum ColorMode{
    ANY_COLOR, BW, RGB, HSV, Opponent, LUV
};

class Descriptor{
    public:
        //Descriptor();
        //~Descriptor();
        void process_patch( const Mat &patch, vector<float> &feature );
        virtual void process_patch( const Mat &patch, vector<float> &feature, const Mat &mask ) = 0;
        //virtual void process_patch( const Mat &image, pair<float,float> &center, vector<float> &feature ) = 0;
        virtual string name( ) const = 0;
        virtual int descriptor_size( ) const = 0;

        virtual int required_channels( ) const { return 0; };
        virtual ColorMode required_color_mode( ) const { return BW; };
        
        virtual void process_image( const Mat &image, const Mat &mask, vector<vector<float> > &features, 
                            vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm,
                            bool verbose=false );
        virtual void process_image( const Mat &image, vector<vector<float> > &features, 
                            vector< PatchDefinition* > &patch_definitions, const PatchMaker &pm,
                            bool verbose=false );

        void get_proper_colors( const Mat &image, Mat &converted_image );
};

#endif
