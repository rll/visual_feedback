/*
 * =====================================================================================
 *
 *       Filename:  make_featuremap.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 04:36:31 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Arjun Singh (arjun), arjun@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <iostream>

#include <patch_vision/extraction/descriptors_common.h>
#include <patch_vision/extraction/feature_io.h>
#include <patch_vision/slicing/patch_makers_common.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

using std::cout;
using std::endl;
using std::strcmp;
using cv::Mat;
using cv::imread;

enum FeatureT{
    RAW_BW,
    RAW_COLOR,
    LBP
};

struct Options
{
    string image_file;
    string output_file;
    string feature_name;
    FeatureT feature;

    int patch_size;
    int step_size;

    bool verbose;
    boost::program_options::options_description desc;
};

namespace po = boost::program_options;

int options(int ac, char ** av, Options& opts)
{
    // Declare the supported options.
    po::options_description desc = opts.desc;
    desc.add_options()("help", "Produce help message.");
    desc.add_options()
      ("image_file,i"   , po::value<string>(&opts.image_file),  "input image file")
      ("output_file,o"  , po::value<string>(&opts.output_file), "output featuremap file")
      ("feature_type,f" , po::value<string>(&opts.feature_name),  "descriptor to use")
      ("patch_size,p"   , po::value<int>(&opts.patch_size),     "patch size")
      ("step_size,s"    , po::value<int>(&opts.step_size),      "step size")
      ("verbose,v", "Whether to print out debugging statements")
      ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);


    if (vm.count("help"))
    {
        cout << desc << endl;
        return 1;
    }
    if (vm.count("verbose")) {
        opts.verbose = true;
    } else{
        opts.verbose = false;
    }
    if ( !strcmp(opts.feature_name.c_str(), "RAW_BW") ){
        opts.feature = RAW_BW;
    }
    else if ( !strcmp(opts.feature_name.c_str(), "RAW_COLOR") ){
        opts.feature = RAW_COLOR;
    }
    else if ( !strcmp(opts.feature_name.c_str(), "LBP") ){
        opts.feature = LBP;
    }
    else{
        cout << opts.feature_name << " is not a valid descriptor" << endl;
        return 1;
    }
    return 0;
}

int main(int argc, char** argv) {
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    // Read in the image
    Mat image = imread(opts.image_file);
    // Create the Patch Maker
    SlidingWindowPatchMaker* pm = new SlidingWindowPatchMaker(opts.patch_size, opts.patch_size, opts.step_size, opts.step_size );
    // Create the descriptor
    Descriptor* descriptor;
    switch ( opts.feature ){
        case RAW_BW:
            //Do nothing
            break;
        
        case RAW_COLOR:
            //Do nothing
            break;

        case LBP:
            descriptor = new LBPDescriptor( opts.patch_size );
            break;
    }
    vector< vector<double> > features;
    vector< PatchDefinition* > patch_definitions;
    descriptor->process_image( image, features, patch_definitions, *pm );
    // Save the featuremap
    FeatureMap fm;
    fm.set_patch_size( opts.patch_size );
    for( size_t i = 0; i < features.size(); i++ ){
      fm.add_feature( patch_definitions[i], features[i] );
    }
    fm.save_to_file( opts.output_file );
    cout << "Successful" << endl;
    
}

