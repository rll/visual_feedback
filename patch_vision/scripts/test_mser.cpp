/*
 * =====================================================================================
 *
 *       Filename:  test_mser.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/02/2011 01:09:27 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <iostream>

#include <patch_vision/extraction/descriptors_common.h>
#include <patch_vision/extraction/feature_io.h>
#include <patch_vision/slicing/patch_makers_common.h>
#include <patch_vision/slicing/pt_io.h>
#include <patch_vision/slicing/label_extraction.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/program_options.hpp>

using std::cout;
using std::endl;
using std::strcmp;
using cv::Mat;
using cv::imread;

using cv::MSER;
using cv::Point;

struct Options
{
    string image_file, mask_file;
    int delta;
    int min_area;
    int max_area;
    float max_variation;
    float min_diversity;
    int max_evolution;
    double area_threshold;
    double min_margin;
    int edge_blur_size;
    boost::program_options::options_description desc;
};

namespace po = boost::program_options;

int options(int ac, char ** av, Options& opts)
{
    // Declare the supported options.
    po::options_description desc = opts.desc;
    desc.add_options()("help", "Produce help message.");
    desc.add_options()
      ("image_file,i"   , po::value<string>(&opts.image_file),                  "input image file")
      ("delta,D"        , po::value<int>(&opts.delta)->default_value(5),        "delta")
      ("min_area,a"     , po::value<int>(&opts.min_area)->default_value(1000),    "min_area")
      ("max_area,A"     , po::value<int>(&opts.max_area)->default_value(144000),    "max_area")
      ("max_variation,V", po::value<float>(&opts.max_variation)->default_value(.25f),    "max_variation")
      ("min_diversity,d", po::value<float>(&opts.min_diversity)->default_value(.2f),     "min_diversity")
      ("max_evolution,E", po::value<int>(&opts.max_evolution)->default_value(200),    "max_evolution")
      ("area_threshold,t", po::value<double>(&opts.area_threshold)->default_value(1.01),"area_threshold")
      ("min_margin,m"   , po::value<double>(&opts.min_margin)->default_value(.003),     "min_margin")
      ("edge_blur_size,b", po::value<int>(&opts.edge_blur_size)->default_value(5),    "edge_blur_size")
      ("mask,M"          , po::value<string>(&opts.mask_file), "Mask file to use")
      ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        cout << desc << endl;
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
    //Mat mask  = imread( opts.mask_file, 0);
    Mat mask (image.rows, image.cols, CV_8UC1 );
    mask = 255;
    // Create the Patch Maker
    MSER mser ( opts.delta, opts.min_area, opts.max_area, opts.max_variation, opts.min_diversity, 
              opts.max_evolution, opts.area_threshold, opts.min_margin, opts.edge_blur_size );
    vector< vector< Point > > contours;
    cout << "Calling MSER";
    mser( image, contours, mask );
    cout << "...done" << endl;
    Mat vis; 
    image.copyTo(vis);
    cout << "Found " << contours.size() << "Contours" << endl;
    //for( size_t i = 0; i < contours.size(); i++ ){
    //    for ( size_t j = 0; j < contours[i].size(); j++ ){
    //        cv::circle( vis, contours[i][j], 0.5, cv::Scalar(0,0,255), -1 );
    //    }
    //}
    cv::namedWindow("MSER",CV_WINDOW_AUTOSIZE);
    Mat Tmp1, Tmp2, Vis;
    image.copyTo(Vis);
    cv::drawContours( Vis, contours, -1, cv::Scalar(0,0,255), 4);
    cv::pyrDown( Vis, Tmp1 );
    cv::pyrDown( Tmp1, Tmp2 );
    cv::imshow("MSER", Tmp2 );
    cv::waitKey();
    Mat test_contours (image.rows, image.cols, CV_8UC1 );
    test_contours = cv::Scalar(0);
    cv::drawContours( test_contours, contours, -1, 255 );
    vector< vector< Point > > contours_external;
    cv::findContours( test_contours, contours_external, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
    cv::drawContours( vis, contours_external, -1, cv::Scalar(0,0,255), 4 );
    Mat tmp1, tmp2;
    cv::pyrDown( vis, tmp1 );
    cv::pyrDown( tmp1, tmp2 );
    cv::imshow("MSER", tmp2 );
    cv::waitKey();
    cv::imwrite("TMP.JPG", tmp2);

}

