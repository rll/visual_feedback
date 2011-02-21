/*
 * File:   imageProcessing.h
 * Author: ted
 *
 * Created on August 11, 2010, 10:47 AM
 */

#ifndef _IMAGEPROCESSING_H
#define	_IMAGEPROCESSING_H

#include "LBP.h"
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
using namespace std;
using namespace cv;

//Constants and functions for creating background mask
const int tol = 15;
const int bg_hue = 75;
const int bg_sat = 80;
const int smooth_size = 15;
const int morph_size = 15;
IplImage* getImageMask(IplImage* img);

//Constants and function for predicting grasp point
CvRect findSockBoundingRect (IplImage *image, int tol=10);
RotatedRect findSockTightBoundingRect (IplImage *image);
Mat rotateImage(const Mat& source, Point2f src_center, double angle);
IplImage* processRotatedImage(IplImage *r_img, IplImage *r_mask);

//Function to extract LBP Features
const int numLBPFeature = 256;
void extractLBPFeature(IplImage* image, vector<double> * features);
void extractLBPFeature(IplImage* img, double* features, CvMat* mask, IplImage* lbp_image = NULL);

//Function to extract HOG Features
#define PI 3.14159265
const int warpWidth = 1024, warpHeight = 512;
const int scalePoint = 16;
const int hollow_size = 150;
const int HOGScale = 1;
const int HOGSmooth = 49;
int computeHOGDesc(IplImage *img, CvRect bound, double** features, int x = -1, int y = -1);

//Misc Functions
IplImage* createHollowImage(IplImage *image, int size=hollow_size);
IplImage* cloneROI(IplImage *img);
IplImage* cvtGrayImage(IplImage *img);
void displayImage(IplImage *img);

#endif	/* _IMAGEPROCESSING_H */
