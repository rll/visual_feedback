/* 
 * File:   LandmarkDetector.h
 * Author: ted
 *
 * Created on February 17, 2011, 3:12 PM
 */

#ifndef _LANDMARKDETECTOR_H
#define	_LANDMARKDETECTOR_H

#include "imageProcessing.h"
#include "LibsvmWrapper.h"
#include <cv.h>
#include <highgui.h>
#include <assert.h>
#include <vector>
using namespace std;
using namespace cv;

enum {OTHER, HEEL, TOE, OPENING};

class LandmarkDetector {
public:
    LandmarkDetector(int s_length = 400, int s_step = 100);
    virtual ~LandmarkDetector();

    void loadNewImage(IplImage * image);
    void loadModelFile(char * name);
    void computeResponseStartingAtPt (CvPoint pt = cvPoint(0,0));
    void getResponseCloseToPt (CvPoint pt, vector<double>& response);
    IplImage* getResponseVisualization();

    // Debugging purposes
    IplImage * getMask();
    vector <IplImage*> * getSegmentImages();
    vector <IplImage*> * getOrientedSegmentImages();

private:
    LibsvmWrapper* _libsvm;
    IplImage * _image;
    IplImage * _hollow;
    bool _set_image;
    bool _set_hollow;
    int _sLength, _sStep;
    vector<CvPoint> *_contour;
    vector<CvPoint> *_centers;
    vector<double> *_angles;
    vector<IplImage*> *_orientedSegments;
    vector<IplImage*> *_segments;

    void getLandmarkFeatures(IplImage *img, vector<double> * features);
    double computeCurvature(IplImage *img, int scale);
    double computeNormalAngle(IplImage *mask, CvPoint point);
    int findClosestIndex (CvPoint pt_a, vector<CvPoint> * contour);
    void freeSegmentInfo();
    vector <IplImage*> * getSegmentImages(IplImage* image, vector<CvPoint> * centers);
    vector <IplImage*> * getOrientedSegmentImages (IplImage *image, vector <IplImage*> * segments, vector <double> *angles);
    vector<double> * getSegmentAngles(IplImage *image, vector<CvPoint> * centers);
    vector<CvPoint> * getSegmentCenters(IplImage *image, vector<CvPoint> * contour, int offset=0);
    vector<CvPoint> * getSockContourSeq(IplImage *image);
};

#endif	/* _LANDMARKDETECTOR_H */

