/* 
 * File:   LandmarkDetector.cpp
 * Author: ted
 * 
 * Created on February 17, 2011, 3:12 PM
 */

#include "LandmarkDetector.h"

LandmarkDetector::LandmarkDetector(int s_length, int s_step){
    assert(s_length > 0);
    assert(s_step > 0);
    _sLength = s_length;
    _sStep = s_step;

    _contour = new vector<CvPoint>;
    _centers = new vector<CvPoint>;
    _angles = new vector<double>;
    _segments = new vector<IplImage*>;
    _orientedSegments = new vector<IplImage*>;
    _libsvm = new LibsvmWrapper();
}

LandmarkDetector::~LandmarkDetector() {
    if(_image != NULL) {
        cvReleaseImage(&_image);
        cvReleaseImage(&_hollow);
    }
    delete _contour;
    delete _libsvm;
    freeSegmentInfo();
}

void LandmarkDetector::freeSegmentInfo() {
    for (int i=0; i < _segments->size(); i++) {
        cvReleaseImage(&(_segments->at(i)));
        cvReleaseImage(&(_orientedSegments->at(i)));
    }
    delete _centers;
    delete _angles;
    delete _segments;
    delete _orientedSegments;
}

void LandmarkDetector::loadNewImage(IplImage * image) {
    assert(image != NULL);
    assert(image->nChannels == 3);

    cvReleaseImage(&_image);
    cvReleaseImage(&_hollow);
    delete _contour;
    
    _image = cvCloneImage(image);
    _hollow = createHollowImage(_image);
    _contour = getSockContourSeq(_hollow);
}

void LandmarkDetector::loadModelFile(char * fileName) {
    delete _libsvm;
    _libsvm = new LibsvmWrapper();
    _libsvm->loadModel(fileName);
    assert(_libsvm->svmModel != NULL);
}

void LandmarkDetector::computeResponseStartingAtPt (CvPoint pt) {
    assert(_image != NULL);
    freeSegmentInfo();
    _centers = getSegmentCenters(_hollow,_contour,findClosestIndex(pt,_contour));
    _angles = getSegmentAngles(_hollow,_centers);
    _segments = getSegmentImages(_hollow,_centers);
    _orientedSegments = getOrientedSegmentImages(_image,_segments,_angles);
}

void LandmarkDetector::getResponseCloseToPt (CvPoint pt, vector<double>& response) {
    assert(_centers->size() > 0);
    assert(_libsvm->svmModel != NULL);
    response.clear();

    int closestSegment = findClosestIndex(pt, _centers);
    vector<double> *features = new vector<double>;
    getLandmarkFeatures(_orientedSegments->at(closestSegment),features);
    
    int nClasses = _libsvm->get_nr_class();
    int *labels = (int *) malloc(nClasses*sizeof(int));
    double* prob = (double *) malloc(nClasses*sizeof(double));
    int label = (int) _libsvm->predict_probability(*features,prob);

    _libsvm->get_labels(labels);
    for (int i=0; i < nClasses; i++) {
        response.push_back(0.0);
    }
    for (int i=0; i < nClasses; i++) {
        response[labels[i]] = prob[i];
    }

    delete features;
    free(prob);
    free(labels);
}

IplImage* LandmarkDetector::getResponseVisualization() {
    int offset = 32;
    int r = 16;
    
    IplImage *visual = cvCloneImage(_image);
    vector<double> response;
    for(int i=0; i < _centers->size(); i++) {
        CvPoint pt = _centers->at(i);
        getResponseCloseToPt(pt,response);
        double normalAngle = computeNormalAngle(_hollow,pt);
        cvCircle(visual,cvPoint(pt.x-1.5*offset*cos(normalAngle),pt.y+1.5*offset*sin(normalAngle))
                ,r,cvScalar(response[OPENING]*255),8);
        cvCircle(visual,cvPoint(pt.x-0.5*offset*cos(normalAngle),pt.y+0.5*offset*sin(normalAngle))
                ,r,cvScalar(response[TOE]*255),8);
        cvCircle(visual,cvPoint(pt.x+0.5*offset*cos(normalAngle),pt.y-0.5*offset*sin(normalAngle))
                ,r,cvScalar(response[HEEL]*255),8);
        cvCircle(visual,cvPoint(pt.x+1.5*offset*cos(normalAngle),pt.y-1.5*offset*sin(normalAngle))
                ,r,cvScalar(response[OTHER]*255),8);
        //cout << response[OTHER] << " " << response[HEEL] << " " << response[TOE] << " " << response[OPENING] << endl;
    }
    return visual;
}

IplImage * LandmarkDetector::getMask() {
    return getImageMask(_image);
}

vector <IplImage*> * LandmarkDetector::getSegmentImages() {
    return _segments;
}

vector <IplImage*> * LandmarkDetector::getOrientedSegmentImages() {
    return _orientedSegments;
}

//Return the sock contour sequence in the image
vector<CvPoint> * LandmarkDetector::getSockContourSeq(IplImage *image) {
    IplImage* img = cvCloneImage(image);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq *contour = NULL;
    cvFindContours(img, storage, &contour, sizeof(CvContour),
                    CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));

    // Finds the largest contour in the image
    CvSeq *largest;
    float largest_area = 0;
    for(;contour;contour = contour->h_next){
        double contourArea= fabs(cvContourArea(contour));
        //Only store the largest contour
        if (contourArea > largest_area) {
            largest = contour;
            largest_area = contourArea;
        }
    }

    vector<CvPoint> * contourPoints = new vector<CvPoint>;
    for(int i=0; i<largest->total; i++) {
        contourPoints->push_back(*((CvPoint*)cvGetSeqElem(largest,i)));
    }

    cvReleaseMemStorage(&storage);
    cvReleaseImage(&img);

    return contourPoints;
}

//Return a list of segment centers, with offset determining the starting point of first center
vector<CvPoint> * LandmarkDetector::getSegmentCenters(IplImage *image, vector<CvPoint> * contour, int offset) {
    vector <CvPoint> * segmentCenters = new vector<CvPoint>;
    //Create split mask
    for (int s=0; s< contour->size()/_sStep; s++) {
        int center = _sStep*s;
        CvPoint pt = contour->at((center+offset)%contour->size());
        segmentCenters->push_back(pt);
    }
    return segmentCenters;
}

//Return the normal to the contour at the center of each segment
vector<double> * LandmarkDetector::getSegmentAngles(IplImage *image, vector<CvPoint> * centers) {
    vector <double> * segmentAngles = new vector<double>;

    //Create split mask
    for (int s=0; s< centers->size(); s++) {
        CvPoint pt = centers->at(s);
        double angle = computeNormalAngle(image,pt)/PI*180;
        segmentAngles->push_back(angle);
    }
    return segmentAngles;
}

//Image given must be a mask.
//Computes and returns the points that are closer to each segment
vector <IplImage*> * LandmarkDetector::getSegmentImages(IplImage* img, vector<CvPoint> * centers) {
    vector <IplImage*> * result = new vector<IplImage*>;
    for(int i=0; i<centers->size(); i++) {
        result->push_back(cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1));
        cvZero(result->at(i));
    }

    //Create split mask
    int h = cvGetSize(img).height, w = cvGetSize(img).width;
    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            if(cvGetReal2D(img,y,x) == 0) {
                continue;
            }
            else{
                vector<double> segmentDistances;
                for(int s=0; s<centers->size(); s++) {
                    CvPoint pt = centers->at(s);
                    double distance = sqrt((pt.x-x)*(pt.x-x) + (pt.y-y)*(pt.y-y));
                    segmentDistances.push_back(distance);
                    //cout << "segment #" << s << " " << segmentDistances.at(s) << endl;
                }

                // Choose the segments that is closests to the point
                for (int j=0; j< _sLength/_sStep; j++) {
                    std::vector<double>::iterator pos = std::min_element(segmentDistances.begin(),segmentDistances.end());
                    int index = pos - segmentDistances.begin();
                    cvSetReal2D(result->at(index),y,x,255);
                    segmentDistances[index] = w+h;
                    //cout << "segment #" << index << " is chosen" << segmentDistances.at(index) << endl;
                }
            }
        }
    }
    return result;
}

vector <IplImage*> * LandmarkDetector::getOrientedSegmentImages (IplImage *image, vector <IplImage*> * segments, vector <double> *angles) {
    vector <IplImage*> * orientedSegments = new vector<IplImage*>;
    IplImage *gray_img = cvtGrayImage(image);

    for(int i=0; i < segments->size(); i++) {
        cvAnd(segments->at(i),gray_img,segments->at(i));
        //Rotate image to have the norm of the center of the contour pointing down
        CvRect boundToRotate = findSockBoundingRect(segments->at(i));
        CvPoint rotateCenter = cvPoint(boundToRotate.x+boundToRotate.width/2,
                                       boundToRotate.y+boundToRotate.height/2);
        cv::Mat rMat = Mat(segments->at(i));
        cv::Mat rMimg = rotateImage(rMat, rotateCenter, 90-angles->at(i));
        IplImage rimg = rMimg;
        IplImage *r_img = &rimg;

        //Crop and Copy image
        CvRect bound = findSockBoundingRect(r_img,20);
        cvSetImageROI(r_img,bound);
        orientedSegments->push_back(cloneROI(r_img));
        rMat.release();
        rMimg.release();
    }

    cvReleaseImage(&gray_img);
    return orientedSegments;
}

int LandmarkDetector::findClosestIndex (CvPoint pt_a, vector<CvPoint> * point_list) {
    vector<double> distances;
    for (int i=0; i < point_list->size(); i++) {
        CvPoint pt_b = point_list->at(i);
        double distance = sqrt((pt_a.x-pt_b.x)*(pt_a.x-pt_b.x) + (pt_a.y-pt_b.y)*(pt_a.y-pt_b.y));
        distances.push_back(distance);
    }
    std::vector<double>::iterator pos = std::min_element(distances.begin(),distances.end());
    return pos - distances.begin();
}

void LandmarkDetector::getLandmarkFeatures(IplImage *img, vector<double> *features) {
    extractLBPFeature(img,features);
}

double LandmarkDetector::computeCurvature(IplImage *img, int scale) {
    
}

//Given an image and its grasping point, we want to find the gradient to
//  get the approach angle
double LandmarkDetector::computeNormalAngle(IplImage *mask, CvPoint point) {
    IplImage *img = cvCloneImage(mask);
    cvSmooth(img,img,CV_GAUSSIAN,11);
    IplImage *dx = cvCreateImage(cvGetSize(mask),IPL_DEPTH_16S,1);
    IplImage *dy = cvCreateImage(cvGetSize(mask),IPL_DEPTH_16S,1);

    //Calculate derivatives
    cvSobel(img,dx,1,0,CV_SCHARR);
    cvSobel(img,dy,0,1,CV_SCHARR);
    //negate dy because of the way these filters are structured
    //-3 -10 -3
    // 0   0  0
    // 3  10  3
    //positive y is downward
    cvScale(dy,dy,-1);

    double dx_at_pt = cvGetReal2D(dx,point.y,point.x);
    double dy_at_pt = cvGetReal2D(dy,point.y,point.x);

    cvReleaseImage(&dx);
    cvReleaseImage(&dy);
    cvReleaseImage(&img);

    return atan2(dy_at_pt,dx_at_pt);
}