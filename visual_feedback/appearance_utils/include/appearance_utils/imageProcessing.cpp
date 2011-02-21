#include "imageProcessing.h"

//Given an image, create a mask image and return it
IplImage* getImageMask(IplImage* image) {
    if (image->nChannels == 1) {
        IplImage *mask = cvCloneImage(image);
        cvThreshold(mask,mask,1,255,CV_THRESH_BINARY);
        return mask;
    }

    IplImage *img = cvCloneImage(image);
    IplImage *hue = cvCreateImage(cvGetSize(img),img->depth,1);
    IplImage *sat = cvCreateImage(cvGetSize(img),img->depth,1);
    //Three Temporary matrices to compute threshold
    CvMat *low_hue = cvCreateMat(cvGetSize(img).height,cvGetSize(img).width,CV_8UC1);
    CvMat *high_hue = cvCreateMat(cvGetSize(img).height,cvGetSize(img).width,CV_8UC1);
    CvMat *combined_hue = cvCreateMat(cvGetSize(img).height,cvGetSize(img).width,CV_8UC1);
    CvMat *low_sat = cvCreateMat(cvGetSize(img).height,cvGetSize(img).width,CV_8UC1);
    CvMat *combined = cvCreateMat(cvGetSize(img).height,cvGetSize(img).width,CV_8UC1);
    //Final masking result
    IplImage *mask = cvCreateImage(cvGetSize(img),img->depth,1);

    //Convert to HSV and get the hue channel
    cvCvtColor(img,img,CV_BGR2HSV);
    cvSplit(img,hue,sat,NULL,NULL);

    //Threshold to get rid of green with some tolerance
    cvThreshold(hue,low_hue,bg_hue-tol,1,CV_THRESH_BINARY_INV);
    cvThreshold(hue,high_hue,bg_hue+tol,1,CV_THRESH_BINARY);
    cvOr(low_hue,high_hue,combined_hue);
    cvThreshold(sat,low_sat,bg_sat-2*tol,1,CV_THRESH_BINARY_INV);

    cvNot(combined_hue,combined_hue);
    cvNot(low_sat,low_sat);
    cvAnd(combined_hue,low_sat,combined);
    cvNot(combined,combined);

    cvConvertScale(combined,mask,255,0);
    //Smooth result and close holes
    int i=morph_size,j=morph_size/2+1;
    IplConvKernel *kernel = cvCreateStructuringElementEx(i,i,j,j,CV_SHAPE_RECT);
    cvDilate(mask,mask,kernel,1);
    cvErode(mask,mask,kernel,1);
    cvSmooth(mask,mask,CV_MEDIAN,smooth_size*11);

    cvReleaseStructuringElement(&kernel);
    cvReleaseMat(&low_hue);
    cvReleaseMat(&high_hue);
    cvReleaseMat(&combined_hue);
    cvReleaseMat(&low_sat);
    cvReleaseMat(&combined);
    cvReleaseImage(&hue);
    cvReleaseImage(&sat);
    cvReleaseImage(&img);
    return mask;
}

// Finds the sock's tight bounding rectangle
RotatedRect findSockTightBoundingRect (IplImage *image) {
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

    // Create Mat type
    CvPoint pointArray[largest->total];
    cvCvtSeqToArray(largest, pointArray);
    vector<Point> ptVect(largest->total);

    for(int i=0; i<largest->total; i++)
        ptVect[i] = Point(pointArray[i].x,pointArray[i].y);

    // Find and return the bounding rect with some modification
    RotatedRect boundingRect = cv::minAreaRect(Mat(ptVect));

    cvReleaseMemStorage(&storage);
    cvReleaseImage(&img);
    return boundingRect;
}

Mat rotateImage(const Mat& source, Point2f src_center, double angle)
{
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}

IplImage* processRotatedImage(IplImage *r_img, IplImage *r_mask) {
    IplImage *processed_image = cvCloneImage(r_img);
    cvCvtColor(r_mask,processed_image,CV_GRAY2BGR);

    //Merge black and green background
    cvScale(processed_image,processed_image,1/255.);
    cvMul(processed_image,r_img,processed_image);

    //Add back the green background
    IplImage *hue = cvCreateImage(cvGetSize(r_img),r_img->depth,1);
    IplImage *sat = cvCreateImage(cvGetSize(r_img),r_img->depth,1);
    IplImage *val = cvCreateImage(cvGetSize(r_img),r_img->depth,1);
    //Convert to HSV and get the hue channel
    cvCvtColor(processed_image,processed_image,CV_BGR2HSV);
    cvSplit(processed_image,hue,sat,val,NULL);
    cvNot(r_mask,r_mask);
    cvScale(r_mask,r_mask,1/255.*bg_hue);
    cvAdd(hue,r_mask,hue);
    cvAdd(sat,r_mask,sat);
    cvAdd(val,r_mask,val);
    cvMerge(hue,sat,val,NULL,processed_image);

    cvCvtColor(processed_image,processed_image,CV_HSV2BGR);

    return processed_image;
}


// Finds the sock's bounding rectangle by searching for the largest
//      contour and using cvContourArea
CvRect findSockBoundingRect (IplImage *image, int tol) {
    IplImage* img = (IplImage*)cvClone(image);
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

    // Find and return the bounding rect with some modification
    CvRect boundingRect= cvBoundingRect(largest,0);
    int x= boundingRect.x;
    int y= boundingRect.y;
    int width= boundingRect.width;
    int height= boundingRect.height;
    // increase bounding box size by 1 pixel

    CvRect bound = cvRect(x-2*tol,y-tol,width+4*tol,height+2*tol);

    cvReleaseMemStorage(&storage);
    cvReleaseImage(&img);
    return bound;
}

void extractLBPFeature(IplImage* image, vector<double> * features) {
    features->clear();

    IplImage *img = cvtGrayImage(image);
    // Convert Image to 4-byte integer-valued image.
    int height = img->height;
    int width = img->width;
    IplImage* scale_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_32S,1);
    cvConvertScale(img,scale_img,1.0,0.0);

    // Compute the feature array by calling LBP functinos. (Uses circular sampl)
    IplImage *mask = getImageMask(img);
    CvMat tmp1;
    calculate_points();
    int results[numLBPFeature];
    lbp_histogram((int*)(scale_img->imageData),height,width,results,1, cvGetMat(mask, &tmp1, 0, 0));

    //L_1 normalization
    CvMat tmp = cvMat(1,numLBPFeature,CV_32SC1,results);
    double norm = cvNorm(&tmp,NULL,CV_L1,NULL);

    for(int i=0; i<numLBPFeature; i++) {
        if(norm != 0)
            features->push_back(results[i]*1.0/norm);
        else
            features->push_back(0.);
    }

    cvReleaseImage(&mask);
    cvReleaseImage(&scale_img);
    cvReleaseImage(&img);
}

/*
 * Extracts the LBP Features from a given image and return the feature array.
 */
void extractLBPFeature(IplImage* image, double* features, CvMat* mask, IplImage* lbp_image){
    IplImage *img = cvtGrayImage(image);

    // Convert Image to 4-byte integer-valued image.
    int height = img->height;
    int width = img->width;
    IplImage* scale_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_32S,1);
    cvConvertScale(img,scale_img,1.0,0.0);

    // Compute the feature array by calling LBP functinos. (Uses circular sampl)
    calculate_points();
    int results[numLBPFeature];
    lbp_histogram((int*)(scale_img->imageData),height,width,results,1,mask,lbp_image);

    //L_1 normalization
    CvMat tmp = cvMat(1,numLBPFeature,CV_32SC1,results);
    double norm = cvNorm(&tmp,NULL,CV_L1,NULL);

    for(int i=0; i<numLBPFeature; i++)
        if(norm != 0)
            features[i] = results[i]*1.0/norm;
        else
            features[i] = 0;

    cvReleaseImage(&scale_img);
    cvReleaseImage(&img);
}

//Given image and a bound, compute HOG Descriptor
int computeHOGDesc(IplImage *img, CvRect bound, double** features, int x, int y) {
    cvSetImageROI(img, bound);

    IplImage* crop= cvCloneImage(img);
    IplImage* warpedImage= cvCreateImage(cvSize(warpWidth, warpHeight), img->depth, img->nChannels); //for normalization purposes send all objects to same size
    cvResize(crop, warpedImage, CV_INTER_CUBIC); // warping the image by resizing it

    cvSmooth(warpedImage,warpedImage,CV_MEDIAN,HOGSmooth);

    HOGDescriptor hogDescriptor(cvSize(warpWidth,warpHeight), cvSize(16*scalePoint,16*scalePoint),
            cvSize(16*scalePoint,16*scalePoint), cvSize(8*scalePoint,8*scalePoint), 9);
    const Mat hogImage= cv::cvarrToMat(warpedImage);
    vector<float> tmpFeatures;
    hogDescriptor.compute(hogImage, tmpFeatures);

    //Display Image
    if(x != -1 && y != -1) {
        cvCircle(warpedImage,cvPoint(x,y),32,CV_RGB(255,0,0),8);
        displayImage(warpedImage);
    }

    int nFeatures = tmpFeatures.size();
    *features = (double*) realloc(*features,nFeatures*sizeof(double));
    for (int i= 0; i<nFeatures; i++) {
        (*features)[i]= (double)tmpFeatures[i]*HOGScale;
    }

    cvResetImageROI(img);
    cvReleaseImage(&crop);
    cvReleaseImage(&warpedImage);

    return nFeatures;
}

//Creates a mask that only has the seam of the image
IplImage* createHollowImage(IplImage *image, int size) {
    IplImage *mask = getImageMask(image);
    IplImage *seam = cvCloneImage(mask);

    //Get smaller image
    int i=size,j=size/2+1;
    IplConvKernel *kernel = cvCreateStructuringElementEx(i,i,j,j,CV_SHAPE_RECT);
    cvErode(mask,mask,kernel,1);

    //NOT and AND to create seam
    cvNot(mask,mask);
    cvAnd(mask,seam,seam);

    cvReleaseImage(&mask);
    cvReleaseStructuringElement(&kernel);
    return seam;
}

IplImage* cloneROI(IplImage *given_image) {
    IplImage *image = cvCreateImage(cvGetSize(given_image),given_image->depth, given_image->nChannels);
    cvCopy(given_image,image);
    return image;
}

/*
 * Return a new image that is the gray-scale conversion of the input image.
 */
IplImage* cvtGrayImage(IplImage *img){
    if (img->nChannels > 1) {
        IplImage *gray_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        cvCvtColor(img,gray_img,CV_BGR2GRAY);
        return gray_img;
    }
    return cvCloneImage(img);
}

// Display the given image in a window
void displayImage(IplImage *image) {
    cvNamedWindow("image", CV_WINDOW_AUTOSIZE);
    cvShowImage("image",image);
    cvWaitKey(0);
    cvDestroyWindow("image");
}