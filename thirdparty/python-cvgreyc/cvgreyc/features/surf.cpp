#include "surflib.h"
#include "ipoint.h"
#include "NDArray.h"

extern "C" {
	void      extract_surf_samples(const char* filename, int npoints, double* points, bool upright,  double* descriptors);
	void      extract_surf_dense  (const char* filename, double scale, int xmin, int xmax, int ymin, int ymax, bool upright, double* descriptors);
	double*   extract_surf        (const char* filename, bool upright, int octaves, int intervals, int init_samples, double thres, int& nkeypoints);
}

using namespace CVGreyc ;

void extract_surf_samples(const char* filename, int npoints, double* points, bool upright, double* descriptors){
	NDArray::CArray<double> _points(points,npoints,4);
	NDArray::CArray<double> _descriptors(descriptors,npoints,64);
	IpVec ipts;
	IplImage *img=cvLoadImage(filename);
	Ipoint ipoint;

	for (int i =0; i<npoints; i++){
		ipoint.x = _points(i,0);
		ipoint.y = _points(i,1);
		ipoint.orientation = _points(i,2);
		ipoint.scale = _points(i,3);
		ipts.push_back(ipoint);
	}	
	surfDes(img,ipts,upright);
	for (int i =0; i<npoints; i++) {
		for (int j = 0; j<64 ; j++) _descriptors(i,j) = ipts[i].descriptor[j];
		_points(i,0) = ipts[i].x;
		_points(i,1) = ipts[i].y;
		_points(i,2) = ipts[i].orientation;
		_points(i,3) = ipts[i].scale;
	}
	cvReleaseImage(&img);
}

void extract_surf_dense(const char* filename, double scale, int xmin, int xmax, int ymin, int ymax, bool upright, double* descriptors) {
	int width  = xmax-xmin + 1;
	int height = ymax-ymin + 1;
	
	NDArray::CArray<double> _descriptors(descriptors,width,height,64);
	IpVec ipts;
	IplImage *img=cvLoadImage(filename);
	Ipoint ipoint;

	for (int i =xmin; i<=xmax; i++){
		for (int j=ymin; j<=ymax; j++){
			ipoint.x = i;
			ipoint.y = j;
			ipoint.orientation = 0.;
			ipoint.scale = scale;
			ipts.push_back(ipoint);
		}
	}	
	surfDes(img,ipts,upright);
	for (int i =0; i<width; i++) {
		for (int j=0; j<height; j++){
			for (int k = 0; k<64 ; k++) _descriptors(i,j,k) = ipts[i].descriptor[j];
		}
	}
	cvReleaseImage(&img);
}

double* extract_surf(const char* filename, bool upright, int octaves, int intervals, int init_samples, double thres, int& nkeypoints){
	IpVec ipts;
	IplImage *img=cvLoadImage(filename);

	surfDetDes(img, ipts, upright, octaves, intervals, init_samples, thres);
	nkeypoints = ipts.size();
	double *result = static_cast<double*>(malloc(sizeof(double) * (nkeypoints*(4+64)) ));
	double *keypoints = &result[0];
	double *descriptors = &result[4*nkeypoints];
	
	for (int i =0; i<(int)ipts.size(); i++) {
		keypoints[i*4]   = ipts[i].x;
		keypoints[i*4+1] = ipts[i].y;
		keypoints[i*4+2] = ipts[i].orientation;
		keypoints[i*4+3] = ipts[i].scale;
		
		for (int j=0;j<64;j++){
			descriptors[i*64+j] = ipts[i].descriptor[j];
		}
	}
	cvReleaseImage(&img);
	return result;
}
