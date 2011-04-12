#include <vector>
#include "cv.h"
using namespace std;
using namespace cv ;

#include <iostream>
extern "C" {
void extract_surf_samples(int height, int width, unsigned char* image, int npoints, float* points, float* descriptors, 
						double hessian_threshold , int nOctaves , int octaveLayers, bool extended){

	Mat_<unsigned char> _image(height,width,image);

					
	Mat_<float> _points(npoints,4,points);

	Mat_<bool>  mask(height,width,true);

	int dsize = extended ? 128 : 64;
	vector<float> _descriptors;
	vector<KeyPoint> ipts;
	KeyPoint ipoint;

	for (int i =0; i<npoints; i++){
		ipoint.pt.x = _points[i][0];
		ipoint.pt.y = _points[i][1];
		ipoint.size = _points[i][2];
		ipoint.angle = _points[i][3];
		ipts.push_back(ipoint);
	}
	SURF surf(hessian_threshold,nOctaves,octaveLayers,extended);
	surf(_image,mask,ipts,_descriptors,true);
	
	for (int i =0; i<npoints; i++) {
		for (int j = 0; j<dsize ; j++) descriptors[i*dsize+j] = _descriptors[i*dsize+j];
		_points[i][0] = ipts[i].pt.x;
		_points[i][1] = ipts[i].pt.y;
		_points[i][2] = ipts[i].size;
		_points[i][3] = ipts[i].angle;
	}
}

void extract_surf_grid(int height, int width, unsigned char* image, double size,
					  int xmin, int xmax, int ymin, int ymax,
                      int nstepsx, int nstepsy, float* descriptors,
                      double hessian_threshold , int nOctaves , int octaveLayers, bool extended) {

	Mat_<unsigned char> _image(height,width,image);
	Mat_<bool> mask(height,width,true);

	double dx = (xmax-xmin)/(nstepsx-1);
	double dy = (ymax-ymin)/(nstepsy-1);

	int dsize = extended ? 128 : 64;
	vector<KeyPoint> ipts;
	vector<float> _descriptors;
	KeyPoint ipoint;
	
	ipoint.size = size ;
	ipoint.angle = 0. ;
	
	for (int i = 0 ; i < nstepsx; i++){
		ipoint.pt.x = xmin + i*dx;
		for (int j = 0 ; j< nstepsy; j++){
			ipoint.pt.y = ymin + j*dy;
			ipts.push_back(ipoint);
		}
	}

	SURF surf(hessian_threshold,nOctaves,octaveLayers,extended);
	surf(_image,mask , ipts,_descriptors,true);

	int n;
	for (int i =0; i<nstepsx; i++) {
		for (int j=0; j<nstepsy; j++){
			for (int k = 0; k<dsize ; k++){
				n = (i*nstepsy + j)*dsize + k;
				descriptors[n] = _descriptors[n];
			}
		}
	}
}

double* extract_surf(int height, int width, unsigned char * image ,
                     int nOctaves, int octaveLayers, double thres, bool extended,
					 int& nkeypoints){

	Mat_<unsigned char> _image(height,width,image);
	Mat_<bool> mask(height,width,true);

	int dsize = extended ? 128 : 64 ;
	int rsize = dsize+4;
	
	vector<KeyPoint> ipts;
	vector<float> _descriptors;

	SURF surf(thres,nOctaves,octaveLayers,extended);
	surf(_image,mask,ipts,_descriptors,false);
	

	nkeypoints = ipts.size();
	double *result = static_cast<double*>(malloc(sizeof(double) * (nkeypoints*rsize) ));
	for (int i =0; i<(int)ipts.size(); i++) {
		result[i*rsize]   = ipts[i].pt.x;
		result[i*rsize+1] = ipts[i].pt.y;
		result[i*rsize+2] = ipts[i].size;
		result[i*rsize+3] = ipts[i].angle;
		
		for (int j=0;j<dsize;j++){
			result[i*rsize+4+j] = _descriptors[i*dsize+j];
		}
	}
	return result;
}

void free_res(double* res){
	free(res);
}

} // extern "C"

//~ #include <iostream>
//~ #include "highgui.h"
//~ 
//~ int main (int argc, char * argv[]) {
	//~ 
	//~ cout<<"loading image..."<<endl;
	//~ Ptr<IplImage> img = cvLoadImage(argv[1]);
	//~ Mat image(img);
	//~ cout<<"image_loaded"<<endl;
	//~ Mat_<unsigned char> gim;
	//~ cvtColor(image,gim,CV_BGR2GRAY);
	//~ cout<<"ready to extract"<<endl;
	//~ float descr[20*20*128];
	//~ extract_surf_grid(gim.rows, gim.cols, gim.data, 5.,
	                  //~ 10,10,200,200,20,20,descr,
					  //~ 4, 2, 100, true);
//~ }
