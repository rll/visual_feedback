#include <iostream>
#include <ros/ros.h>
#include <socks/LandmarkDetection/LandmarkDetector.h>
#include <appearance_utils/LoadImage.h>
#include <appearance_utils/LandmarkResponse.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

LandmarkDetector* landmarkDetector_;
sensor_msgs::CvBridge bridge_;

bool load_image_srv         (   appearance_utils::LoadImage::Request    &req,
                                appearance_utils::LoadImage::Response   &res )
{
    cout << "Called load_image service" << endl;
    sensor_msgs::Image image = req.image;
    sensor_msgs::ImagePtr img_ptr(new sensor_msgs::Image(image));
    
    IplImage *cv_image = NULL;    
    try
        {
                cv_image = bridge_.imgMsgToCv(img_ptr, "bgr8");
        }
        catch (sensor_msgs::CvBridgeException error)
        {
                ROS_ERROR("Error bridging to openCV format");
                return false;
        }
    cout << "Converted to cv_image" << endl;
    landmarkDetector_->loadNewImage(cv_image);
    cout << "Loaded image" << endl;
    landmarkDetector_->computeResponseStartingAtPt(cvPoint(0,0),true);
    cout << "Computed response" << endl;
    return true;
}

bool landmark_response_srv  (   appearance_utils::LandmarkResponse::Request    &req,
                                appearance_utils::LandmarkResponse::Response   &res )
{
    vector<double> response;
    landmarkDetector_->getResponseCloseToPt(cvPoint(req.x,req.y), response);
    if(req.mode == req.HEEL){
        res.response = response[HEEL];
    }
    else if(req.mode == req.TOE){
        res.response = response[TOE];
    }
    else if(req.mode == req.OPENING){
        res.response = response[OPENING];
    }
    else if(req.mode == req.OTHER){
        res.response = response[OTHER];
    }
    else{
        cout << "Given invalid filter type" << endl;
        return false;
    }
    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "landmark_response_server");
    ros::NodeHandle n("~");
    landmarkDetector_ = new LandmarkDetector();

    //Initialize by loading the model
    string model_file;
    n.param<string>("model_file", model_file, "model.txt");
    landmarkDetector_->loadModelFile(const_cast<char*>(model_file.c_str()));

    //Start the services
    ros::ServiceServer image_service = n.advertiseService("load_image", load_image_srv);
    ROS_INFO("load_image service ready.");
    ros::ServiceServer landmark_service = n.advertiseService("landmark_response", landmark_response_srv);
    ROS_INFO("landmark_response service ready.");
    ros::spin();
    return 0; 
}
