#include <iostream>
#include <ros/ros.h>
#include <socks/LandmarkDetection/LandmarkDetector.h>
#include <appearance_utils/LoadImage.h>
#include <appearance_utils/LandmarkResponse.h>
#include <appearance_utils/LandmarkResponseAll.h>
#include <appearance_utils/SaveVisualization.h>
#include <appearance_utils/ExtractLBPFeatures.h>
#include <appearance_utils/GetMask.h>
#include <appearance_utils/PatchResponse.h>
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
    if (req.mode == req.CONTOUR){
        landmarkDetector_->computeResponseStartingAtPt(cvPoint(0,0),true,false);
    cout << "Computed response (contour only)" << endl;
    }else{
        landmarkDetector_->computeResponseStartingAtPt(cvPoint(0,0),false,true);
    cout << "Computed response (inside only)" << endl;
    }
}

bool extract_lbp_features_srv (   appearance_utils::ExtractLBPFeatures::Request     &req,
                                  appearance_utils::ExtractLBPFeatures::Response    &res )
{
    cout << "Called extract_lbp_features service" << endl;
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
    landmarkDetector_->getLandmarkFeatures(cv_image, &res.features,req.type);

    return true;
}

bool landmark_response_srv  (   appearance_utils::LandmarkResponse::Request    &req,
                                appearance_utils::LandmarkResponse::Response   &res )
{
    vector<double> response;
    landmarkDetector_->getResponseCloseToPt(cvPoint(req.x,req.y), response,req.theta,req.use_nearest);
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

/* Pulls all feature responses for all patches in the sock
 * Input: None (maybe a bool to include inside?)
 * Output: Two vectors: one of feature centers, and another of 
 * */
bool landmark_response_all_srv (    appearance_utils::LandmarkResponseAll::Request      &req,
                                    appearance_utils::LandmarkResponseAll::Response     &res )
{
    
    cout << "Called landmark_response_all service" << endl;
    vector <CvPoint>* centers =      landmarkDetector_->getCenters();
    vector <IplImage*>* patches =    landmarkDetector_->getOrientedSegmentImages();
    for (int i=0; i < centers->size(); i++){
        appearance_utils::PatchResponse pr;
        CvPoint center = centers->at(i);
        pr.x = center.x;
        pr.y = center.y;
        landmarkDetector_->getResponse (patches->at(i), pr.responses,req.type);
        
        //Hard Coded now, find principled approach later
        pr.response_types.push_back(pr.OTHER);
        pr.response_types.push_back(pr.HEEL);
        pr.response_types.push_back(pr.TOE);
        pr.response_types.push_back(pr.OPENING);
        res.patch_responses.push_back(pr);
    }
    cout << "Computed features" << endl;
    return true;    
}


//Assumes LoadImage has been called
bool get_visualization_srv     (   appearance_utils::SaveVisualization::Request    &req,
                                    appearance_utils::SaveVisualization::Response   &res )
{
    cout << "Called get_visualization service" << endl;
    IplImage *cv_image = landmarkDetector_->getSeparateResponseVisualization();
    sensor_msgs::ImagePtr img_ptr;
    sensor_msgs::Image img;
    try
    {
            img_ptr = bridge_.cvToImgMsg(cv_image, "bgr8");
            img = *img_ptr;
    }
    catch (sensor_msgs::CvBridgeException error)
    {
            ROS_ERROR("error");
            return false;
    }
    res.image = img;
    return true;
}


//Assumes LoadImage has been called
bool get_mask_srv              (    appearance_utils::GetMask::Request    &req,
                                    appearance_utils::GetMask::Response   &res )
{
    cout << "Called get_mask service" << endl;
    IplImage *cv_image = landmarkDetector_->getMask();
    sensor_msgs::ImagePtr img_ptr;
    sensor_msgs::Image img;
    try
    {
            img_ptr = bridge_.cvToImgMsg(cv_image, "mono8");
            img = *img_ptr;
    }
    catch (sensor_msgs::CvBridgeException error)
    {
            ROS_ERROR("error");
            return false;
    }
    res.mask = img;
    return true;
}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "landmark_response_server");
    ros::NodeHandle n("~");
    landmarkDetector_ = new LandmarkDetector(600,260,100,200,200,100,3);

    //Initialize by loading the model
    string model_file;
    n.param<string>("model_file", model_file, "model.txt");
    landmarkDetector_->loadModelFile(const_cast<char*>(model_file.c_str()));

    //Start the services
    ros::ServiceServer image_service = n.advertiseService("load_image", load_image_srv);
    ROS_INFO("load_image service ready.");
    ros::ServiceServer landmark_service = n.advertiseService("landmark_response", landmark_response_srv);
    ROS_INFO("landmark_response service ready.");
    ros::ServiceServer landmark_all_service = n.advertiseService("landmark_response_all", landmark_response_all_srv);
    ROS_INFO("landmark_response_all service ready.");
    ros::ServiceServer extract_lbp_features_service = n.advertiseService("extract_lbp_features", extract_lbp_features_srv);
    ROS_INFO("extract_lbp_features service ready.");
    ros::ServiceServer mask_service = n.advertiseService("get_mask", get_mask_srv);
    ROS_INFO("get_mask service ready.");
    ros::spin();
    return 0; 
}
