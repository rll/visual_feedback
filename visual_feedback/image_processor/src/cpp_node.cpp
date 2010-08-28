#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

sensor_msgs::CvBridge bridge_;



IplImage *process(IplImage *input_cv_image, double *pts2d, double *params, int& num_pts, int& num_params){
    return = cvCloneImage(cv_image);
}

bool cpp_process(image_processor::ProcessBridge::Request    &req,
                 image_processor::ProcessBridge::Response &res )
{
    sensor_msgs::Image image = req->image;
    sensor_msgs::ImagePtr img_ptr(new sensor_msgs::Image(image))
    sensor_msgs::CameraInfo cam_info = snapshot_ptr->info;
    
    IplImage *cv_image = NULL;    
    try
        {
                cv_image = bridge_.imgMsgToCv(img_ptr, "bgr8");
        }
        catch (sensor_msgs::CvBridgeException error)
        {
                ROS_ERROR("error");
                return false;
        }
        double temp_pts[128];
        double temp_params[128];
        
        IplImage *output_cv_image;
        int num_pts = 0;
        int num_params = 0;
        output_cv_image = process(input_cv_image,temp_pts,temp_params,output_cv_image,num_pts,num_params);
        
        double pts[num_pts];
        double params[num_params];
        for i = 0; i < num_pts; i++{
            pts[i] = temp_pts[i];
        }
        for i = 0; i < num_params; i++{
            params[i] = num_params[i];
        }
        sensor_msgs::ImagePtr output_img_ptr;
        sensor_msgs::Image output_img;
        try
        {
                output_img_ptr = bridge_.cvToImgMsg(output_cv_image, "bgr8");
                output_img = *output_img_ptr;
        }
        catch (sensor_msgs::CvBridgeException error)
        {
                ROS_ERROR("error");
                return false;
        }
    res->image_annotated = output_img
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_image_processing_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("cpp_process", cpp_process);
    ROS_INFO("Ready to bridge cpp and python...");
    ros::spin();

    return 0;
}

