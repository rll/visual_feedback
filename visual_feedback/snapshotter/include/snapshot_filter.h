#ifndef SNAPSHOT_FILTER_H
#define SNAPSHOT_FILTER_H
#include <ros/ros.h>
#include "snapshotter/Snapshot.h"
#include <string>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class SnapshotFilter {

    public:

    ros::NodeHandle n_;
    sensor_msgs::CvBridge bridge_;
    std::string name;
    std::string input_topic;
    std::string output_topic;
    ros::Publisher output_pub;
    ros::Subscriber input_sub;

    SnapshotFilter();
    SnapshotFilter(ros::NodeHandle &n,ros::NodeHandle &glob);
    ~SnapshotFilter();
    
    virtual void get_extended_params();
    virtual IplImage* image_filter(IplImage* img_ptr,sensor_msgs::CameraInfo* cam_info_ptr);

    void handle_my_input(const snapshotter::SnapshotConstPtr& snapshot_ptr);

};
#endif
