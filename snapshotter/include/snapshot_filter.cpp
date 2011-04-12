#ifndef SNAPSHOT_FILTER_CPP
#define SNAPSHOT_FILTER_CPP
#include "snapshot_filter.h"

SnapshotFilter::SnapshotFilter(){
    //Do nothing
}

SnapshotFilter::SnapshotFilter(ros::NodeHandle &n,ros::NodeHandle &glob) 
{
    ROS_INFO("constructor called");
    n_ = n;
    //n.getParam("name",name);
    name = ros::this_node::getName();
    n.getParam("input",input_topic);
    n.getParam("output",output_topic);
    get_extended_params();
    input_sub = glob.subscribe<snapshotter::Snapshot>(input_topic,1,&SnapshotFilter::handle_my_input,this);
    output_pub = glob.advertise<snapshotter::Snapshot>(output_topic,1);
    ROS_INFO("constructor finished");
}

SnapshotFilter::~SnapshotFilter()
{
    //Do nothing
}

void SnapshotFilter::get_extended_params(){};

IplImage* SnapshotFilter::image_filter(IplImage* img_ptr,sensor_msgs::CameraInfo* cam_info_ptr){
    return (IplImage*) img_ptr;
};

void SnapshotFilter::handle_my_input(const snapshotter::SnapshotConstPtr& snapshot_ptr)
{
    ROS_INFO("received");
    sensor_msgs::Image image = snapshot_ptr->image;
    sensor_msgs::ImagePtr img_ptr(new sensor_msgs::Image(image));
    sensor_msgs::CameraInfo cam_info = snapshot_ptr->info;
    
    IplImage *cv_image = NULL;
    
    try
    {
        cv_image = bridge_.imgMsgToCv(img_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("error");
        return;
    }

    IplImage *output_cv_image = image_filter(cv_image,&cam_info);
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
        return;
    }

    snapshotter::Snapshot snapshot;
    snapshot.image = output_img;
    snapshot.info = cam_info;

    ROS_INFO("Preparing to publish");
    output_pub.publish(snapshot);
    ROS_INFO("Published");

}

class CircleFilter : public SnapshotFilter{
    public:
    CircleFilter(ros::NodeHandle &n,ros::NodeHandle &glob) : SnapshotFilter(n,glob){
    };
    ~CircleFilter(){
    
    };
    
    void get_extended_params(){
    };
    
    IplImage* image_filter(IplImage* img_ptr,sensor_msgs::CameraInfo* cam_info_ptr){
        return (IplImage*) img_ptr;
    };
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "snapshot_filter");
    ROS_INFO("Main Called");
    ros::NodeHandle n("~");
    ros::NodeHandle glob;
    ROS_INFO("NH instantiated");
    SnapshotFilter ic(n,glob);
    ROS_INFO("Successfully created snapshotfilter");
    ros::spin();
    return 0;
}
#endif

