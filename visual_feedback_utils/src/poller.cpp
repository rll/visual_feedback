#include <cstdio>
#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include "polled_camera/GetPolledImage.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poller", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  double hz_val;
  nh.param<double>("hz",hz_val,2.0);
  double hz = boost::lexical_cast<double>(hz_val);
  
  
  std::string service_name = nh.resolveName("camera") + "/request_image";
  ros::ServiceClient client = nh.serviceClient<polled_camera::GetPolledImage>(service_name);

  polled_camera::GetPolledImage::Request req;
  polled_camera::GetPolledImage::Response rsp;
  req.response_namespace = nh.resolveName("output");

  ros::Rate loop_rate(hz);
  while (nh.ok()) {
    if (client.call(req, rsp)) {
      std::cout << "Timestamp: " << rsp.stamp << std::endl;
      loop_rate.sleep();
    }
    else {
      ROS_ERROR("Service call failed");
      client.waitForExistence();
    }
  }
}
