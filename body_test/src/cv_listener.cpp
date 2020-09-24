#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
void callback(sensor_msgs::ImageConstPtr msg){
    ROS_INFO("U are in callback");
    std::cout<<msg->data[0]<<std::endl;
    std::cout<<msg->encoding<<std::endl;
    std::cout<<msg->data.size()<<std::endl;
    // cv::imwrite("/home/seven/test.jpg", cv_bridge::toCvShare(msg, "bgr8")->image);

    cv::imwrite("/home/seven/test.jpg", cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
    ros::shutdown();
} 
int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_sub");


  ros::NodeHandle node;
  image_transport::ImageTransport transport(node);
  image_transport::Subscriber image_sub=transport.subscribe("/camera/depth/image_rect_raw",1000,callback);
 
  ros::spin();
  return 0;
}