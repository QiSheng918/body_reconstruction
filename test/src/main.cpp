#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const char WINDOW[] = "Image window";
// static void help()
// {
//     printf("\nThis program demonstrates converting OpenCV Image to ROS Image messages  \n"
//         );
 
// }
 
int main(int argc, char** argv)
{
//   help();
  ros::init(argc, argv, "image_converter");
 
  //Reading an image from the file
  
  cv::Mat cv_image = cv::imread("/home/seven/lake.png");
  if(cv_image.empty() )
    {
        ROS_ERROR("Read the picture failed!");
        return -1;
    }
 
  //Convert OpenCV image to ROS message
  ros::NodeHandle node;
  image_transport::ImageTransport transport(node);
  image_transport::Publisher image_pub; 
  image_pub=transport.advertise("OutImage", 1);
  ros::Time time=ros::Time::now(); 
 
  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.header.frame_id = "image";
  cvi.encoding = "bgr8";
  cvi.image = cv_image;
 
  sensor_msgs::Image im;
  cvi.toImageMsg(im);
  ros::Rate loop_rate(50);
  while(ros::ok()){
      image_pub.publish(im);
      loop_rate.sleep();
  }
  
//   ROS_INFO("Converted Successfully!");
 
  //Show the image
//   cv::namedWindow(WINDOW);
//   cv::imshow(WINDOW,cv_image);
//   cv::waitKey(0);
 
//   ros::spin();
  return 0;
}