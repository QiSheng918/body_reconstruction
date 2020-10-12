#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>  




ros::Publisher pcl_pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("hello world");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);            //设置输入点云
  pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (0.2, 1.5);        //设置在过滤字段的范围
//     pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
//   pass.setFilterLimits (-1.0, 1.0);        //设置在过滤字段的范围
//       pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
//   pass.setFilterLimits (-1.0, 1.0);        //设置在过滤字段的范围
  //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter (*cloud_filtered);  
  pass.setInputCloud (cloud_filtered);  
  pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (-0.5, 0.5);   
  pass.filter (*cloud_filtered); 
     
       pass.setInputCloud (cloud_filtered);  
  pass.setFilterFieldName ("y");       
  pass.setFilterLimits (-0.5, 0.5);   
  pass.filter (*cloud_filtered); 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);//体素大小设置为30*30*30cm
	sor.filter(*cloud_filtered);

     sensor_msgs::PointCloud2 output;
	//  ros::Time::
	 output.header.stamp=ros::Time::now();
    output.header.frame_id = "camera_depth_optical_frame";
    pcl::toROSMsg(*cloud_filtered, output);
    pcl_pub.publish(output);

}


int main(int argc, char* argv[])
{
	 ros::init(argc, argv, "filter");
     ros::NodeHandle nh;

 
     ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);
	 pcl_pub= nh.advertise<sensor_msgs::PointCloud2> ("filter_output", 1);
     ros::spin();

 
	return 0;
}
