#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


ros::Publisher pcl_pub;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("hello world");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(50);
	// ne.setRadiusSearch (0.05); 
	ne.compute(*pcNormal);	
 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud, *pcNormal, *cloud_with_normals);

     sensor_msgs::PointCloud2 output;
	//  ros::Time::
	 output.header.stamp=ros::Time::now();
    output.header.frame_id = "world";
    pcl::toROSMsg(*cloud_with_normals, output);
    pcl_pub.publish(output);
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
	// ros::shutdown();
}


int main(int argc, char* argv[])
{
	 ros::init(argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;

 
     ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);
	 pcl_pub= nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
     ros::spin();

 
	return 0;
}
