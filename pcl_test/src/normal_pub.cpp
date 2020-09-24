#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseArray.h>


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
	geometry_msgs::PoseArray msg;
	msg.header.frame_id="base_link";
	msg.header.stamp=ros::Time::now();
	msg.poses.resize(2);
	msg.poses[0].position.x=0;
	msg.poses[0].position.y=0;
	msg.poses[0].position.z=1;
	msg.poses[0].orientation.x=0;
	msg.poses[0].orientation.y=0;
	msg.poses[0].orientation.z=0;
	msg.poses[0].orientation.w=1;

	msg.poses[1].position.x=0;
	msg.poses[1].position.y=0;
	msg.poses[1].position.z=0.5;
	msg.poses[1].orientation.x=0;
	msg.poses[1].orientation.y=0;
	msg.poses[1].orientation.z=1;
	msg.poses[1].orientation.w=0;


    pcl_pub.publish(msg);
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
	// ros::shutdown();
}


int main(int argc, char* argv[])
{
	 ros::init(argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;

 
     ros::Subscriber sub = nh.subscribe ("filter_output", 1, cloud_cb);
	 pcl_pub= nh.advertise<geometry_msgs::PoseArray>("normal_output", 1);
     ros::spin();

 
	return 0;
}
