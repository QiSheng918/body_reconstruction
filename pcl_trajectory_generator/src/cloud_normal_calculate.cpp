// 利用点云数据求解人体表面法向量

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
	ROS_INFO("point cloud normal calculating");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setViewPoint(0,0,2);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(50);
	// ne.setRadiusSearch (0.05); 
	ne.compute(*pcNormal);	
 
 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud, *pcNormal, *cloud_with_normals);

	sensor_msgs::PointCloud2 msg;
	
	pcl::toROSMsg(*cloud_with_normals, msg);
	msg.header.frame_id=input->header.frame_id;
	msg.header.stamp=ros::Time::now();
    pcl_pub.publish(msg);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cloud_normal_calculate");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("cloud_preprocessed", 1, cloud_cb);
	pcl_pub= nh.advertise<sensor_msgs::PointCloud2>("cloud_normal", 1);
    ros::spin();
	return 0;
}
