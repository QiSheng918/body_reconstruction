#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>  
#include <geometry_msgs/PoseArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>


ros::Publisher pcl_pub;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("hello world");
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	double x_min=-0.5;
	double x_max=0.5;
	double delta_x=0.01;
	pcl::PointNormal searchPoint = cloud->at(0);
	float radius = 0.01;	
	int K = 10;	
	geometry_msgs::PoseArray msg;
	msg.header.frame_id="camera_depth_optical_frame";
	msg.header.stamp=ros::Time::now();
	msg.poses.resize(int((x_max-x_min)/delta_x));
	int index=0;
	for(double x=x_min;x<x_max;x+=delta_x){
		searchPoint.x=x;
		searchPoint.y=0;
		searchPoint.z=0;

		double x_center,y_center,z_center;
		std::vector<double> center(3,0);

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
				center[0]+=cloud->points[pointIdxNKNSearch[i]].x;
				center[1]+=cloud->points[pointIdxNKNSearch[i]].y;
				center[2]+=cloud->points[pointIdxNKNSearch[i]].z;
				// cloud->points[pointIdxNKNSearch[i]].
			}
			for(int i=0;i<3;i++) center[i]/=pointIdxNKNSearch.size();
			msg.poses[index].position.x=center[0];
			msg.poses[index].position.y=center[1];
			msg.poses[index].position.z=center[2];
			msg.poses[index].orientation.x=0;
			msg.poses[index].orientation.y=0;
			msg.poses[index].orientation.z=0;
			msg.poses[index].orientation.w=1;
			index++;
		}
	}
	std::cout<<index<<std::endl;
	std::cout<<msg.poses.size()<<std::endl;

	// msg.header.frame_id="base_link";
	// msg.header.stamp=ros::Time::now();
	// msg.poses.resize(2);
	// msg.poses[0].position.x=0;
	// msg.poses[0].position.y=0;
	// msg.poses[0].position.z=1;
	// msg.poses[0].orientation.x=0;
	// msg.poses[0].orientation.y=0;
	// msg.poses[0].orientation.z=0;
	// msg.poses[0].orientation.w=1;

	// msg.poses[1].position.x=0;
	// msg.poses[1].position.y=0;
	// msg.poses[1].position.z=0.5;
	// msg.poses[1].orientation.x=0;
	// msg.poses[1].orientation.y=0;
	// msg.poses[1].orientation.z=1;
	// msg.poses[1].orientation.w=0;


    pcl_pub.publish(msg);
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
	// ros::shutdown();
}


int main(int argc, char* argv[])
{
	 ros::init(argc, argv, "linear_normal_pub");
     ros::NodeHandle nh;

 
     ros::Subscriber sub = nh.subscribe ("normal_output", 1, cloud_cb);
	 pcl_pub= nh.advertise<geometry_msgs::PoseArray>("linear_normal_output", 1);
     ros::spin();

 
	return 0;
}
