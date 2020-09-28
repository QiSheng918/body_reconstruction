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
	pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = -1;


	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内点，使用的点

	pcl::SACSegmentation<pcl::PointNormal> seg;
	//可选设置
	seg.setOptimizeCoefficients(true);
	//必须设置
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);    //分割操作

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

	pcl::ExtractIndices<pcl::PointNormal> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	//除去平面之外的数据
	extract.setNegative(true);
	extract.filter(*output_cloud);

	sensor_msgs::PointCloud2 msg;
	msg.header.stamp=ros::Time::now();
    msg.header.frame_id = "kinect_link";
	pcl::toROSMsg(*output_cloud, msg);
	pcl_pub.publish(msg);

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	double x_min=-0.5;
	double x_max=0.5;
	double delta_x=0.01;
	pcl::PointNormal searchPoint = output_cloud->at(0);
	float radius = 0.005;		
	for(double x=x_min;x<x_max;x+=delta_x){
		searchPoint.x=x;
		searchPoint.y=0;
		// searchPoint.z=0;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
					std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
					<< " " << cloud->points[pointIdxRadiusSearch[i]].y
					<< " " << cloud->points[pointIdxRadiusSearch[i]].z
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
			}
	}
	// geometry_msgs::PoseArray msg;
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


    // pcl_pub.publish(msg);
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
	// ros::shutdown();
}


int main(int argc, char* argv[])
{
	 ros::init(argc, argv, "linear_normal_pub");
     ros::NodeHandle nh;

 
     ros::Subscriber sub = nh.subscribe ("normal_output", 1, cloud_cb);
	 pcl_pub= nh.advertise<sensor_msgs::PointCloud2>("linear_normal_output", 1);
     ros::spin();

 
	return 0;
}
