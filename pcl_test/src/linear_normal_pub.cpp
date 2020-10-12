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
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace std;
ros::Publisher pcl_pub;
ros::Publisher marker_pub;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	vector<vector<double> >  normal_vec(10000,vector<double>(5,0));
	for(int i=0;i<cloud->points.size();i++){
		int m=int(cloud->points[i].x*50+50)*100+int(cloud->points[i].y*50+50);
		normal_vec[m][0]+=cloud->points[i].z;
		normal_vec[m][1]+=cloud->points[i].normal_x;
		normal_vec[m][2]+=cloud->points[i].normal_y;
		normal_vec[m][3]+=cloud->points[i].normal_z;
		normal_vec[m][4]+=1;
	}
	ROS_INFO("hello world!");
	geometry_msgs::PoseArray msg;
	msg.header.frame_id="camera_depth_optical_frame";
	msg.header.stamp=ros::Time::now();
	double x_min=-0.4,x_max=0.4;
	double delta_x=0.01;
	double y=0;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "camera_depth_optical_frame";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.001;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
	for(double x=x_min;x<x_max;x+=delta_x){
		int m=int(x*50+50)*100+int(y*50+50);
		// geometry_msgs::Pose temp_pose;
		geometry_msgs::Point p;
		if(normal_vec[m][4]!=0){
			p.x=x;
			p.y=y;
			p.z=normal_vec[m][0]/normal_vec[m][4];
			line_list.points.push_back(p);
		
			// temp_pose.position.x=x;
			// temp_pose.position.y=y;
			// temp_pose.position.z=normal_vec[m][0]/normal_vec[m][4];
			double v[3]={normal_vec[m][1]/normal_vec[m][4],normal_vec[m][2]/normal_vec[m][4],normal_vec[m][3]/normal_vec[m][4]};
			double norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
			for(int i=0;i<3;i++) v[i]/=norm;
			p.x+=0.1*v[0];
			p.y+=0.1*v[1];
			p.z+=0.1*v[2];
			line_list.points.push_back(p);
    		marker_pub.publish(line_list);
			// for(int i=0;i<3;i++) v[i]/=norm;
			// temp_pose.orientation.x=v[0]*sin(norm/2);
			// temp_pose.orientation.y=v[1]*sin(norm/2);
			// temp_pose.orientation.z=v[2]*sin(norm/2);
			// temp_pose.orientation.w=cos(norm/2);
			// msg.poses.push_back(temp_pose);
			
		}
		// else{
		// 	temp_pose.position.x=x;
		// 	temp_pose.position.y=y;
		// 	temp_pose.position.z=-1;
		// }
		
	}
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

    marker_pub= nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  	
     ros::Subscriber sub = nh.subscribe ("normal_output", 1, cloud_cb);
	 pcl_pub= nh.advertise<geometry_msgs::PoseArray>("linear_normal_output", 1);
     ros::spin();

 
	return 0;
}
