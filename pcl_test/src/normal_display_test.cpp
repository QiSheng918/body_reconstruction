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
ros::Publisher pose_pub;
ros::Publisher marker_pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	vector<vector<double> >  normal_vec(10000,vector<double>(5,0));
	for(int i=0;i<cloud->points.size();i++){
		int m=int(cloud->points[i].x*100+50)*100+int(cloud->points[i].y*100+50);
		normal_vec[m][0]+=cloud->points[i].z;
		normal_vec[m][1]+=cloud->points[i].normal_x;
		normal_vec[m][2]+=cloud->points[i].normal_y;
		normal_vec[m][3]+=cloud->points[i].normal_z;
		normal_vec[m][4]+=1;
	}
	ROS_INFO("hello world!");
	
	double x_min=-0.5,x_max=0.5;
	double delta_x=0.01;
	double y=0;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = input->header.frame_id;
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

	geometry_msgs::PoseArray msg;
	msg.header.frame_id=input->header.frame_id;
	msg.header.stamp=ros::Time::now();
	//进行末端轨迹提取，提取结果为一系列路点的位姿
	Eigen::Vector3d norm1{0,0,1};

	for(double x=x_min;x<x_max;x+=delta_x){
		int m=int(x*100+50)*100+int(y*100+50);
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


			Eigen::Vector3d norm2;
			// std::cout<<normal_vec[m][1]<<','<<normal_vec[m][2]<<","<<normal_vec[m][3]<<std::endl;
			norm2<<normal_vec[m][1],normal_vec[m][2],normal_vec[m][3];
			norm2.normalize();
			std::cout<<norm2<<std::endl;
			Eigen::Vector3d n=norm1.cross(norm2);
			// n.normalize();
			double theta=acos(norm1.dot(norm2));

			Eigen::AngleAxisd angle_axis(theta,n);

		 	Eigen::Quaterniond q(angle_axis);
			std::vector<double> temp(7,0);
			temp[0]=x;
			temp[1]=y;
			temp[2]=normal_vec[m][0]/normal_vec[m][4]+0.05;
            temp[3]=q.x();
            temp[4]=q.y();
            temp[5]=q.z();
            temp[6]=q.w();
			geometry_msgs::Pose pose_temp;
			pose_temp.position.x=temp[0];
			pose_temp.position.y=temp[1];
			pose_temp.position.z=temp[2];

			pose_temp.orientation.x=temp[3];
			pose_temp.orientation.y=temp[4];
			pose_temp.orientation.z=temp[5];
			pose_temp.orientation.w=temp[6];
			msg.poses.push_back(pose_temp);
			
		}		
	}
    pose_pub.publish(msg);

}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "linear_normal_pub");
    ros::NodeHandle nh;

    marker_pub= nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	pose_pub=nh.advertise<geometry_msgs::PoseArray>("pose_array",1000);

  	ROS_INFO("STARTING");
	ros::Subscriber sub = nh.subscribe ("cloud_normal", 1, cloud_cb);
	ros::spin();

 
	return 0;
}
