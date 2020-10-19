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
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf/transform_listener.h>

using namespace std;
ros::Subscriber sub;
tf::TransformListener listener;
vector<Eigen::Matrix4d> points_base;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
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

	
	double x_min=-0.4,x_max=0.4;
	double delta_x=0.01;
	double y=0;
	tf::StampedTransform transform;
    try{
      	listener.lookupTransform("base", "tool0",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
    }
	Eigen::Matrix4d tool2base_transform_matrix=Eigen::Matrix4d::Identity();
	double _x=transform.getRotation().getX();
    double _y=transform.getRotation().getY();
    double _z=transform.getRotation().getZ();
    double _w=transform.getRotation().getW();
	Eigen::Quaterniond q(_w,_x,_y,_z);
	tool2base_transform_matrix.block(0,0,3,3)=q.toRotationMatrix();
	tool2base_transform_matrix(0,3)=transform.getOrigin().getX();
	tool2base_transform_matrix(1,3)=transform.getOrigin().getY();
	tool2base_transform_matrix(2,3)=transform.getOrigin().getZ();

	Eigen::Vector3d norm1{0,0,1};
	
	for(double x=x_min;x<x_max;x+=delta_x){

		int m=int(x*50+50)*100+int(y*50+50);
		// geometry_msgs::Pose temp_pose;
		geometry_msgs::Point p;
		if(normal_vec[m][4]!=0){
			Eigen::Vector3d norm2;
			norm2<<normal_vec[m][1],normal_vec[m][2],normal_vec[m][3];
			norm2.normalize();
			Eigen::Vector3d n=norm1.cross(norm2);
			// n.normalize();
			double theta=acos(norm1.dot(norm2));

			Eigen::AngleAxisd rotationVector(theta,n);

    		// Eigen::Vector3d eulerAngle=rotationVector.matrix().eulerAngles(0,1,2);
			Eigen::Matrix3d rotation_matrix=rotationVector.toRotationMatrix();
			Eigen::Matrix4d transform_matrix=Eigen::Matrix4d::Identity();
			transform_matrix.block(0,0,3,3)=rotationVector.toRotationMatrix();
			transform_matrix(0,3)=x;
			transform_matrix(1,3)=y;
			transform_matrix(2,3)=normal_vec[m][0]/normal_vec[m][4];
			points_base.emplace_back(tool2base_transform_matrix*transform_matrix);
		}		
	}
	sub.shutdown();
}

void  speedlCmdGenerate()
{
	
   
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "linear_normal_pub");
    ros::NodeHandle nh;
    sub = nh.subscribe ("normal_output", 1, cloud_cb);
    ros::spin();
	sub.shutdown();
 
	return 0;
}
