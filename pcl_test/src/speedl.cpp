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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/StdVector>


// using namespace std;
// ros::Publisher pcl_pub;
// ros::Publisher marker_pub;
// void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
	
// 	Eigen::Vector3d norm1,norm2;
// 	norm1.normalize();
// 	norm2.normalize();
// 	Eigen::Vector3d n=norm1.cross(norm2);
// 	double theta=acos(norm1.dot(norm2));
// 	std::cout<<theta<<std::endl;
// 	std::cout<<n<<std::endl;

// }


int main(int argc, char* argv[])
{
	Eigen::Vector3d norm1,norm2;
	// std::cin>>norm1;
	// std::cin>>norm2;
	for(int i=0;i<3;i++){
		std::cin>>norm1(i);
	}
	for(int i=0;i<3;i++){
		std::cin>>norm2(i);
	}
	norm1.normalize();
	norm2.normalize();
	Eigen::Vector3d n=norm1.cross(norm2);
	n.normalize();
	double theta=acos(norm1.dot(norm2));
	std::cout<<theta<<std::endl;
	Eigen::AngleAxisd rotationVector(theta,n);

    Eigen::Vector3d eulerAngle=rotationVector.matrix().eulerAngles(0,1,2);
	// std::cout<<eulerAngle(0)<<std::endl;
	std::cout<<eulerAngle(0)<<","<<eulerAngle(1)<<","<<eulerAngle(2)<<std::endl;
	std::cout<<n<<std::endl;
	return 0;
}
