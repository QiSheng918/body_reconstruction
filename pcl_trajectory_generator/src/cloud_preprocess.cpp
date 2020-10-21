// 进行点云数据预处理，包括坐标系转换，滤波等

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
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>




std::string frame="camera_color_optical_frame";
ros::Publisher pcl_pub;

std::string frmae_id="base";
Eigen::Matrix4d tool2base_transform_matrix;

void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *source_cloud);

   //进行点云坐标系转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, tool2base_transform_matrix);

 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;

    //进行点云直通滤波
    pass.setInputCloud (transformed_cloud);            
    pass.setFilterFieldName ("z");         
    pass.setFilterLimits (-1, 1);       
    pass.filter (*cloud_filtered);  

    pass.setInputCloud (cloud_filtered);  
    pass.setFilterFieldName ("x");         
    pass.setFilterLimits (-1.5, 0);   
    pass.filter (*cloud_filtered); 
     
    pass.setInputCloud (cloud_filtered);  
    pass.setFilterFieldName ("y");       
    pass.setFilterLimits (-0.5, 0.5);   
    pass.filter (*cloud_filtered); 

    sensor_msgs::PointCloud2 output;
	
    pcl::toROSMsg(*cloud_filtered, output);
    // pcl::toROSMsg(*transformed_cloud, output);
    output.header.stamp=ros::Time::now();
    output.header.frame_id = frmae_id;
    pcl_pub.publish(output);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pcl_preprocess_node");
    ros::NodeHandle nh;
    tf::TransformListener listener;
	pcl_pub= nh.advertise<sensor_msgs::PointCloud2> ("cloud_preprocessed", 1);
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, pclCallback);
    ros::Rate loop_rate(250);
    ROS_INFO("LOOPING");
    while(ros::ok()){
        tf::StampedTransform transform; 
        try{
      	listener.lookupTransform("base", frame,
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
      	    ROS_ERROR("%s",ex.what());
      	    ros::Duration(1.0).sleep();
        }
        tool2base_transform_matrix=Eigen::Matrix4d::Identity();
        double _x=transform.getRotation().getX();
        double _y=transform.getRotation().getY();
        double _z=transform.getRotation().getZ();
        double _w=transform.getRotation().getW();
        std::cout<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl;
        Eigen::Quaterniond q(_w,_x,_y,_z);
        std::cout<<q.w()<<","<<_w<<std::endl;
        tool2base_transform_matrix.block(0,0,3,3)=q.toRotationMatrix();
        Eigen::Vector3d p(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
        p=-q.toRotationMatrix().transpose()*p;
        tool2base_transform_matrix(0,3)=transform.getOrigin().getX();
        tool2base_transform_matrix(1,3)=transform.getOrigin().getY();
        tool2base_transform_matrix(2,3)=transform.getOrigin().getZ();
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}
