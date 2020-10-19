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



std::string frame="camera_depth_optical_frame";
ros::Publisher pcl_pub;
tf::TransformListener listener;
std::string frmae_id="base";
Eigen::Matrix4d tool2base_transform_matrix
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *source_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, tool2base_transform_matrix);

 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (transformed_cloud);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (0.2, 1.5);        //设置在过滤字段的范围
    pass.filter (*cloud_filtered);  

    pass.setInputCloud (cloud_filtered);  
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-0.5, 0.5);   
    pass.filter (*cloud_filtered); 
     
    pass.setInputCloud (cloud_filtered);  
    pass.setFilterFieldName ("y");       
    pass.setFilterLimits (-0.5, 0.5);   
    pass.filter (*cloud_filtered); 

    sensor_msgs::PointCloud2 output;
	output.header.stamp=ros::Time::now();
    output.header.frame_id = frmae_id
    pcl::toROSMsg(*cloud_filtered, output);
    pcl_pub.publish(output);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "filter");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
	pcl_pub= nh.advertise<sensor_msgs::PointCloud2> ("cloud_preprocessed", 1);
    ros::Rate loop_rate(250);
    while(ros::ok()){
        tf::StampedTransform transform; 
        try{
      	listener.lookupTransform("base", "tool0",  
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
        Eigen::Quaterniond q(_w,_x,_y,_z);
        tool2base_transform_matrix.block(0,0,3,3)=q.toRotationMatrix();
        tool2base_transform_matrix(0,3)=transform.getOrigin().getX();
        tool2base_transform_matrix(1,3)=transform.getOrigin().getY();
        tool2base_transform_matrix(2,3)=transform.getOrigin().getZ();
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}
