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
#include <unordered_map>


ros::Publisher pcl_pub;
std::vector<std::vector<std::vector<double> > > normal_vec;
geometry_msgs::PoseArray msg;
int flag=0;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("hello world");
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);
    // cloud->points[0].
    std::cout<<cloud->points.size()<<std::endl;
    // if(flag==1){
    //     for(int i=0;i<400;i++){
    //         for(int j=0;j<400;j++){
    //             if(normal_vec[i][j].size()!=5) std::cout<<normal_vec[i][j].size()<<std::endl;
    //         }
    //     }
    // }
    std::cout<<normal_vec[187][160].size()<<std::endl;
    for(int i=0;i<cloud->points.size();i++){
        if(flag==1){
            std::cout<<int(cloud->points[i].x*100+200)<<","<<int(cloud->points[i].y*100+200)<<std::endl;
            std::cout<<cloud->points[i].z<<std::endl;
            std::cout<<cloud->points[i].normal_x<<std::endl;
            std::cout<<cloud->points[i].normal_y<<std::endl;
            std::cout<<cloud->points[i].normal_z<<std::endl;
            std::cout<<normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200].size()<<std::endl;
            std::cout<<std::endl;
        }
        
        // if((cloud->points[i].x*100+200)>=400 || (cloud->points[i].y*100+200)>=400 || (cloud->points[i].x*100+200)<0 || (cloud->points[i].y*100+200)<0) std::cout<<"false"<<std::endl;
        normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200][0]+=cloud->points[i].z;
        normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200][1]+=cloud->points[i].normal_x;
        normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200][2]+=cloud->points[i].normal_y;
        normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200][3]+=cloud->points[i].normal_z;
        normal_vec[cloud->points[i].x*100+200][cloud->points[i].y*100+200][4]+=1;
        // if(flag==1) std::cout<<"flag"<<std::endl;
    }
    flag++;

	msg.header.stamp=ros::Time::now();
   
   
    for(int i=0;i<400;i++){
        for(int j=0;j<400;j++){
            if(normal_vec[i][j].size()!=5) std::cout<<normal_vec[i][j].size()<<std::endl;
        }
    }
    ROS_INFO("start record");
    std::cout<<msg.poses.size()<<std::endl;
    ROS_INFO("stop record");
    for(int i=0;i<400;i++){
        for(int j=0;j<400;j++){
            // if(normal_vec[i][j][4]!=0){
            //     // geometry_msgs::Pose pose_msg;
                
                msg.poses[i*400+j].position.x=(i-200)*0.01;
			    msg.poses[i*400+j].position.y=(j-200)*0.01;
                msg.poses[i*400+j].position.z=0;
			    // msg.poses[i*400+j].orientation.x=0;
                // msg.poses[i*400+j].orientation.y=0;
                // msg.poses[i*400+j].orientation.z=0;
                // msg.poses[i*400+j].orientation.w=1;
            //     // normal_vec[i][j]={0,0,0,0,0}
            // }
            // else{
            //     msg.poses[i*400+j].position.x=(i-200)*0.01;
			//     msg.poses[i*400+j].position.y=(j-200)*0.01;
			//     msg.poses[i*400+j].position.z=-1;
			//     msg.poses[i*400+j].orientation.x=0;
            //     msg.poses[i*400+j].orientation.y=0;
            //     msg.poses[i*400+j].orientation.z=0;
            //     msg.poses[i*400+j].orientation.w=1;
            // }
            // for(int m=0;m<5;m++) normal_vec[i][j][m]=0;
            // normal_vec[i][j]={0,0,0,0,0};
            // if(normal_vec[i][j].size()!=5) std::cout<<normal_vec[i][j].size()<<std::endl;
        }
    }
    pcl_pub.publish(msg);
    ROS_INFO("stop record");
            for(int i=0;i<400;i++){
            for(int j=0;j<400;j++){
                if(normal_vec[i][j].size()!=5) std::cout<<normal_vec[i][j].size()<<std::endl;
            }
        }
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
	// ros::shutdown();
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "hash_test_node");
    ros::NodeHandle nh;
    // normal_vec.resize(400);
     msg.poses.resize(1600);
     	msg.header.frame_id="camera_depth_optical_frame";
    // std::vector<double> temp(5,0);
    // temp[0]=-1;
    // std::vector<std::vector<double> > temp_vec2(400,temp);
    for(int i=0;i<400;i++){
        normal_vec.push_back(std::vector<std::vector<double> >(400,std::vector<double>(5,0)));
    }
    // for(int i=0;i<400;i++){
    //     for(int j=0;j<400;j++){
    //         if(normal_vec[i][j].size()!=5) std::cout<<normal_vec[i][j].size()<<std::endl;
    //     }
    // }
    // for(int i=0;i<400;i++){
    //     for(int j=0;j<400;j++){
    //         if
    //         }
    
    
    ros::Subscriber sub = nh.subscribe ("normal_output", 1, cloud_cb);
	pcl_pub= nh.advertise<geometry_msgs::PoseArray>("linear_normal_output", 1);
    ros::spin();

 
	return 0;
}
