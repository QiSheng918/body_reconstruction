// 利用点云数据提取人体表面法向量并生成一条轨迹

#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <string>
#include "geometry_msgs/WrenchStamped.h"


const double velocity_limit=0.5;
const double desire_fz=8;

class SpeedCmdGenerator
{
public:
	SpeedCmdGenerator(){
		ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);
		pcl_sub = nh.subscribe ("cloud_normal", 1, &SpeedCmdGenerator::pclCallback,this);
		wrench_sub = nh.subscribe("compensate_wrench_base_filter", 1000, &SpeedCmdGenerator::wrenchCallback,this);
		for(int i=0;i<6;i++){
			command_vel.push_back(0);
			wrench_now.push_back(0);
			pos_now.push_back(0);
		}
		
		ros::Duration(5).sleep();
		this->posCmdGenerator();
		// ros::Rate loop_rate(250);
		// while(ros::ok()){
			
		// 	ros::spinOnce();
		// 	loop_rate.sleep();
		// }
	}

	

private:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub,wrench_sub;
	ros::Publisher ur_pub;
	tf::TransformListener listener;

	std::vector<double> pos_now;
	Eigen::Matrix3d rotation_matrix;
	std::vector<std::vector<double> > pos_desire;
	std::vector<double> wrench_now;

	std::vector<double> command_vel;

	void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input);
	void wrenchCallback(const geometry_msgs::WrenchStampedConstPtr &msg);
	void posCmdGenerator();
	void urMove();
	void limitVelocity(std::vector<double> &velocity);
	void getTransform();
	std::string double2string(double input);
};


void SpeedCmdGenerator::pclCallback(const sensor_msgs::PointCloud2ConstPtr& input){
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	std::vector<std::vector<double> >  normal_vec(10000,std::vector<double>(5,0));
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

	Eigen::Vector3d norm1{0,0,1};
	
	for(double x=x_min;x<x_max;x+=delta_x){
		int m=int(x*50+50)*100+int(y*50+50);
		if(normal_vec[m][4]!=0){
			Eigen::Vector3d norm2;
			norm2<<normal_vec[m][1],normal_vec[m][2],normal_vec[m][3];
			norm2.normalize();
			Eigen::Vector3d n=norm1.cross(norm2);
			// n.normalize();
			double theta=acos(norm1.dot(norm2));

			Eigen::AngleAxisd angle_axis(theta,n);


			Eigen::Vector3d euler_angle=angle_axis.toRotationMatrix().eulerAngles(2,1,0);
		
			std::vector<double> temp(6,0);
			temp[0]=x;
			temp[1]=y;
			temp[2]=normal_vec[m][0]/normal_vec[m][4];
			for(int i=0;i<3;i++) temp[i+3]=euler_angle(i);
			pos_desire.push_back(temp);
		}		
	}
	pcl_sub.shutdown();
};


void SpeedCmdGenerator::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg){
	wrench_now[0] = msg->wrench.force.x;
    wrench_now[1] = msg->wrench.force.y;
    wrench_now[2] = msg->wrench.force.z;
    wrench_now[3] = msg->wrench.torque.x;
    wrench_now[4] = msg->wrench.torque.y;
    wrench_now[5] = msg->wrench.torque.z;
};


void  SpeedCmdGenerator::posCmdGenerator()
{
	ros::Rate loop_rate(50);	
	for(int i=0;i<pos_desire.size();i++){
		int distance=(pos_desire[i][0]-pos_now[0])*(pos_desire[i][0]-pos_now[0])+(pos_desire[i][1]-pos_now[1])*(pos_desire[i][1]-pos_now[1]);
		while(distance<0.1){
			
			this->getTransform();
			Eigen::Vector3d linear_speed,angular_speed;
			for(int m=0;m<3;m++){
				linear_speed(m)=0.001*(pos_desire[i][m]);
			}
			for(int m=3;m<6;m++){
				angular_speed(m-3)=0.0001*(pos_desire[i][m]);
			}
			linear_speed=rotation_matrix.transpose()*linear_speed;
			linear_speed(2)=0.005*(desire_fz-wrench_now[2]);
			linear_speed=rotation_matrix*linear_speed;
			for(int m=0;m<3;m++) command_vel[m]=linear_speed(m);
			for(int m=0;m<3;m++) command_vel[m+3]=angular_speed(m);
			this->urMove();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
   
}

void SpeedCmdGenerator::getTransform(){
	tf::StampedTransform transform; 
	try{
		listener.lookupTransform("base", "tool0",ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	double _x=transform.getRotation().getX();
	double _y=transform.getRotation().getY();
	double _z=transform.getRotation().getZ();
	double _w=transform.getRotation().getW();
	Eigen::Quaterniond q(_w,_x,_y,_z);
	Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);
	rotation_matrix=q.toRotationMatrix();
	pos_now[0]=transform.getOrigin().getX();
	pos_now[1]=transform.getOrigin().getY();
	pos_now[2]=transform.getOrigin().getZ();
	for(int i=0;i<3;i++) pos_now[i+3]=eulerAngle(i);
}


//浮点数转string
std::string SpeedCmdGenerator::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}


//限制速度大小
void SpeedCmdGenerator::limitVelocity(std::vector<double> &velocity){
    for(int i=0;i<velocity.size();i++){
        if(fabs(velocity[i])<1e-3) velocity[i]=0;
        if(velocity[i]>velocity_limit) velocity[i]=velocity_limit;
        else if(velocity[i]<-velocity_limit) velocity[i]=-velocity_limit;
        else ;
    }
}


//UR机器人回调函数
void SpeedCmdGenerator::urMove()
{
    this->limitVelocity(command_vel);
    std_msgs::String ur_script_msgs;
    double time2move = 0.2;
    double acc=0.5;
    std::string move_msg;
    move_msg = "speedl([";
    move_msg = move_msg + double2string(command_vel[0]) + ",";
    move_msg = move_msg + double2string(command_vel[1]) + ",";
    move_msg = move_msg + double2string(command_vel[2]) + ",";
    move_msg = move_msg + double2string(command_vel[3]) + ",";
    move_msg = move_msg + double2string(command_vel[4]) + ",";
    move_msg = move_msg + double2string(command_vel[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(acc) + ",";
    move_msg = move_msg + double2string(time2move) + ")";
    move_msg = move_msg + "\n";
    ur_script_msgs.data=move_msg;
    ur_pub.publish(ur_script_msgs);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "speed_cmd_generator_node");
	ros::AsyncSpinner spinner(2);
    spinner.start();
	SpeedCmdGenerator speed_cmd_generator;
	return 0;
}
