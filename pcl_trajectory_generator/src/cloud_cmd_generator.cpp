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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>


const double velocity_limit=0.5;
const double desire_fz=-5;
static const std::string PLANNING_GROUP = "manipulator";


class SpeedCmdGenerator
{
public:
	SpeedCmdGenerator():move_group(PLANNING_GROUP)
	{
		flag=true;
		for(int i=0;i<6;i++){
			command_vel.push_back(0);
			wrench_now.push_back(0);	
		}
		for(int i=0;i<7;i++) pos_now.push_back(0);

		std::string pos="vision_pose";
        move_group.setNamedTarget(pos);
        bool plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success) return;
        std::cout<<"plan success"<<std::endl;
        bool execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success) return;
        std::cout<<"execute success"<<std::endl;

		ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);
		pcl_sub = nh.subscribe ("cloud_normal", 1, &SpeedCmdGenerator::pclCallback,this);
		wrench_sub = nh.subscribe("compensate_wrench_tool", 1000, &SpeedCmdGenerator::wrenchCallback,this);
		pose_pub=nh.advertise<geometry_msgs::PoseArray>("pose_array",1000);
		ros::Duration(10).sleep();
		while(flag) ;
		// ros::spin();
		this->posCmdGenerator();

		ros::Duration(5).sleep();
        move_group.setNamedTarget(pos);
        plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success) return;
        std::cout<<"plan success"<<std::endl;
        execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success) return;
        std::cout<<"execute success"<<std::endl;
	}


private:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub,wrench_sub;
	ros::Publisher ur_pub,pose_pub;
	tf::TransformListener listener;

	std::vector<double> pos_now;
	Eigen::Matrix3d rotation_matrix;
	std::vector<std::vector<double> > pos_desire;
	std::vector<double> wrench_now;

	std::vector<double> command_vel;
	bool flag;

	moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


	void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input);
	void wrenchCallback(const geometry_msgs::WrenchStampedConstPtr &msg);
	void posCmdGenerator();
	void urMove();
	void limitVelocity(std::vector<double> &velocity);
	void getTransform();
	std::string double2string(double input);
};


//利用点云数据获取人体表面法向量并提取出一条机器人末端执行的轨迹
void SpeedCmdGenerator::pclCallback(const sensor_msgs::PointCloud2ConstPtr& input){
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg (*input, *cloud);

	// 利用hash表存储点云位置以及对应位置的法向量
	std::vector<std::vector<double> >  normal_vec(15000,std::vector<double>(5,0));
	for(int i=0;i<cloud->points.size();i++){
		int m=int(cloud->points[i].x*100+150)*100+int(cloud->points[i].y*100+50);
		normal_vec[m][0]+=cloud->points[i].z;
		normal_vec[m][1]+=cloud->points[i].normal_x;
		normal_vec[m][2]+=cloud->points[i].normal_y;
		normal_vec[m][3]+=cloud->points[i].normal_z;
		normal_vec[m][4]+=1;
	}

	double x_min=-0.55,x_max=-0.3;
	double delta_x=0.01;
	double y=0;
	geometry_msgs::PoseArray msg;
	msg.header.frame_id="base";
	msg.header.stamp=ros::Time::now();
	//进行末端轨迹提取，提取结果为一系列路点的位姿
	Eigen::Vector3d norm1{1,0,0};
	for(double x=x_min;x<x_max;x+=delta_x){
		int m=int(x*100+150)*100+int(y*100+50);
		if(normal_vec[m][4]!=0){
			Eigen::Vector3d norm2;
			// std::cout<<normal_vec[m][1]<<','<<normal_vec[m][2]<<","<<normal_vec[m][3]<<std::endl;
			norm2<<-normal_vec[m][1],-normal_vec[m][2],-normal_vec[m][3];
			norm2.normalize();
			std::cout<<norm2<<std::endl;
			Eigen::Vector3d n=norm1.cross(norm2);
			// n.normalize();
			double theta=acos(norm1.dot(norm2));

			Eigen::AngleAxisd angle_axis(theta,n);
			Eigen::AngleAxisd angle_axis1(M_PI/2,Eigen::Vector3d(0,1,0));
			Eigen::Matrix3d T=Eigen::Matrix3d::Identity();
			T(1,1)=-1;
			T(2,2)=-1;
		 	Eigen::Quaterniond q(angle_axis.toRotationMatrix()*angle_axis1.toRotationMatrix());
			std::vector<double> temp(7,0);
			temp[0]=x;
			temp[1]=y;
			temp[2]=normal_vec[m][0]/normal_vec[m][4]+0.02;
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
			
			pos_desire.push_back(temp);
		}		
	}
	pose_pub.publish(msg);
	ROS_INFO("calculate normal finished");
	for(int i=0;i<pos_desire.size();i++){
		for(int j=0;j<7;j++) std::cout<<pos_desire[i][j]<<",";
		std::cout<<std::endl;
	}
	flag=false;
	pcl_sub.shutdown();
};

//力矩传感器回调函数
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
	double distance=1;	
	while(distance>1e-4){
		distance=(pos_desire[0][0]-pos_now[0])*(pos_desire[0][0]-pos_now[0])+(pos_desire[0][1]-pos_now[1])*(pos_desire[0][1]-pos_now[1])+(pos_desire[0][2]-pos_now[2])*(pos_desire[0][2]-pos_now[2]);

			std::cout<<distance<<std::endl;
			this->getTransform();
			Eigen::Vector3d linear_speed,angular_speed;
			for(int m=0;m<3;m++) linear_speed(m)=2*(pos_desire[0][m]-pos_now[m]);
			
			double q_dot=0;
            for(int m=3;m<7;m++) q_dot+=pos_now[m]*pos_desire[0][m];
            if(q_dot<0){
                for(int m=3;m<7;m++) pos_now[m]=-pos_now[m];
            }
            Eigen::Matrix<double,3,1> epsilon, epsilon_d;                 
            epsilon << pos_now[3], pos_now[4], pos_now[5];          
            epsilon_d << pos_desire[0][3], pos_desire[0][4],pos_desire[0][5];          
            Eigen::Matrix<double,3,3> skew_matrix;
            skew_matrix << 0,-epsilon_d(2),epsilon_d(1),epsilon_d(2),0,-epsilon_d(0),-epsilon_d(1),epsilon_d(0),0;    
            Eigen::Matrix<double,3,1> orient_error = pos_desire[0][6] * epsilon - pos_now[6] * epsilon_d + skew_matrix * epsilon;                
			for(int m=0;m<3;m++) angular_speed(m)=  -1*orient_error(m); 

			linear_speed=rotation_matrix.transpose()*linear_speed;
			
			// linear_speed(2)=-0.005*(desire_fz-wrench_now[2]);
			linear_speed=rotation_matrix*linear_speed;
			for(int m=0;m<3;m++) command_vel[m]=linear_speed(m);
			for(int m=0;m<3;m++) command_vel[m+3]=angular_speed(m);
			this->urMove();
			loop_rate.sleep();
			ros::spinOnce();
	}
	for(int i=0;i<pos_desire.size();i++){
		
		distance=1;
		while(distance>1e-4){
			distance=(pos_desire[i][0]-pos_now[0])*(pos_desire[i][0]-pos_now[0])+(pos_desire[i][1]-pos_now[1])*(pos_desire[i][1]-pos_now[1]);
			std::cout<<distance<<std::endl;
			this->getTransform();
			Eigen::Vector3d linear_speed,angular_speed;
			for(int m=0;m<3;m++) linear_speed(m)=1*(pos_desire[i][m]-pos_now[m]);
			
			double q_dot=0;
            for(int m=3;m<7;m++) q_dot+=pos_now[m]*pos_desire[i][m];
            if(q_dot<0){
                for(int m=3;m<7;m++) pos_now[m]=-pos_now[m];
            }
            Eigen::Matrix<double,3,1> epsilon, epsilon_d;                 
            epsilon << pos_now[3], pos_now[4], pos_now[5];          
            epsilon_d << pos_desire[i][3], pos_desire[i][4],pos_desire[i][5];          
            Eigen::Matrix<double,3,3> skew_matrix;
            skew_matrix << 0,-epsilon_d(2),epsilon_d(1),epsilon_d(2),0,-epsilon_d(0),-epsilon_d(1),epsilon_d(0),0;    
            Eigen::Matrix<double,3,1> orient_error = pos_desire[i][6] * epsilon - pos_now[6] * epsilon_d + skew_matrix * epsilon;                
			for(int m=0;m<3;m++) angular_speed(m)=  -1*orient_error(m); 
			std::cout<<wrench_now[2]<<std::endl;
			linear_speed=rotation_matrix.transpose()*linear_speed;
			linear_speed(2)=-0.002*(desire_fz-wrench_now[2]);
			linear_speed=rotation_matrix*linear_speed;
			for(int m=0;m<3;m++) command_vel[m]=linear_speed(m);
			for(int m=0;m<3;m++) command_vel[m+3]=angular_speed(m);
			this->urMove();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
   
}

//获取机器人当前状态信息
void SpeedCmdGenerator::getTransform(){
	tf::StampedTransform transform; 
	try{
		listener.lookupTransform("base", "contact_frame",ros::Time(0), transform);
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
	rotation_matrix=q.toRotationMatrix();
    
	pos_now[0]=transform.getOrigin().getX();
	pos_now[1]=transform.getOrigin().getY();
	pos_now[2]=transform.getOrigin().getZ();
    pos_now[3]=transform.getRotation().getX();
    pos_now[4]=transform.getRotation().getY();
    pos_now[5]=transform.getRotation().getZ();
    pos_now[6]=transform.getRotation().getW();
	for(int i=0;i<7;i++) std::cout<<pos_now[i]<<";";
	std::cout<<std::endl;
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
        if(fabs(velocity[i])<1e-4) velocity[i]=0;
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
