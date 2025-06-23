#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

double pos_x = 0,pos_y=0,pos_z=0;
double pos_x_max = -10000,pos_y_max=-10000,pos_z_max=-10000;
double pos_x_min = 10000,pos_y_min=10000,pos_z_min=10000;
double pos_x_sum = 0,pos_y_sum=0,pos_z_sum=0;
double pos_x_avg = 0,pos_y_avg=0,pos_z_avg=0;
double cnt = 0.0;

tf::Quaternion quat;
double roll_=0,pitch_=0,yaw_=0;
double yaw_max = -10000,yaw_min = 10000,yaw_sum = 0,yaw_avg = 0;
double time_begin = 0,time_end=0;
bool first_data = false;

void odo_cbk(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(!first_data)
	{
		time_begin = msg->header.stamp.toSec();
		first_data = true;
	}
	time_end = msg->header.stamp.toSec() - time_begin;
	cnt ++;
	pos_x = msg->pose.pose.position.x;
	pos_y = msg->pose.pose.position.y;
	pos_z = msg->pose.pose.position.z;
	
	if(pos_x > pos_x_max) pos_x_max=pos_x;
	if(pos_y > pos_y_max) pos_y_max=pos_y;
	if(pos_z > pos_z_max) pos_z_max=pos_z;
	
	if(pos_x < pos_x_min) pos_x_min=pos_x;
	if(pos_y < pos_y_min) pos_y_min=pos_y;
	if(pos_z < pos_z_min) pos_z_min=pos_z;
	
	pos_x_sum += pos_x;
	pos_y_sum += pos_y;
	pos_z_sum += pos_z;
	
	pos_x_avg = pos_x_sum/cnt;
	pos_y_avg = pos_y_sum/cnt;
	pos_z_avg = pos_z_sum/cnt;

	
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
	
	if(yaw_ > yaw_max) yaw_max=yaw_;
	if(yaw_ < yaw_min) yaw_min=yaw_;
	
	yaw_sum += yaw_;
	yaw_avg = yaw_sum/cnt;
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pos_record");
  	ros::NodeHandle nh;
	ros::Subscriber sub_odo = nh.subscribe("/Odometry", 1, odo_cbk);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		if(time_end > 30.0)
		{
			break;
		}
		rate.sleep();
	}
	std::cout<<"pos_x_max:"<<pos_x_max<<" pos_y_max:"<<pos_y_max<<" pos_z_max:"<<pos_z_max<<std::endl;
	std::cout<<"pos_x_min:"<<pos_x_min<<" pos_y_min:"<<pos_y_min<<" pos_z_min:"<<pos_z_min<<std::endl;
	std::cout<<"pos_x_avg:"<<pos_x_avg<<" pos_y_avg:"<<pos_y_avg<<" pos_z_avg:"<<pos_z_avg<<std::endl;
	std::cout<<"yaw_max:"<<yaw_max<<" yaw_min:"<<yaw_min<<" yaw_avg:"<<yaw_avg<<std::endl;
	std::cout<<"time:"<<time_end<<std::endl;
	std::cout<<"cnt:"<<cnt<<std::endl;
	
	return 0;
}
