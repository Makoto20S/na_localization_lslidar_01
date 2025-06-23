#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

double pos_x = 0,pos_y=0,pos_z=0;
double pos_x_max = -10000,pos_y_max=-10000,pos_z_max=-10000;
double pos_x_min = 10000,pos_y_min=10000,pos_z_min=10000;
double pos_x_sum = 0,pos_y_sum=0,pos_z_sum=0;
double pos_x_var = 0, pos_y_var=0, pos_z_var=0;
double pos_x_avg = 0,pos_y_avg=0,pos_z_avg=0;
double pos_x_avg_new = 0.0, pos_y_avg_new=0.0, pos_z_avg_new=0.0;
double cnt = 0.0, cnt_n = 0.0; //过去含有n个样本 

tf::Quaternion quat;
double roll_=0,pitch_=0,yaw_=0;
double yaw_max = -10000,yaw_min = 10000,yaw_sum = 0,yaw_avg = 0,yaw_avg_new = 0,yaw_var = 0;
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
	cnt_n = cnt - 1;
	pos_x = msg->pose.pose.position.x;
	pos_y = msg->pose.pose.position.y;
	pos_z = msg->pose.pose.position.z;
	
	if(pos_x > pos_x_max) pos_x_max=pos_x;
	if(pos_y > pos_y_max) pos_y_max=pos_y;
	if(pos_z > pos_z_max) pos_z_max=pos_z;
	
	if(pos_x < pos_x_min) pos_x_min=pos_x;
	if(pos_y < pos_y_min) pos_y_min=pos_y;
	if(pos_z < pos_z_min) pos_z_min=pos_z;

	pos_x_avg = pos_x_avg_new;
	pos_y_avg = pos_y_avg_new;
	pos_z_avg = pos_z_avg_new;

	pos_x_avg_new = ((cnt_n * pos_x_avg) + pos_x)/(cnt);
	pos_z_avg_new = ((cnt_n * pos_z_avg) + pos_z)/(cnt);
	pos_y_avg_new = ((cnt_n * pos_y_avg) + pos_y)/(cnt);

	pos_x_var = (cnt_n * (pos_x_var+(pos_x_avg_new - pos_x_avg)*(pos_x_avg_new - pos_x_avg)) + (pos_x_avg_new - pos_x)*(pos_x_avg_new - pos_x))/(cnt);
    pos_y_var = (cnt_n * (pos_y_var+(pos_y_avg_new - pos_y_avg)*(pos_y_avg_new - pos_y_avg)) + (pos_y_avg_new - pos_y)*(pos_y_avg_new - pos_y))/(cnt);
    pos_z_var = (cnt_n * (pos_z_var+(pos_z_avg_new - pos_z_avg)*(pos_z_avg_new - pos_z_avg)) + (pos_z_avg_new - pos_z)*(pos_z_avg_new - pos_z))/(cnt);

	pos_x_sum += pos_x;
	pos_y_sum += pos_y;
	pos_z_sum += pos_z;
	

	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
	
	if(yaw_ > yaw_max) yaw_max=yaw_;
	if(yaw_ < yaw_min) yaw_min=yaw_;

	yaw_avg = yaw_avg_new;
	yaw_avg_new = ((cnt_n * yaw_avg) + yaw_)/(cnt);
	yaw_var= (cnt_n * (yaw_var+(yaw_avg_new - yaw_avg)*(yaw_avg_new - yaw_avg)) + (yaw_avg_new - pos_x)*(yaw_avg_new - yaw_))/(cnt);
	
	yaw_sum += yaw_;
	
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
	std::cout<<"pos_x_avg:"<<pos_x_avg_new<<" pos_y_avg:"<<pos_y_avg_new<<" pos_z_avg:"<<pos_z_avg_new<<std::endl;
	std::cout<<"pos_x_var:"<<pos_x_var<<" pos_y_var:"<<pos_y_var<<" pos_z_var:"<<pos_z_var<<std::endl;
	std::cout<<"yaw_max:"<<yaw_max<<" yaw_min:"<<yaw_min<<" yaw_avg:"<<yaw_avg_new<<" yaw_var:"<<yaw_var<<std::endl;
	std::cout<<"time:"<<time_end<<std::endl;
	std::cout<<"cnt:"<<cnt<<std::endl;
	
	return 0;
}
