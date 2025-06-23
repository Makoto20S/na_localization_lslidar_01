//C++
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include <boost/filesystem.hpp>
//eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
//tf_my
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <inc_octree.h>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

double fusionTrans0[6]; //x,y,z,roll,pitch,yaw
double fusionTrans1[6];
double fusionTrans2[6];
double fusionTrans3[6];
ros::Publisher pub_trans1;
ros::Publisher pub_trans2;
ros::Publisher pub_trans3;
bool rec_rbo = false, rec_rb1 = false, rec_rb2 = false, rec_rb3 = false;

void BasetoENUHandler0(const grid_map::inc_octree::ConstPtr& TransMsg) 
{
  std::cout<<"00000000000000"<<std::endl;
  if (TransMsg->robotID == "Base_ENU") {
    std::cout << " receive  0 TransMsg" <<"; "<<TransMsg->poseX<<"; "<<TransMsg->poseY<<"; "<<TransMsg->poseZ<<"; "<<TransMsg->poseRoll<<"; " <<TransMsg->posePitch<<"; "<<TransMsg->poseYaw<<std::endl;
    fusionTrans0[0] = TransMsg->poseX;
    fusionTrans0[1] = TransMsg->poseY;
    fusionTrans0[2] = TransMsg->poseZ;
    fusionTrans0[3] = TransMsg->poseRoll;
    fusionTrans0[4] = TransMsg->posePitch;
    fusionTrans0[5] = TransMsg->poseYaw;
    rec_rbo =  true;
  }
}
void BasetoENUHandler1(const grid_map::inc_octree::ConstPtr& TransMsg) {
  std::cout<<"1111111111111111111"<<std::endl;
  if (TransMsg->robotID == "Base_ENU") {
    std::cout << " receive 1 TransMsg" <<"; "<<TransMsg->poseX<<"; "<<TransMsg->poseY<<"; "<<TransMsg->poseZ<<"; "<<TransMsg->poseRoll<<"; " <<TransMsg->posePitch<<"; "<<TransMsg->poseYaw<<std::endl;
    fusionTrans1[0] = TransMsg->poseX;
    fusionTrans1[1] = TransMsg->poseY;
    fusionTrans1[2] = TransMsg->poseZ;
    fusionTrans1[3] = TransMsg->poseRoll;
    fusionTrans1[4] = TransMsg->posePitch;
    fusionTrans1[5] = TransMsg->poseYaw;
    rec_rb1 =  true;
  }
}
void BasetoENUHandler2(const grid_map::inc_octree::ConstPtr& TransMsg) {
  std::cout<<"22222222222"<<std::endl;
  if (TransMsg->robotID == "Base_ENU") {
    std::cout << " receive 2 TransMsg" <<"; "<<TransMsg->poseX<<"; "<<TransMsg->poseY<<"; "<<TransMsg->poseZ<<"; "<<TransMsg->poseRoll<<"; " <<TransMsg->posePitch<<"; "<<TransMsg->poseYaw<<std::endl;
    fusionTrans2[0] = TransMsg->poseX;
    fusionTrans2[1] = TransMsg->poseY;
    fusionTrans2[2] = TransMsg->poseZ;
    fusionTrans2[3] = TransMsg->poseRoll;
    fusionTrans2[4] = TransMsg->posePitch;
    fusionTrans2[5] = TransMsg->poseYaw;
    rec_rb2 =  true;
  }
}
void BasetoENUHandler3(const grid_map::inc_octree::ConstPtr& TransMsg) {
  if (TransMsg->robotID == "Base_ENU") {
    std::cout << " receive 3 TransMsg" <<"; "<<TransMsg->poseX<<"; "<<TransMsg->poseY<<"; "<<TransMsg->poseZ<<"; "<<TransMsg->poseRoll<<"; " <<TransMsg->posePitch<<"; "<<TransMsg->poseYaw<<std::endl;
    fusionTrans3[0] = TransMsg->poseX;
    fusionTrans3[1] = TransMsg->poseY;
    fusionTrans3[2] = TransMsg->poseZ;
    fusionTrans3[3] = TransMsg->poseRoll;
    fusionTrans3[4] = TransMsg->posePitch;
    fusionTrans3[5] = TransMsg->poseYaw;
    rec_rb3 =  true;
  }
}

void Cal_Trans() {
  Eigen::Affine3f rb0_GL = pcl::getTransformation(fusionTrans0[0], fusionTrans0[1], fusionTrans0[2], fusionTrans0[3], fusionTrans0[4], fusionTrans0[5]);
  Eigen::Affine3f rb1_GL = pcl::getTransformation(fusionTrans1[0], fusionTrans1[1], fusionTrans1[2], fusionTrans1[3], fusionTrans1[4], fusionTrans1[5]);
  Eigen::Affine3f rb2_GL = pcl::getTransformation(fusionTrans2[0], fusionTrans2[1], fusionTrans2[2], fusionTrans2[3], fusionTrans2[4], fusionTrans2[5]);
  Eigen::Affine3f rb3_GL = pcl::getTransformation(fusionTrans3[0], fusionTrans3[1], fusionTrans3[2], fusionTrans3[3], fusionTrans3[4], fusionTrans3[5]);

  Eigen::Affine3f pose1_base = rb0_GL.inverse() * rb1_GL;  Eigen::Affine3f pose2_base = rb0_GL.inverse() * rb2_GL;  Eigen::Affine3f pose3_base = rb0_GL.inverse() * rb3_GL;
  float rb1_to_base[6]; float rb2_to_base[6]; float rb3_to_base[6];
  pcl::getTranslationAndEulerAngles(pose1_base, rb1_to_base[0], rb1_to_base[1], rb1_to_base[2], rb1_to_base[3], rb1_to_base[4], rb1_to_base[5]);
  std::cout << "pose1_base matrix:" << pose1_base.matrix() << std::endl;
  std::cout<<"rb1_to_base[0]:"<<rb1_to_base[0]<<"rb1_to_base[1]:"<<rb1_to_base[1]<<"rb1_to_base[2]:"<<rb1_to_base[2]<<"rb1_to_base[3]:"<<rb1_to_base[3]<<"rb1_to_base[4]:"<<rb1_to_base[4]<<"rb1_to_base[5]:"<<rb1_to_base[5]<<std::endl;
  pcl::getTranslationAndEulerAngles(pose2_base, rb2_to_base[0], rb2_to_base[1], rb2_to_base[2], rb2_to_base[3], rb2_to_base[4], rb2_to_base[5]);
  pcl::getTranslationAndEulerAngles(pose3_base, rb3_to_base[0], rb3_to_base[1], rb3_to_base[2], rb3_to_base[3], rb3_to_base[4], rb3_to_base[5]);
  nav_msgs::Odometry Robot1_base, Robot2_base, Robot3_base;
  Robot1_base.header.stamp.fromSec(ros::Time::now().toSec()); Robot2_base.header.stamp.fromSec(ros::Time::now().toSec()); Robot3_base.header.stamp.fromSec(ros::Time::now().toSec());
  Robot1_base.header.frame_id = "ouster1/base_link";   Robot2_base.header.frame_id = "ouster2/base_link";   Robot3_base.header.frame_id = "ouster3/base_link";
  Robot1_base.child_frame_id = "ouster1/base_link/odom2map";   Robot2_base.child_frame_id = "ouster2/base_link/odom2map";   Robot3_base.child_frame_id = "ouster3/base_link/odom2map";

  Robot1_base.pose.pose.position.x = rb1_to_base[0]; Robot1_base.pose.pose.position.y = rb1_to_base[1];  Robot1_base.pose.pose.position.z = rb1_to_base[2];
  Robot1_base.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rb1_to_base[3], rb1_to_base[4], rb1_to_base[5]);
  pub_trans1.publish(Robot1_base);

  Robot2_base.pose.pose.position.x = rb2_to_base[0]; Robot2_base.pose.pose.position.y = rb2_to_base[1];  Robot2_base.pose.pose.position.z = rb2_to_base[2];
  Robot2_base.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rb2_to_base[3], rb2_to_base[4], rb2_to_base[5]);
  pub_trans2.publish(Robot2_base);

  Robot3_base.pose.pose.position.x = rb3_to_base[0]; Robot3_base.pose.pose.position.y = rb3_to_base[1];  Robot3_base.pose.pose.position.z = rb3_to_base[2];
  Robot3_base.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rb3_to_base[3], rb3_to_base[4], rb3_to_base[5]);
  pub_trans3.publish(Robot3_base);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "rp_pub");
  ros::NodeHandle nh;
  ros::Subscriber subRb0toGL, subRb1toGL, subRb2toGL, subRb3toGL;
  fusionTrans0[0] = 0; fusionTrans0[1] = 0; fusionTrans0[2] = 0; fusionTrans0[3] = 0; fusionTrans0[4] = 0; fusionTrans0[5] = 0;
  fusionTrans1[0] = 0; fusionTrans1[1] = 0; fusionTrans1[2] = 0; fusionTrans1[3] = 0; fusionTrans1[4] = 0; fusionTrans1[5] = 0;
  fusionTrans2[0] = 0; fusionTrans2[1] = 0; fusionTrans2[2] = 0; fusionTrans2[3] = 0; fusionTrans2[4] = 0; fusionTrans2[5] = 0;
  fusionTrans3[0] = 0; fusionTrans3[1] = 0; fusionTrans3[2] = 0; fusionTrans3[3] = 0; fusionTrans3[4] = 0; fusionTrans3[5] = 0;
  subRb0toGL = nh.subscribe<grid_map::inc_octree>("/jackal0/lio_sam/mapping/inc_octree", 2000, BasetoENUHandler0);
  subRb1toGL = nh.subscribe<grid_map::inc_octree>("/jackal1/lio_sam/mapping/inc_octree", 2000, BasetoENUHandler1);
  subRb2toGL = nh.subscribe<grid_map::inc_octree>("/jackal2/lio_sam/mapping/inc_octree", 2000, BasetoENUHandler2);
  subRb3toGL = nh.subscribe<grid_map::inc_octree>("/jackal3/lio_sam/mapping/inc_octree", 2000, BasetoENUHandler3);
  pub_trans1 = nh.advertise<nav_msgs::Odometry> ("/jackal1/context/trans_map", 1);
  pub_trans2 = nh.advertise<nav_msgs::Odometry> ("/jackal2/context/trans_map", 1);
  pub_trans3 = nh.advertise<nav_msgs::Odometry> ("/jackal3/context/trans_map", 1);
  int num = 0;
  std::cout<<"1111"<<std::endl;
   while (ros::ok()) 
   {
     ros::spinOnce();
     if (num < 3 && rec_rbo == true && rec_rb2 == true) {// && num < 3 && rec_rb3 == true  rec_rbo == true  && rec_rb1 == true
       Cal_Trans();
       num++;
       std::cout<<"3333"<<std::endl;
       std::cout << " pub context : " << num <<std::endl;
    }
   }

  return 0;
}

