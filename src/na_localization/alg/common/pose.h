/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/
#ifndef POSE_H
#define POSE_H

#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <iomanip>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

using namespace std;

namespace utils {

class Pose
{
public:
  Pose(){
    q_ = Eigen::Quaterniond::Identity();
    t_ = Eigen::Vector3d::Zero();
    T_ = Eigen::Matrix4d::Identity();
    td_ = 0;
    cov_.setZero();
  }

  Pose(const Pose &pose){
    q_ = pose.q_;
    t_ = pose.t_;
    T_ = pose.T_;
    td_ = pose.td_;
    cov_ = pose.cov_;
  }

  Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const double &td=0){
    q_ = q; q_.normalize();
    t_ = t;
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
    td_ = td;
    cov_.setZero();
  }

  Pose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const double &td=0){
    q_ = Eigen::Quaterniond(R);
    t_ = t;
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = R; T_.topRightCorner<3, 1>() = t_;
    td_ = td;
    cov_.setZero();
  }

  Pose(const Eigen::Matrix4d &T, const double &td=0){
    q_ = Eigen::Quaterniond(T.topLeftCorner<3, 3>()); q_.normalize();
    t_ = T.topRightCorner<3, 1>();
    T_ = T;
    td_ = td;
    cov_.setZero();
  }

  Pose(double x,  double y,  double z,  double roll,  double pitch,  double yaw, const double &td=0){
	  Eigen::Matrix3d R;
    R =
		Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Vector3d t(x, y, z);
    q_ = Eigen::Quaterniond(R);
    t_ = t;
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = R; T_.topRightCorner<3, 1>() = t_;
    td_ = td;
    cov_.setZero();
  }

  Pose(const nav_msgs::Odometry &odom){
    q_ = Eigen::Quaterniond(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);
    t_ = Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
    for (size_t i = 0; i < 6; i++)
      for (size_t j = 0; j < 6; j++)
        cov_(i, j) = double(odom.pose.covariance[i * 6 + j]);
  }

  Pose(const geometry_msgs::Pose &pose){
    q_ = Eigen::Quaterniond(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
    t_ = Eigen::Vector3d(
      pose.position.x,
      pose.position.y,
      pose.position.z);
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
  }

  Eigen::Vector3d Quat2rpy(const Eigen::Quaterniond& quat) const{
    Eigen::Vector3d atti;
    Eigen::Vector4d q(quat.w(), quat.x(), quat.y(), quat.z());
    q.normalize();
    double t0 = -2.0 * (q(2) * q(2) + q(3) * q(3)) + 1.0;
    double t1 = +2.0 * (q(1) * q(2) + q(0) * q(3));
    double t2 = -2.0 * (q(1) * q(3) - q(0) * q(2));
    double t3 = +2.0 * (q(2) * q(3) + q(0) * q(1));
    double t4 = -2.0 * (q(1) * q(1) + q(2) * q(2)) + 1.0;
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    double yaw = atan2(t1, t0);
    double pitch = asin(t2);
    double roll = atan2(t3, t4);

    atti[0] = yaw;
    atti[1] = pitch;
    atti[2] = roll;
    return atti;
  }

  double normalizeAngle(double angle) const{
    while (angle > M_PI)
    {
      angle -= 2. * M_PI;
    }

    while (angle < -M_PI)
    {
      angle += 2. * M_PI;
    }
    
    return angle;
  }

  inline double x() const{
    return t_[0];
  }

  inline double y() const{
    return t_[1];
  }

  inline double z() const{
    return t_[2];
  }

  inline double roll() const{
    Eigen::Vector3d euler = Quat2rpy(q_);
    return normalizeAngle(euler[2]);
  }

  inline double pitch() const{
    Eigen::Vector3d euler = Quat2rpy(q_);
    return normalizeAngle(euler[1]);
  }

  inline double yaw() const{
    Eigen::Vector3d euler = Quat2rpy(q_);
    return normalizeAngle(euler[0]);
  }


  static Pose poseTransform(const Pose &pose1, const Pose &pose2){
    // t12 = t1 + q1 * t2;
    // q12 = q1 * q2;
    return Pose(pose1.q_*pose2.q_, pose1.q_*pose2.t_+pose1.t_);
  }

  void update(){
    T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); 
    T_.topRightCorner<3, 1>() = t_;
  }

  Pose inverse() const{
    return Pose(q_.inverse(), -(q_.inverse()*t_));
  }
  Pose operator * (const Pose &pose){
    return Pose(q_*pose.q_, q_*pose.t_+t_);
  }
  friend ostream &operator << (ostream &out, const Pose &pose){
    out << std::fixed << std::setprecision(3)
      << "t [" << pose.t_(0) << "," << pose.t_(1) << "," << pose.t_(2) 
      << "], q [" << pose.q_.x() << "," << pose.q_.y() << "," << pose.q_.z() << "," << pose.q_.w()
      << "], td [" << pose.td_ << "]";
    return out;
  }

  double td_;
  Eigen::Quaterniond q_; // q = [cos(theta/2), u*sin(theta/2)]
  Eigen::Vector3d t_;
  Eigen::Matrix4d T_;
  Eigen::Matrix<double, 6, 6> cov_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // TODO: the Eigen bugs in initializing the class
};

}


#endif
