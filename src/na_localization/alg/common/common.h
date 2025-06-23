#ifndef ALG_COMMON_HPP_
#define ALG_COMMON_HPP_


#include <ros/ros.h>

#include <iostream>           
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>    

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

#include "pose.h"
#include "./ivox3d/ivox3d.h"
#include "json.hpp"

#include "log/log.h"
#include <omp.h>

using json = nlohmann::json;

struct PointXYZIR
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

namespace utils{

  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> PointICloud;
  typedef PointICloud::Ptr PointICloudPtr;

  // inline PointICloudPtr transfromPointCloud(const PointICloudPtr& cloudIn, utils::Pose& pose){
  //   PointICloudPtr cloudOut(new PointICloud());
  //   for(auto const &point:cloudIn->points){
  //     PointI pointOut;
  //     pointOut.x = pose.T_(0,0)*point.x + pose.T_(0,1)*point.y + pose.T_(0,2)*point.z + pose.T_(0,3);
  //     pointOut.y = pose.T_(1,0)*point.x + pose.T_(1,1)*point.y + pose.T_(1,2)*point.z + pose.T_(1,3);
  //     pointOut.z = pose.T_(2,0)*point.x + pose.T_(2,1)*point.y + pose.T_(2,2)*point.z + pose.T_(2,3);
  //     pointOut.intensity = point.intensity;
  //     cloudOut->push_back(pointOut);
  //   }
  // }

  inline void transfromPointCloud(const PointICloud& cloudIn, const utils::Pose& pose, PointICloud& cloudOut){
    for(auto const &point:cloudIn.points){
      PointI pointOut;
      pointOut.x = pose.T_(0,0)*point.x + pose.T_(0,1)*point.y + pose.T_(0,2)*point.z + pose.T_(0,3);
      pointOut.y = pose.T_(1,0)*point.x + pose.T_(1,1)*point.y + pose.T_(1,2)*point.z + pose.T_(1,3);
      pointOut.z = pose.T_(2,0)*point.x + pose.T_(2,1)*point.y + pose.T_(2,2)*point.z + pose.T_(2,3);
      pointOut.intensity = point.intensity;
      cloudOut.push_back(pointOut);
    }
  }
}

 


#endif