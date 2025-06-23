#ifndef PLUGINS_RELOC_HPP
#define PLUGINS_RELOC_HPP

#include <ros/ros.h>
#include "alg/relocService/relocService.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace plugins
{
    class RelocPlugin{
      public:
        RelocPlugin(ros::NodeHandle& nh, const std::string& params_filename);
        // ~RelocPlugin();

        // 获取激光数据
        void readLidar(const sensor_msgs::PointCloud2& cloud_msgs);

        // 设置位姿（用于rviz手动重定位）
        bool initPoseSrv(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose_msg_ptr);

        // 调用SC重定位服务
        bool relocBySC();
        bool relocBySC(const sensor_msgs::PointCloud2& cloud_msgs);

        // 调用overlap重定位服务
        bool relocalByOverlap();
        bool relocalByOverlap(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs);
        bool localRelocByBfs(const utils::Pose& init_pose, const double& search_radius);

        void readOverlapLoopMsg(const nav_msgs::Odometry::ConstPtr& overlapLoopMsgPtr);
        void overlapLoopMsgCallback();

        utils::Pose getRelocPose();

      public:
        bool manualReloc_{false};
        bool reloSuccess_{false};
        bool relocByOverlap_{true};
        utils::Pose relocPose_;

      private:
        ros::Publisher overlapCloudPublisher_;
        ros::Publisher overlapSignPublisher_;
        sensor_msgs::PointCloud2 cloudmsgforOverlap_;
        ros::Subscriber overlapLoopMsgSubscriber_;


        std::shared_ptr<alg::RelocService> reloc_service_ptr_;
        sensor_msgs::PointCloud2 new_cloud_msg_;

        bool get_scan_{false};

        nav_msgs::Odometry overlap_msg_;
        bool get_new_overlap_msg_{false};

        std::mutex lidar_mutex_;


    };
} // namespace plugins



#endif