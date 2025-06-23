#ifndef RELOC_SERVICE_H
#define RELOC_SERVICE_H

#include "common/common.h"
#include "../matchRateCal/match_rate_cal.h"
#include "./ScanContext/Scancontext.h"
#include "BfsSearch/bfsSearch.h"

namespace alg{

  // 使用方法
  // -------0.0 初始化-------
  // RelocService()
  // ------1.0 设置参数------
  // setParams();
  // setGlobalMap();
  // setKeyFramePoseAndSCMAP();
  // ------2.0 调用函数------
  // localRelocByBfs();
  // relocBySC();
  // relocByOverlap();

  class RelocService{
    public:

      struct relocParams
      {
        double local_map_radius{100.0};
        double match_rate_threshold{0.80};
      };
      

      RelocService(ros::NodeHandle& nh);

      // 根据初值进行重定位
      // @params: input_cloud_ptr 输入点云， init_pose 初始位姿， out_pose 输出位姿
      bool relocWithAccuratePose(const utils::PointICloudPtr& input_cloud_ptr, const utils::Pose& init_pose, utils::Pose& out_pose);

      // 设置参数
      void setParams(const alg::RelocService::relocParams& params);

      // 设置点云地图
      void setGlobalMap(const utils::PointICloudPtr& gloabMapPtr);

      // 设置SC描述子地图
      // @params: descriptorFileDirectory 地图路径
      // sc map 文件名: polarcontexts.txt
      // 关键帧位姿 文件名: poses.txt， 格式：kitti
      bool setKeyFramePoseAndSCMAP(const std::string& descriptorFileDirectory);

      // 采用SC进行重定位
      // @params: input_cloud_ptr 输入点云， out_pose 输出位姿
      bool relocBySC(const utils::PointICloudPtr& input_cloud_ptr, utils::Pose& out_pose);

      // 采用Overlap进行重定位
      // @params: input_cloud_ptr 输入点云， idx overlap搜索到的回环点序号, out_pose 输出位姿
      bool relocByOverlap(const utils::PointICloudPtr& input_cloud_ptr, const int& idx,
                          utils::Pose& out_pose);

      // 采用暴力搜索进行局部重定位
      bool localRelocByBfs(const utils::PointICloudPtr& input_cloud_ptr, const alg::bfsSearch::searchParams& params, utils::Pose& out_pose);

      bool reloc_success{false};
      utils::Pose reloc_pose;
    
    private:
      json param_json_;

      ros::Publisher MapCloudPublisher_;
      ros::Publisher finalCloudPublisher_;
      ros::Publisher initCloudPublisher_;
      ros::Publisher surrouddingCloudPublisher_;
      
      utils::PointICloud cur_cloud_;

      utils::PointICloudPtr gloabMapPtr_;
      pcl::KdTreeFLANN<utils::PointI>::Ptr kdtreeGloabMapPtr_;
      pcl::VoxelGrid<utils::PointI> downSizeCurCloud_;


      std::shared_ptr<alg::MatchRateCal> match_rate_cal_ptr_;

      SCManager scReloc_;
      std::vector<Eigen::Matrix4d> KeyFramesPose_;

      bool SC_enable_{false};

      relocParams params_;


  };


}



#endif