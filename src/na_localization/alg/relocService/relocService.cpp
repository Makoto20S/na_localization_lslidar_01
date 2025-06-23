#include "relocService.h"

using namespace std;
using namespace utils;

namespace alg{

RelocService::RelocService(ros::NodeHandle& nh){
  
  // 发布相关话题
  MapCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("mapCloud", 1, true);
  finalCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("finalCloud", 1);
  initCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("initCloud", 1);
  surrouddingCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("surrouddingCloud", 1);

  gloabMapPtr_.reset(new PointICloud());
  kdtreeGloabMapPtr_.reset(new pcl::KdTreeFLANN<utils::PointI>());
  
  match_rate_cal_ptr_ = std::make_shared<alg::MatchRateCal>();

}

void RelocService::setParams(const alg::RelocService::relocParams& params){
  params_ = params;
}

void RelocService::setGlobalMap(const utils::PointICloudPtr& gloabMapPtr){
  
  // NLOG_INFO("[RelocService][setGlobalMap] get Global Map, global map size: {}", gloabMapPtr->size());
  // if(gloabMapPtr->size() < 0 ){
  //   NLOG_WARN("[RelocService][setGlobalMap] Global map is empty!!!");
  //   return;
  // }

  gloabMapPtr_ = gloabMapPtr;//地图点云

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*gloabMapPtr, msg_cloud);
  msg_cloud.header.stamp = ros::Time::now();
  msg_cloud.header.frame_id = "map";
  MapCloudPublisher_.publish(msg_cloud);

  kdtreeGloabMapPtr_->setInputCloud(gloabMapPtr);
  match_rate_cal_ptr_->setMap(gloabMapPtr);

  // NLOG_INFO("[RelocService][setGlobalMap] Map is init!");

}


bool RelocService::setKeyFramePoseAndSCMAP(const std::string& descriptorFileDirectory){
  
  int SC_size{-1};
  if(scReloc_.ReadformTxt(descriptorFileDirectory, SC_size)){
    SC_enable_ = true;
    // NLOG_INFO("[RelocService] SCMap ReadformTxt Success");
  }
  
  std::ifstream fin_pose;
  fin_pose.open(descriptorFileDirectory + "poses.txt");
  while (!fin_pose.eof()){
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 4; j++){
        fin_pose >> T(i, j);
      }
    }
    KeyFramesPose_.push_back(T);
  }
  fin_pose.close();

  if(KeyFramesPose_.size() > 0 && SC_size == KeyFramesPose_.size()-1){
    // NLOG_INFO("[RelocService] SC ReadPose Success");
    return false;
  }else{
    // NLOG_INFO("[RelocService] SC ReadPose Failed, SC size: {}, keyframe size: {}", SC_size, KeyFramesPose_.size());
    return true;
  }

}
//init_pose是search_result，即匹配率最高的位姿
bool RelocService::relocWithAccuratePose(const utils::PointICloudPtr& input_cloud_ptr, const utils::Pose& init_pose, 
                                          utils::Pose& out_pose){

  if(input_cloud_ptr->size() <= 0){
    cout<<"here???"<<endl;
    return false;
  }
  
  utils::PointI curpose;
  curpose.x = init_pose.t_(0);
  curpose.y = init_pose.t_(1);
  curpose.z = init_pose.t_(2);

  // extract local map
  utils::PointICloudPtr localMapPtr(new utils::PointICloud());
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  kdtreeGloabMapPtr_->radiusSearch(curpose, params_.local_map_radius, pointSearchInd, pointSearchSqDis);//kd树搜索经过bfsreloc的最优位姿的100米范围内的点，kdtreeGloabMapPtr_设置输入为地图点云
  for(const auto iter:pointSearchInd){
    localMapPtr->push_back(gloabMapPtr_->points[iter]);//组成局部地图点云
  }

  // get cur cloud 
  utils::PointICloudPtr curCloudDSPtr(new utils::PointICloud());
  downSizeCurCloud_.setInputCloud(input_cloud_ptr);
  downSizeCurCloud_.setLeafSize(0.4,0.4,0.4);
  downSizeCurCloud_.filter(*curCloudDSPtr);

  // predict
  utils::PointICloudPtr initCloudPtr(new utils::PointICloud());
  pcl::transformPointCloud(*curCloudDSPtr, *initCloudPtr, init_pose.T_.cast<float>());

  if(initCloudPtr->size() > 0){
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*initCloudPtr, msg_cloud);
    msg_cloud.header.stamp = ros::Time::now();
    msg_cloud.header.frame_id = "camera_init";//map
    initCloudPublisher_.publish(msg_cloud);
  }

  if(localMapPtr->size() > 0){
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*localMapPtr, msg_cloud);
    msg_cloud.header.stamp = ros::Time::now();
    msg_cloud.header.frame_id = "camera_init";//map
    surrouddingCloudPublisher_.publish(msg_cloud);
  }

  PointICloudPtr cloudFinalPtr(new PointICloud());
  pcl::IterativeClosestPoint<PointI, PointI> icp;
  icp.setInputSource(curCloudDSPtr);
  icp.setInputTarget(localMapPtr);
  icp.setMaxCorrespondenceDistance(1.0);//设置对应点对之间的最大距离（此值对配准结果影响较大）。
  icp.setEuclideanFitnessEpsilon(0.001);// 设置收敛条件是均方误差和小于阈值， 停止迭代；
  icp.setMaximumIterations(100); //最大迭代次数，icp是一个迭代的方法，最多迭代这些次（若结合可视化并逐次显示，可将次数设置为1）；
  icp.align(*cloudFinalPtr, init_pose.T_.cast<float>());//icp执行计算，并将变换后的点云保存在cloud_final里
  
  Eigen::Matrix4f icp_matrix4f=icp.getFinalTransformation();   
  cout<<"score:"<<icp.getFitnessScore()<<endl;
  if(cloudFinalPtr->size() > 0){
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*cloudFinalPtr, msg_cloud);
    msg_cloud.header.stamp = ros::Time::now();
    msg_cloud.header.frame_id = "camera_init";//map
    finalCloudPublisher_.publish(msg_cloud);
    cout<<"cloudFinalPtr->size() :"<<cloudFinalPtr->size()<<endl;
  }
  else
    cout<<"Reloc no points!!!!"<<endl;

  double match_rate;
  bool match_success = match_rate_cal_ptr_->calculateMatchRate(cloudFinalPtr, match_rate);
  // NLOG_INFO("[RelocService][relocWithAccuratePose] match_rate:{:.3f}", match_rate);

  cout << "match rate:" << match_rate << endl;
  if (match_rate > params_.match_rate_threshold)
  {
    out_pose = utils::Pose(icp_matrix4f.cast<double>());
    reloc_pose = out_pose;
    // NLOG_INFO("[RelocService][relocWithAccuratePose] Final pose:{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
    //            reloc_pose.x(), reloc_pose.y(), reloc_pose.z(), reloc_pose.roll(), reloc_pose.pitch(), reloc_pose.yaw());

    reloc_success = true;
    return true;
  }
  else
  {
    reloc_success = false;
    cout<<"reloc_success = false"<<endl;
    return false;
  }
}

bool RelocService::relocBySC(const utils::PointICloudPtr& input_cloud_ptr,
                             utils::Pose& out_pose){
  if(KeyFramesPose_.size() <= 0){
    NLOG_WARN("[RelocService][relocBySC] sc descriptorMap is empty !!!");
    return false;
  }

  if(!SC_enable_){
    NLOG_WARN("[RelocService][relocBySC] sc is not init!!!");
    return false;
  }        

  scReloc_.makeAndSaveScancontextAndKeys(*input_cloud_ptr);
  int SCclosestHistoryFrameID = -1; // init with -1
  auto detectResult = scReloc_.detectLoopClosureID(); // first: nn index, second: yaw diff 
  SCclosestHistoryFrameID = detectResult.first;
  double yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
  scReloc_.deleteLastScancontext();

  if (SCclosestHistoryFrameID == -1){ 
    NLOG_WARN("[RelocService][relocBySC] SC Find Loop error !!!");
    return false;
  }

  Eigen::Matrix4d scLoopPose_M = KeyFramesPose_[SCclosestHistoryFrameID];
  
  utils::Pose scLoopPose(scLoopPose_M(0,3), scLoopPose_M(1,3), scLoopPose_M(2,3), 0, 0, -yawDiffRad);
  // NLOG_INFO("[RelocService][relocBySC] SC init pose:{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
  //            scLoopPose.x(), scLoopPose.y(), scLoopPose.z(), scLoopPose.roll(), scLoopPose.pitch(), scLoopPose.yaw());

  return relocWithAccuratePose(input_cloud_ptr, scLoopPose, out_pose);
} 

bool RelocService::relocByOverlap(const utils::PointICloudPtr& input_cloud_ptr, const int& idx,
                                  utils::Pose& out_pose){
  float yaw_diff_rad{0.0};

  // 如果SC可用，则采用SC计算角度
  if(SC_enable_){
    scReloc_.calculateAngle( *input_cloud_ptr, idx, yaw_diff_rad );
  } 
  // cout << "yaw_diff_rad" << yaw_diff_rad << endl;

  // 循环搜索角度
  for(int i=0; i<36; ++i){
    double yaw = yaw_diff_rad*180/M_PI + 10*i;
    if(yaw > 180){
      yaw -=360;
    }
    if(yaw < -180){
      yaw +=360;
    }

    yaw = yaw/180 * M_PI;
    utils::Pose overlapLoopPose(KeyFramesPose_[idx](0,3), KeyFramesPose_[idx](1,3), KeyFramesPose_[idx](2,3), 0 , 0, -yaw); 
    cout << "overlapLoopPose: " << overlapLoopPose << endl;
    if(relocWithAccuratePose(input_cloud_ptr, overlapLoopPose, out_pose)){
      return true;
    }
  }

  return false;
  
} 

bool RelocService::localRelocByBfs(const utils::PointICloudPtr& input_cloud_ptr, const alg::bfsSearch::searchParams& params, utils::Pose& out_pose){

  // NLOG_INFO("[RelocService][localRelocByBfs] start localRelocByBfs, searchParams:"
  //           "threadNums:{}, cloud_range:{}, search_radius:{}, pose_size:{}, map_size:{}"
  //           "init pose:{}, {}, {}, {}, {}, {}",
  //           params.threadNums, params.cloud_range, params.search_radius, params.pose_size, params.map_size,
  //           params.init_pose.x(), params.init_pose.y(), params.init_pose.z(), params.init_pose.roll(), params.init_pose.pitch(), params.init_pose.yaw());

  alg::bfsSearch local_reloc;
  utils::Pose search_result;
  local_reloc.bfsreloc(*gloabMapPtr_, *input_cloud_ptr, params, search_result);
  // NLOG_INFO("[RelocService][localRelocByBfs] localRelocByBfs search_result:{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
  //           search_result.x(), search_result.y(), search_result.z(), search_result.roll(), search_result.pitch(), search_result.yaw());

  return relocWithAccuratePose(input_cloud_ptr, search_result, out_pose);//当前帧点云，匹配率最高的位姿，输出位姿

}

}




