#include "reloc_plugin.h"

using namespace std;

namespace plugins
{
    RelocPlugin::RelocPlugin(ros::NodeHandle& nh, const std::string& params_filename){
        
        // 1.0 读取参数
        json param_json;
        std::ifstream infile(params_filename);
        infile >> param_json;
        string point_map_directory = param_json["point_map_directory"].get<string>();
        string sc_map_directory = param_json["sc_map_directory"].get<string>();
        
        alg::RelocService::relocParams reloc_params;
        try
        {
            reloc_params.match_rate_threshold = param_json["match_rate_threshold"].get<double>();
            reloc_params.local_map_radius = param_json["local_map_radius"].get<double>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        utils::PointICloudPtr map_cloud_ptr(new utils::PointICloud());
        if (pcl::io::loadPCDFile<utils::PointI> (point_map_directory + ".pcd", *map_cloud_ptr) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        }

        // NLOG_INFO("[RelocPlugin] point_map_directory:{}, sc_map_directory:{}, match_rate_threshold:{}, local_map_radius:{}"
        //             "map_cloud siz:{}",
        //             point_map_directory, sc_map_directory, reloc_params.match_rate_threshold, reloc_params.local_map_radius,
        //             map_cloud_ptr->size());

        if(map_cloud_ptr->size() < 10) return;

        // 2.0 初始化重定位服务
        reloc_service_ptr_ = std::make_shared<alg::RelocService>(nh);
        reloc_service_ptr_->setParams(reloc_params);
        reloc_service_ptr_->setGlobalMap(map_cloud_ptr);//地图点云
        // reloc_service_ptr_->setKeyFramePoseAndSCMAP(sc_map_directory);

        overlapCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("overlapCloud", 1);
        overlapSignPublisher_ = nh.advertise<nav_msgs::Odometry>("reloc_sign", 1);
        overlapLoopMsgSubscriber_ = nh.subscribe("/overlapLoopMsg", 1, &RelocPlugin::readOverlapLoopMsg, this);

    }

    

    void RelocPlugin::readLidar(const sensor_msgs::PointCloud2& cloud_msgs){
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        new_cloud_msg_ = cloud_msgs;
        if(!get_scan_) get_scan_ = true;
    }

    bool RelocPlugin::initPoseSrv(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose_msg_ptr){
        if(!reloc_service_ptr_) 
            return false;

        if(!get_scan_)  return false;   

        Eigen::Vector3d pose_t(init_pose_msg_ptr->pose.pose.position.x, init_pose_msg_ptr->pose.pose.position.y, init_pose_msg_ptr->pose.pose.position.z);
        Eigen::Quaterniond pose_q;
        pose_q.x() = init_pose_msg_ptr->pose.pose.orientation.x;
        pose_q.y() = init_pose_msg_ptr->pose.pose.orientation.y;
        pose_q.z() = init_pose_msg_ptr->pose.pose.orientation.z;
        pose_q.w() = init_pose_msg_ptr->pose.pose.orientation.w;
        utils::Pose initPose(pose_q, pose_t); 

        std::lock_guard<std::mutex> lock(lidar_mutex_);
        utils::PointICloudPtr input_cloud_ptr(new utils::PointICloud());
        pcl::fromROSMsg(new_cloud_msg_, *input_cloud_ptr);
        
        if(input_cloud_ptr->empty()){
            std::cout << "no points cloud!" << std::endl;
            return false;
        }

        // utils::Pose out_pose;
        reloSuccess_ = reloc_service_ptr_->relocWithAccuratePose(input_cloud_ptr, initPose, relocPose_);
        return reloSuccess_;
    }
  
    bool RelocPlugin::relocBySC(){

        if(!reloc_service_ptr_) 
            return false;

        if(!get_scan_)  return false;   

        std::lock_guard<std::mutex> lock(lidar_mutex_);
        utils::PointICloudPtr input_cloud_ptr(new utils::PointICloud());
        pcl::fromROSMsg(new_cloud_msg_, *input_cloud_ptr);
        if(input_cloud_ptr->empty()){
            std::cout << "no points cloud!" << std::endl;
            return false;
        }

        reloSuccess_ = reloc_service_ptr_->relocBySC(input_cloud_ptr, relocPose_);
        return reloSuccess_;
    }

    bool RelocPlugin::relocalByOverlap(){
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        overlapCloudPublisher_.publish(new_cloud_msg_);
        return true;
    }

    bool RelocPlugin::localRelocByBfs(const utils::Pose& init_pose, const double& search_radius){
        
        if(!reloc_service_ptr_) 
            return false;

        if(!get_scan_)  return false;
        
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        utils::PointICloudPtr input_cloud_ptr(new utils::PointICloud());
        pcl::fromROSMsg(new_cloud_msg_, *input_cloud_ptr);
        if(input_cloud_ptr->empty()){
            std::cout << "no points cloud!" << std::endl;
            return false;
        }

        alg::bfsSearch::searchParams search_params;
        search_params.init_pose = init_pose;
        search_params.search_radius = search_radius;

        reloSuccess_ = reloc_service_ptr_->localRelocByBfs(input_cloud_ptr, search_params, relocPose_);
        return reloSuccess_;
    } 

    void RelocPlugin::readOverlapLoopMsg(const nav_msgs::Odometry::ConstPtr& overlapLoopMsgPtr){
        overlap_msg_ = *overlapLoopMsgPtr;
        get_new_overlap_msg_ = true;
    }


    void RelocPlugin::overlapLoopMsgCallback(){

        if(!reloc_service_ptr_) 
            return;

        if(!get_scan_)  return;
        
        if(manualReloc_) return;

        if(!get_new_overlap_msg_) return;
        get_new_overlap_msg_ = false;

        Eigen::Vector3d pose_t(overlap_msg_.pose.pose.position.x, overlap_msg_.pose.pose.position.y, overlap_msg_.pose.pose.position.z);
        std::vector<Eigen::Vector3d> failedPose;

        std::vector<int> failidx;

        for(int i=0; i<36; ++i){

            if(manualReloc_) return;

            int idx = overlap_msg_.pose.covariance[i];
            
            bool tooclose{false};
            for(const auto lastidx:failidx){
                if(abs(idx - lastidx) < 5){
                    tooclose = true;
                    break;
                }
            }

            if(tooclose) continue;

            cout << "get msg for overlap: " << idx << endl;

            utils::PointICloudPtr input_cloud_ptr(new utils::PointICloud());
            pcl::fromROSMsg(new_cloud_msg_, *input_cloud_ptr);

            // utils::Pose out_pose;
            reloSuccess_ = reloc_service_ptr_->relocByOverlap(input_cloud_ptr, idx, relocPose_);
            if(reloSuccess_) 
            {
                break;
            }else{
                failidx.push_back(idx);
            }
                
        
        }

        return;
        
    }

    utils::Pose RelocPlugin::getRelocPose(){
        return relocPose_;
    }


} // namespace plugins
