#include "bfsSearch.h"

using namespace alg;
using namespace std;

namespace alg{
//初始化栅格地图，计算每个地图点对应的栅格索引，将该栅格置为true；   计算过程没太看懂
void bfsSearch::generateGridMap(const utils::PointICloud& map, const utils::Pose& init_pose, const float& map_range, const float& grid_size){
    float inv_resolution = 1.0 / grid_size;
    //基于rviz/GPS给的初始位姿栅格地图边界
    map_bottom_(0) = static_cast<int>(floor(init_pose.x() - map_range) * inv_resolution);
    map_bottom_(1) = static_cast<int>(floor(init_pose.y() - map_range) * inv_resolution);
    map_bottom_(2) = static_cast<int>(floor(init_pose.z() - map_range/2) * inv_resolution);
    //地图长宽高
    uint32_t dx = static_cast<uint32_t>(2 * map_range * inv_resolution) + 1;
    uint32_t dy = static_cast<uint32_t>(2 * map_range * inv_resolution) + 1;
    uint32_t dz = static_cast<uint32_t>(map_range * inv_resolution) + 1;
    uint32_t grid_nums =  dx * dy * dz;
    divb_mul_ = Eigen::Vector3i(1, dx, dx*dy);

    pc_gridmap_.clear();
    pc_gridmap_.resize(grid_nums, false);

    for(const auto p:map.points){
        int i = static_cast<int>(floor(p.x * inv_resolution)- map_bottom_(0));
        int j = static_cast<int>(floor(p.y * inv_resolution)- map_bottom_(1));
        int z = static_cast<int>(floor(p.z * inv_resolution)- map_bottom_(2));
        uint32_t idx = i*divb_mul_(0) + j*divb_mul_(1) + z*divb_mul_(2);
        if(!pc_gridmap_[idx]){
            pc_gridmap_[idx] = true;
        }
    }
}

void bfsSearch::generateRotatedCloud(const utils::PointICloud& cloud, const utils::Pose& init_pose, std::vector<pcIdx>& rotated_cloud_idx, const float& grid_size){
    float inv_resolution = 1.0 / grid_size;
    //遍历每一个角度
    for(int angle=-180; angle<180; ++angle){
        pcIdx pc_idx;
        pc_idx.cloud_angle = float(angle)/180 * M_PI;

        utils::Pose cur_pose(init_pose.x(), init_pose.y(), init_pose.z(), 0, 0, pc_idx.cloud_angle);
        utils::PointICloud tmp_cloud;
        utils::transfromPointCloud(cloud, cur_pose, tmp_cloud);//当前点云转到地图系
        // pcl::transformPointCloud(cloud, tmp_cloud, cur_pose.T_.cast<float>());

        //计算当前点云每个点对应的栅格索引
        for(const auto p:tmp_cloud.points){
            int i = static_cast<int>(floor(p.x * inv_resolution)- map_bottom_(0));
            int j = static_cast<int>(floor(p.y * inv_resolution)- map_bottom_(1));
            int z = static_cast<int>(floor(p.z * inv_resolution)- map_bottom_(2)); 
            uint32_t idx = i*divb_mul_(0) + j*divb_mul_(1) + z*divb_mul_(2);
            pc_idx.idx.push_back(idx);
        }
        rotated_cloud_idx.push_back(pc_idx);
    }
}

void bfsSearch::bfsreloc(const utils::PointICloud& map, const utils::PointICloud& cloud,
                         const searchParams& params, utils::Pose& result_pose)            
{
    //pose_grid存储可能的位姿，与设置的search_radius有关
    int pose_grid_div = static_cast<int>(floor(params.search_radius/ params.pose_size));   
    std::vector<Eigen::Vector2d> pose_grid;
    for(int i=-pose_grid_div; i<pose_grid_div+1; ++i){
        for(int j=-pose_grid_div; j<pose_grid_div+1; ++j){
            Eigen::Vector2d coord(i*params.pose_size, j*params.pose_size);
            pose_grid.push_back(coord);
        }
    }
    //当前点云处理，滤除100米外的点，滤出nan点
    utils::PointICloudPtr cloud_extract_ptr(new utils::PointICloud());
    for(const auto p:cloud.points){
        if(p.x < -params.cloud_range || p.x > params.cloud_range || p.y < -params.cloud_range || p.y > params.cloud_range || p.z < -params.cloud_range || p.z > params.cloud_range) continue;
        if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
        cloud_extract_ptr->push_back(p);
    }

    utils::PointICloudPtr curCloudDSPtr(new utils::PointICloud());
    pcl::VoxelGrid<utils::PointI> downSizeCurCloud;
    downSizeCurCloud.setInputCloud(cloud_extract_ptr);
    downSizeCurCloud.setLeafSize(0.4,0.4,0.4);//0.4
    downSizeCurCloud.filter(*curCloudDSPtr);

    float map_range = params.cloud_range + params.search_radius;
    utils::PointICloud map_extract;
    for(const auto p:map.points){
        if(p.x < (params.init_pose.x() - map_range) || p.x > (params.init_pose.x() + map_range) 
        || p.y < (params.init_pose.y() - map_range) || p.y > (params.init_pose.y() + map_range) 
        || p.z < (params.init_pose.z() - map_range/2) || p.z > (params.init_pose.z() + map_range/2)) continue;
        map_extract.push_back(p);
    }
    generateGridMap(map_extract, params.init_pose, map_range, params.map_size);

    std::vector<project_result> project_result_v;
    double init_x = static_cast<int>(floor(params.init_pose.x()/ params.pose_size)) * params.pose_size; 
    double init_y = static_cast<int>(floor(params.init_pose.y()/ params.pose_size)) * params.pose_size; 
    utils::Pose init_grid_pose(init_x, init_y, params.init_pose.z(), 0, 0, 0);

    std::vector<pcIdx> rotated_cloud_idx;
    generateRotatedCloud(*curCloudDSPtr, init_grid_pose, rotated_cloud_idx, params.map_size);

    std::mutex v_mute;
    omp_set_num_threads(params.threadNums);
    #pragma omp parallel for
    //遍历每一个可能的位置
    for(int j=0; j<pose_grid.size(); ++j){
        Eigen::Vector2d local_grid = pose_grid[j];
        double x_global = static_cast<int>(floor((params.init_pose.x() + local_grid(0))/ params.pose_size)) * params.pose_size; 
        double y_global = static_cast<int>(floor((params.init_pose.y() + local_grid(1))/ params.pose_size)) * params.pose_size; 

        Eigen::Vector2d offest;
        offest(0) = static_cast<int>(floor((x_global - init_x)/ params.map_size));
        offest(1) = static_cast<int>(floor((y_global - init_y)/ params.map_size));
        //遍历每一个角度下的当前帧点云
        for(const auto pc_idx:rotated_cloud_idx){
            utils::Pose cur_pose(x_global, y_global, params.init_pose.z(), 0, 0, pc_idx.cloud_angle);
            float vaild_nums{0.0};
            //遍历每个点的栅格索引
            for(const auto idx:pc_idx.idx){
                uint32_t real_idx = idx + offest(0) * divb_mul_(0) + offest(1) * divb_mul_(1);//计算索引
                if(pc_gridmap_[real_idx]) ++vaild_nums;//若地图点也存在该栅格，则有效点加一；
            }
            double match_rate = vaild_nums/float(pc_idx.idx.size());//匹配率 = 有效点数量 / 当前帧点云点数量

            project_result result;
            result.pose = cur_pose;
            result.match_rate = match_rate;

            {
                std::lock_guard<std::mutex> buf_lock(v_mute);
                project_result_v.push_back(result);
            }
        }
    }

    std::sort(project_result_v.begin(), project_result_v.end(), std::less<project_result>());//升序
    project_result final_result = project_result_v.back();//取出匹配率最高的一次

    result_pose = final_result.pose;

    // NLOG_INFO("[bfsSearch][bfsreloc] Final match_rate:{:.3f}.", final_result.match_rate);


}


}
