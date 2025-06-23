#ifndef _RELOC_BFSSERACH_
#define _RELOC_BFSSERACH_

#include "alg/common/common.h"

namespace alg{

class bfsSearch
{

public:
    struct searchParams
    {
        int threadNums{2};
        float cloud_range{100.0};
        float search_radius{10.0};
        float pose_size{0.5};//0.5
        float map_size{0.5};//0.5
        utils::Pose init_pose;
    };

    void bfsreloc(const utils::PointICloud& map, const utils::PointICloud& cloud,
                  const searchParams& params, utils::Pose& result_pose);

public:

    struct pcIdx
    {
        std::vector<uint32_t> idx;
        float cloud_angle;
    };

    struct project_result
    {
        utils::Pose pose;
        float match_rate;

        project_result (){};
        project_result (utils::Pose pose_, float match_rate_): pose(pose_), match_rate(match_rate_) {}
        bool operator < (const project_result& p) const{return (match_rate < p.match_rate);}//这个运算符重载，使排序按照匹配率进行排序
    };

private:
    std::vector<bool> pc_gridmap_;
    Eigen::Vector3d map_bottom_;

    searchParams params_;
    Eigen::Vector3i divb_mul_;

private:
    void generateGridMap(const utils::PointICloud& map, const utils::Pose& init_pose, const float& map_range, const float& grid_size);

    void generateRotatedCloud(const utils::PointICloud& cloud, const utils::Pose& init_pose, std::vector<pcIdx>& rotated_cloud_idx, const float& grid_size);
    
                      
    

};


}


#endif