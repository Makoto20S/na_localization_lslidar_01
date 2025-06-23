#ifndef Local_Map_EXT_HPP_
#define Local_Map_EXT_HPP_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include "common/common.h"

namespace alg{

class LocalMapExtract{
  public:
    struct Grid
    {
        int grid_id;
        int grid_id_x;
        int grid_id_y;
        double lower_bound_x;
        double lower_bound_y;
        double upper_bound_x;
        double upper_bound_y;
        utils::PointICloud pc;
    };

    void setParameter(double GRID_SIZE, double LIDAR_RANGE);
    void generateGridMap(const utils::PointICloudPtr& pc);
    void getLocalMap(utils::Pose cur_pose, utils::PointICloudPtr& localMapPtr);
    void DownSampleLargeMap(const utils::PointICloudPtr& pc, string saveDirectory);

  private:
    double GRID_SIZE_{20.0}, LIDAR_RANGE_{60};
    std::vector<Grid> GridMap_;
    double min_x_b_, min_y_b_, max_x_b_, max_y_b_;
    int div_x_, div_y_, grid_nums;

};

}



#endif