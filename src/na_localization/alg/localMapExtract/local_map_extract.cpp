#include "local_map_extract.h"

namespace alg{

void LocalMapExtract::setParameter(double GRID_SIZE, double LIDAR_RANGE){
    GRID_SIZE_ = GRID_SIZE;
    LIDAR_RANGE_ = LIDAR_RANGE;
}

void LocalMapExtract::generateGridMap(const utils::PointICloudPtr& pc){
    float min_x = 1000000, min_y =1000000;
    float max_x = -1000000, max_y =-1000000;

    for(const auto point:pc->points){
        if(point.x > max_x)  max_x = point.x;
        if(point.y > max_y)  max_y = point.y;
        if(point.x <= min_x) min_x = point.x;
        if(point.y <= min_y) min_y = point.y;
    }
    cout << min_x << " " <<  min_y << " " << max_x << " " << max_y << endl;

    min_x_b_ = GRID_SIZE_ * static_cast<int>(floor(min_x / GRID_SIZE_));
    min_y_b_ = GRID_SIZE_ * static_cast<int>(floor(min_y / GRID_SIZE_));
    max_x_b_ = GRID_SIZE_ * static_cast<int>(floor(max_x / GRID_SIZE_)+1);
    max_y_b_ = GRID_SIZE_ * static_cast<int>(floor(max_y / GRID_SIZE_)+1);

    cout << min_x_b_ << " " <<  min_y_b_ << " " << max_x_b_ << " " << max_y_b_ << endl;
    
    div_x_ = (max_x_b_ - min_x_b_) / GRID_SIZE_;
    div_y_ = (max_y_b_ - min_y_b_) / GRID_SIZE_;
    grid_nums = div_x_* div_y_;

    cout << div_x_ << " " <<  div_y_ << " " << grid_nums << endl;

    for(int y=0; y<div_y_; ++y){
        for(int x=0; x<div_x_; ++x){
            int grid_id = x +div_x_*y;
            Grid subgrid;
            subgrid.grid_id = grid_id;
            subgrid.grid_id_x = x;
            subgrid.grid_id_y = y;
            subgrid.lower_bound_x = min_x_b_ + GRID_SIZE_*x;
            subgrid.lower_bound_y = min_y_b_ + GRID_SIZE_*y;
            subgrid.upper_bound_x = min_x_b_ + GRID_SIZE_*(x+1);
            subgrid.upper_bound_y = min_y_b_ + GRID_SIZE_*(y+1);
            GridMap_.push_back(subgrid);
        }
    }

    for(const auto point:pc->points){
        int idx = static_cast<int>(floor((point.x - static_cast<int>(min_x_b_)) / GRID_SIZE_));
        int idy = static_cast<int>(floor((point.y - static_cast<int>(min_y_b_)) / GRID_SIZE_));
        int id = idx + div_x_*idy;
        GridMap_[id].pc.push_back(point);
    }

}

void LocalMapExtract::DownSampleLargeMap(const utils::PointICloudPtr& pc, string saveDirectory){
    float min_x = 1000000, min_y =1000000;
    float max_x = -1000000, max_y =-1000000;

    for(const auto point:pc->points){
        if(point.x > max_x)  max_x = point.x;
        if(point.y > max_y)  max_y = point.y;
        if(point.x <= min_x) min_x = point.x;
        if(point.y <= min_y) min_y = point.y;
    }
    cout << min_x << " " <<  min_y << " " << max_x << " " << max_y << endl;

    min_x_b_ = GRID_SIZE_ * static_cast<int>(floor(min_x / GRID_SIZE_));
    min_y_b_ = GRID_SIZE_ * static_cast<int>(floor(min_y / GRID_SIZE_));
    max_x_b_ = GRID_SIZE_ * static_cast<int>(floor(max_x / GRID_SIZE_)+1);
    max_y_b_ = GRID_SIZE_ * static_cast<int>(floor(max_y / GRID_SIZE_)+1);

    cout << min_x_b_ << " " <<  min_y_b_ << " " << max_x_b_ << " " << max_y_b_ << endl;
    
    div_x_ = (max_x_b_ - min_x_b_) / GRID_SIZE_;
    div_y_ = (max_y_b_ - min_y_b_) / GRID_SIZE_;
    grid_nums = div_x_* div_y_;

    cout << div_x_ << " " <<  div_y_ << " " << grid_nums << endl;

    for(int y=0; y<div_y_; ++y){
        for(int x=0; x<div_x_; ++x){
            int grid_id = x +div_x_*y;
            Grid subgrid;
            subgrid.grid_id = grid_id;
            subgrid.grid_id_x = x;
            subgrid.grid_id_y = y;
            subgrid.lower_bound_x = min_x_b_ + GRID_SIZE_*x;
            subgrid.lower_bound_y = min_y_b_ + GRID_SIZE_*y;
            subgrid.upper_bound_x = min_x_b_ + GRID_SIZE_*(x+1);
            subgrid.upper_bound_y = min_y_b_ + GRID_SIZE_*(y+1);
            GridMap_.push_back(subgrid);
        }
    }

    for(const auto point:pc->points){
        int idx = static_cast<int>(floor((point.x - static_cast<int>(min_x_b_)) / GRID_SIZE_));
        int idy = static_cast<int>(floor((point.y - static_cast<int>(min_y_b_)) / GRID_SIZE_));
        int id = idx + div_x_*idy;
        GridMap_[id].pc.push_back(point);
    }

    utils::PointICloudPtr GlobalMap(new utils::PointICloud());

    for(int id=0; id<grid_nums; ++id) {
        utils::PointICloudPtr points(new utils::PointICloud());
        *points = GridMap_[id].pc;
        utils::PointICloudPtr pointsDS(new utils::PointICloud());
        pcl::VoxelGrid<utils::PointI> sor;
        sor.setInputCloud (points);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.filter(*pointsDS);
        *GlobalMap += *pointsDS;
        GridMap_[id].pc.clear();
        GridMap_[id].pc = *pointsDS;
        cout << GridMap_[id].pc.size() << " ";
    }
    cout << GlobalMap->size() << endl;
    pcl::io::savePCDFileASCII(saveDirectory+"globalMap.pcd", *GlobalMap);
}

void LocalMapExtract::getLocalMap(utils::Pose cur_pose, utils::PointICloudPtr& localMapPtr){
    int idx = static_cast<int>(floor((cur_pose.t_(0) - static_cast<int>(min_x_b_)) / GRID_SIZE_));
    int idy = static_cast<int>(floor((cur_pose.t_(1) - static_cast<int>(min_y_b_)) / GRID_SIZE_));
    int select_grid_num = static_cast<int>(floor(LIDAR_RANGE_/GRID_SIZE_));
    localMapPtr->clear();

    for(int y=-select_grid_num; y<select_grid_num+1; ++y){
        int dy = idy+y;
        if(dy < 0) continue;
        if(dy >= div_y_) continue;
        for(int x=-select_grid_num; x<select_grid_num+1; ++x){
            int dx = idx+x;
            // cout << dx << " " << dy << endl;
            if(dx < 0) continue;
            if(dx >= div_x_) continue;

            int id = dx + dy*div_x_;
            *localMapPtr += GridMap_[id].pc;
            // cout << GridMap_[id].pc.size() << endl;
        }
    }
}




}

