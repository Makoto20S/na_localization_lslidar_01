#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

using namespace std;

typedef pcl::PointXYZINormal PointType;

class map_management
{
public:
    map_management();
    ~map_management();

    void set_input_PCD(string read_dir);
    void set_ds_size(double ds_size_);
    void print_xy();
    void voxel_process();
    void output_map();
    bool get_map(double posx,double posy);
    bool get_map_add(double posx,double posy);
    void find_neibor(int dx,int dy);
    void set_input_cloud(pcl::PointCloud<PointType>::Ptr cloud);  // 添加这个方法
    bool need_relocal;
    std::mutex mutex_;

    pcl::PointCloud<PointType>::Ptr pointcloud_output; //输出的地图点云
    pcl::PointCloud<PointType>::Ptr pointcloud_add; //输出的地图点云
    pcl::PointCloud<PointType>::Ptr cloud_copy;
    private:
    pcl::PointCloud<PointType>::Ptr pointcloudmap; //输入的地图点云
    // pcl::PointCloud<PointType>::Ptr pointcloud_output; //输入的地图点云
    double xmin,xmax,ymin,ymax;
    double voxel_size;
    double voxel_num;
    double ds_size;
    int Dx,Dy; //每个轴能划分的栅格大小
    int dx_last,dy_last;
    bool first_pos;
    vector<pcl::PointCloud<PointType>::Ptr> vec_voxel;

    void calculate_DxDy();
    void Initialize();
    // void find_neibor(int dx,int dy);
    
};