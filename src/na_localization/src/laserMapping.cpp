#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <Eigen/Dense>
#include "IMU_Processing.hpp"

// gstam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstring>

#include "GNSS_Processing.hpp"
#include <sensor_msgs/NavSatFix.h>
#include "na_localization/rtk_pos_raw.h"
#include "na_localization/rtk_heading_raw.h"
#include "na_localization/sensor_vaild.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <chrono>
#include <thread>

#include <GeographicLib/UTMUPS.hpp>

#include "map_management.h"

#include "alg/common/common.h"
#include "alg/relocService/relocService.h"
#include "plugins/reloc_plugin/reloc_plugin.h"


#include "lla_enu.hpp"

// 函数声明
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

// 提前声明的关键变量
/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf kf;
state_ikfom state_point;
state_ikfom state_point_last;//上一时刻的状态
state_ikfom state_point_lastframe; // 上一关键帧的状态

/***优化部分相关变量***/
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   // 历史所有关键帧的平面点集合（降采样）
pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

//重定位相关变量
double max_z,min_z;//构图过程中最大高度与最低高度

// #include "matchRateCal/match_rate_cal.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)//0.001
#define PUBFRAME_PERIOD     (20)
#include "cloud_info.h" 
#include "inc_octree.h"
float  keyframe_threshold_pos=1,keyframe_threshold_rot = 0.2; //关键帧阈值，m 和 度
shared_ptr<LLAENU> gnss2enu(new LLAENU());

// 添加robot_id相关变量
// std::string robot_id0 = "jackal0";
// std::string robot_id2 = "jackal2";
int current_robot_id = 0; // 当前机器人ID

// 地图数据传播相关变量
pcl::PointCloud<PointType>::Ptr shared_MapKeyFramesDS(new pcl::PointCloud<PointType>());
double shared_max_z = 0.0;
double shared_min_z = 0.0;
bool map_data_ready = false;
bool mapping_completed = false;
std::mutex map_data_mutex;

// 添加坐标变换相关变量
double fusionTrans0[6]; // robot_0位姿: x,y,z,roll,pitch,yaw
double fusionTrans2[6]; // robot_2位姿: x,y,z,roll,pitch,yaw
bool rec_rb0 = false;   // 是否接收到robot_0位姿
bool rec_rb2 = false;   // 是否接收到robot_2位姿
std::mutex pose_mutex;  // 位姿数据互斥锁

// 坐标变换发布器
ros::Publisher pub_trans0; // 发布robot_0相对于robot_2的变换
ros::Publisher pub_robot0_pose; // 发布robot_0位姿
ros::Publisher pub_robot2_pose; // 发布robot_2位姿

// 构图相关变量（从goutu文件移植）
double globalMapVisualizationPoseDensity = 1.0;
double globalMapVisualizationLeafSize = 0.4;
ros::Publisher pubSavedCloud;
ros::Publisher pubSavedPose;

// 键盘输入监听相关
bool enter_key_pressed = false;
std::mutex enter_key_mutex;

// 移植的saveMapService函数（修改为直接调用版本）
bool saveMapToMemory()
{
    std::lock_guard<std::mutex> lock(map_data_mutex);
    
    cout<<"start save map to memory"<<endl;

    pcl::PointCloud<PointType>::Ptr MapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr MapKeyFrames(new pcl::PointCloud<PointType>());

    pcl::VoxelGrid<PointType> downSizeFilterMapKeyPoses;
    downSizeFilterMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
    downSizeFilterMapKeyPoses.setInputCloud(cloudKeyPoses3D);
    downSizeFilterMapKeyPoses.filter(*MapKeyPosesDS);
    
    for (int i = 0; i < (int)MapKeyPosesDS->size(); ++i)
    {
        int thisKeyInd = (int)MapKeyPosesDS->points[i].intensity;
        if(thisKeyInd < surfCloudKeyFrames.size()) {
            *MapKeyFrames += *surfCloudKeyFrames[thisKeyInd];
        }
    }

    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
    downSizeFilterGlobalMapKeyFrames.setInputCloud(MapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*shared_MapKeyFramesDS);

    shared_max_z = max_z;
    shared_min_z = min_z;
    
    // 发布地图点云
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*shared_MapKeyFramesDS, cloudMsg);
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "camera_init";
    pubSavedCloud.publish(cloudMsg);

    // 发布位姿信息
    geometry_msgs::PoseArray poseMsg;  // 确保正确的类型声明
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "camera_init";
    
    for(int i = 0; i < cloudKeyPoses6D->size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = cloudKeyPoses6D->points[i].x;
        pose.position.y = cloudKeyPoses6D->points[i].y;
        pose.position.z = cloudKeyPoses6D->points[i].z;
        
        tf::Quaternion q;
        q.setRPY(cloudKeyPoses6D->points[i].roll, cloudKeyPoses6D->points[i].pitch, cloudKeyPoses6D->points[i].yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        
        poseMsg.poses.push_back(pose);
    }
    pubSavedPose.publish(poseMsg);

    map_data_ready = true;
    mapping_completed = true;
    
    cout<<"save map to memory done"<<endl;
    cout<<"已成功构建地图"<<endl;
    return true;
}

// 获取共享地图数据
bool getSharedMapData(pcl::PointCloud<PointType>::Ptr& mapData, double& maxZ, double& minZ)
{
    std::lock_guard<std::mutex> lock(map_data_mutex);
    if(!map_data_ready) {
        return false;
    }
    
    *mapData = *shared_MapKeyFramesDS;
    maxZ = shared_max_z;
    minZ = shared_min_z;
    cout<<"地图已传入id00"<<endl;  // 添加这行调试信息
    return true;
}

// 键盘输入监听线程
void keyboardInputThread()
{
    while(ros::ok()) {
        char input = getchar();
        if(input == '\n') {
            std::lock_guard<std::mutex> lock(enter_key_mutex);
            enter_key_pressed = true;
            cout << "Enter key pressed, enabling relocalization for robot_0" << endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// robot_0位姿回调函数
void robot0PoseHandler(const grid_map::inc_octree::ConstPtr& TransMsg) 
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    if (TransMsg->robotID == "Base_ENU") {
        std::cout << "receive robot_0 pose: " << TransMsg->poseX << ", " << TransMsg->poseY << ", " << TransMsg->poseZ 
                  << ", " << TransMsg->poseRoll << ", " << TransMsg->posePitch << ", " << TransMsg->poseYaw << std::endl;
        
        fusionTrans0[0] = TransMsg->poseX;
        fusionTrans0[1] = TransMsg->poseY;
        fusionTrans0[2] = TransMsg->poseZ;
        fusionTrans0[3] = TransMsg->poseRoll;
        fusionTrans0[4] = TransMsg->posePitch;
        fusionTrans0[5] = TransMsg->poseYaw;
        rec_rb0 = true;
    }
}

// robot_2位姿回调函数
void robot2PoseHandler(const grid_map::inc_octree::ConstPtr& TransMsg) 
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    if (TransMsg->robotID == "Base_ENU") {
        std::cout << "receive robot_2 pose: " << TransMsg->poseX << ", " << TransMsg->poseY << ", " << TransMsg->poseZ 
                  << ", " << TransMsg->poseRoll << ", " << TransMsg->posePitch << ", " << TransMsg->poseYaw << std::endl;
        
        fusionTrans2[0] = TransMsg->poseX;
        fusionTrans2[1] = TransMsg->poseY;
        fusionTrans2[2] = TransMsg->poseZ;
        fusionTrans2[3] = TransMsg->poseRoll;
        fusionTrans2[4] = TransMsg->posePitch;
        fusionTrans2[5] = TransMsg->poseYaw;
        rec_rb2 = true;
    }
}
// 计算并发布坐标变换
void calculateAndPublishTransform() 
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    
    if (!rec_rb0 || !rec_rb2) {
        return; // 等待两个机器人的位姿都接收到
    }
    
    // 使用PCL计算变换矩阵
    Eigen::Affine3f rb0_GL = pcl::getTransformation(fusionTrans0[0], fusionTrans0[1], fusionTrans0[2], 
                                                    fusionTrans0[3], fusionTrans0[4], fusionTrans0[5]);
    Eigen::Affine3f rb2_GL = pcl::getTransformation(fusionTrans2[0], fusionTrans2[1], fusionTrans2[2], 
                                                    fusionTrans2[3], fusionTrans2[4], fusionTrans2[5]);
    
    // 计算robot_0相对于robot_2的变换
    Eigen::Affine3f pose0_to_base2 = rb2_GL.inverse() * rb0_GL;
    
    float rb0_to_base2[6];
    pcl::getTranslationAndEulerAngles(pose0_to_base2, rb0_to_base2[0], rb0_to_base2[1], rb0_to_base2[2], 
                                     rb0_to_base2[3], rb0_to_base2[4], rb0_to_base2[5]);
    
    std::cout << "robot_0 to robot_2 transform: " << rb0_to_base2[0] << ", " << rb0_to_base2[1] << ", " << rb0_to_base2[2] 
              << ", " << rb0_to_base2[3] << ", " << rb0_to_base2[4] << ", " << rb0_to_base2[5] << std::endl;
    
    // 发布变换
    nav_msgs::Odometry Robot0_to_base2;
    Robot0_to_base2.header.stamp = ros::Time::now();
    Robot0_to_base2.header.frame_id = "jackal0/base_link";
    Robot0_to_base2.child_frame_id = "jackal0/base_link/odom2map";
    
    Robot0_to_base2.pose.pose.position.x = rb0_to_base2[0];
    Robot0_to_base2.pose.pose.position.y = rb0_to_base2[1];
    Robot0_to_base2.pose.pose.position.z = rb0_to_base2[2];
    Robot0_to_base2.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rb0_to_base2[3], rb0_to_base2[4], rb0_to_base2[5]);
    
    pub_trans0.publish(Robot0_to_base2);
}

// 发布当前机器人位姿到inc_octree话题
void publishCurrentRobotPose()
{
    if (current_robot_id == 0 || current_robot_id == 2) {
        grid_map::inc_octree poseMsg;
        poseMsg.robotID = "Base_ENU";
        
        // 从当前状态获取位姿
        poseMsg.poseX = state_point.pos[0];
        poseMsg.poseY = state_point.pos[1];
        poseMsg.poseZ = state_point.pos[2];
        
        // 将旋转矩阵转换为欧拉角
        Eigen::Vector3d euler = state_point.rot.matrix().eulerAngles(0, 1, 2);
        poseMsg.poseRoll = euler[0];
        poseMsg.posePitch = euler[1];
        poseMsg.poseYaw = euler[2];
        
        if (current_robot_id == 0) {
            pub_robot0_pose.publish(poseMsg);
        } else if (current_robot_id == 2) {
            pub_robot2_pose.publish(poseMsg);
        }
    }
}
/*** Time Log Variables ***/
int    add_point_size = 0, kdtree_delete_counter = 0;
bool   pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
int effct_feat_num_ = 0;
int vaild_points_ = 0;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0,last_timestamp_leg= -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    scan_count = 0, publish_count = 0;
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;

bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

/* 发布PX4Odometry相关 */
bool isLocalFrameInitialized = false;
Eigen::Vector3d localFrameOrigin; // 局部坐标系原点在全局坐标系下的位置
// Sophus::SO3d localFrameOriginRot; // 局部坐标系原点的旋转
Eigen::Matrix3d localFrameOriginRot;
/*********************/
int count_first_odometry=0,count_odometry_threshold=20;   //初始变换的帧数
nav_msgs::Odometry odom_all;
grid_map::inc_octree  inc_octree_;
bool pub_firstodo_en = false;

vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<double>                     time_end_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_rtk_buffer;
deque<nav_msgs::Odometry::ConstPtr> leg_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());          //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());         //畸变纠正后降采样的单帧点云，W系

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;
KD_TREE<PointType> ikdtree2;



V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// 移除extern声明，保留实际定义
/*** EKF inputs and output ***/

double package_end_time_last = 0;
Eigen::Vector3d pos_lid;  //估计的W系下的位置

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;
/*fzh*/
geometry_msgs::Pose origin_pose;
geometry_msgs::Pose current_pose_in_map;
nav_msgs::Odometry odom;

shared_ptr<Preprocess> p_pre(new Preprocess());

// 移除extern声明，保留实际定义
/***优化部分相关变量***/
float transformTobeMapped[6]; //  当前帧的位姿(world系下)

pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

nav_msgs::Path globalPath;
bool visual_ikdtree;

bool usertk;
string gnss_topic;
string gnss_heading_topic;
double rtk_time_grift = 0;
double last_timestamp_gnss = -1.0 ;
deque<nav_msgs::Odometry> gnss_buffer;
deque<double> gnss_heading_buffer;
deque<double> heading_time_buffer;
geometry_msgs::PoseStamped msg_gnss_pose;
shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
bool gnss_inited = false ;                        //  是否完成gnss初始化
double lat0,lon0,alt0;
nav_msgs::Path gps_path ;

std::mutex mtx;

string leg_topic;
bool useleg;

//重定位
// bool flag_reposition=false;
bool rtk_p0_init = false;//gps原点初始化标志位

bool need_relocal = true;
int Reposition_type;
// bool flag_lidar=false;
// bool flag_extpos=false;
bool flag_manualpos=false;
bool flag_rtkpos=false;
bool flag_rtkheading = false;
//bfsreloc
double search_radius = 10.0;
// 移除extern声明，保留实际定义
// double max_z = 0,min_z = 0;//构图过程中最大高度与最低高度
bool flag_relocbfs = false;
// pcl::PointCloud<PointType>::Ptr ds_pl_orig (new pcl::PointCloud<PointType>());//声明源点云
pcl::PointCloud<PointType>::Ptr pointcloudmap(new pcl::PointCloud<PointType>());//地图点云
// double extposx,extposy,extposz;
Eigen::Vector3d manualpos = Eigen::Vector3d::Zero();
Eigen::Quaterniond ext_q;
string loadmappath; //读取pcd的路径
string loadposepath; //读取pcd的初始经纬高
string params_filename;
int map_count = 0;
sensor_msgs::PointCloud2 globalmapmsg;
ros::Publisher pubglobalmap;
double rtk_heading;
double kf_heading;
bool rtk_vaild = false;
bool rtk_heading_vaild = false;
V3D rtk_T_wrt_Lidar(Zero3d);
vector<double> rtk2Lidar_T(3, 0.0);
ros::Publisher pubRelocal_flag,pubLocal_flag;
std::shared_ptr<plugins::RelocPlugin> reloc_plugin_ptr_; // bfs搜索功能指针

//重构ikd树
bool flag_ikdtree_initial=false;
PointType pos_last,pos_now;
pcl::PointCloud<PointType>::Ptr mapikdtree(new pcl::PointCloud<PointType>());//存储新的ikd树点云
bool flag_reikdtree =false;
std::mutex mtx_reikd;
bool map_update = false;
//int flag_thread1 = 0; //是否正在调用树进行查找：0表示空闲，1表示正在处理kdtree1,2表示正在处理kdtree2
//int flag_thread2 = 0; //是否正在更新点云：     0表示空闲，1表示正在处理kdtree1,2表示正在处理kdtree2
int status_tree1 = 0;//0表示空闲，1表示正在更新点云，2表示正在用该树查找；
int status_tree2 = 0;
int last_ikdtree = 1; //最新更新的ikdtree

//kd树
pcl::KdTreeFLANN<PointType> kdtree1;
pcl::KdTreeFLANN<PointType> kdtree2;

ros::Publisher pubSensorVaild;
ros::Publisher pubLocalizationVaild;
ros::Publisher pubICP_in;
ros::Publisher pubICP_out;
ros::Publisher pubICP_target;

bool imu_vaild = true; //imu是否有效的标志位
double imu_fault_time; //imu故障的时间

map_management MM;
//ikdtree包围盒
BoxPointType LocalMap_Points;           // ikd-tree地图立方体的2个角点
bool Localmap_Initialized = false;      // 局部地图是否初始化

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

int count_fault=0;
bool first_lidar = false;


// typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointCloud<PointType> PointCloudXYZI;
// PointCloudXYZI pl_full,pl_surf;  //todo 把livox消息类型转换到pcl 再转换到 sensor msg
//                                 //todo 为了能给bfs搜索的接口
sensor_msgs::PointCloud2 trans_cloud;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    if(!first_lidar)
        first_lidar = true;
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr); //todo 把livox消息类型转换成pcl

    lidar_buffer.clear();
    time_buffer.clear();
    time_end_buffer.clear();

    lidar_buffer.push_back(ptr);
    time_end_buffer.push_back(msg->header.stamp.toSec()+ptr->points.back().curvature/1000.0);

    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();

    pcl::toROSMsg(*ptr, trans_cloud);
    
}

// sensor_msgs::PointCloud2 overlap_cloud;
// robot_0的点云回调函数
void robot0_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    std::cout << "接收到id0的livox点云" << std::endl;
    if(current_robot_id == 0) {
        // 处理robot_0的livox点云数据
        mtx_buffer.lock();
        if(!first_lidar)
            first_lidar = true;
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr); //把livox消息类型转换成pcl

        lidar_buffer.clear();
        time_buffer.clear();
        time_end_buffer.clear();

        lidar_buffer.push_back(ptr);
        time_end_buffer.push_back(msg->header.stamp.toSec()+ptr->points.back().curvature/1000.0);

        time_buffer.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar = msg->header.stamp.toSec();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        
        std::cout << "=====ID000=====" << std::endl;
    }
}

// robot_0的IMU回调函数
void robot0_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg) {
    std::cout << "接收到id0的imu" << std::endl;
    if(current_robot_id == 0) {
        // 处理robot_0的IMU数据
        imu_cbk(msg);
    }
}

// robot_2的点云回调函数
void robot2_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    std::cout << "接收到id2的livox点云" << std::endl;
    if(current_robot_id == 2) {
        // 处理robot_2的livox点云数据
        mtx_buffer.lock();
        if(!first_lidar)
            first_lidar = true;
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr); //把livox消息类型转换成pcl

        lidar_buffer.clear();
        time_buffer.clear();
        time_end_buffer.clear();

        lidar_buffer.push_back(ptr);
        time_end_buffer.push_back(msg->header.stamp.toSec()+ptr->points.back().curvature/1000.0);

        time_buffer.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar = msg->header.stamp.toSec();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        
        std::cout << "=====ID222=====" << std::endl;
    }
}

// robot_2的IMU回调函数
void robot2_imu_cbk(const sensor_msgs::Imu::ConstPtr &msg) {
    std::cout << "接收到id2的imu" << std::endl;
    if(current_robot_id == 2) {
        // 处理robot_2的IMU数据
        imu_cbk(msg);
    }
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    //容错测试
    // count_fault++;
    // if(count_fault>=50 && count_fault<=200) return;
    mtx_buffer.lock();
    if(!first_lidar)
        first_lidar = true;
    // scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    
    // 检查是否是八叉树转换的点云（通过frame_id标识）
    if(msg->header.frame_id == "map") {
        // 直接转换，跳过预处理
        pcl::fromROSMsg(*msg, *ptr);
        cout<<"Received octree converted point cloud, skipping preprocessing"<<endl;
    } else {
        // 原有的预处理流程
        p_pre->process(msg, ptr);
    }

    //************将点云由左后上转到前左上*******************/
    // for(auto& point : ptr->points)
    // {
    //     double x_tmp = -point.y;
    //     double y_tmp = point.x;
    //     point.x = x_tmp;
    //     point.y = y_tmp;
    // }

    // for(auto& point : ptr->points)
    // {
    //     double x_tmp = -0.2588 * point.x + 0.9659 * point.y;
    //     double y_tmp = -0.9659 * point.x - 0.2588 * point.y;
    //     point.x = x_tmp;
    //     point.y = y_tmp;
    // }

    //如果雷达点云没有数据，直接return
    // if(ptr->points.size() == 0)
    if(ptr->points.size() < 100)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
        std::cout<<"没有雷达数据"<<std::endl;
    }
        
    //定位程序需要避免缓存区数据堆积
    lidar_buffer.clear();
    time_buffer.clear();
    time_end_buffer.clear();

    lidar_buffer.push_back(ptr);
    time_end_buffer.push_back(msg->header.stamp.toSec());

     cout<<"lidar size:"<<ptr->points.size()<<endl;

    time_buffer.push_back(msg->header.stamp.toSec()-ptr->points.back().curvature/1000.0);//因为rslidar的时间戳是雷达帧结束时间，所以要转成开始时间.
    last_timestamp_lidar = msg->header.stamp.toSec();
    // cout<<"lidar buff have data!!!"<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    pcl::toROSMsg(*ptr, trans_cloud);
    // overlap_cloud = *msg;
}


bool chengxin_change = false;
double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
int imu_fault = 0;
bool time_no_sync = true;
double delta_time = 0;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    // imu_fault++;
    // if(imu_fault>1000 && imu_fault<1800) return ;

    //如果imu无效，则置有效
    if(imu_vaild == false)
    {
        // imu_fault_time
        imu_vaild = true;
        ROS_INFO("IMU falut recovery! recovery time:%lf",msg_in->header.stamp.toSec()-imu_fault_time);
    }
    
    if(time_no_sync)
    {
        delta_time = ros::Time::now().toSec() - msg_in->header.stamp.toSec();
        cout<<"delta_time = "<<delta_time<<endl;
        time_no_sync = false;
    }

    // publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    if (fabs(msg->linear_acceleration.x)>1000||fabs(msg->linear_acceleration.y)>1000||fabs(msg->linear_acceleration.z)>1000||fabs(msg->angular_velocity.x)>1000||fabs(msg->angular_velocity.y)>1000||fabs(msg->angular_velocity.z)>1000)
        
    {
        cout<<"IMU数据异常!!!"<<endl;
            return;
    }

    // //*********若已对齐到雷达（左后上），则不执行下列语句*********//
    // if(!chengxin_change)
    // {
    //     msg->linear_acceleration.y = -msg->linear_acceleration.y;
    //     msg->linear_acceleration.z = -msg->linear_acceleration.z;

    //     msg->angular_velocity.y = -msg->angular_velocity.y;
    //     msg->angular_velocity.z = -msg->angular_velocity.z;
    // }
    
    
    
    // //*********左后上转前左上*********//
    // double x_acc_tmp = -msg->linear_acceleration.y;
    // double y_acc_tmp = msg->linear_acceleration.x;
    // msg->linear_acceleration.x = x_acc_tmp;
    // msg->linear_acceleration.y = y_acc_tmp;

    // double x_ang_tmp = -msg->angular_velocity.y;
    // double y_ang_tmp = msg->angular_velocity.x;
    // msg->angular_velocity.x = x_ang_tmp;
    // msg->angular_velocity.y = y_ang_tmp;

    //msg->linear_acceleration.y = -msg->linear_acceleration.y;
    //msg->linear_acceleration.z = -msg->linear_acceleration.z;
    //msg->angular_velocity.y = -msg->angular_velocity.y;
    //msg->angular_velocity.z = -msg->angular_velocity.z;

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
        imu_rtk_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    if(usertk)
    {
        imu_rtk_buffer.push_back(msg);
    } 
    //监视缓存区的大小，大小超出限制就pop掉最一开始的数据
    if(imu_buffer.size()>1000)
    {
        imu_buffer.pop_front();
        // ROS_WARN("imu buffer is too large!");
    }

    if(imu_rtk_buffer.size()>1000)
    {
        imu_rtk_buffer.pop_front();
        // ROS_WARN("imu_rtk buffer is too large!");
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void leg_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
    if(!useleg) return;
    
    nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry(*msg_in));
    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_leg)
    {
        ROS_WARN("leg loop back, clear buffer");
        leg_buffer.clear();
    }

    last_timestamp_leg = timestamp;

    leg_buffer.push_back(msg);

    if(leg_buffer.size()>1000)
    {
        leg_buffer.pop_front();
        // ROS_WARN("leg buffer is too large!");
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    
}

double lidar_mean_scantime = 0.0;
// int    scan_num = 0;
/* last_time_packed是否初始化 */
bool initialized_last_time_packed = false;
double last_time_packed=-1;
/* 数据打包的时间间隔,如果以某个传感器为打包断点,那么时间间隔就是传感器频率的倒数,unit:s */
double time_interval_packed = 0.1;//0.1
/* lidar数据异常(没有收到数据)情况下的最大等待时间,对于低频传感器通常设置为帧间隔时间的一半,
* 超过该时间数据还没到来,就认为本次打包该传感器数据异常,unit:s */
double time_wait_max_lidar = 0.10;

bool TryToFetchOneLidarFrame(MeasureGroup &meas)
{
#ifdef ENABLE_SENSOR_FAULT
    /* 如果lidar帧的时间戳晚于上一次包结束时间,说明lidar数据延迟太大,导致上一次打包时判定lidar为故障,
    * 因此需要丢弃过时的lidar帧 */
    while ((!lidar_buffer.empty()) && (time_end_buffer.front()<=last_time_packed)) 
    {
        cout<<"雷达队列长度 : "<<lidar_buffer.size()<<endl;
        cout<<"*********雷达帧尾:"<<time_end_buffer.front()<<endl;
        cout<<"*********上一次打包结束时间:"<<last_time_packed<<endl;
        cout<<"雷达帧尾 - 上一次打包结束时间"<<time_end_buffer.front()-last_time_packed<<endl;
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        time_end_buffer.pop_front();
    }
#endif   
    if (lidar_buffer.empty()) {
        return false;
    }    
    
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    time_end_buffer.pop_front();

    if (meas.lidar->points.size() <= 5) // time too little
    {
        lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        ROS_WARN("Too few input point cloud!\n");
    }
    else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
    {
        lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    }
    else
    {
        // scan_num ++;
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        // lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        lidar_mean_scantime = 0.1 ;
    }
    return true;

}

bool flag_roswarn_lidar = false;
//把当前要处理的LIDAR和IMU数据打包到meas
bool sync_packages(MeasureGroup &meas)
{
    // 添加调试输出
    cout << "=== sync_packages调试信息 ===" << endl;
    cout << "first_lidar: " << first_lidar << endl;
    cout << "imu_vaild: " << imu_vaild << endl;
    cout << "imu_buffer.size(): " << imu_buffer.size() << endl;
    cout << "last_timestamp_imu: " << last_timestamp_imu << endl;
    
    if(!first_lidar)
    {
        cout << "返回false: first_lidar未设置" << endl;
        return false;
    }
    
#ifdef ENABLE_SENSOR_FAULT
    
    /* 初始化last_time_packed为当前时间 */
    if (!initialized_last_time_packed) {
        cout<<"12334"<<endl;
        last_time_packed = ros::Time::now().toSec();
        cout<<"第一次last_time_packed:"<<last_time_packed<<endl;
        meas.package_end_time = last_time_packed;
        initialized_last_time_packed = true;
        return false;
    }
#endif
    // if (lidar_buffer.empty() || imu_buffer.empty()) {
    //     return false;
    // }
    
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        if(TryToFetchOneLidarFrame(meas))
        {
            //cout<<"828282"<<endl;
            meas.lidar_end_time = lidar_end_time;
            meas.lidar_vaild = true;   
            lidar_pushed = true;  
            package_end_time_last = meas.package_end_time;
            meas.package_end_time = lidar_end_time; 

            if(flag_roswarn_lidar == true)
            {
                ROS_INFO("Lidar Data Recovery!");
                flag_roswarn_lidar = false;
            }   
        }
        else
        {
#ifdef ENABLE_SENSOR_FAULT
            /* 计时,距离上次打包成功后,一定时间内都没有数据,代表lidar故障 */
            if ((ros::Time::now().toSec() - last_time_packed) > (time_interval_packed + time_wait_max_lidar)) 
                {
                    cout<<"故障当前时间："<<ros::Time::now().toSec()<<endl;
                    //cout<<"11111"<<endl;
                    //cout<<"imu_vaild ="<<imu_vaild<<endl;
                    //cout<<"flag_roswarn_lidar ="<<flag_roswarn_lidar<<endl;
                    // printf("  Error: SyncPackages: Lidar is malfunctioning!\n");
                    // cout<<"delta time:"<<ros::Time::now().toSec() - last_time_packed<<endl;
                    if(flag_roswarn_lidar == false && imu_vaild)
                    {
                        ROS_ERROR("Lidar Data Error !");
                        flag_roswarn_lidar = true;
                    }

                    lidar_pushed = true;
                    //雷达数据指向空指针
                    
                    PointCloudXYZI::Ptr  point_cloud_tmp(new PointCloudXYZI());
                    meas.lidar = point_cloud_tmp;
                    meas.lidar_vaild = false;
                    //测试一下imu递推是不是这个的问题
                    meas.lidar_beg_time = meas.package_end_time;
                    meas.lidar_end_time = meas.package_end_time + time_interval_packed;
                    /* 如果没有lidar数据,每次打包结束时间就是上一次加上time_interval_packed */
                    package_end_time_last = meas.package_end_time;
                    meas.package_end_time += time_interval_packed;
                }
#endif            
        }
    }
    // cout<<"1.1"<<endl;   
    /* 如果lidar还没取出(或判断为故障) */
    if (!lidar_pushed)
    return false;

    // if (last_timestamp_imu < lidar_end_time)
    // {
    //     return false;
    // }  

    if(imu_vaild)
    {
        //判断imu异常(lidar打包后0.5s都没有接收到imu数据)
        if((ros::Time::now().toSec() - meas.package_end_time)>0.5)
        {
            cout<<"ros::Time::now().toSec() = "<<ros::Time::now().toSec()<<endl;
            cout<<"dt:"<<ros::Time::now().toSec()-meas.package_end_time<<endl;
            ROS_ERROR("IMU data error!!!");
            imu_vaild = false;
            lidar_pushed = false; //imu异常需要把lidar pop出去，否则会一直卡在这里
            if(imu_buffer.empty())
            {
                cout<<"imu队列空的"<<endl;
                imu_fault_time = meas.package_end_time;
            }
            else
                imu_fault_time = imu_buffer.back()->header.stamp.toSec();
            // flg_exit = true;
            return false;
        }
    }
    else
    {
        lidar_pushed = false;
        return false;
    }

    //cout<<"last_timestamp_imu = "<<last_timestamp_imu<<endl;
    //cout<<"meas.package_end_time "<<meas.package_end_time<<endl;

    //确保具备提取imu数据的条件，即最后一个imu时间大于包的结束时间
    if (last_timestamp_imu <= meas.package_end_time)
        return false;

    //确保具备提取leg数据的条件，即最后一个leg时间大于包的结束时间
    // if(useleg)
    // {
    //     if (last_timestamp_leg <= meas.package_end_time)
    //     return false;
    // }
    // cout<<"1.2"<<endl;  
    /*** push imu data, and pop from imu buffer ***/
    if(imu_buffer.empty()) 
        return false;

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    // while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    while ((!imu_buffer.empty()) && (imu_time <= meas.package_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        // if(meas.package_end_time - imu_time > 0.2)
        // {
        //     imu_buffer.pop_front();
        //     continue;
        // }           
        // if(imu_time > lidar_end_time) break;
        if(imu_time > meas.package_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    // cout<<"imu_vaild:"<<!meas.imu.empty()<<endl;

    /*** push leg data, and pop from leg buffer ***/
    if(!leg_buffer.empty() && useleg)
    {
        double leg_time = leg_buffer.front()->header.stamp.toSec();
        meas.leg.clear();
        while ((!leg_buffer.empty()) && (leg_time <= meas.package_end_time))
        {
            leg_time = leg_buffer.front()->header.stamp.toSec();
            if(leg_time > meas.package_end_time) break;
            meas.leg.push_back(leg_buffer.front());
            leg_buffer.pop_front();
        }
    }
    // cout<<"1.3"<<endl;  
    //判断leg数据是否有效
    if(!meas.leg.empty())
    {
        meas.leg_vaild = true;
    }
    else
    {
        meas.leg_vaild = false;
    }

    // cout<<"leg_vaild:"<<meas.leg_vaild<<endl;

    lidar_pushed = false;

    last_time_packed = meas.package_end_time;
    // cout<<"1.4"<<endl;
    return true;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);
    // V3D p_global(state_point.rot.matrix() * p_body + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}


void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix()*p_body_lidar + state_point.offset_T_L_I);
    // V3D p_body_imu = p_body_lidar;

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}


PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull_)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull_.publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    // publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];  
}

// void setPose(nav_msgs::Odometry& odom, const Eigen::Vector3d& localPos, const Sophus::SO3d& localRot) 
// {    
//     // 设置局部坐标系下的位置
//     odom.pose.pose.position.x = localPos.x();
//     odom.pose.pose.position.y = localPos.y();
//     odom.pose.pose.position.z = localPos.z();
    
//     // 设置局部坐标系下的旋转
//     Eigen::Quaterniond q(localRot.matrix());
//     odom.pose.pose.orientation.x = q.coeffs()[0];
//     odom.pose.pose.orientation.y = q.coeffs()[1];
//     odom.pose.pose.orientation.z = q.coeffs()[2];
//     odom.pose.pose.orientation.w = q.coeffs()[3];
// }

void setPose(nav_msgs::Odometry& odom, const Eigen::Vector3d& localPos, const Eigen::Matrix3d& localRot) 
{    
    // 设置局部坐标系下的位置
    odom.pose.pose.position.x = localPos.x();
    odom.pose.pose.position.y = localPos.y();
    odom.pose.pose.position.z = localPos.z();
    
    // 设置局部坐标系下的旋转
    Eigen::Quaterniond q(localRot);
    odom.pose.pose.orientation.x = q.coeffs()[0];
    odom.pose.pose.orientation.y = q.coeffs()[1];
    odom.pose.pose.orientation.z = q.coeffs()[2];
    odom.pose.pose.orientation.w = q.coeffs()[3];
}


void publish_odometry(const ros::Publisher & pubOdomAftMapped, const ros::Publisher & pubOdomFromOrigin)
{
    // 发布先验地图坐标系下odom
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    // 发布局部坐标系下odom
    if (!isLocalFrameInitialized)
    {	
        localFrameOrigin = state_point.pos;
        localFrameOriginRot = state_point.rot.matrix();
    }
    double z = state_point.pos(2) - localFrameOrigin(2);
    nav_msgs::Odometry odomFromOrigin;
    odomFromOrigin.header.frame_id = "camera_init";
    odomFromOrigin.header.stamp = ros::Time().fromSec(lidar_end_time);
    
    Eigen::Vector3d localPos = localFrameOriginRot.inverse() * (state_point.pos - localFrameOrigin);
    // Sophus::SO3d localRot = localFrameOriginRot.inverse() * state_point.rot;
    // setPose(odomFromOrigin, localPos, localRot);
    Eigen::Matrix3d localRot = localFrameOriginRot.inverse() * state_point.rot.matrix();
    localPos(2) = z;
    setPose(odomFromOrigin, localPos, localRot);
    geometry_msgs::Quaternion RPYQuat = odomFromOrigin.pose.pose.orientation;
    geometry_msgs::Quaternion RPYQuat_ = odomAftMapped.pose.pose.orientation;
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(RPYQuat_.x, RPYQuat_.y, RPYQuat_.z, RPYQuat_.w)).getRPY(roll, pitch, yaw);
	// std::cout<<"Odometry:"<<std::endl;
    // std::cout<<"roll  = "<<roll*180/3.14<<std::endl;
	// std::cout<<"pitch = "<<pitch*180/3.14<<std::endl;
    // std::cout<<"yaw   = "<<yaw*180/3.14<<std::endl;
    tf::Matrix3x3(tf::Quaternion(RPYQuat.x, RPYQuat.y, RPYQuat.z, RPYQuat.w)).getRPY(roll, pitch, yaw);
	// std::cout<<"PX4_Odometry:"<<std::endl;
    // std::cout<<"roll  = "<<roll*180/3.14<<std::endl;
	// std::cout<<"pitch = "<<pitch*180/3.14<<std::endl;
    // std::cout<<"yaw   = "<<yaw*180/3.14<<std::endl;
    if(!isLocalFrameInitialized){
        if(abs(roll) < 0.5 && abs(pitch) < 0.5 && abs(yaw) < 0.5 && abs(localPos(0)) < 0.2 && abs(localPos(1)) < 0.2 && abs(localPos(2)) < 0.2){
            isLocalFrameInitialized = true;
            std::cout << "初始化局部坐标系成功！！！" << std::endl;
            std::cout << "初始化局部坐标系成功！！！" << std::endl;
            std::cout << "初始化局部坐标系成功！！！" << std::endl;
            std::cout << "初始化局部坐标系成功！！！" << std::endl;
            std::cout << "初始化局部坐标系成功！！！" << std::endl;
        }
    }
    
    pubOdomFromOrigin.publish(odomFromOrigin);
}

// void publish_odometry(const ros::Publisher & pubOdomAftMapped, const ros::Publisher & pubOdomFromOrigin)
// {
//     // 发布先验地图坐标系下odom
//     odomAftMapped.header.frame_id = "camera_init";
//     odomAftMapped.child_frame_id = "body";
//     odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
//     set_posestamp(odomAftMapped.pose);
//     pubOdomAftMapped.publish(odomAftMapped);
//     // 发布以起点为原点的坐标系下odom
//     nav_msgs::Odometry odomFromOrigin;
//     tf::Quaternion quaternion;
//     Eigen::Matrix3d rotationMatrix = state_point.rot.matrix();
//     Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(0, 1, 2);
//     float x, y, z, roll, pitch, yaw;
//     Eigen::Affine3f start;
//     roll = euler_angles(0);
//     pitch = euler_angles(1);
//     yaw = euler_angles(2);
//     if(!odo_init_flag){
//         odo_init_flag = true;
//         Eigen::Quaterniond q(state_point.rot.unit_quaternion());
//         start = pcl::getTransformation(
//             state_point.pos(0),state_point.pos(1),state_point.pos(2),
//             roll,pitch,yaw);
//         // 发布给px4
//         odomFromOrigin.pose.pose.position.x = 0;
//         odomFromOrigin.pose.pose.position.y = 0;
//         odomFromOrigin.pose.pose.position.z = 0;
//         odomFromOrigin.pose.pose.orientation.x = quaternion.x();
//         odomFromOrigin.pose.pose.orientation.y = quaternion.y();
//         odomFromOrigin.pose.pose.orientation.z = quaternion.z();
//         odomFromOrigin.pose.pose.orientation.w = quaternion.w();
//     }
//     else{
//         Eigen::Quaterniond q(state_point.rot.unit_quaternion());
//         Eigen::Affine3f now = pcl::getTransformation(
//             state_point.pos(0),state_point.pos(1),state_point.pos(2),
//             roll,pitch,yaw);
//         Eigen::Affine3f trans =start.inverse() * now;
//         pcl::getTranslationAndEulerAngles(trans, x, y, z, roll, pitch, yaw);
//         odomFromOrigin.pose.pose.position.x = x;
//         odomFromOrigin.pose.pose.position.y = y;
//         odomFromOrigin.pose.pose.position.z = z;
//         quaternion.setRPY(roll, pitch, yaw);
//         odomFromOrigin.pose.pose.orientation.x = quaternion.x();
//         odomFromOrigin.pose.pose.orientation.y = quaternion.y();
//         odomFromOrigin.pose.pose.orientation.z = quaternion.z();
//         odomFromOrigin.pose.pose.orientation.w = quaternion.w();
//     }
//     odomFromOrigin.header.stamp = ros::Time::now();
//     odomFromOrigin.header.frame_id = "camera_init";  
//     odomFromOrigin.child_frame_id = "body";
//     pubOdomFromOrigin.publish(odomFromOrigin);

//     auto P = kf.get_P();
//     for (int i = 0; i < 6; i ++)
//     {
//         int k = i < 3 ? i + 3 : i - 3;
//         odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
//         odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
//         odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
//         odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
//         odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
//         odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
//     }

//     static tf::TransformBroadcaster br;
//     tf::Transform                   transform;
//     tf::Quaternion                  q;
//     transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
//                                     odomAftMapped.pose.pose.position.y, \
//                                     odomAftMapped.pose.pose.position.z));
//     q.setW(odomAftMapped.pose.pose.orientation.w);
//     q.setX(odomAftMapped.pose.pose.orientation.x);
//     q.setY(odomAftMapped.pose.pose.orientation.y);
//     q.setZ(odomAftMapped.pose.pose.orientation.z);
//     transform.setRotation( q );
//     br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
// }

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        //队列长度超过100，移除最早的path
        if(path.poses.size()>100)
        {
            path.poses.erase(path.poses.begin());
        }
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

//发布更新后的轨迹
void publish_path_update(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
    string odometryFrame = "camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        if (kkk % 10 == 0)
        {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
}

//  发布gnss 轨迹
void publish_gnss_path(const ros::Publisher pubPath)
{
    gps_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        pubPath.publish(gps_path);
    }
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

//  eulerAngle 2 Quaterniond
Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q ;            //   四元数 q 和 -q 是相等的
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll ;
    q.normalize();
    return q ;
}

/**
 * 两点之间距离
 */
float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void gnss_cbk(const na_localization::rtk_pos_raw::ConstPtr& msg_in) //todo yaml更改 usertk
{
    //判断是否使用rtk融合
    if (usertk==false)
        return;
    //判断gps是否有效

     if (msg_in->pos_type!=50)  //todo =50 rtk 模式  //todo =16 gnss单点定位
         return;

     if (msg_in->hgt_std_dev > 0.05) //todo 同理 这个判定是rtk用的判定
         return;

    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec()+rtk_time_grift;

    mtx_buffer.lock();
    // 没有进行时间纠正
    if (timestamp < last_timestamp_gnss)
    {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = msg_in->header.stamp.toSec()+rtk_time_grift;
    gnss_data.status = msg_in->pos_type;
    gnss_data.service = 0;

    //通过gps_qual给出协方差
    double posecov;
    posecov=0.05*0.05;

    gnss_data.pose_cov[0] = posecov;
    gnss_data.pose_cov[1] = posecov;
    gnss_data.pose_cov[2] = 2.0*posecov;

    mtx_buffer.unlock();
    double Lat = msg_in->lat, Lon = msg_in->lon;
    int zone;
    bool northp;
    double x_,y_,z_;
    GeographicLib::UTMUPS::Forward(Lat, Lon, zone, northp, x_, y_);
    z_ = msg_in->hgt;
    //dsq变电站参数
    // x_ -= 669369.25000;
    // y_ -= 3528527.25000;
    // z_ -= 17.28488;
    x_ -= 669368.31250;
    y_ -= 3528523.75000;
    z_ -= 20.90000;

    M3D rotation_tmp;
    V3D drift_tmp,position_tmp;
    // dsq变电站参数
    // drift_tmp << -196.486,153.127,3.44237;
    // rotation_tmp << 0.0972805, -0.995257, -0.000182575,
    //                 0.99526, 0.0972791, -0.000187838,
    //                 0.00020471, -0.000163439, 1;
    drift_tmp << -194.23625,151.38426,3.97583;
    rotation_tmp << 0.10763, -0.99419, -0.00106,
                    0.99414, 0.10763, -0.01050,
                    0.01055, -0.00008, 0.99995;
    position_tmp << x_,y_,z_;
    position_tmp = rotation_tmp * position_tmp + drift_tmp;
    x_ = position_tmp[0];
    y_ = position_tmp[1];
    z_ = position_tmp[2];
    // gnss_data.UpdateXYZ(msg_in->lat, msg_in->lon, msg_in->hgt) ;             //  WGS84 -> ENU  ???  调试结果好像是 NED 北东地
    nav_msgs::Odometry gnss_data_enu ;
    // add new message to buffer:
    gnss_data_enu.header.stamp = ros::Time().fromSec(gnss_data.time);
    gnss_data_enu.pose.pose.position.x =  x_ ;  //gnss_data.local_E ;   北
    gnss_data_enu.pose.pose.position.y =  y_ ;  //gnss_data.local_N;    东
    gnss_data_enu.pose.pose.position.z =  z_;  //  地

    gnss_data_enu.pose.covariance[0] = posecov ;
    gnss_data_enu.pose.covariance[7] = posecov ;
    gnss_data_enu.pose.covariance[14] = 2.0*posecov ;

    gnss_buffer.push_back(gnss_data_enu);
    if(gnss_buffer.size()>50)
    {
        gnss_buffer.pop_front();
        // ROS_WARN("gnss buffer is too large!");
    }
    // visial gnss path in rviz:
    msg_gnss_pose.header.frame_id = "camera_init";
    msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

    Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();

    gnss_pose(0,3) = x_ ;
    gnss_pose(1,3) = y_ ;
    gnss_pose(2,3) = z_ ; 


    msg_gnss_pose.pose.position.x = gnss_pose(0,3) ;  
    msg_gnss_pose.pose.position.y = gnss_pose(1,3) ;
    msg_gnss_pose.pose.position.z = gnss_pose(2,3) ;
    
    //队列长度超过100，移除最早的path
    if(gps_path.poses.size()>100)
    {
        gps_path.poses.erase(gps_path.poses.begin());
    }
    gps_path.poses.push_back(msg_gnss_pose); 

    if(need_relocal)
    {
        if(flag_rtkpos == false)
        {
            flag_rtkpos = true;
        }
    } 

}

void gnss_cbk_sensor_msgs_2enu(const sensor_msgs::NavSatFix::ConstPtr& msg_in) 
{
    //判断是否使用rtk融合
    if (usertk==false)
        return;
    //判断gps是否有效
    if (msg_in->status.status!=4 && msg_in->status.status!=5)
        return;
    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec()+rtk_time_grift ;

    mtx_buffer.lock();
    // 没有进行时间纠正
    if (timestamp < last_timestamp_gnss)
    {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = msg_in->header.stamp.toSec()+rtk_time_grift ;
    gnss_data.status = msg_in->status.status;
    gnss_data.service = 0;

    //通过gps_qual给出协方差
    double posecov;
    posecov=0.05*0.05;

    gnss_data.pose_cov[0] = posecov;
    gnss_data.pose_cov[1] = posecov;
    gnss_data.pose_cov[2] = 2.0*posecov;

    mtx_buffer.unlock();

    
    Eigen::Vector3d enupose;
    enupose = gnss2enu->LLAToENU(msg_in->longitude*M_PI/180,msg_in->latitude*M_PI/180,msg_in->altitude);
    cout<<"enupose:::"<<enupose[0]<<","<<enupose[1]<<","<<enupose[2]<<endl;
    
    
    nav_msgs::Odometry gnss_data_enu ;
    // add new message to buffer:
    gnss_data_enu.header.stamp = ros::Time().fromSec(gnss_data.time);
    gnss_data_enu.pose.pose.position.x =  enupose[0] ;  //gnss_data.local_E ;   北
    gnss_data_enu.pose.pose.position.y =  enupose[1] ;  //gnss_data.local_N;    东
    gnss_data_enu.pose.pose.position.z =  enupose[2];  //  地

    gnss_data_enu.pose.covariance[0] = posecov ;
    gnss_data_enu.pose.covariance[7] = posecov ;
    gnss_data_enu.pose.covariance[14] = 2.0*posecov ;

    gnss_buffer.push_back(gnss_data_enu);
    if(gnss_buffer.size()>50)
    {
        gnss_buffer.pop_front();
        // ROS_WARN("gnss buffer is too large!");
    }
    // visial gnss path in rviz:
    msg_gnss_pose.header.frame_id = "camera_init";
    msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

    Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();

    gnss_pose(0,3) = enupose[0] ;
    gnss_pose(1,3) = enupose[1] ;
    gnss_pose(2,3) = enupose[2] ; 


    msg_gnss_pose.pose.position.x = gnss_pose(0,3) ;  
    msg_gnss_pose.pose.position.y = gnss_pose(1,3) ;
    msg_gnss_pose.pose.position.z = gnss_pose(2,3) ;
    
    //队列长度超过100，移除最早的path
    if(gps_path.poses.size()>100)
    {
        gps_path.poses.erase(gps_path.poses.begin());
    }
    gps_path.poses.push_back(msg_gnss_pose); 

    if(need_relocal)
    {
        if(flag_rtkpos == false)
        {
            flag_rtkpos = true;
        }
    } 

}

void gnss_heading_cbk(const na_localization::rtk_heading_raw::ConstPtr& msg_in)
{
    //如果不是固定解或不使用rtk的航向,return;
    if(msg_in->pos_type != 50)
    {
        return ;
    }
    //cout<<"1111"<<endl;
    rtk_heading = 360.0 - msg_in->heading + 87.0;


    if(rtk_heading<0)
    {
        rtk_heading += 360.0;
    }
    else if(rtk_heading>360.0)
    {
        rtk_heading -= 360;
    }

    rtk_heading = rtk_heading * M_PI / 180.0;

    gnss_heading_buffer.push_back(rtk_heading);
    heading_time_buffer.push_back(msg_in->header.stamp.toSec());
    if(gnss_heading_buffer.size()>50)
    {
        gnss_heading_buffer.pop_front();
        heading_time_buffer.pop_front();
        // ROS_WARN("gnss_heading_buffer is too large!");
    }
    if(need_relocal)
    {
        if(flag_rtkheading == false)
        {
            flag_rtkheading = true;
        }
    }
}

void ManualPos_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg_in)
{
    // if (Reposition_type!=2)
    //     return; 

    if (flag_manualpos==false)
    {
       manualpos[0]=msg_in->pose.position.x;
       manualpos[1]=msg_in->pose.position.y;
       manualpos[2]=msg_in->pose.position.z; 

       ext_q.x()=msg_in->pose.orientation.x;
       ext_q.y()=msg_in->pose.orientation.y;
       ext_q.z()=msg_in->pose.orientation.z;
       ext_q.w()=msg_in->pose.orientation.w;
       need_relocal = true;
       flag_manualpos=true;


       cout<<"Receive External Position Successful!"<<endl;
       cout<<manualpos[0]<<" "<<manualpos[1]<<" "<<manualpos[2]<<endl;
    } 

}

void manualpos_reposition()
{
    //将标志位置false;
    flag_manualpos = false;

    double min_score=100.0;
    Eigen::Matrix4f transformation=Eigen::Matrix4f::Identity();

    pcl::PointCloud<PointType>::Ptr mapreposition(new pcl::PointCloud<PointType>());
    PointType pos_repos;

    pos_repos.x=manualpos[0];
    pos_repos.y=manualpos[1];
    pos_repos.z=manualpos[2];
    //初始匹配的地图，半径设置为80m
    for(int i=0;i<(int)pointcloudmap->size();i++)
    {
        if (pointDistance(pointcloudmap->points[i], pos_repos) > 80.0)//80
        continue;
                        
        mapreposition->push_back(pointcloudmap->points[i]);
    }

    if(mapreposition->points.size() < 100)
    {
        ROS_WARN("Invaild Position");
        return;
    }


    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=ext_q.toRotationMatrix();
    // rotation_matrix=Angle2Matrix(yaw0);`
    Eigen::Matrix4f preTF=Matrix4f::Identity();

    preTF.block<3,3>(0,0)=rotation_matrix.cast<float>();
    preTF(0,3)=(float)manualpos[0];
    preTF(1,3)=(float)manualpos[1];
    preTF(2,3)=(float)manualpos[2];
    Eigen::Vector3d transpose;
    transpose<< preTF(0,3),preTF(1,3),preTF(2,3);

    // pcl::PointCloud<PointCloudXYZI> pl_orig;
    PointCloudXYZI::Ptr pl_orig (new PointCloudXYZI);//声明源点云
    pcl::PointCloud<PointType>::Ptr ds_pl_orig (new pcl::PointCloud<PointType>());//声明源点云
    pcl::PointCloud<PointType>::Ptr test_ds_pl_orig (new pcl::PointCloud<PointType>());
    //如果没有雷达点云数据，直接return
    if(!first_lidar) 
    {
        cout<<"lidar no points"<<endl;
        return;
    }

    pl_orig = Measures.lidar;
    //cout<<"point cloud size:"<<pl_orig->points.size()<<endl;
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(pl_orig);
    sor.setLeafSize(0.1f,0.1f,0.1f);
    sor.filter(*ds_pl_orig);

    // flag_lidar=true;
    // cout<<"lidar ready!"<<endl;

    // cout<<preTF<<endl;
    //调用icp匹配，得到精位姿
    *test_ds_pl_orig = *ds_pl_orig;
    for(auto &point:test_ds_pl_orig->points )
    {
        Eigen::Vector3d pointpose;
        pointpose<<point.x,point.y,point.z;
        pointpose = rotation_matrix*pointpose + transpose;
        point.x = pointpose[0];
        point.y = pointpose[1];
        point.z = pointpose[2];
    }
    sensor_msgs::PointCloud2 ICP_in_cloud;
    pcl::toROSMsg(*test_ds_pl_orig, ICP_in_cloud);
    ICP_in_cloud.header.stamp = ros::Time().fromSec(lidar_end_time);
    ICP_in_cloud.header.frame_id = "camera_init";
    pubICP_in.publish(ICP_in_cloud);


    sensor_msgs::PointCloud2 ICP_target_cloud;
    pcl::toROSMsg(*mapreposition, ICP_target_cloud);
    ICP_target_cloud.header.stamp = ros::Time().fromSec(lidar_end_time);
    ICP_target_cloud.header.frame_id = "camera_init";
    pubICP_target.publish(ICP_target_cloud);


    pcl::IterativeClosestPoint<PointType,PointType> icp1;
    icp1.setInputSource(ds_pl_orig);
    icp1.setInputTarget(mapreposition);
    icp1.setMaxCorrespondenceDistance (150);
    icp1.setMaximumIterations (20);
    icp1.setTransformationEpsilon (1e-6);
    icp1.setEuclideanFitnessEpsilon (1e-6);
    icp1.setRANSACIterations(0);
    pcl::PointCloud<PointType> Final;
    icp1.align(Final,preTF);

    sensor_msgs::PointCloud2 ICP_out_cloud;
    pcl::toROSMsg(Final, ICP_out_cloud);
    ICP_out_cloud.header.stamp = ros::Time().fromSec(lidar_end_time);
    ICP_out_cloud.header.frame_id = "camera_init";
    pubICP_out.publish(ICP_out_cloud);

    //todo NDT
    // pcl::NormalDistributionsTransform<PointType,PointType> ndt;  //todo NDT
    // ndt.setInputSource(ds_pl_orig);
    // ndt.setInputTarget(mapreposition);
    // ndt.setStepSize(0.1);
    // ndt.setResolution(0.5);
    // ndt.setMaximumIterations(30);
    // ndt.setTransformationEpsilon(0.01);
    // pcl::PointCloud<PointType> Final;
    // ndt.align(Final,preTF);
    // transformation = ndt.getFinalTransformation();

    transformation = icp1.getFinalTransformation();
    // cout<<"score:"<<min_score<<endl;
    cout<<"score:"<<icp1.getFitnessScore()<<endl;
    // cout<<"score:"<<ndt.getFitnessScore()<<endl;
    //ICP匹配度太差，重定位失败
    std_msgs::Bool Relocal_flag_msg;
    if(icp1.getFitnessScore() > 1.0)
    {
        
        Relocal_flag_msg.data = false;
        pubRelocal_flag.publish(Relocal_flag_msg);
        ROS_WARN("ICP Score is too big!");
        return;
    }
    Relocal_flag_msg.data = true;
    pubRelocal_flag.publish(Relocal_flag_msg);
    // if(ndt.getFitnessScore() > 1.0)
    // {
    //     ROS_WARN("NDT Score is too big!");
    //     return;
    // }
    cout<<transformation<<endl;

    //规范一下旋转矩阵，否则Sophus库会出问题
    Eigen::Matrix3d tmp_matrix =transformation.block<3,3>(0,0).cast<double>();
    Eigen::Quaterniond tmp_quat= Eigen::Quaterniond(tmp_matrix);
    tmp_matrix = tmp_quat.normalized().toRotationMatrix();
    state_point = state_point_last;
    state_point.rot=Sophus::SO3d(tmp_matrix);
    state_point.pos=transformation.block<3,1>(0,3).cast<double>();
    kf.change_x(state_point);
    cout<<"重定位完成！"<<endl;
    need_relocal = false;
}

bool read_keytxt = false;

void manualpos_reposition2()
{
    //将标志位置false;
    flag_manualpos = false;
    static double high = min_z;
    //double min_score=100.0;
    Eigen::Matrix4f transformation=Eigen::Matrix4f::Identity();

    pcl::PointCloud<PointType>::Ptr mapreposition(new pcl::PointCloud<PointType>());
    PointType pos_repos;

    pos_repos.x=manualpos[0];
    pos_repos.y=manualpos[1];
    pos_repos.z= high;
    
    
    reloc_plugin_ptr_->readLidar(trans_cloud);  
    /*
    if(!read_keytxt)
    {
        string keyposepath = "/home/nrc/na/na_localization_lslidar/src/na_localization/PCD/lidar_path.txt";
        std::ifstream inputFile(keyposepath);
        double time, xx, yy,zz;
        double min_dis = 10;
        string line;
        if(!inputFile.is_open())
        {
            cout<<"无法打开文件"<<endl;
        }
        while(inputFile >> time>> xx >> yy >> zz);
        {
            cout<<"121"<<endl;
            

                cout<<"time = "<<time<<endl;
                    cout<<"xx = "<<xx<<"yy = "<<yy<<"zz  ="<<zz<<endl;
                    if(sqrt(abs(pos_repos.x - xx)* abs(pos_repos.x - xx)+abs(pos_repos.y - yy)* abs(pos_repos.y - yy))< min_dis)
                    {
                        
                        pos_repos.z=zz;
                        min_dis = sqrt(abs(pos_repos.x - xx)* abs(pos_repos.x - xx)+abs(pos_repos.y - yy)* abs(pos_repos.y - yy));
                    }
                    
            
        }
        inputFile.close();
        if(pos_repos.z == 0)
        {
            cout<<"没有合适的关键帧位姿"<<endl;
        }
        read_keytxt = true;
    }
    */
    cout<<"pos_repos.z = "<<pos_repos.z<<endl;

    cerr << "manu local relco call !!!" << endl;

    //search_radius = 5.0;
    cout<<"位姿可能半径： "<<search_radius<<endl;
    bool reloSuccess_ = false;
    utils::Pose init_pose(pos_repos.x, pos_repos.y, pos_repos.z, 0, 0, 0);

    reloSuccess_ = reloc_plugin_ptr_->localRelocByBfs(init_pose, search_radius);
    //reloSuccess_ = reloc_plugin_ptr_->relocBySC();
    if(reloSuccess_)
        {
            utils::Pose reloc_pose = reloc_plugin_ptr_->getRelocPose();
            state_point = kf.get_x(); 
            state_point.rot=Sophus::SO3d(reloc_pose.q_);
            state_point.pos=reloc_pose.t_;

            kf.change_x(state_point);
            need_relocal = false;
            
            cout<<"手动局部重定位成功"<<endl;
        }
        else
        {
            
            high +=1.0;
            if(high >max_z)
            {
                cout<<"手动局部重定位失败"<<endl;
                high = min_z;
            }
                
        }

    
}


void rtk_reloc()
{
    //将标志位置false;
    flag_rtkpos = false; //todo 仅使用rtk三维坐标信息 不使用航向信息

    //cout << 1111 << endl;

    nav_msgs::Odometry gnss_tmp = gnss_buffer.back(); //Odometry数据类型  把GNNS数据存储内容放进去
    PointType pos_repos; //pointXYZ类型
    pos_repos.x = gnss_tmp.pose.pose.position.x; //point类型 xyz 等于 gnss获取的xyz
    pos_repos.y = gnss_tmp.pose.pose.position.y;
    pos_repos.z = gnss_tmp.pose.pose.position.z;

    if(first_lidar == false)
            return;

    reloc_plugin_ptr_->readLidar(trans_cloud);  


    cerr << " single GPS local relco call !!!" << endl;

    //search_radius = 10.0;
    cout<<"位姿可能半径： "<<search_radius<<endl;
    bool reloSuccess_ = false;
    utils::Pose init_pose(pos_repos.x, pos_repos.y, 0.0, 0, 0, 0);

    reloSuccess_ = reloc_plugin_ptr_->localRelocByBfs(init_pose, search_radius);

    if(reloSuccess_)
        {
            utils::Pose reloc_pose = reloc_plugin_ptr_->getRelocPose();
            state_point = kf.get_x(); 
            state_point.rot=Sophus::SO3d(reloc_pose.q_);
            state_point.pos=reloc_pose.t_;

            kf.change_x(state_point);
            need_relocal = false;

            cout<<"局部重定位成功"<<endl;
        }
        else
        {
            cout<<"局部重定位失败"<<endl;
        }
}


void rtk_reposition()    //todo 2024.1.4 use before
{
    //将标志位置false;
    flag_rtkpos = false;
    flag_rtkheading = false;

    pcl::PointCloud<PointType>::Ptr mapreposition(new pcl::PointCloud<PointType>()); //点云类型智能指针 
    PointType pos_repos; //pointXYZ类型
    nav_msgs::Odometry gnss_tmp = gnss_buffer.back(); //Odometry数据类型  把GNNS数据存储内容放进去
    pos_repos.x = gnss_tmp.pose.pose.position.x; //point类型 xyz 等于 gnss获取的xyz
    pos_repos.y = gnss_tmp.pose.pose.position.y;
    pos_repos.z = gnss_tmp.pose.pose.position.z;
    
    //初始匹配的地图，半径设置为80m
    for(int i=0;i<(int)pointcloudmap->size();i++)
    {
        if (pointDistance(pointcloudmap->points[i], pos_repos) > 80.0)//80
        continue;
                        
        mapreposition->push_back(pointcloudmap->points[i]);
    }

    //初始位置错误导致点云数量太少
    if(mapreposition->points.size() < 100)
    {
        ROS_WARN("Invaild Position");
        return;
    }

    Eigen::AngleAxisd rot_z_btol(rtk_heading, Eigen::Vector3d::UnitZ());//旋转向量 （旋转角 旋转轴 只用z ）
    Eigen::Matrix3d rotation_matrix = rot_z_btol.matrix();//矩阵化
    Eigen::Matrix4f preTF=Matrix4f::Identity(); //初始化四维单位矩阵

    preTF.block<3,3>(0,0)=rotation_matrix.cast<float>();
    preTF(0,3)=(float)pos_repos.x; //第一列第四个
    preTF(1,3)=(float)pos_repos.y;
    preTF(2,3)=(float)pos_repos.z;

    PointCloudXYZI::Ptr pl_orig (new PointCloudXYZI);//声明源点云
    pcl::PointCloud<PointType>::Ptr ds_pl_orig (new pcl::PointCloud<PointType>());//声明源点云
    //如果没有雷达点云数据，直接return
    if(!first_lidar) 
    {
        cout<<"lidar no points"<<endl;
        return;
    }

    pl_orig = Measures.lidar;
    if(pl_orig->points.size()<5)
    return;
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(pl_orig);
    sor.setLeafSize(0.1f,0.1f,0.1f);
    sor.filter(*ds_pl_orig);

    //调用icp匹配，得到精位姿
    pcl::IterativeClosestPoint<PointType,PointType> icp1;
    icp1.setInputSource(ds_pl_orig);
    icp1.setInputTarget(mapreposition);
    icp1.setMaxCorrespondenceDistance (150);
    icp1.setMaximumIterations (20);
    icp1.setTransformationEpsilon (1e-6);
    icp1.setEuclideanFitnessEpsilon (1e-6);
    icp1.setRANSACIterations(0);
    pcl::PointCloud<PointType> Final;
    icp1.align(Final,preTF);

    cout<<"score:"<<icp1.getFitnessScore()<<endl;
    //ICP匹配度太差，重定位失败
    if(icp1.getFitnessScore() > 1.0)
    {
        ROS_WARN("ICP Score is too big!");
        return;
    }

    Eigen::Matrix4f transformation=Eigen::Matrix4f::Identity();
    transformation = icp1.getFinalTransformation();

    Eigen::Matrix3d tmp_matrix =transformation.block<3,3>(0,0).cast<double>();
    Eigen::Quaterniond tmp_quat= Eigen::Quaterniond(tmp_matrix);
    tmp_matrix = tmp_quat.normalized().toRotationMatrix();
    state_point = state_point_last;
    state_point.rot=Sophus::SO3d(tmp_matrix);
    state_point.pos=transformation.block<3,1>(0,3).cast<double>();

    kf.change_x(state_point);
    cout<<"重定位完成！"<<endl;
    need_relocal = false;
}



//重定位
void ReLocalization()
{
    //判断是否需要重定位
    if(!need_relocal) 
    {
        return;
    }

    //定期发布全局地图      
    if(map_count<100)
    {
        map_count++;
    }
    else
    {
        pubglobalmap.publish(globalmapmsg);
        map_count=0;
    }

    //选择重定位方式
    if(flag_rtkpos==true )//rtk重定位   && flag_rtkheading==true
    {
        // rtk_reposition(); //todo rtk重定位
        // return ;
        rtk_reloc();  //todo gnss 单点重定位
    }
    else if(flag_manualpos==true)//手动重定位 
    {
         for(int i = 0;i<max_z-min_z+1;i++)//遍历可能的高度值
         if(need_relocal)
        manualpos_reposition2();
        // return ;
    }
    

    //重定位完成
    if(!need_relocal)
    {
        //ikdtree初始化完成，说明本次重定位是定位丢失后的重定位，需要重置ikdtree和包围盒
        if(flag_ikdtree_initial)
        {
            MM.get_map(state_point.pos[0],state_point.pos[1]);
            // ikdtree.reconstruct(MM.pointcloud_output->points);

            kdtree1.setInputCloud(MM.pointcloud_output);
            kdtree2.setInputCloud(MM.pointcloud_output);

        }
    }
    
}

void sensor_vaild_judge(Eigen::Vector3d &z_leg, Eigen::Vector3d &z_rtk)
{
    //判断leg的可用性
    // Eigen::Vector3d z_leg = Eigen::Vector3d::Zero();
    if(Measures.leg_vaild == true)
    {
        nav_msgs::Odometry::ConstPtr leg_back = Measures.leg.back();
        //z_leg只有前向和左向的速度
        z_leg(0)=leg_back->twist.twist.linear.x;
        z_leg(1)=leg_back->twist.twist.linear.y;
        // cout<<"velocity forward:"<<z_leg(0)<<" velocity left:"<<z_leg(1)<<endl;
    }
            
    //判断rtk的可用性
    rtk_vaild = false;
    // Eigen::Vector3d z_rtk = Eigen::Vector3d::Zero();
    while (!gnss_buffer.empty())
    {
        // 删除当前帧0.1s之前的rtk
        if (gnss_buffer.front().header.stamp.toSec() < Measures.package_end_time - 0.10)
        {
            gnss_buffer.pop_front();
        }
        // 超过当前帧0.05s之后，退出
        else if (gnss_buffer.front().header.stamp.toSec() > Measures.package_end_time)
        {
            break;
        }
        else
        {
            nav_msgs::Odometry thisGPS = gnss_buffer.front();
            gnss_buffer.pop_front();

            // V3D dp;
            // dp=state_point.rot.matrix()*rtk_T_wrt_Lidar;

            bool first_imu = true;
            double dt_rtk = 0.005;
            Sophus::SO3d rot_rtk = state_point_last.rot;
            V3D vel_rtk = state_point_last.vel;
            V3D angvel_avr,acc_avr;
            double imu_time_last = package_end_time_last;
            //丢掉上一个lidar时刻前的imu数据
            while(!imu_rtk_buffer.empty() && (package_end_time_last>0) && (imu_rtk_buffer.front()->header.stamp.toSec()<package_end_time_last))
            {
                imu_rtk_buffer.pop_front();
            }
            //更新姿态
            while(!imu_rtk_buffer.empty() && (package_end_time_last>0) && (imu_rtk_buffer.front()->header.stamp.toSec() <= thisGPS.header.stamp.toSec()))
            {
                //计算dt
                if(first_imu)
                {
                    dt_rtk = imu_rtk_buffer.front()->header.stamp.toSec() - package_end_time_last;
                    first_imu = false;
                    imu_time_last = imu_rtk_buffer.front()->header.stamp.toSec();
                }
                else
                {
                    dt_rtk = imu_rtk_buffer.front()->header.stamp.toSec() - imu_time_last;
                    imu_time_last = imu_rtk_buffer.front()->header.stamp.toSec();
                }

                
                angvel_avr << imu_rtk_buffer.front()->angular_velocity.x,
                              imu_rtk_buffer.front()->angular_velocity.y,
                              imu_rtk_buffer.front()->angular_velocity.z;

                acc_avr << imu_rtk_buffer.front()->linear_acceleration.x,
                           imu_rtk_buffer.front()->linear_acceleration.y,
                           imu_rtk_buffer.front()->linear_acceleration.z;

                rot_rtk = rot_rtk * Sophus::SO3d::exp(angvel_avr * dt_rtk);     
                acc_avr = rot_rtk * acc_avr + state_point_last.grav;
                vel_rtk += acc_avr * dt_rtk;

                imu_rtk_buffer.pop_front();           
            }

            V3D dp;
            dp = rot_rtk.matrix()*rtk_T_wrt_Lidar;
            //补偿杆臂
            z_rtk[0] = thisGPS.pose.pose.position.x-dp[0];
            z_rtk[1] = thisGPS.pose.pose.position.y-dp[1];
            z_rtk[2] = thisGPS.pose.pose.position.z-dp[2];

            //丢掉rtk时刻前的imu数据
            while(!imu_rtk_buffer.empty() && (package_end_time_last>0) && (imu_rtk_buffer.front()->header.stamp.toSec()<thisGPS.header.stamp.toSec()))
            {
                imu_rtk_buffer.pop_front();
            }
            //计算雷达结束时刻姿态和速度
            while(!imu_rtk_buffer.empty() && (package_end_time_last>0) && (imu_rtk_buffer.front()->header.stamp.toSec()<Measures.package_end_time))
            {
                dt_rtk = imu_rtk_buffer.front()->header.stamp.toSec() - imu_time_last;
                imu_time_last = imu_rtk_buffer.front()->header.stamp.toSec();

                angvel_avr << imu_rtk_buffer.front()->angular_velocity.x,
                              imu_rtk_buffer.front()->angular_velocity.y,
                              imu_rtk_buffer.front()->angular_velocity.z;

                acc_avr << imu_rtk_buffer.front()->linear_acceleration.x,
                           imu_rtk_buffer.front()->linear_acceleration.y,
                           imu_rtk_buffer.front()->linear_acceleration.z;

                rot_rtk = rot_rtk * Sophus::SO3d::exp(angvel_avr * dt_rtk); 
                acc_avr = rot_rtk * acc_avr + state_point_last.grav;
                vel_rtk += acc_avr * dt_rtk;   

                z_rtk[0] += vel_rtk[0] * dt_rtk;
                z_rtk[1] += vel_rtk[1] * dt_rtk;
                z_rtk[2] += vel_rtk[2] * dt_rtk;

                imu_rtk_buffer.pop_front();
            }

            rtk_vaild = true;
        }
    }

    //判断heading的可用性
    rtk_heading_vaild = false;
    while (!gnss_heading_buffer.empty())
    {
        // 删除当前帧0.05s之前的rtk
        if (heading_time_buffer.front() < Measures.package_end_time - 0.10)
        {
            gnss_heading_buffer.pop_front();
            heading_time_buffer.pop_front();
        }
        // 超过当前帧0.05s之后，退出
        else if (heading_time_buffer.front() > Measures.package_end_time)
        {
            break;
        }
        else
        {
            kf_heading = gnss_heading_buffer.front();

            gnss_heading_buffer.pop_front();
            heading_time_buffer.pop_front();

            rtk_heading_vaild = true;
        }
    }
}

void publish_sensor_vaild()
{
    na_localization::sensor_vaild SensorVaild;
    SensorVaild.header.stamp = ros::Time().fromSec(Measures.package_end_time);
    SensorVaild.imu_vaild = imu_vaild;
    SensorVaild.lidar_vaild = Measures.lidar_vaild;
    SensorVaild.rtk_vaild = rtk_vaild;
    SensorVaild.leg_vaild = Measures.leg_vaild;
    pubSensorVaild.publish(SensorVaild);
}

void publish_localization_vaild()
{
    std_msgs::Bool msg_localization;

    if(need_relocal)
    {
        msg_localization.data = false;
    }
    else
    {
        msg_localization.data = true;
    }

    pubLocalizationVaild.publish(msg_localization);
}

// BoxPointType LocalMap_Points;           // ikd-tree地图立方体的2个角点
// bool Localmap_Initialized = false;      // 局部地图是否初始化
void lasermap_fov_segment()
{
    cub_needrm.clear();     // 清空需要移除的区域
    V3D pos_LiD = state_point.pos;  // W系下位置
    //初始化局部地图范围，以pos_LiD为中心,长宽高均为cube_len
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    //各个方向上pos_LiD与局部地图边界的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离（1.5*300m）太小，标记需要移除need_move(FAST-LIO2论文Fig.3)
        // if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
        if (dist_to_map_edge[i][0] <= 80 || dist_to_map_edge[i][1] <= 80) need_move = true;
    }
    if (!need_move) return;  //如果不需要，直接返回，不更改局部地图
    cout<<"remove points"<<endl;
    double time_remove1 = omp_get_wtime();
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    //需要移动的距离
    // float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    float mov_dist = 20;
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        // if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
        if (dist_to_map_edge[i][0] <= 80){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        // } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
        } else if (dist_to_map_edge[i][1] <= 80){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }    
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);

    if(cub_needrm.size() > 0) ikdtree.Delete_Point_Boxes(cub_needrm); //删除指定范围内的点

    double time_remove2 = omp_get_wtime();
    cout<<"remove time:"<<time_remove2 - time_remove1<<endl;
}

void map_incremental()
{
    double time_addpoint1 = omp_get_wtime();
    PointVector PointNoNeedDownsample;
    PointNoNeedDownsample.reserve(MM.pointcloud_add->points.size());
    for(int i=0;i<MM.pointcloud_add->points.size();i++)
    {
        PointNoNeedDownsample.push_back(MM.pointcloud_add->points[i]);
    }
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    double time_addpoint2 = omp_get_wtime();
    cout<<"points size:"<<MM.pointcloud_add->points.size()<<endl;
    cout<<"ikdtree size:"<<ikdtree.validnum()<<endl;
    cout<<"add point time:"<<time_addpoint2 - time_addpoint1<<endl;
}

//重构ikdtree线程
void ReikdtreeThread()
{
    // cout<<"******************"<<endl;
    ros::Rate rate1(1);
    while (ros::ok())
    {
        if (flg_exit)
            break;
        //如果需要重定位,continue
        if(need_relocal)
            continue;

        if (flag_ikdtree_initial==true)
        {

            pos_now.x=state_point.pos[0];
            pos_now.y=state_point.pos[1];
            pos_now.z=state_point.pos[2];

            /***********新增代码*************/
            
            if(MM.get_map(state_point.pos[0],state_point.pos[1]))
            {   
                //mtx_reikd.lock();
                if(status_tree1 == 0  && last_ikdtree == 2)
                {
                    status_tree1 = 1;
                   // mtx_reikd.unlock();

                    double time_update1 = omp_get_wtime();
                    //cout<<"aaa111"<<endl;
                    pcl::PointCloud<PointType>::Ptr cloud_copy;
                    cloud_copy.reset(new pcl::PointCloud<PointType>());
                    *cloud_copy = *(MM.pointcloud_output);
                    kdtree1.setInputCloud(cloud_copy);
                    //kdtree1.setInputCloud(MM.pointcloud_output);
                    //cout<<"aaa222"<<endl;
                    double time_update2 = omp_get_wtime();


                    //mtx_reikd.lock();
                    status_tree1 = 0;
                    last_ikdtree = 1;
                    //mtx_reikd.unlock();
                }

                else if(status_tree2 == 0  && last_ikdtree == 1)
                {
                    status_tree2 = 1;
                    //mtx_reikd.unlock();

                    double time_update1 = omp_get_wtime();
                    cout<<"1111"<<endl;
                    pcl::PointCloud<PointType>::Ptr cloud_copy;
                    cloud_copy.reset(new pcl::PointCloud<PointType>());
                    *cloud_copy = *(MM.pointcloud_output);
                    kdtree2.setInputCloud(cloud_copy);
                    //cout<<"2222"<<endl;
                    double time_update2 = omp_get_wtime();

                    //mtx_reikd.lock();
                    status_tree2 = 0;
                    last_ikdtree = 2;
                    //mtx_reikd.unlock();
                }
                else
                {
                    ROS_ERROR("Thread2 Error!");
                    mtx_reikd.unlock();
                }

            }
            
        }
        rate1.sleep();
    }  
}

bool isKeyFrame(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& last_pos, const Eigen::Matrix3d& current_rot, const Eigen::Matrix3d& last_rot, double keyframe_threshold_pos, double keyframe_threshold_rot)
{
    // 计算位置变化
    Eigen::Vector3d pos_diff = current_pos - last_pos;
    double pos_change = pos_diff.norm();

    // 计算旋转变化
    Eigen::Matrix3d rot_diff = last_rot.inverse() * current_rot;
   // Eigen::AngleAxisd rot_change(rot_diff);
     //double rot_angle = rot_change.angle() * 180.0 / M_PI; // 转换为度
   
    double roll = std::atan2(rot_diff(2, 1), rot_diff(2, 2))* 180.0 / M_PI;
    double pitch = std::asin(-rot_diff(2, 0))* 180.0 / M_PI;
    double yaw = std::atan2(rot_diff(1, 0), rot_diff(0, 0))* 180.0 / M_PI;
  
    // 判断是否为关键帧
    //if (abs(pos_diff(0))> keyframe_threshold_pos ||abs(pos_diff(1))> keyframe_threshold_pos ||abs(pos_diff(2))> keyframe_threshold_pos || rot_angle > keyframe_threshold_rot)
    if (pos_change > keyframe_threshold_pos)// || abs(roll)> keyframe_threshold_rot||abs(pitch)> keyframe_threshold_rot||abs(yaw)> keyframe_threshold_rot)
    //if (pos_change > keyframe_threshold_pos )
    {
        return true; // 当前帧是关键帧
    }
    return false; // 当前帧不是关键帧
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    // 读取robot_id参数
    nh.param<int>("robot_id", current_robot_id, 2);
    cout << "Current robot_id: " << current_robot_id << endl;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);                // 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);              // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic 
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);     // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，需要该变量和上一个变量同时为true才发布
    nh.param<bool>("publish/visual_ikdtree", visual_ikdtree, true);              // 是否发布ikdtree点云
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);                        // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path",map_file_path,"");                         // 地图保存路径
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");              // 雷达点云topic名称
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");               // IMU的topic名称
    nh.param<bool>("common/time_sync_en", time_sync_en, false);                 // 是否需要时间同步，只有当外部未进行时间同步时设为true
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);          // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);                          // 地图的局部区域的长度（FastLio2论文中有解释）
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);                       // 激光雷达的最大探测范围
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);                            // IMU陀螺仪的协方差
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);                            // IMU加速度计的协方差
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);                     // IMU陀螺仪偏置的协方差
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);                     // IMU加速度计偏置的协方差
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);                   // 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);            // 激光雷达的类型
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);                  // 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);              // 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);    // 是否提取特征点（FAST_LIO2默认不进行特征点提取）
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); // 雷达相对于IMU的外参R
    nh.param<vector<double>>("mapping/rtk2Lidar_T", rtk2Lidar_T, vector<double>()); // rtk相对于雷达的外参T（即rtk在Lidar坐标系中的坐标）
        // 将 std::vector<double> 转换为 Eigen::Matrix3
Eigen::Matrix3d extrinR_matrix;
for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        extrinR_matrix(i, j) = extrinR[i * 3 + j];
    }
}

// 将旋转矩阵转换为四元数
Eigen::Quaterniond Rtt(extrinR_matrix);

// 将四元数转换回旋转矩阵
Sophus::SO3d Rtt1(Rtt);
Eigen::Matrix3d result_matrix = Rtt1.matrix();

// 将 Eigen::Matrix3d 转换回 std::vector<double>
std::vector<double> result_vector;
for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        result_vector.push_back(result_matrix(i, j));
    }
}

// 将结果存储回 extrinR
extrinR = result_vector;
    //bfsreloc
    nh.param<double>("Search_Radius",search_radius,10.0); //查找半径，一般10够用，室外视情况而定可提高至15，室内地图较小杂点较多可改为5
    
    //rtk
    nh.param<bool>("usertk", usertk, false); //是否使用rtk
    nh.param<string>("common/gnss_topic", gnss_topic,"/rtk_pos_raw");   //gps的topic名称

    nh.param<string>("common/gnss_heading_topic", gnss_heading_topic,"/rtk_heading_raw");   //gps_heading的topic名称 

    nh.param<std::string>("loadmappath", loadmappath, "/home/ywb/NR_mapping/src/FAST_LIO_SAM/FAST_LIO_SAM/PCD/cloud_map.pcd"); //加载地图的路径
    nh.param<std::string>("loadposepath", loadposepath, "/home/ywb/NR_mapping/src/FAST_LIO_SAM/FAST_LIO_SAM/PCD/pose.txt"); //加载初始位置的路径（只有用rtk才需要）
    nh.param<std::string>("params_filename", params_filename, "/home/nrc/na/na_localization_general/src/na_localization/PCD/param.json"); //加载重定位
    nh.param<string>("common/leg_topic", leg_topic,"/leg_odom");   //leg的topic名称
    nh.param<bool>("useleg",useleg,false); //是否使用leg

    nh.param<int>("Reposition_type", Reposition_type, 2);
    nh.param<bool>("common/pub_firstodo_en", pub_firstodo_en, false);              // 是否发布初始变换

    nh.param<bool>("chengxin_change", chengxin_change, false); //是否加入gps因子
    cout<<"chengxin_change = "<<chengxin_change<<endl;
    
    // 添加构图相关参数
    nh.param<double>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 1.0);
    nh.param<double>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 0.4);

    cout<<"Lidar_type: "<<p_pre->lidar_type<<endl;
    // 初始化path的header（包括时间戳和帧id），path用于保存odemetry的路径
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    
    int keyframe_count = 0;
    int frame_count = 0;
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl, sub_imu;
    ros::Subscriber sub_robot2_pcl, sub_robot2_imu;
    ros::Subscriber sub_robot0_pcl, sub_robot0_imu;

    // 根据robot_id订阅主要处理的话题
    if(current_robot_id == 2) {
        // sub_pcl = nh.subscribe("/jackal2/point_local_octree", 1, robot2_pcl_cbk);
        // sub_imu = nh.subscribe("/jackal2/point_local_octree", 1, robot2_imu_cbk);
        sub_pcl = nh.subscribe("/livox/lidar", 1, robot2_pcl_cbk);
        sub_imu = nh.subscribe("/livox/imu", 1, robot2_imu_cbk);
    
        cout << "Robot 2: 构图模式启动" << endl;

        // 启动键盘输入监听线程 - 移动到这里更合理
        std::thread keyboard_thread(keyboardInputThread);
        keyboard_thread.detach();
        cout << "键盘监听线程已启动，按回车键保存地图并通知robot_0开始重定位" << endl;
    } else if(current_robot_id == 0) {
        sub_pcl = nh.subscribe("/jackal0/point_local_octree", 1, robot0_pcl_cbk);
        // sub_imu = nh.subscribe("/jackal0/point_local_octree", 1, robot0_imu_cbk);
        cout << "Robot 0: Relocalization mode activated" << endl;
    }

    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk_sensor_msgs_2enu); //gnss

    ros::Subscriber sub_gnss_heading = nh.subscribe(gnss_heading_topic, 200000, gnss_heading_cbk);   

    ros::Subscriber sub_leg = nh.subscribe(leg_topic, 200000, leg_cbk); 
    // ros::Subscriber sub_ExterPos=nh.subscribe("/move_base_simple/goal",1,ExtPos_cbk); //手动重定位话题
    ros::Subscriber sub_ManualPos=nh.subscribe("/move_base_simple/goal",1,ManualPos_cbk); //手动重定位话题
    
    // 只订阅位姿话题用于坐标变换计算
    ros::Subscriber sub_robot0_pose = nh.subscribe<grid_map::inc_octree>("/jackal0/lio_sam/mapping/inc_octree", 2000, robot0PoseHandler);
    ros::Subscriber sub_robot2_pose = nh.subscribe<grid_map::inc_octree>("/jackal2/lio_sam/mapping/inc_octree", 2000, robot2PoseHandler);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    

    pubICP_in = nh.advertise<sensor_msgs::PointCloud2>("/icp_in", 100000);
    pubICP_target = nh.advertise<sensor_msgs::PointCloud2>("/icp_target", 100000);

    pubICP_out = nh.advertise<sensor_msgs::PointCloud2>("/icp_out", 100000);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 100000);
    ros::Publisher pubOdomFromOrigin = nh.advertise<nav_msgs::Odometry> ("/PX4_Odometry",100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> ("/path", 100000);
    ros::Publisher pubPathUpdate = nh.advertise<nav_msgs::Path>("s_fast_lio/path_update", 100000);                   //  isam更新后的path
    ros::Publisher pubGnssPath = nh.advertise<nav_msgs::Path>("/gnss_path", 100000);
    ros::Publisher pub_save = nh.advertise<std_msgs::Bool>("/save_data", 100); //发布保存数据的话题
    pubSensorVaild = nh.advertise<na_localization::sensor_vaild>("/sensor_vaild", 100000); //发布目前传感器的可用性
    pubLocalizationVaild = nh.advertise<std_msgs::Bool>("/localization_vaild", 1000); //发布定位的有效性
    //发布全局地图,用来手动重定位
    pubglobalmap =nh.advertise<sensor_msgs::PointCloud2>("/globalmap",1);
    //发布定位结果的个数，用来统计定位频率
    ros::Publisher pubOdoCnt = nh.advertise<std_msgs::Int32>("/odometry_count", 100);

    //test ikdtree
    ros::Publisher pubEffCnt = nh.advertise<std_msgs::Int32>("/Effct_feat_num", 100);
    ros::Publisher pubPointsCnt = nh.advertise<std_msgs::Int32>("/vaild_points_num", 100);
    ros::Publisher pubCloudeffct = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effct", 100000);

    ros::Publisher pubKeyFrames = nh.advertise<lio_sam::cloud_info> ("/jackal0/lio_sam/mapping/cloud_info", 100);//发布关键帧的话题
    ros::Publisher pub_inc_octree_ = nh.advertise<grid_map::inc_octree>("/jackal0/lio_sam/mapping/inc_octree", 1);//发布0号车位姿
    
    // 添加构图相关发布器
    pubSavedCloud = nh.advertise<sensor_msgs::PointCloud2>("/saved_cloud", 100);
    pubSavedPose = nh.advertise<std_msgs::String>("/saved_pose", 100);
    
    //输出与web端通讯的相关标志位
    pubRelocal_flag =nh.advertise<std_msgs::Bool>("/Relocal_flag",1);/***重定位成功/失败标志位：true为成功、false为失败***/
    pubLocal_flag =nh.advertise<std_msgs::Bool>("/Local_flag",1);/***定位丢失标志位：true为成功、false为丢失***/
    
    // 初始化位姿数组
    for(int i = 0; i < 6; i++) {
        fusionTrans0[i] = 0.0;
        fusionTrans2[i] = 0.0;
    }
    
    // 添加坐标变换相关发布器
    pub_trans0 = nh.advertise<nav_msgs::Odometry>("/jackal0/context/trans_map", 1);
    
    // 根据当前robot_id发布对应的位姿
    if (current_robot_id == 0) {
        pub_robot0_pose = nh.advertise<grid_map::inc_octree>("/jackal0/lio_sam/mapping/inc_octree", 1);
    } else if (current_robot_id == 2) {
        pub_robot2_pose = nh.advertise<grid_map::inc_octree>("/jackal2/lio_sam/mapping/inc_octree", 1);
    }
    
    //string params_filename = string("/home/nrc/na/na_localization_lslidar/src/na_localization/PCD/param.json");
    reloc_plugin_ptr_ = std::make_shared<plugins::RelocPlugin>(nh, params_filename);    
    /****************************/

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    shared_ptr<ImuProcess> p_imu1(new ImuProcess());
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    rtk_T_wrt_Lidar<<VEC_FROM_ARRAY(rtk2Lidar_T);
    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov), 
                        V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    

    /*************************重定位****************************/
    string read_dir = loadmappath;
    pcl::PCDReader pcd_reader;
    // pcd_reader.read(read_dir, *pointcloudmap);
    // cout<<"read pcd success!"<<endl;

    pcl::VoxelGrid<PointType> downSizepointcloudmap;
    pcl::PointCloud<PointType>::Ptr DSpointcloudmap(new pcl::PointCloud<PointType>());//地图点云

    // downSizepointcloudmap.setInputCloud(pointcloudmap);
    // downSizepointcloudmap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    // downSizepointcloudmap.filter(*pointcloudmap);

    //需要对地图点云降采样，不然在rviz里显示太卡
    downSizepointcloudmap.setInputCloud(pointcloudmap);
    downSizepointcloudmap.setLeafSize(1.0f, 1.0f, 1.0f);
    downSizepointcloudmap.filter(*DSpointcloudmap);

    // sensor_msgs::PointCloud2 globalmapmsg;
    pcl::toROSMsg(*DSpointcloudmap, globalmapmsg);
    globalmapmsg.header.frame_id = "camera_init"; //todo 这里发布一个从读取点云里面稀疏后的降采样点云 就是为了方便在RVIZ里面观察

    /****************加载地图******************/
    MM.set_ds_size(filter_size_map_min);
    MM.set_input_PCD(loadmappath);
    MM.voxel_process();

    /****************加载原始经纬高,最大/最小高度******************/
    if(!rtk_p0_init)
    {
        std::ifstream inputFile(loadposepath);
        double lat0, lon0, alt0;
        if(inputFile >> lat0 >> lon0 >> alt0 >> max_z >>min_z) 
        {
            max_z = ceil(max_z);
            min_z = floor(min_z);
            printf("先验地图原点经纬度坐标： 经度 = %3.7lf; 纬度 =  %3.7lf;  高度 =  %3.3lf; 构图过程位姿最大高度 =  %2.2f; 构图过程位姿最小高度 =  %2.2f; \n", lon0, lat0, alt0, max_z, min_z);
        }
        if(lon0 == 0 || lat0 == 0 || alt0 == 0)
        {
            cout<<"无法读取地图rtk原点,需进行手动重定位"<<endl;
        }
        else if(usertk)
        {
            gnss2enu->SetOriginLLA(lon0*M_PI/180,lat0*M_PI/180,alt0); 
        }
        
           
        rtk_p0_init = true;
    }

    /****************************************/
    //重构ikd树的线程
    std::thread ikdtreethread(&ReikdtreeThread);
    
    ros::Rate ratemap(1); //todo 接上文 发布了一个在RVIZ里面方便观察的稀疏地图 （源自读取）要延迟1S 发布在RVIZ中 否则会RVIZ收不到
    ratemap.sleep();
    pubglobalmap.publish(globalmapmsg);
    // ratemap.sleep();
    
    Eigen::Matrix3d Sigma_leg = Eigen::Matrix3d::Identity(); //leg里程计的协方差
    double sigmaleg = 0.0025;//0.01
    Sigma_leg(0, 0) = sigmaleg;
    Sigma_leg(1, 1) = sigmaleg;
    Sigma_leg(2, 2) = sigmaleg;

    Eigen::Matrix3d Sigma_rtk = Eigen::Matrix3d::Identity(); //rtk的协方差
    double sigmartk = 0.05*0.05;
    Sigma_rtk(0, 0) = sigmartk;
    Sigma_rtk(1, 1) = sigmartk;
    Sigma_rtk(2, 2) = sigmartk;

    Eigen::Vector3d z_leg = Eigen::Vector3d::Zero();
    Eigen::Vector3d z_rtk = Eigen::Vector3d::Zero();

    bool save_data = true;

    double odo_time = 0;
    int odo_cnt = 0;

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    
    int transform_calc_counter = 0;

    while (ros::ok())
    {
        if (flg_exit) break;
        ros::spinOnce();

        // 重定位逻辑
        if(current_robot_id == 0) {
            // robot_0的重定位逻辑
            std::lock_guard<std::mutex> lock(enter_key_mutex);
            if(enter_key_pressed && map_data_ready && need_relocal) {
                // 加载共享地图数据进行重定位
                pcl::PointCloud<PointType>::Ptr mapData(new pcl::PointCloud<PointType>());
                double maxZ, minZ;
                if(getSharedMapData(mapData, maxZ, minZ)) {
                    // 使用mapData进行重定位初始化
                    *pointcloudmap = *mapData;
                    max_z = maxZ;
                    min_z = minZ;
                    
                    // 重新初始化地图管理器
                    // 先保存mapData为临时PCD文件
                    string temp_pcd_path = "/tmp/shared_map.pcd";
                    pcl::PCDWriter pcd_writer;
                    pcd_writer.write(temp_pcd_path, *mapData);
                    
                    // 使用set_input_PCD方法
                    MM.set_input_PCD(temp_pcd_path);
                    MM.voxel_process();
                    
                    cout << "Loaded shared map data for relocalization, starting relocalization..." << endl;
                    enter_key_pressed = false; // 重置标志
                }
            }
        }

        ReLocalization();


        if(sync_packages(Measures))  //把一次的IMU和LIDAR数据打包到Measures
        {
            std::cout<<"sync_packages(Measures)!!!!!!!!!!!"<<std::endl;
            if(current_robot_id == 0 && need_relocal==true)
            {
                continue;
            }

            double time_all1 = omp_get_wtime();
            double t00 = omp_get_wtime();

            if (flg_first_scan)
            {
                
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                
                continue;
            }
            
            state_point_last = state_point;
            double t_process1 = omp_get_wtime();
            p_imu1->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            double t_process2 = omp_get_wtime();


            if(!p_imu1->feats_undistort_vaild)
                continue;

            state_point = kf.get_x();
            //cout<<"预测："<<state_point.pos[0]<<","<<state_point.pos[1]<<","<<state_point.pos[2]<<endl;
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            // lasermap_fov_segment();
            //点云下采样
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            
            double time_all2 = omp_get_wtime();

            if(flag_ikdtree_initial == false)
            {
                PointType pos_initial;
                pos_initial.x=state_point.pos[0];
                pos_initial.y=state_point.pos[1];
                pos_initial.z=state_point.pos[2];
                //获得初始的局部地图
                MM.get_map(state_point.pos[0],state_point.pos[1]);

                pos_last=pos_initial;

                flag_ikdtree_initial=true;

                kdtree1.setInputCloud(MM.pointcloud_output);
                kdtree2.setInputCloud(MM.pointcloud_output);


            }
            
            //判断传感器的可用性
            sensor_vaild_judge(z_leg,z_rtk);

            //计算当前roll、pitch、yaw
            Eigen::Vector3d cur_atti = Eigen::Vector3d::Zero();
            cur_atti[0] = atan2(state_point.rot.matrix()(2,1),state_point.rot.matrix()(2,2));
            cur_atti[1] = -asin(state_point.rot.matrix()(2,0));
            cur_atti[2] = atan2(state_point.rot.matrix()(1,0),state_point.rot.matrix()(0,0));

            Nearest_Points.resize(feats_down_size);         //存储近邻点的vector

            rtk_vaild = false;
            rtk_heading_vaild = false;

            //mtx_reikd.lock();
            if(status_tree1 == 0 && last_ikdtree == 1)
            {
                status_tree1 = 2;
                //mtx_reikd.unlock();
                

                if(visual_ikdtree)
                {
                    featsFromMap->clear();

                    *featsFromMap = *kdtree1.getInputCloud();

                }
                double time_kf1 = omp_get_wtime();
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, kdtree1, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en,effct_feat_num_,vaild_points_,
                                                  Sigma_leg,z_leg,useleg,Measures.leg_vaild,Measures.lidar_vaild,need_relocal,Sigma_rtk,z_rtk,rtk_vaild,cur_atti,kf_heading,rtk_heading_vaild,pubLocal_flag,pubCloudeffct,lidar_end_time);
                //cout<<"effct_feat_num_en1 = "<<effct_feat_num_en<<endl;
                std_msgs::Int32 eff_cnt_;
                eff_cnt_.data = effct_feat_num_;
                pubEffCnt.publish(eff_cnt_);
                //cout<<"vaild_points_1 = "<<vaild_points_<<endl;
                std_msgs::Int32 points_cnt_;
                points_cnt_.data = vaild_points_;
                pubPointsCnt.publish(points_cnt_);
                double time_kf2 = omp_get_wtime();
                

                //mtx_reikd.lock();
                status_tree1 = 0;
               // mtx_reikd.unlock();
            }
            else if(status_tree2 == 0 && last_ikdtree == 2)
            {
                status_tree2 = 2;
                //mtx_reikd.unlock();

                if(visual_ikdtree)
                {
                    featsFromMap->clear();

                    *featsFromMap = *kdtree2.getInputCloud();

                }
                double time_kf1 = omp_get_wtime();
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, kdtree2, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en,effct_feat_num_,vaild_points_,
                                                  Sigma_leg,z_leg,useleg,Measures.leg_vaild,Measures.lidar_vaild,need_relocal,Sigma_rtk,z_rtk,rtk_vaild,cur_atti,kf_heading,rtk_heading_vaild,pubLocal_flag,pubCloudeffct,lidar_end_time);
                //cout<<"effct_feat_num_en2 = "<<effct_feat_num_en<<endl;
                std_msgs::Int32 eff_cnt_;
                eff_cnt_.data = effct_feat_num_;
                pubEffCnt.publish(eff_cnt_);
                //cout<<"vaild_points_2 = "<<vaild_points_<<endl;
                std_msgs::Int32 points_cnt_;
                points_cnt_.data = vaild_points_;
                pubPointsCnt.publish(points_cnt_);
                double time_kf2 = omp_get_wtime();


                //mtx_reikd.lock();
                status_tree2 = 0;
                //mtx_reikd.unlock();
            }
            else
            {
                ROS_ERROR("Thread2 Error!");
                //mtx_reikd.unlock();
            }

            /***************************/
            double time_all3 = omp_get_wtime();


            state_point = kf.get_x();
            std::cout << "卡尔曼滤波状态更新成功" << std::endl;
        // 检查state_point是否有效
        std::cout << "状态点位置: " << state_point.pos.transpose() << std::endl;
        std::cout << "状态点旋转矩阵有效性: " << (state_point.rot.matrix().allFinite() ? "有效" : "无效") << std::endl;

        // 安全地访问四元数
        try {
            auto quat = state_point.rot.unit_quaternion();
            std::cout << "四元数: w=" << quat.w() << ", x=" << quat.x() << ", y=" << quat.y() << ", z=" << quat.z() << std::endl;
        } catch (const std::exception& e) {
            std::cout << "四元数访问失败: " << e.what() << std::endl;
        }   
        
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
            // printf("雷达结束时间： %7.7lf \n",lidar_end_time);
            // cout<<"滤波后："<<state_point.pos[0]<<","<<state_point.pos[1]<<","<<state_point.pos[2]<<endl;
            // cout<<endl;

            std::cout<<"AAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!"<<std::endl;
            /*
            if(pub_firstodo_en)
            {
              std::cout<<"pub_firstodo_en!!!!!!!!!!!!"<<std::endl;
              if (count_first_odometry == 0)
              {
                  odom_all.pose.pose.position.x = state_point.pos[0];
                  odom_all.pose.pose.position.y = state_point.pos[1];
                  odom_all.pose.pose.position.z = state_point.pos[2];
                  odom_all.pose.pose.orientation.x = state_point.rot.unit_quaternion().x();
                  odom_all.pose.pose.orientation.y = state_point.rot.unit_quaternion().y();
                  odom_all.pose.pose.orientation.z = state_point.rot.unit_quaternion().z();
                  odom_all.pose.pose.orientation.w = state_point.rot.unit_quaternion().w();
              
                  count_first_odometry++;
              }
              else if (count_first_odometry < count_odometry_threshold )
              {
                  odom_all.pose.pose.position.x += state_point.pos[0];
                  odom_all.pose.pose.position.y += state_point.pos[1];
                  odom_all.pose.pose.position.z += state_point.pos[2];
                  odom_all.pose.pose.orientation.x += state_point.rot.unit_quaternion().x();
                  odom_all.pose.pose.orientation.y += state_point.rot.unit_quaternion().y();
                  odom_all.pose.pose.orientation.z += state_point.rot.unit_quaternion().z();
                  odom_all.pose.pose.orientation.w += state_point.rot.unit_quaternion().w();
              
                  count_first_odometry++;
              }
              if (count_first_odometry == count_odometry_threshold)
              {

                  // 计算平均位置
                  odom_all.pose.pose.position.x /= count_first_odometry;
                  odom_all.pose.pose.position.y /= count_first_odometry;
                  odom_all.pose.pose.position.z /= count_first_odometry;
              
                  // 计算平均四元数并归一化
                  Eigen::Quaterniond q_avg(
                      odom_all.pose.pose.orientation.w / count_first_odometry,
                      odom_all.pose.pose.orientation.x / count_first_odometry,
                      odom_all.pose.pose.orientation.y / count_first_odometry,
                      odom_all.pose.pose.orientation.z / count_first_odometry
                  );
                  q_avg.normalize(); // 归一化四元数
              
                  odom_all.pose.pose.orientation.w = q_avg.w();
                  odom_all.pose.pose.orientation.x = q_avg.x();
                  odom_all.pose.pose.orientation.y = q_avg.y();
                  odom_all.pose.pose.orientation.z = q_avg.z();
              
                  odom_all.header.stamp.fromSec(lidar_end_time);
                  odom_all.header.frame_id = "Base_ENU";
              
                  inc_octree_.time = odom_all.header.stamp.toSec();
                  inc_octree_.poseX = odom_all.pose.pose.position.x; // 因子图优化的位姿
                  inc_octree_.poseY = odom_all.pose.pose.position.y;
                  inc_octree_.poseZ = odom_all.pose.pose.position.z;
                  Eigen::Quaterniond q_odom(
                      odom_all.pose.pose.orientation.w,
                      odom_all.pose.pose.orientation.x,
                      odom_all.pose.pose.orientation.y,
                      odom_all.pose.pose.orientation.z
                  );
                  tf::Quaternion q_o(q_odom.x(), q_odom.y(), q_odom.z(), q_odom.w());
                  double roll, pitch, yaw;
                  tf::Matrix3x3(q_o).getRPY(roll, pitch, yaw);
                  inc_octree_.poseRoll = roll;
                  inc_octree_.posePitch = pitch;
                  inc_octree_.poseYaw = yaw;
                  inc_octree_.robotID= "Base_ENU";
              
                  for (int count_octree_ = 0; count_octree_ < 10; count_octree_++)
                  {
                      pub_inc_octree_.publish(inc_octree_);
                      cout<<"sucess"<<endl ;
                      // std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒
                  }
                    count_first_odometry++;
              }
            }
            */
                   //关键帧提取
            if(!pub_firstodo_en)
            {
                    if (isKeyFrame(state_point.pos, state_point_lastframe.pos, state_point.rot.matrix(), state_point_lastframe.rot.matrix(), keyframe_threshold_pos, keyframe_threshold_rot)||keyframe_count==0)
                {
                    // 保存关键帧

                    PointTypePose keyframe_pose;
                    keyframe_pose.x=state_point.pos[0];
                    keyframe_pose.y=state_point.pos[1];
                    keyframe_pose.z=state_point.pos[2];
                    // keyframe_pose.roll = atan2(state_point.rot.matrix()(2, 1), state_point.rot.matrix()(2, 2))* 180.0 / M_PI;//度
                    // keyframe_pose.pitch = -asin(state_point.rot.matrix()(2, 0))* 180.0 / M_PI;
                    // keyframe_pose.yaw = atan2(state_point.rot.matrix()(1, 0), state_point.rot.matrix()(0, 0))* 180.0 / M_PI;
                    Eigen::Quaterniond q_internal = state_point.rot.unit_quaternion();
                    tf::Quaternion ql(q_internal.x(), q_internal.y(), q_internal.z(),q_internal.w());
                    double roll, pitch, yaw;
                    tf::Matrix3x3(ql).getRPY(roll, pitch, yaw);
                    keyframe_pose.roll = roll ;
                    keyframe_pose.pitch = pitch ;
                    keyframe_pose.yaw = yaw;
                    keyframe_pose.time=lidar_end_time;
                    cloudKeyPoses6D->points.push_back(keyframe_pose);

                    int keysize = feats_down_body->points.size();
                    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(keysize, 1));
                
                    for (int keyi = 0; keyi < keysize; keyi++)
                    {
                        RGBpointBodyLidarToIMU(&feats_down_body->points[keyi], \
                                            &laserCloudIMUBody->points[keyi]);
                    }



                    lio_sam::cloud_info keyframe_info;
                    sensor_msgs::PointCloud2 tempkeyCloud;
                    pcl::toROSMsg(*laserCloudIMUBody, tempkeyCloud);
              

                    // lio_sam::cloud_info keyframe_info;
                    // sensor_msgs::PointCloud2 tempkeyCloud;
                    // pcl::toROSMsg(*feats_down_body, tempkeyCloud);

                    tempkeyCloud.header.stamp.fromSec(lidar_end_time);
                    tempkeyCloud.header.frame_id = "camera_init";
                    keyframe_info.header = tempkeyCloud.header;
                    keyframe_info.cloud_deskewed = tempkeyCloud;//----原始点云用于sc
                    keyframe_info.cloud_corner.data.clear();
                    keyframe_info.cloud_surface = tempkeyCloud;//----原始点云用于回环icp

                    keyframe_info.initialGuessX =  keyframe_pose.x;//fast-lio-sam优化出的位姿
                    keyframe_info.initialGuessY =  keyframe_pose.y;
                    keyframe_info.initialGuessZ =  keyframe_pose.z;
                    keyframe_info.initialGuessRoll  =  roll;
                    keyframe_info.initialGuessPitch =  pitch;
                    keyframe_info.initialGuessYaw   = yaw;
                    keyframe_info.imuRollInit  =  roll;
                    keyframe_info.imuPitchInit =  pitch;
                    keyframe_info.imuYawInit   =  yaw;
                    //cloudInfo.imuAvailable = cloudKeyPoses6D->size() - 1;
                    
                    pubKeyFrames.publish(keyframe_info);
                    
                    // 保存关键帧点云到surfCloudKeyFrames
                    surfCloudKeyFrames.push_back(laserCloudIMUBody);
                    
                    // 保存关键帧位置到cloudKeyPoses3D
                    PointType thisPose3D;
                    thisPose3D.x = keyframe_pose.x;
                    thisPose3D.y = keyframe_pose.y;
                    thisPose3D.z = keyframe_pose.z;
                    thisPose3D.intensity = keyframe_count; // 使用关键帧索引作为intensity
                    cloudKeyPoses3D->points.push_back(thisPose3D);
                    
                    keyframe_count++;
                    state_point_lastframe = state_point;//更新上一关键帧位姿

                    cout<<"keyframe true:"<<keyframe_count<<endl;
                    ROS_INFO("keyframe:t=%lf,x=%f,y=%f,z=%f,roll=%f,pitch=%f,yaw=%f",keyframe_pose.time,keyframe_pose.x,keyframe_pose.y,keyframe_pose.z,keyframe_pose.roll,keyframe_pose.pitch,keyframe_pose.yaw);
                    
                    // robot_2的构图逻辑
                    if(current_robot_id == 2) {
                        // 添加构图开始提示
                        static bool mapping_started = false;
                        if(!mapping_started) {
                            cout << "已经开始构图 - Robot ID 02 进入构图循环" << endl;
                            mapping_started = true;
                        }
                        
                        // 检查是否按下回车键来保存地图
                        std::lock_guard<std::mutex> lock(enter_key_mutex);
                        if(enter_key_pressed) {
                            saveMapToMemory();
                            enter_key_pressed = false;
                            cout << "Map saved to memory by robot_2, robot_0 can now start localization" << endl;
                        }
                        
                        // 或者每50个关键帧自动保存一次
                        if(keyframe_count % 50 == 0 && keyframe_count > 0) {
                            saveMapToMemory();
                        }
                    }
                    
                    // robot_0的重定位逻辑
                    if(current_robot_id == 0) {
                        std::lock_guard<std::mutex> lock(enter_key_mutex);
                        if(enter_key_pressed) {
                            saveMapToMemory();
                            enter_key_pressed = false;
                            cout << "Map saved to memory, robot_0 can now start localization" << endl;
                        }
                    } 
                }
            }
            
            
            // frame_count++;
            // cout<<"frame_count:"<<frame_count<<endl;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped,pubOdomFromOrigin);
            //测试定位频率
            if(0 == odo_time)
            {
                odo_time = ros::Time::now().toSec();
                odo_cnt ++;
            }
            else
            {
                odo_cnt ++;
                if((ros::Time::now().toSec() - odo_time) > 10.0)
                {
                    std_msgs::Int32 odo_cnt_;
                    odo_cnt_.data = odo_cnt;
                    pubOdoCnt.publish(odo_cnt_);

                    odo_time = ros::Time::now().toSec();
                    odo_cnt = 0;
                }
            }

            /*** add the feature points to map kdtree ***/
            feats_down_world->resize(feats_down_size);
            
            /******* Publish points *******/
            if (path_en)
            {
                publish_path(pubPath);
                publish_path_update(pubPathUpdate);             //   发布经过isam2优化后的路径
                publish_gnss_path(pubGnssPath);                        //   发布gnss轨迹
            }                         
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            if (visual_ikdtree)   publish_map(pubLaserCloudMap);
            
            double t11 = omp_get_wtime();
            double time_all4 = omp_get_wtime();

            double yaw = atan2(state_point.rot.matrix()(1,0),state_point.rot.matrix()(0,0)) * 180 / M_PI;
            
            // 在处理完成后发布当前机器人位姿
            publishCurrentRobotPose();
            
            // 每隔一定帧数计算并发布坐标变换
            transform_calc_counter++;
            if (transform_calc_counter >= 10) { // 每10帧计算一次变换
                calculateAndPublishTransform();
                transform_calc_counter = 0;
            }
        }

        rate.sleep();

        //发布传感器的有效性
        publish_sensor_vaild();
        //发布定位的有效性
        publish_localization_vaild();  

        if(need_relocal)
        {
            if(save_data == true)
            {
                std_msgs::Bool tmp_save_data ;
                tmp_save_data.data = true;
                pub_save.publish(tmp_save_data);
                save_data = false;
            }
        }
        else
        {
            save_data = true;
        }
    }

    ikdtreethread.join();
    
    // 程序结束时保存地图（robot_2）
    if(current_robot_id == 2) {
        saveMapToMemory();
    }

    return 0;
}
