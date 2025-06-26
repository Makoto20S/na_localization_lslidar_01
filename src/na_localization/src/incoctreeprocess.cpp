#include"incoctreeprocess.h"

// void IncOctreeProcess::IncOctreeHandler(const grid_map::inc_octreeConstPtr& msgIn)
// {
//     string ROBOT_ID = msgIn->robotID;
//     //lio_sam::inc_octree::Ptr inc_octree_ptr(new lio_sam::inc_octree(*msgIn));
//     //inc_octree_buffer.push_back(inc_octree_ptr);

    
//     timestamp = timestamp.fromSec(msgIn->time);
//     double t1 = ros::Time::now().toSec();
    
//     cout<<ROBOT_ID<<endl;
    

//     PointTypePose pose_octree;
//     pose_octree.x = msgIn->poseX;
//     pose_octree.y = msgIn->poseY;
//     pose_octree.z = msgIn->poseZ;
//     pose_octree.roll = msgIn->poseRoll;
//     pose_octree.pitch = msgIn->posePitch;
//     pose_octree.yaw = msgIn->poseYaw;
//     //将位置存入对应点云中
//     PointType point_octree;
//     point_octree.x = msgIn->poseX;
//     point_octree.y = msgIn->poseY;
//     point_octree.z = msgIn->poseZ;
//     //将八叉树int8数组存入对应队列中
//     std_msgs::Int8MultiArray data_octree;
//     data_octree.data = msgIn->incr_octree.data;

//     // if(ROBOT_ID == "ouster0")
//     if(ROBOT_ID == robot_id0)
//     {
//         std::cout<<"enter"<<std::endl;
//         inc_octree_buffer0.push_back(data_octree);
//         cloud_octree0->push_back(point_octree);
//         ResetValues();
//         double tt1 = ros::Time::now().toSec();
//         Search_insert_octree(cloud_octree0,inc_octree_buffer0,m_merged_difftree_);
//         double tt2 = ros::Time::now().toSec();
//         //std::cout<<" time 1 is : "<<(tt2 - tt1)<<std::endl;
//         Get_pts_from_octree(m_merged_difftree_,cloud_trans_,cloud_octree0->back().x,cloud_octree0->back().y,cloud_octree0->back().z);
//         double tt3 = ros::Time::now().toSec();
//         //std::cout<<" time 2 is : "<<(tt3 - tt2)<<std::endl;
//         Trans_pts_to_lidar(cloud_trans_,cloud,pose_octree);
//         double tt4 = ros::Time::now().toSec();
//         //std::cout<<" time 3 is : "<<(tt4 - tt3)<<std::endl;
//         cloudInfo0.imuRollInit = pose_octree.roll;
//         cloudInfo0.imuPitchInit = pose_octree.pitch;
//         cloudInfo0.imuYawInit = pose_octree.yaw;
//         cloudInfo0.initialGuessRoll = pose_octree.roll;
//         cloudInfo0.initialGuessPitch = pose_octree.pitch;
//         cloudInfo0.initialGuessYaw = pose_octree.yaw;
//         cloudInfo0.initialGuessX = point_octree.x;
//         cloudInfo0.initialGuessY = point_octree.y;
//         cloudInfo0.initialGuessZ = point_octree.z;
//         cloudInfo0.header.stamp = timestamp;
//         cloudInfo0.header.frame_id = ROBOT_ID + "/odom";
//         publish_cloudinfo(cloud,pub_laser_cloud0,ROBOT_ID,cloudInfo0,pub_cloudinfo0);
//     }
//     // else if(ROBOT_ID == "ouster1")
//     else if(ROBOT_ID == robot_id1)
//     {
//         inc_octree_buffer1.push_back(data_octree);
//         cloud_octree1->push_back(point_octree);
//         ResetValues();
//         Search_insert_octree(cloud_octree1,inc_octree_buffer1,m_merged_difftree_);
//         Get_pts_from_octree(m_merged_difftree_,cloud_trans_,cloud_octree1->back().x,cloud_octree1->back().y,cloud_octree1->back().z);
//         Trans_pts_to_lidar(cloud_trans_,cloud,pose_octree);
//         cloudInfo1.imuRollInit = pose_octree.roll;
//         cloudInfo1.imuPitchInit = pose_octree.pitch;
//         cloudInfo1.imuYawInit = pose_octree.yaw;
//         cloudInfo1.initialGuessRoll = pose_octree.roll;
//         cloudInfo1.initialGuessPitch = pose_octree.pitch;
//         cloudInfo1.initialGuessYaw = pose_octree.yaw;
//         cloudInfo1.initialGuessX = point_octree.x;
//         cloudInfo1.initialGuessY = point_octree.y;
//         cloudInfo1.initialGuessZ = point_octree.z;
//         cloudInfo1.header.stamp = timestamp;
//         cloudInfo1.header.frame_id = ROBOT_ID + "/odom";
//         publish_cloudinfo(cloud,pub_laser_cloud1,ROBOT_ID,cloudInfo1,pub_cloudinfo1);
//     }
//     // else if(ROBOT_ID == "ouster2")
//     else if(ROBOT_ID == robot_id2)
//     {
//         inc_octree_buffer2.push_back(data_octree);
//         cloud_octree2->push_back(point_octree);
//         ResetValues();
//         Search_insert_octree(cloud_octree2,inc_octree_buffer2,m_merged_difftree_);
//         Get_pts_from_octree(m_merged_difftree_,cloud_trans_,cloud_octree2->back().x,cloud_octree2->back().y,cloud_octree2->back().z);
//         Trans_pts_to_lidar(cloud_trans_,cloud,pose_octree);
//         cloudInfo2.imuRollInit = pose_octree.roll;
//         cloudInfo2.imuPitchInit = pose_octree.pitch;
//         cloudInfo2.imuYawInit = pose_octree.yaw;
//         cloudInfo2.initialGuessRoll = pose_octree.roll;
//         cloudInfo2.initialGuessPitch = pose_octree.pitch;
//         cloudInfo2.initialGuessYaw = pose_octree.yaw;
//         cloudInfo2.initialGuessX = point_octree.x;
//         cloudInfo2.initialGuessY = point_octree.y;
//         cloudInfo2.initialGuessZ = point_octree.z;
//         cloudInfo2.header.stamp = timestamp;
//         cloudInfo2.header.frame_id = ROBOT_ID + "/odom";
//         publish_cloudinfo(cloud,pub_laser_cloud2,ROBOT_ID,cloudInfo2,pub_cloudinfo2);
//     }
//     // else if(ROBOT_ID == "ouster3")
//     else if(ROBOT_ID == robot_id3)
//     {
//         inc_octree_buffer3.push_back(data_octree);
//         cloud_octree3->push_back(point_octree);
//         ResetValues();
//         Search_insert_octree(cloud_octree3,inc_octree_buffer3,m_merged_difftree_);
//         Get_pts_from_octree(m_merged_difftree_,cloud_trans_,cloud_octree3->back().x,cloud_octree3->back().y,cloud_octree3->back().z);
//         Trans_pts_to_lidar(cloud_trans_,cloud,pose_octree);
//         cloudInfo3.imuRollInit = pose_octree.roll;
//         cloudInfo3.imuPitchInit = pose_octree.pitch;
//         cloudInfo3.imuYawInit = pose_octree.yaw;
//         cloudInfo3.initialGuessRoll = pose_octree.roll;
//         cloudInfo3.initialGuessPitch = pose_octree.pitch;
//         cloudInfo3.initialGuessYaw = pose_octree.yaw;
//         cloudInfo3.initialGuessX = point_octree.x;
//         cloudInfo3.initialGuessY = point_octree.y;
//         cloudInfo3.initialGuessZ = point_octree.z;
//         cloudInfo3.header.stamp = timestamp;
//         cloudInfo3.header.frame_id = ROBOT_ID + "/odom";
//         publish_cloudinfo(cloud,pub_laser_cloud3,ROBOT_ID,cloudInfo3,pub_cloudinfo3);
//     }
//         // else if(ROBOT_ID == "ouster4")
//         else if(ROBOT_ID == robot_id4)
//         {
//             inc_octree_buffer4.push_back(data_octree);
//             cloud_octree4->push_back(point_octree);
//             ResetValues();
//             Search_insert_octree(cloud_octree4,inc_octree_buffer4,m_merged_difftree_);
//             Get_pts_from_octree(m_merged_difftree_,cloud_trans_,cloud_octree4->back().x,cloud_octree4->back().y,cloud_octree4->back().z);
//             Trans_pts_to_lidar(cloud_trans_,cloud,pose_octree);
//             cloudInfo4.imuRollInit = pose_octree.roll;
//             cloudInfo4.imuPitchInit = pose_octree.pitch;
//             cloudInfo4.imuYawInit = pose_octree.yaw;
//             cloudInfo4.initialGuessRoll = pose_octree.roll;
//             cloudInfo4.initialGuessPitch = pose_octree.pitch;
//             cloudInfo4.initialGuessYaw = pose_octree.yaw;
//             cloudInfo4.initialGuessX = point_octree.x;
//             cloudInfo4.initialGuessY = point_octree.y;
//             cloudInfo4.initialGuessZ = point_octree.z;
//             cloudInfo4.header.stamp = timestamp;
//             cloudInfo4.header.frame_id = ROBOT_ID + "/odom";
//             publish_cloudinfo(cloud,pub_laser_cloud4,ROBOT_ID,cloudInfo4,pub_cloudinfo4);
//         }
//     else
//     {
//         cout<<"no right ID"<<endl;
//     }
//         double t2 = ros::Time::now().toSec();
//         std::cout<<"sum time is  : "<<(t2-t1)<<std::endl;
// }

void IncOctreeProcess::IncOctreeHandler0(const grid_map::inc_octreeConstPtr& msgIn)
{
    timestamp = timestamp.fromSec(msgIn->time);
    double t1 = ros::Time::now().toSec();
    
    cout << robot_id0 << endl;
    
    PointTypePose pose_octree;
    pose_octree.x = msgIn->poseX;
    pose_octree.y = msgIn->poseY;
    pose_octree.z = msgIn->poseZ;
    pose_octree.roll = msgIn->poseRoll;
    pose_octree.pitch = msgIn->posePitch;
    pose_octree.yaw = msgIn->poseYaw;
    
    PointType point_octree;
    point_octree.x = msgIn->poseX;
    point_octree.y = msgIn->poseY;
    point_octree.z = msgIn->poseZ;
    
    std_msgs::Int8MultiArray data_octree;
    data_octree.data = msgIn->incr_octree.data;
    
    std::cout << "enter robot0" << std::endl;
    inc_octree_buffer0.push_back(data_octree);
    cloud_octree0->push_back(point_octree);
    ResetValues();
    
    double tt1 = ros::Time::now().toSec();
    Search_insert_octree(cloud_octree0, inc_octree_buffer0, m_merged_difftree_);
    double tt2 = ros::Time::now().toSec();
    
    Get_pts_from_octree(m_merged_difftree_, cloud_trans_, cloud_octree0->back().x, cloud_octree0->back().y, cloud_octree0->back().z);
    double tt3 = ros::Time::now().toSec();
    
    Trans_pts_to_lidar(cloud_trans_, cloud, pose_octree);
    double tt4 = ros::Time::now().toSec();
    
    cloudInfo0.imuRollInit = pose_octree.roll;
    cloudInfo0.imuPitchInit = pose_octree.pitch;
    cloudInfo0.imuYawInit = pose_octree.yaw;
    cloudInfo0.initialGuessRoll = pose_octree.roll;
    cloudInfo0.initialGuessPitch = pose_octree.pitch;
    cloudInfo0.initialGuessYaw = pose_octree.yaw;
    cloudInfo0.initialGuessX = point_octree.x;
    cloudInfo0.initialGuessY = point_octree.y;
    cloudInfo0.initialGuessZ = point_octree.z;
    cloudInfo0.header.stamp = timestamp;
    cloudInfo0.header.frame_id = robot_id0 + "/odom";
    
    publish_cloudinfo(cloud, pub_laser_cloud0, robot_id0, cloudInfo0, pub_cloudinfo0);
    
    double t2 = ros::Time::now().toSec();
    // std::cout << "robot0 sum time is: " << (t2-t1) << std::endl;
}

void IncOctreeProcess::IncOctreeHandler1(const grid_map::inc_octreeConstPtr& msgIn)
{
    timestamp = timestamp.fromSec(msgIn->time);
    double t1 = ros::Time::now().toSec();
    
    cout << robot_id1 << endl;
    
    PointTypePose pose_octree;
    pose_octree.x = msgIn->poseX;
    pose_octree.y = msgIn->poseY;
    pose_octree.z = msgIn->poseZ;
    pose_octree.roll = msgIn->poseRoll;
    pose_octree.pitch = msgIn->posePitch;
    pose_octree.yaw = msgIn->poseYaw;
    
    PointType point_octree;
    point_octree.x = msgIn->poseX;
    point_octree.y = msgIn->poseY;
    point_octree.z = msgIn->poseZ;
    
    std_msgs::Int8MultiArray data_octree;
    data_octree.data = msgIn->incr_octree.data;
    
    inc_octree_buffer1.push_back(data_octree);
    cloud_octree1->push_back(point_octree);
    ResetValues();
    
    Search_insert_octree(cloud_octree1, inc_octree_buffer1, m_merged_difftree_);
    Get_pts_from_octree(m_merged_difftree_, cloud_trans_, cloud_octree1->back().x, cloud_octree1->back().y, cloud_octree1->back().z);
    Trans_pts_to_lidar(cloud_trans_, cloud, pose_octree);
    
    cloudInfo1.imuRollInit = pose_octree.roll;
    cloudInfo1.imuPitchInit = pose_octree.pitch;
    cloudInfo1.imuYawInit = pose_octree.yaw;
    cloudInfo1.initialGuessRoll = pose_octree.roll;
    cloudInfo1.initialGuessPitch = pose_octree.pitch;
    cloudInfo1.initialGuessYaw = pose_octree.yaw;
    cloudInfo1.initialGuessX = point_octree.x;
    cloudInfo1.initialGuessY = point_octree.y;
    cloudInfo1.initialGuessZ = point_octree.z;
    cloudInfo1.header.stamp = timestamp;
    cloudInfo1.header.frame_id = robot_id1 + "/odom";
    
    publish_cloudinfo(cloud, pub_laser_cloud1, robot_id1, cloudInfo1, pub_cloudinfo1);
    
    double t2 = ros::Time::now().toSec();
    // std::cout << "robot1 sum time is: " << (t2-t1) << std::endl;
}

void IncOctreeProcess::IncOctreeHandler2(const grid_map::inc_octreeConstPtr& msgIn)
{
    timestamp = timestamp.fromSec(msgIn->time);
    double t1 = ros::Time::now().toSec();
    
    cout << robot_id2 << endl;
    
    PointTypePose pose_octree;
    pose_octree.x = msgIn->poseX;
    pose_octree.y = msgIn->poseY;
    pose_octree.z = msgIn->poseZ;
    pose_octree.roll = msgIn->poseRoll;
    pose_octree.pitch = msgIn->posePitch;
    pose_octree.yaw = msgIn->poseYaw;
    
    PointType point_octree;
    point_octree.x = msgIn->poseX;
    point_octree.y = msgIn->poseY;
    point_octree.z = msgIn->poseZ;
    
    std_msgs::Int8MultiArray data_octree;
    data_octree.data = msgIn->incr_octree.data;
    
    inc_octree_buffer2.push_back(data_octree);
    cloud_octree2->push_back(point_octree);
    ResetValues();
    
    Search_insert_octree(cloud_octree2, inc_octree_buffer2, m_merged_difftree_);
    Get_pts_from_octree(m_merged_difftree_, cloud_trans_, cloud_octree2->back().x, cloud_octree2->back().y, cloud_octree2->back().z);
    Trans_pts_to_lidar(cloud_trans_, cloud, pose_octree);
    
    cloudInfo2.imuRollInit = pose_octree.roll;
    cloudInfo2.imuPitchInit = pose_octree.pitch;
    cloudInfo2.imuYawInit = pose_octree.yaw;
    cloudInfo2.initialGuessRoll = pose_octree.roll;
    cloudInfo2.initialGuessPitch = pose_octree.pitch;
    cloudInfo2.initialGuessYaw = pose_octree.yaw;
    cloudInfo2.initialGuessX = point_octree.x;
    cloudInfo2.initialGuessY = point_octree.y;
    cloudInfo2.initialGuessZ = point_octree.z;
    cloudInfo2.header.stamp = timestamp;
    cloudInfo2.header.frame_id = robot_id2 + "/odom";
    
    publish_cloudinfo(cloud, pub_laser_cloud2, robot_id2, cloudInfo2, pub_cloudinfo2);
    
    double t2 = ros::Time::now().toSec();
    // std::cout << "robot2 sum time is: " << (t2-t1) << std::endl;
}

void IncOctreeProcess::IncOctreeHandler3(const grid_map::inc_octreeConstPtr& msgIn)
{
    timestamp = timestamp.fromSec(msgIn->time);
    double t1 = ros::Time::now().toSec();
    
    cout << robot_id3 << endl;
    
    PointTypePose pose_octree;
    pose_octree.x = msgIn->poseX;
    pose_octree.y = msgIn->poseY;
    pose_octree.z = msgIn->poseZ;
    pose_octree.roll = msgIn->poseRoll;
    pose_octree.pitch = msgIn->posePitch;
    pose_octree.yaw = msgIn->poseYaw;
    
    PointType point_octree;
    point_octree.x = msgIn->poseX;
    point_octree.y = msgIn->poseY;
    point_octree.z = msgIn->poseZ;
    
    std_msgs::Int8MultiArray data_octree;
    data_octree.data = msgIn->incr_octree.data;
    
    inc_octree_buffer3.push_back(data_octree);
    cloud_octree3->push_back(point_octree);
    ResetValues();
    
    Search_insert_octree(cloud_octree3, inc_octree_buffer3, m_merged_difftree_);
    Get_pts_from_octree(m_merged_difftree_, cloud_trans_, cloud_octree3->back().x, cloud_octree3->back().y, cloud_octree3->back().z);
    Trans_pts_to_lidar(cloud_trans_, cloud, pose_octree);
    
    cloudInfo3.imuRollInit = pose_octree.roll;
    cloudInfo3.imuPitchInit = pose_octree.pitch;
    cloudInfo3.imuYawInit = pose_octree.yaw;
    cloudInfo3.initialGuessRoll = pose_octree.roll;
    cloudInfo3.initialGuessPitch = pose_octree.pitch;
    cloudInfo3.initialGuessYaw = pose_octree.yaw;
    cloudInfo3.initialGuessX = point_octree.x;
    cloudInfo3.initialGuessY = point_octree.y;
    cloudInfo3.initialGuessZ = point_octree.z;
    cloudInfo3.header.stamp = timestamp;
    cloudInfo3.header.frame_id = robot_id3 + "/odom";
    
    publish_cloudinfo(cloud, pub_laser_cloud3, robot_id3, cloudInfo3, pub_cloudinfo3);
    
    double t2 = ros::Time::now().toSec();
    // std::cout << "robot3 sum time is: " << (t2-t1) << std::endl;
}

void IncOctreeProcess::IncOctreeHandler4(const grid_map::inc_octreeConstPtr& msgIn)
{
    timestamp = timestamp.fromSec(msgIn->time);
    double t1 = ros::Time::now().toSec();
    
    cout << robot_id4 << endl;
    
    PointTypePose pose_octree;
    pose_octree.x = msgIn->poseX;
    pose_octree.y = msgIn->poseY;
    pose_octree.z = msgIn->poseZ;
    pose_octree.roll = msgIn->poseRoll;
    pose_octree.pitch = msgIn->posePitch;
    pose_octree.yaw = msgIn->poseYaw;
    
    PointType point_octree;
    point_octree.x = msgIn->poseX;
    point_octree.y = msgIn->poseY;
    point_octree.z = msgIn->poseZ;
    
    std_msgs::Int8MultiArray data_octree;
    data_octree.data = msgIn->incr_octree.data;
    
    inc_octree_buffer4.push_back(data_octree);
    cloud_octree4->push_back(point_octree);
    ResetValues();
    
    Search_insert_octree(cloud_octree4, inc_octree_buffer4, m_merged_difftree_);
    Get_pts_from_octree(m_merged_difftree_, cloud_trans_, cloud_octree4->back().x, cloud_octree4->back().y, cloud_octree4->back().z);
    Trans_pts_to_lidar(cloud_trans_, cloud, pose_octree);
    
    cloudInfo4.imuRollInit = pose_octree.roll;
    cloudInfo4.imuPitchInit = pose_octree.pitch;
    cloudInfo4.imuYawInit = pose_octree.yaw;
    cloudInfo4.initialGuessRoll = pose_octree.roll;
    cloudInfo4.initialGuessPitch = pose_octree.pitch;
    cloudInfo4.initialGuessYaw = pose_octree.yaw;
    cloudInfo4.initialGuessX = point_octree.x;
    cloudInfo4.initialGuessY = point_octree.y;
    cloudInfo4.initialGuessZ = point_octree.z;
    cloudInfo4.header.stamp = timestamp;
    cloudInfo4.header.frame_id = robot_id4 + "/odom";
    
    publish_cloudinfo(cloud, pub_laser_cloud4, robot_id4, cloudInfo4, pub_cloudinfo4);
    
    double t2 = ros::Time::now().toSec();
    // std::cout << "robot4 sum time is: " << (t2-t1) << std::endl;
}

IncOctreeProcess::IncOctreeProcess(ros::NodeHandle& nh):nh_(nh)
{
    nh_.param<std::string>("robot_id0", robot_id0, "jackal0");
    nh_.param<std::string>("robot_id1", robot_id1, "jackal1");
    nh_.param<std::string>("robot_id2", robot_id2, "jackal2");
    nh_.param<std::string>("robot_id3", robot_id3, "jackal3");
    nh_.param<std::string>("robot_id4", robot_id4, "jackal4");
    nh_.param<std::string>("inc_octree_topic",inc_octree_topic, "lio_sam/mapping/inc_octree");
    //std::cout<< "robot_id0" << robot_id0 << ";" << "robot_id3" << robot_id3 <<std::endl;

    sub_inc_octree0 = nh_.subscribe<grid_map::inc_octree>(robot_id0 + "/" + inc_octree_topic, 10000, &IncOctreeProcess::IncOctreeHandler0, this);
    sub_inc_octree1 = nh_.subscribe<grid_map::inc_octree>(robot_id1 + "/" + inc_octree_topic, 10000, &IncOctreeProcess::IncOctreeHandler1, this);
    sub_inc_octree2 = nh_.subscribe<grid_map::inc_octree>(robot_id2 + "/" + inc_octree_topic, 10000, &IncOctreeProcess::IncOctreeHandler2, this);
    sub_inc_octree3 = nh_.subscribe<grid_map::inc_octree>(robot_id3 + "/" + inc_octree_topic, 10000, &IncOctreeProcess::IncOctreeHandler3, this);
    sub_inc_octree4 = nh_.subscribe<grid_map::inc_octree>(robot_id4 + "/" + inc_octree_topic, 10000, &IncOctreeProcess::IncOctreeHandler4, this);
    //为了在rviz里显示，输出sensor_msgs的雷达系下点云
    /*
    pub_laser_cloud0 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id0 + "/point_local_octree", 10000);
    pub_laser_cloud1 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id1 + "/point_local_octree", 10000);
    pub_laser_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id2 + "/point_local_octree", 10000);
    pub_laser_cloud3 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id3 + "/point_local_octree", 10000);
    */
    pub_laser_cloud0 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id0 + "/point_local_octree", 10000);
    pub_laser_cloud1 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id1 + "/point_local_octree", 10000);
    pub_laser_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id2 + "/point_local_octree", 10000);
    pub_laser_cloud3 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id3 + "/point_local_octree", 10000);
    pub_laser_cloud4 = nh_.advertise<sensor_msgs::PointCloud2>(robot_id4 + "/point_local_octree", 10000);
    //为了后续代码使用的cloudinfo发布
    /*
     pub_cloudinfo0 = nh_.advertise<lio_sam::cloud_info>(robot_id0 + "/cloudinfo_registered", 10000);
    pub_cloudinfo1 = nh_.advertise<lio_sam::cloud_info>(robot_id1 + "/cloudinfo_registered", 10000);
    pub_cloudinfo2 = nh_.advertise<lio_sam::cloud_info>(robot_id2 + "/cloudinfo_registered", 10000);
    pub_cloudinfo3 = nh_.advertise<lio_sam::cloud_info>(robot_id3 + "/cloudinfo_registered", 10000);   
    */
    pub_cloudinfo0 = nh_.advertise<lio_sam::cloud_info>(robot_id0 + "/lio_sam/mapping/cloud_info", 10000);
    pub_cloudinfo1 = nh_.advertise<lio_sam::cloud_info>(robot_id1 + "/lio_sam/mapping/cloud_info", 10000);
    pub_cloudinfo2 = nh_.advertise<lio_sam::cloud_info>(robot_id2 + "/lio_sam/mapping/cloud_info", 10000);
    pub_cloudinfo3 = nh_.advertise<lio_sam::cloud_info>(robot_id3 + "/lio_sam/mapping/cloud_info", 10000);
    pub_cloudinfo4 = nh_.advertise<lio_sam::cloud_info>(robot_id4 + "/lio_sam/mapping/cloud_info", 10000);
    nh_.param<double>("/resolution_octree", resolution_octree_, 0.1);//加载八叉树分辨率
    nh_.param<double>("/probHit", probHit, 0.7);//hit概率更新步长
    nh_.param<double>("/probMiss", probMiss, 0.4);//miss概率更新步长
    nh_.param<double>("/thresMin", thresMin, 0.12);//占据概率下限
    nh_.param<double>("/thresMax", thresMax, 0.97);//占据概率上限
    nh_.param<double>("/octree_range", octree_range_, 50.0);//占据概率上限

    m_merged_difftree_ = new octomap::OcTree(resolution_octree_);
    m_merged_difftree_->setProbHit(probHit);
    m_merged_difftree_->setProbMiss(probMiss);
    m_merged_difftree_->setClampingThresMin(thresMin);
    m_merged_difftree_->setClampingThresMax(thresMax);

    temp_octree_ = new octomap::OcTree(resolution_octree_);
    temp_octree_->setProbHit(probHit);
    temp_octree_->setProbMiss(probMiss);
    temp_octree_->setClampingThresMin(thresMin);
    temp_octree_->setClampingThresMax(thresMax);

    cloud_octree0.reset(new pcl::PointCloud<PointType>());
    cloud_octree1.reset(new pcl::PointCloud<PointType>());
    cloud_octree2.reset(new pcl::PointCloud<PointType>());
    cloud_octree3.reset(new pcl::PointCloud<PointType>());
    cloud_octree4.reset(new pcl::PointCloud<PointType>());

    
}

void IncOctreeProcess::ResetValues()
{
    m_merged_difftree_->clear();
    temp_octree_->clear();
    cloud_trans_.clear();
    cloud.reset(new pcl::PointCloud<PointType>());
    
}


void IncOctreeProcess::Search_insert_octree(pcl::PointCloud<PointType>::Ptr& OctreePoses, std::vector<std_msgs::Int8MultiArray> _octree_lis,
                              octomap::OcTree*& m_merged_difftree) 
{
    m_merged_difftree->clear();
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    std::vector<int> pointSearchIndGlobalMap;//找到的20m内的位姿点的索引
    std::vector<float> pointSearchSqDisGlobalMap;
    kdtreeGlobalMap->setInputCloud(OctreePoses);
    kdtreeGlobalMap->radiusSearch(OctreePoses->back(), 10.0, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    for (int i = (int)pointSearchIndGlobalMap.size() - 1; i >= 0; --i) 
    {
//        cout<<pointSearchIndGlobalMap[i]<<endl;
        std::string datastr;
//        if(_octree_lis[pointSearchIndGlobalMap[i]].data.empty())
//        cout<<"empty!!!"<<endl;
        for(auto it = std::begin(_octree_lis[pointSearchIndGlobalMap[i]].data);it != std::end(_octree_lis[pointSearchIndGlobalMap[i]].data);++it) 
        {
            datastr += char(*it);//int8数组转字符串
        }
        
        
        //temp_octree_->clear();
        
        std::stringstream datastream(datastr);//字符串转为字符串流
        if(!datastr.empty())
        {
                    if (!temp_octree_->readBinary(datastream))
        ROS_ERROR("Error serializing readBinary"); //bt文件输出为字符串流
        }

       
        

        
        ///temp_octree->expand();
        for (octomap::OcTree::iterator it = temp_octree_->begin(), end = temp_octree_->end(); it != end; ++it) 
        {
            
            octomap::OcTreeKey nodeKey = it.getKey();
            m_merged_difftree->setNodeValue(nodeKey, it->getLogOdds());//拼接成局部总八叉树
        }
        
    }
}

void IncOctreeProcess::Get_pts_from_octree(octomap::OcTree*& m_merged_difftree, pcl::PointCloud<PointType>& cloud_trans, float x, float y, float z)
{
        octomap::point3d center(x, y, z);
        for (octomap::OcTree::iterator it = m_merged_difftree->begin(), end = m_merged_difftree->end(); it != end; ++it) {
            octomap::point3d point = it.getCoordinate();
            octomap::OcTreeKey key = m_merged_difftree->coordToKey(point, 16);//将octomap坐标转换为对应的键值
            octomap::OcTreeNode* node = m_merged_difftree->search(key, 16);//在最大深度搜索键值对应的节点
            PointType pointTmp;
            if(node->getOccupancy() >= 0.5) {
                if ((point - center).norm() <= 100.0) {
                    pointTmp.x = point.x();
                    pointTmp.y = point.y();
                    pointTmp.z = point.z();
                    cloud_trans.push_back(pointTmp);
                }
            }
        }
}

void IncOctreeProcess::Trans_pts_to_lidar(pcl::PointCloud<PointType>& cloud_trans, pcl::PointCloud<PointType>::Ptr& cloud_pub,
                             PointTypePose pose) 
{
        cloud_pub->resize(cloud_trans.size());
        Eigen::Affine3f _T_w_l_ = pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
        Eigen::Isometry3d T_l_w ;          //   world2body
        T_l_w.matrix() = _T_w_l_.inverse().matrix().cast<double>();
        for (int i = 0; i < cloud_trans.points.size(); ++i) {
            cloud_pub->points[i].x = T_l_w(0, 0) * cloud_trans.points.at(i).x + T_l_w(0, 1) * cloud_trans.points.at(i).y + T_l_w(0, 2) * cloud_trans.points.at(i).z + T_l_w(0, 3);
            cloud_pub->points[i].y = T_l_w(1, 0) * cloud_trans.points.at(i).x + T_l_w(1, 1) * cloud_trans.points.at(i).y + T_l_w(1, 2) * cloud_trans.points.at(i).z + T_l_w(1, 3);
            cloud_pub->points[i].z = T_l_w(2, 0) * cloud_trans.points.at(i).x + T_l_w(2, 1) * cloud_trans.points.at(i).y + T_l_w(2, 2) * cloud_trans.points.at(i).z + T_l_w(2, 3);
            cloud_pub->points[i].intensity = 100;
        }
}

void IncOctreeProcess::publish_cloudinfo(pcl::PointCloud<PointType>::Ptr cloud_need,const ros::Publisher & pubcloud,std::string robot_id,lio_sam::cloud_info &cloudIn,const ros::Publisher & pubcloudinfo)
{
    std::cout<<"pub "<< robot_id<<"; points is : "<<cloud_need->points.size()<<std::endl;
    if (cloud_need->points.size() < 10)
        return;
    sensor_msgs::PointCloud2 laser_cloud;
    pcl::toROSMsg(*cloud_need, laser_cloud);
    laser_cloud.header.stamp = timestamp;
    laser_cloud.header.frame_id = "map";
    pubcloud.publish(laser_cloud);
    cloudIn.cloud_deskewed = laser_cloud;
    
    pubcloudinfo.publish(cloudIn);
}
