#include "map_management.h"

map_management::map_management()
{
    pointcloudmap.reset(new pcl::PointCloud<PointType>()); 
    pointcloud_output.reset(new pcl::PointCloud<PointType>());
    pointcloud_add.reset(new pcl::PointCloud<PointType>());
    cloud_copy.reset(new pcl::PointCloud<PointType>());
    xmin = FLT_MAX;
    xmax = FLT_MIN;
    ymin = FLT_MAX;
    ymax = FLT_MIN;

    voxel_size = 10; //10
    voxel_num = 8;//6
    ds_size = 0.4;
    first_pos = true;
    need_relocal = false;
}

map_management::~map_management() {}

//初始化点云数组
void map_management::Initialize()
{
    vec_voxel.reserve((Dx + 1)*(Dy + 1));

    for(int i=0;i<(Dx + 1)*(Dy + 1);i++)
    {
        pcl::PointCloud<PointType>::Ptr pointcloud_tmp(new pcl::PointCloud<PointType>());
        vec_voxel.push_back(pointcloud_tmp);
    }
}

void map_management::set_ds_size(double ds_size_)
{
    ds_size = ds_size_;
}

void map_management::set_input_PCD(string read_dir)
{
    pcl::PCDReader pcd_reader;
    pcd_reader.read(read_dir, *pointcloudmap);

    pcl::VoxelGrid<PointType> DSpointcloudmap;
    DSpointcloudmap.setInputCloud(pointcloudmap);
    DSpointcloudmap.setLeafSize(ds_size,ds_size,ds_size);
    DSpointcloudmap.filter(*pointcloudmap);

    for(size_t i = 0;i<pointcloudmap->points.size();i++)
    {
        if(pointcloudmap->points[i].x > xmax)
            xmax = pointcloudmap->points[i].x;

        if(pointcloudmap->points[i].x < xmin)
            xmin = pointcloudmap->points[i].x;

        if(pointcloudmap->points[i].y > ymax)
            ymax = pointcloudmap->points[i].y;

        if(pointcloudmap->points[i].y < ymin)
            ymin = pointcloudmap->points[i].y;
    }

    calculate_DxDy();

    Initialize();
}

void map_management::calculate_DxDy()
{
    Dx = (xmax - xmin) / voxel_size;
    Dy = (ymax - ymin) / voxel_size;

    // cout<<"Dx:"<<Dx<<" Dy:"<<Dy<<endl;
}

void map_management::print_xy()
{
    printf("xmax:%lf xmin:%lf ymax:%lf ymin:%lf \n",xmax,xmin,ymax,ymin);
}

//把点放到对应栅格里
void map_management::voxel_process()
{
    // cout<<"size:"<<vec_voxel.size()<<endl;
    for(size_t i = 0;i<pointcloudmap->points.size();i++)
    {
        int dx = (pointcloudmap->points[i].x - xmin) / voxel_size;
        int dy = (pointcloudmap->points[i].y - ymin) / voxel_size;
        int index = dx + dy * Dx;
        // if(index >= vec_voxel.size())
        // {
        //     cout<<"dx:"<<dx<<" dy:"<<dy<<" index:"<<index<<endl;
        // }
        vec_voxel[index]->push_back(pointcloudmap->points[i]);
    }
}

void map_management::output_map()
{
    for(size_t i=0;i<1500;i++)
    {
        *pointcloud_output += *vec_voxel[i];
    }

    string savemappath = "/home/firefly/na_mapping_lslidar/src/na_mapping/PCD/map_output.pcd";
    pcl::io::savePCDFileBinary(savemappath,*pointcloud_output);
}

void map_management::find_neibor(int dx,int dy)
{
    pointcloud_output->clear();
    //cout<<"333333333"<<endl;
    for(int i=dx-voxel_num;i<=dx+voxel_num;i++)
    {
        if(i <0 || i > Dx) continue;
        for(int j=dy-voxel_num;j<=dy+voxel_num;j++)
        {
            if(j < 0 || j > Dy) continue;
            int index = i + j * Dx;
            *pointcloud_output += *vec_voxel[index];
        }
    }
}

bool map_management::get_map(double posx,double posy)
{
    int dx = (posx - xmin) / voxel_size;
    int dy = (posy - ymin) / voxel_size;

    if(dx < 0 || dx > Dx || dy < 0 || dy > Dy)
    {
        ROS_ERROR("Invaild position");
        need_relocal = true;
        return false;
    }
        
        
    if(first_pos)
    {
        dx_last = dx;
        dy_last = dy;
        first_pos = false;
        find_neibor(dx,dy);
        return true;
    }
    else
    {
        if(dx == dx_last && dy == dy_last)
            return false;

        find_neibor(dx,dy);
        dx_last = dx;
        dy_last = dy;
        return true;
    }
}
//计算新增点云
bool map_management::get_map_add(double posx,double posy)
{
    int dx = (posx - xmin) / voxel_size;
    int dy = (posy - ymin) / voxel_size;   
    
    //输入位置超出了地图范围
    if(dx < 0 || dx > Dx || dy < 0 || dy > Dy)
    {
        ROS_ERROR("Invaild Position, Need Relocal");
        need_relocal = true;
        return false;
    }
    else
        need_relocal = false;

    if(dx == dx_last && dy == dy_last)
        return false;

    pointcloud_add->clear();
    //判断移动方向
    int delta_x = dx - dx_last;
    int delta_y = dy - dy_last;

    if(delta_x == 0) //x方向上没有移动
    {
        for(int i=dx-voxel_num;i<=dx+voxel_num;i++)
        {
            if(i < 0 || i > Dx) continue;

            int index;

            if(delta_y > 0)
            {
                index = i + Dx * (dy + voxel_num);
                if(dy + voxel_num > Dy) continue;
            }
            else if(delta_y < 0)
            {
                index = i + Dx * (dy - voxel_num);
                if(dy - voxel_num < 0) continue;
            }

            *pointcloud_add += *vec_voxel[index];               
        }
    }
    else if(delta_y == 0) //y方向上没有移动
    {
        for(int i=dy-voxel_num;i<=dy+voxel_num;i++)
        {
            if(i < 0 || i > Dy) continue;

            int index;

            if(delta_x > 0)
            {
                index = dx + voxel_num + Dx * i;
                if(dx + voxel_num > Dx) continue;
            }
            else if(delta_x < 0)
            {
                index = dx - voxel_num + Dx * i;
                if(dx - voxel_num < 0) continue;
            }

            *pointcloud_add += *vec_voxel[index];               
        }        
    }
    else //x、y方向上都发生移动
    {
        // cout<<"both move!"<<endl;

        for(int i=dx-voxel_num;i<=dx+voxel_num;i++)
        {
            if(i < 0 || i > Dx) continue;

            int index;

            if(delta_y > 0)
            {
                index = i + Dx * (dy + voxel_num);
                if(dy + voxel_num > Dy) continue;
            }
            else if(delta_y < 0)
            {
                index = i + Dx * (dy - voxel_num);
                if(dy - voxel_num < 0) continue;
            }

            *pointcloud_add += *vec_voxel[index];               
        } 

        for(int i=dy-voxel_num;i<=dy+voxel_num;i++)
        {
            if(i < 0 || i > Dy) continue;

            int index;

            if(delta_x > 0)
            {
                index = dx + voxel_num + Dx * i;
                if(dx + voxel_num > Dx) continue;
            }
            else if(delta_x < 0)
            {
                index = dx - voxel_num + Dx * i;
                if(dx - voxel_num < 0) continue;
            }

            *pointcloud_add += *vec_voxel[index];               
        }         
    }

    *pointcloud_output +=  *pointcloud_add;

    dx_last = dx;
    dy_last = dy;

    return true;
}
