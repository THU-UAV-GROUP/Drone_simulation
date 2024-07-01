#include "BPRFD/utils.h"

bool point_cmp(MyPointType& a, MyPointType& b)
{
    return a.z < b.z;
}

void transformPointType(const pcl::PointCloud<MyPointType>::Ptr& old_cloud, pcl::PointCloud<PointType>::Ptr& new_cloud)
{
    for (auto& old_point: old_cloud->points)
    {
        PointType new_point;

        new_point.x = old_point.x;
        new_point.y = old_point.y;
        new_point.z = old_point.z;
        new_point.intensity = old_point.intensity;

        new_cloud->points.push_back(new_point);
    }
}

void transformPointType(const pcl::PointCloud<PointType>::Ptr& old_cloud, pcl::PointCloud<MyPointType>::Ptr& new_cloud)
{
    for (auto& old_point: old_cloud->points)
    {
        MyPointType new_point;

        new_point.x = old_point.x;
        new_point.y = old_point.y;
        new_point.z = old_point.z;
        new_point.intensity = old_point.intensity;
        new_point.global_id = 0;
        new_point.is_ground = 0;
        new_point.too_low = 0;
        new_point.is_invalid = 0;

        new_cloud->points.push_back(new_point);
    }
}

void pcd2Matrix(const pcl::PointCloud<MyPointType>::Ptr& cloud, Eigen::MatrixXf& matrix)
{
    matrix.resize(cloud->points.size(), 3);
    int j = 0;
    for (auto& point : cloud->points)
    {
        matrix.row(j++) << point.x, point.y, point.z;
    }
}

void setGlobalId(pcl::PointCloud<MyPointType>::Ptr& ori, int& globalId)
{
    for (auto & point : ori->points)
    {
        point.global_id = globalId;
        globalId++;
    }  
}