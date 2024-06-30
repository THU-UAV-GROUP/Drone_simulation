#pragma once

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>

struct PointXYZIWithFlag : public pcl::PointXYZI
{
    int     global_id;      
    int     is_ground;
    int     too_low;     
    int     is_invalid;     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT
(PointXYZIWithFlag,
    (float, x,          x)
    (float, y,          y)
    (float, z,          z)
    (float, intensity,  intensity)  
    (int,   too_low,    too_low)
    (int,   is_invalid,   is_invalid)
    (int,   is_ground,  is_ground)
)

using PointType = pcl::PointXYZI;
using MyPointType = PointXYZIWithFlag;

bool point_cmp(MyPointType& a, MyPointType& b);

void transformPointType(const pcl::PointCloud<PointType>::Ptr& old_cloud, pcl::PointCloud<MyPointType>::Ptr& new_cloud);

void transformPointType(const pcl::PointCloud<MyPointType>::Ptr& old_cloud, pcl::PointCloud<PointType>::Ptr& new_cloud);

void pcd2Matrix(const pcl::PointCloud<MyPointType>::Ptr& cloud, Eigen::MatrixXf& matrix);

void setGlobalId(pcl::PointCloud<MyPointType>::Ptr& ori, int& globalId);