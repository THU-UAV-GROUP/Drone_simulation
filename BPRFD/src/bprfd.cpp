#include "BPRFD/bprfd.h"

BPRFD::BPRFD()
{
    getParams();

    allocateMemory();
}

BPRFD::~BPRFD() {}

void BPRFD::getParams()
{
    ROS_INFO_STREAM("\033[1;32m Getting params begin. \033[0m");
    std::cout << std::endl;

    // base control
    nh.param<bool>("bprfd/base_control/debug_mode", debug_mode, false);
    ROS_INFO_STREAM("\033[1;32m -- whether use debug mode: " << (debug_mode ? "true" : "false") << " \033[0m");
    std::cout << std::endl;

    // ros node
    nh.param<std::string>("bprfd/ros_control/raw_topic", raw_topic, "");
    raw_pcd_subscriber = nh.subscribe(raw_topic, 10, &BPRFD::laserCallback, this); 
    ROS_INFO_STREAM("\033[1;32m -- raw topic node: " << raw_topic << " \033[0m");
    std::cout << std::endl;

    nh.param<std::string>("bprfd/ros_control/ground_topic", ground_topic, "");
    ground_pcd_publisher = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    ROS_INFO_STREAM("\033[1;32m -- ground topic node: " << ground_topic << " \033[0m");
    std::cout << std::endl;

    nh.param<std::string>("bprfd/ros_control/non_ground_topic", non_ground_topic, "");
    non_ground_pcd_publisher = nh.advertise<sensor_msgs::PointCloud2>(non_ground_topic, 10);
    ROS_INFO_STREAM("\033[1;32m -- non-ground topic node: " << non_ground_topic << " \033[0m");
    std::cout << std::endl;

    nh.param<std::string>("bprfd/ros_control/invalid_topic", invalid_topic, "");
    invalid_pcd_publisher = nh.advertise<sensor_msgs::PointCloud2>(invalid_topic, 10);
    ROS_INFO_STREAM("\033[1;32m -- invalid topic node: " << invalid_topic << " \033[0m");
    std::cout << std::endl;
    

    // ground detection
    nh.param<float>("bprfd/detection_param/relative_sensor_height", relative_sensor_height, 1.77);

    nh.param<int>("bprfd/detection_param/initial_seeds_count", initial_seeds_count, 20);

    nh.param<float>("bprfd/detection_param/seed_z_thre", seed_z_thre, 1.2);

    nh.param<int>("bprfd/detection_param/ground_extraction_count", ground_extraction_count, 20);

    nh.param<float>("bprfd/detection_param/seed_dist_thre", SEED_DIST_THRE, 0.3);

    nh.param<float>("bprfd/detection_param/too_high_thre", too_high_thre, 4.0);

    nh.param<float>("bprfd/detection_param/min_dist", min_dist, 2.0);

    nh.param<float>("bprfd/detection_param/max_dist", max_dist, 75.0);

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- relative sensor height: " << relative_sensor_height << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- initial seeds count: " << initial_seeds_count << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- seed z threshold: " << seed_z_thre << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- ground extraction count: " << ground_extraction_count << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- seed distance threshold: " << SEED_DIST_THRE << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- too high threshold: " << too_high_thre << " \033[0m");
        std::cout << std::endl;

        ROS_INFO_STREAM("\033[1;32m -- min distance threshold: " << min_dist << " \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- max distance threshold: " << max_dist << " \033[0m");
        std::cout << std::endl;
    }

    ROS_INFO_STREAM("\033[1;32m Getting params done. \033[0m");
    std::cout << std::endl;
}

void BPRFD::allocateMemory()
{
    // reset
    ori_pcd.reset(new pcl::PointCloud<PointType>());

    raw_pcd.reset(new pcl::PointCloud<MyPointType>());

    ground_pcd.reset(new pcl::PointCloud<MyPointType>());
    non_ground_pcd.reset(new pcl::PointCloud<MyPointType>());

    invalid_pcd.reset(new pcl::PointCloud<MyPointType>());

    // clear
    ori_pcd->clear();

    raw_pcd->clear();

    ground_pcd->clear();
    non_ground_pcd->clear();

    invalid_pcd->clear();
}

void BPRFD::laserCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // preprocess 
    preprocess(cloud_msg, raw_pcd);

    // process
    process(raw_pcd, ground_pcd, non_ground_pcd);

    // postprocess
    postprocess(raw_pcd, ground_pcd, non_ground_pcd, invalid_pcd);
}

void BPRFD::preprocess(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<MyPointType>::Ptr raw_pcd)
{
    ROS_INFO_STREAM("\033[1;32m Preprocess begin. \033[0m");
    std::cout << std::endl;

    // get raw pcd info
    pcl::fromROSMsg(*cloud_msg, *ori_pcd); 

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- this frame has " << ori_pcd->size() << " points. \033[0m");
        std::cout << std::endl;
    }

    // transform pcd type
    transformPointType(ori_pcd, raw_pcd);

    setGlobalId(raw_pcd, global_id);

    // remove the point too low
    detectTooLowPoint(raw_pcd);

    ROS_INFO_STREAM("\033[1;32m Preprocess done. \033[0m");
    std::cout << std::endl;
}

void BPRFD::process(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud)
{
    ROS_INFO_STREAM("\033[1;32m Process begin. \033[0m");
    std::cout << std::endl;

    // get initial ground seeds
    extractInitialSeeds(input_cloud, ground_cloud);

    // get ground points by iteration
    extractGroundPoints(input_cloud, ground_cloud, non_ground_cloud);

    ROS_INFO_STREAM("\033[1;32m Process done. \033[0m");
    std::cout << std::endl;
}

void BPRFD::postprocess(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, pcl::PointCloud<MyPointType>::Ptr& invalid_cloud)
{
    ROS_INFO_STREAM("\033[1;32m Postprocess begin. \033[0m");
    std::cout << std::endl;

    detectInvalidPoint(input_cloud);

    parseAndPubFinalPCD(input_cloud, ground_cloud, non_ground_cloud, invalid_cloud);

    ROS_INFO_STREAM("\033[1;32m Postprocess done. \033[0m");
    std::cout << std::endl;
}

void BPRFD::extractInitialSeeds(const pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& seeds_cloud)
{
    assert (0 < initial_seeds_count && initial_seeds_count <= input_cloud->points.size());

    double seed_z_sum = 0.0;
    double seed_z_mean = 0.0;

    for (int i = low_invalid_count; i < low_invalid_count + initial_seeds_count; ++i)
    {
        seed_z_sum += input_cloud->points[i].z;
    }

    seed_z_mean = seed_z_sum / initial_seeds_count;

    seeds_cloud->clear();
    int cnt = 0;
    for (auto& point : input_cloud->points)
    {
        if (point.too_low == 1)
        {
            continue;
        }

        if (point.z < seed_z_mean + seed_z_thre)
        {
            seeds_cloud->points.push_back(point);
            cnt++;
        }
    }

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- the mean height of ground seeds: " << seed_z_mean << " \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- now the ground seeds have " << seeds_cloud->size() << " points. \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::extractGroundPoints(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud)
{
    for (int i = 0; i < ground_extraction_count; ++i)
    {
        // estimate adaptive distance threshold
        estimateGroundThre(ground_cloud, seeds_normal_vector, seeds_dist_thre);

        // pcd to matrix
        pcd2Matrix(input_cloud, point_matrix);

        // detect ground points using threshold
        detectGroundPoints(input_cloud, ground_cloud, non_ground_cloud, seeds_normal_vector, seeds_dist_thre);
    }
}

void BPRFD::estimateGroundThre(const pcl::PointCloud<MyPointType>::Ptr& ground_cloud, Eigen::MatrixXf& normal_vector, float& dist_thre)
{
    // compute mean and covariance of seeds
    Eigen::Matrix3f seeds_cov;
    Eigen::Vector4f seeds_mean;
    pcl::computeMeanAndCovarianceMatrix(*ground_cloud, seeds_cov, seeds_mean);

    // use svd to get normal vector and mean coordinate
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(seeds_cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_vector = (svd.matrixU().col(2));
    seeds_mean_vector = seeds_mean.head<3>();

    // get the distance between seed's mean coordinate and seed's plane
    seeds_dist = -(seeds_normal_vector.transpose() * seeds_mean_vector)(0, 0);
    dist_thre = SEED_DIST_THRE - seeds_dist;

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- the distance between seed's mean coordinate and seed's plane: " << seeds_dist << " \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- the adaptive seeds distance threshold: " << dist_thre << " \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::detectGroundPoints(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, Eigen::MatrixXf& normal_vector, float& dist_thre)
{
    ground_cloud->clear();
    non_ground_cloud->clear();

    dist_result = point_matrix * normal_vector;

    for (int r = 0; r < dist_result.rows(); ++r)
    {
        if (dist_result[r] < dist_thre)
        {
            input_cloud->points[r].is_ground = 1;
            ground_cloud->points.push_back(input_cloud->points[r]);
        }
        else
        {
            input_cloud->points[r].is_ground = 0;
            non_ground_cloud->points.push_back(input_cloud->points[r]);
        }
    }

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- there are " << ground_cloud->size() << " ground points. \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- there are " << non_ground_cloud->size() << " non-ground points. \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::detectTooLowPoint(pcl::PointCloud<MyPointType>::Ptr& cloud)
{
    // sort by z-axis value, from low to high
    sort(cloud->points.begin(), cloud->points.end(), point_cmp);

    // find all worng points and mark them
    for (auto& point : cloud->points)
    {
        if (point.z < -1.5 * relative_sensor_height)
        {
            point.too_low = 1;
            low_invalid_count++;
        }
        else
        {
            break;
        }
    }

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- there are " << low_invalid_count << " points have too low z value to be ground points, will be marked. \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::detectInvalidPoint(pcl::PointCloud<MyPointType>::Ptr& cloud)
{
    int cnt = 0;

    for (auto& point : cloud->points)
    {
        float dist = sqrt(point.x * point.x + point.y * point.y + point.y * point.y);
        if (point.is_ground == 0)
        {
            if (point.z > too_high_thre || min_dist < dist || dist < max_dist)
            {
                point.is_invalid = 1;
                cnt++;
        }
        }
    }

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- there are " << cnt << " points in non-ground pcd that is invalid. \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::parseAndPubFinalPCD(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, pcl::PointCloud<MyPointType>::Ptr& invalid_cloud)
{
    non_ground_cloud->clear();

    for (auto& point : input_cloud->points)
    {
        if (point.is_ground == 0)
        {
            if (point.is_invalid == 1)
            {
                invalid_cloud->points.push_back(point);
            }
            else
            {
                non_ground_cloud->points.push_back(point);
            }
        }
    }

    pubPCD(ground_pcd_publisher, ground_cloud);
    pubPCD(non_ground_pcd_publisher, non_ground_cloud);
    pubPCD(invalid_pcd_publisher, invalid_cloud);

    if (debug_mode)
    {
        ROS_INFO_STREAM("\033[1;32m -- there are " << input_cloud->size() << " points in this pcd. \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- there are " << ground_cloud->size() << " ground points in this pcd. \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- there are " << non_ground_cloud->size() << " non-ground points in this pcd. \033[0m");
        ROS_INFO_STREAM("\033[1;32m -- there are " << invalid_cloud->size() << " invalid points in this pcd. \033[0m");
        std::cout << std::endl;
    }
}

void BPRFD::pubPCD(const ros::Publisher& scan_publisher, const pcl::PointCloud<MyPointType>::Ptr& scan)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*scan, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = "map";
    scan_publisher.publish(tempCloud);
}

void BPRFD::run()
{
    
}