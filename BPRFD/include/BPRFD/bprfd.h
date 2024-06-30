#pragma once

#include "BPRFD/utils.h"

class BPRFD
{
private:
    ros::NodeHandle nh;

    bool debug_mode;

    std::string raw_topic;

    std::string ground_topic;
    std::string non_ground_topic;
    std::string invalid_topic;

    ros::Subscriber raw_pcd_subscriber;
    ros::Publisher  ground_pcd_publisher;
    ros::Publisher  non_ground_pcd_publisher;
    ros::Publisher  invalid_pcd_publisher;

    pcl::PointCloud<PointType>::Ptr ori_pcd;

    pcl::PointCloud<MyPointType>::Ptr raw_pcd;

    pcl::PointCloud<MyPointType>::Ptr ground_pcd;
    pcl::PointCloud<MyPointType>::Ptr non_ground_pcd;
    pcl::PointCloud<MyPointType>::Ptr invalid_pcd;

    int global_id = 0;

    int low_invalid_count = 0;
    
    float relative_sensor_height;

    int initial_seeds_count;

    float seed_z_thre;

    int ground_extraction_count;

    Eigen::MatrixXf seeds_normal_vector;
    Eigen::Vector3f seeds_mean_vector;

    float seeds_dist;
    float seeds_dist_thre;

    float SEED_DIST_THRE;

    Eigen::MatrixXf point_matrix;
    Eigen::VectorXf dist_result;

    float too_high_thre;

    float min_dist;
    float max_dist;

public:
    BPRFD();
    ~BPRFD();

    void getParams();

    void allocateMemory();

    void run();

    void laserCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void preprocess(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<MyPointType>::Ptr raw_pcd);

    void process(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud);

    void postprocess(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, pcl::PointCloud<MyPointType>::Ptr& invalid_cloud);

    void detectTooLowPoint(pcl::PointCloud<MyPointType>::Ptr& cloud);

    void detectInvalidPoint(pcl::PointCloud<MyPointType>::Ptr& cloud);

    void extractInitialSeeds(const pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& seeds_cloud);

    void extractGroundPoints(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud);

    void estimateGroundThre(const pcl::PointCloud<MyPointType>::Ptr& ground_cloud, Eigen::MatrixXf& normal_vector, float& dist_thre);

    void detectGroundPoints(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, Eigen::MatrixXf& normal_vector, float& dist_thre);

    void parseAndPubFinalPCD(pcl::PointCloud<MyPointType>::Ptr& input_cloud, pcl::PointCloud<MyPointType>::Ptr& ground_cloud, pcl::PointCloud<MyPointType>::Ptr& non_ground_cloud, pcl::PointCloud<MyPointType>::Ptr& invalid_cloud);

    void pubPCD(const ros::Publisher& scan_publisher, const pcl::PointCloud<MyPointType>::Ptr& scan);
};