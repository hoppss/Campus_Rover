#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/foreach.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

// for sensors
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// pointcloud transform
#include <pcl_ros/transforms.h>

#define NUM_THRE1AD 8

static const double HEAT_MIN = 1.6;
static const double HEAT_MAX = 20;

static const double COLOR_MAP_SIZE = 20;

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 254;
static const unsigned char FREE_SPACE = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

//PCL visualization
std::vector<PointCloudPtr> g_cloud_viz_list;
bool g_pcl_viz;

class CampusroverCostmap
{
  public:
    CampusroverCostmap(ros::NodeHandle n);

  private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub_;
    // ros parameters - common
    std::string base_frame_;
    ros::Publisher costmap_pub_;
    // ros parameters - costmap
    double map_size_width_;
    double map_size_height_;
    double map_update_frequency_;
    double map_resolution_;
    // ros parameters - obstacle
    double max_obstacle_height_;
    double min_obstacle_height_;
    // ros parameters - inflation
    double inflation_radius_;
    double cost_scaling_factor_;

    void PointCloud2Callback( const PointCloud::ConstPtr& msg);

    void pubPointcloud(ros::Publisher& pub, pcl::PointXYZ& pt);

    void passThroughFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    void cropBoxFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    void voxelGridFliter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    void toCostmap( PointCloudPtr &cloud_in, nav_msgs::OccupancyGrid &occupancy_grid);

    Eigen::MatrixXd getCostMat(const std::vector<Eigen::Vector2i> obstacle_list, 
                               int width, int height, int num_thread);
    
    void computeMatDis(const std::vector<Eigen::Vector2i> obstacle_list, 
                       Eigen::MatrixXd &dst_mat,
                       int min_i, int max_i);

    double computeCost(const double dis);
};