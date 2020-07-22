#include <ros/ros.h>
#include <tf/transform_listener.h>

//ROS Msgs
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// Laser to Pointcloud
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <campusrover_map_server/SavePcd.h>

// Use namespace
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::Ptr CloudPtr;

// ROS parameters
bool show_pc_, use_scan_, use_pointcloud2_;
string map_frame_, file_path_;

// Global variables
laser_geometry::LaserProjection projector_;
ros::Publisher cloud_pub_;
ros::Subscriber scan_sub_;

// PCL visualization
CloudPtr cloud_show_, cloud_acc_;

// Functions
void get_parameters(ros::NodeHandle n_private);
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
void pointcloud2Callback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
void viz_cb (pcl::visualization::PCLVisualizer& viz);
bool save_3d_cloud(campusrover_map_server::SavePcd::Request  &req,
                   campusrover_map_server::SavePcd::Response &res);
