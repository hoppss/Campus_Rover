#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
// Seg
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

// ROS Parameters
int visualization_;
double min_z_, max_z_, seg_distance_th_;
string pcd_file_;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<int>("visualization", visualization_, 0);
  n_private.param<double>("min_z", min_z_, -0.70);
  n_private.param<double>("max_z", max_z_, 0.1);
  n_private.param<string>("pcd_file", pcd_file_, "mycloud.pcd");
  n_private.param<double>("seg_distance_th", seg_distance_th_, 0.15);

}

void cloud_to_map( pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr ground,
                   nav_msgs::OccupancyGrid &map)
{
  double max_x, max_y, mini_x, mini_y;
  for (int i = 0; i < main_cloud->points.size(); i++)
  {
    if(i == 0)
    {
      max_x = mini_x = main_cloud->points[i].x;
      max_y = mini_y = main_cloud->points[i].y;
    }
    else
    {
      if (main_cloud->points[i].x > max_x)
      {
        max_x = main_cloud->points[i].x;
      }
      if (main_cloud->points[i].x < mini_x)
      {
        mini_x = main_cloud->points[i].x;
      }
      if (main_cloud->points[i].y > max_y)
      {
        max_y = main_cloud->points[i].y;
      }
      if (main_cloud->points[i].y < mini_y)
      {
        mini_y = main_cloud->points[i].y;
      }
    }
  }
  cout << "Size in meter: " << max_x-mini_x << " x " << max_y-mini_y << endl;
  // Transform to (0, 0) to cloud center
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1 (0,3) = -(max_x + mini_x) / 2.0;
  transform_1 (1,3) = -(max_y + mini_y) / 2.0;
  pcl::transformPointCloud (*ground, *ground, transform_1);
  pcl::transformPointCloud (*obstacle, *obstacle, transform_1);
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.map_load_time = ros::Time::now();
  map.info.width = (int)(abs(max_x-mini_x) / map.info.resolution);
  map.info.height = (int)(abs(max_y-mini_y) / map.info.resolution);
  map.info.origin.position.x = mini_x;
  map.info.origin.position.y = mini_y;
  map.info.origin.position.z = 0;
  map.info.origin.orientation.w = 1;
  // initial array
  // char data[map.info.height][map.info.width];
  char** data = new char*[map.info.height]; // each element is a pointer to an array.
  for(size_t i = 0; i < map.info.height; ++i)
  {
    data[i] = new char[map.info.width]; // build rows
  }
  for(int i = 0; i < map.info.height; i++)
  {
    for(int j = 0; j < map.info.width; j++)
    {
      data[i][j] = -1;
    }
  }
  cout << "Initialed map" << endl;
  int x, y, drop_count;
  drop_count = 0;
  // put ground
  for (int i = 0; i < ground->points.size(); i++)
  {
    x = (int)(ground->points[i].x / map.info.resolution + map.info.width / 2);
    y = (int)(ground->points[i].y / map.info.resolution + map.info.height / 2);
    // Make sure point in region
    if(y >= 0 && y <  map.info.height && x >=0 && map.info.width)
    {
      data[y][x] = 0;
    }
    else
    {
      drop_count++;
    }
    // data[y+1][x] = 0;
    // data[y][x+1] = 0;
    // data[y+1][x+1] = 0;
    // data[y][x-1] = 0;
    // data[y-1][x] = 0;
    // data[y-1][x-1] = 0;
  }
  cout << "Put ground on map and droped " << drop_count << " points" << endl;
  drop_count = 0;
  for (int i = 0; i < obstacle->points.size(); i++)
  {
    x = (int)(obstacle->points[i].x / map.info.resolution + map.info.width / 2);
    y = (int)(obstacle->points[i].y / map.info.resolution + map.info.height / 2);
    // Make sure point in region
    if(y >= 0 && y <  map.info.height && x >=0 && map.info.width)
    {
      data[y][x] = 100;
    }
    else
    {
      drop_count++;
    }
    // data[y+1][x] = 100;
    // data[y][x+1] = 100;
    // data[y+1][x+1] = 100;
    // data[y][x-1] = 100;
    // data[y-1][x] = 100;
    // data[y-1][x-1] = 100;
  }
  cout << "Put obstacle on map and droped " << drop_count << " points" << endl;
  // put data into msg
  for(int i = 0; i < map.info.height; i++)
  {
    for(int j = 0; j < map.info.width; j++)
    {
      map.data.push_back(data[i][j]);
    }
  }
}

void cloud_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud_neg)
{
  // segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (seg_distance_th_);
  seg.setInputCloud (src);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
     return;
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (src);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*seg_cloud);
  std::cerr << "PointCloud representing the planar component: " << seg_cloud->width * seg_cloud->height << " data points." << std::endl;
  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*seg_cloud_neg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autolabor_3d_map");
  nav_msgs::OccupancyGrid map_msg;
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  ros::Publisher map_pub;
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  get_parameters(n_private);
  //PCL
  pcl::visualization::CloudViewer *viewer;
  if(visualization_)
  {
    viewer = new pcl::visualization::CloudViewer("Simple Cloud Viewer");
  }
  //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_neg(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Start Loading PointCloud ..." << '\n';
  double Start = ros::Time::now().toSec();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_, *cloud) == -1) //* load the file
  {
    ROS_ERROR("can not open file: %s", pcd_file_.c_str());
    return (-1);
  }
  std::cout << "Use: " << (ros::Time::now().toSec() - Start)*1000 << " ms" << '\n';
  cloud_segmentation(cloud, cloud_seg, cloud_seg_neg);
  std::cout << "Start pass filter..." << std::endl;
  // PassThrough
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_seg_neg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_z_, max_z_);
  pass.filter (*cloud_pass);
  // segmentation
  
  cloud_to_map(cloud, cloud_pass, cloud_seg, map_msg);
  std::cout << "map publishing..." << std::endl;
  map_pub.publish( map_msg );
  if (visualization_)
  {
    switch (visualization_) {
      case 1:
        viewer->showCloud (cloud, "input");
        break;
      case 2:
        viewer->showCloud (cloud_pass, "PassThrough");
        break;
      case 3:
        viewer->showCloud (cloud_seg, "segmentation");
        break;
      case 4:
        viewer->showCloud (cloud_seg_neg, "segmentation_neg");
        break;
      default:
        break;
    }
  }
  ros::spin();
}
