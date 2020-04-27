#include "autolabor_costmap.h"

AutolaborCostmap::AutolaborCostmap(ros::NodeHandle n):
  n_(n)
{
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  nh_private.param<double>("map_size_width", map_size_width_, 10.0);
  nh_private.param<double>("map_size_height", map_size_height_, 10.0);
  nh_private.param<double>("map_update_frequency", map_update_frequency_, 5.0);
  nh_private.param<double>("map_resolution", map_resolution_, 0.1);
  nh_private.param<bool>("pcl_viz", g_pcl_viz, false);
  nh_private.param<double>("max_obstacle_height", max_obstacle_height_, 2.0);
  nh_private.param<double>("min_obstacle_height", min_obstacle_height_, 0.0);

  nh_private.param<double>("inflation_radius", inflation_radius_, 0.55);
  nh_private.param<double>("cost_scaling_factor", cost_scaling_factor_, 10.0);

  // ros publish
  costmap_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("autolabor_local_costmap", 10);
  cloud_sub_ = n_.subscribe<PointCloud>("points2", 100, 
                                        &AutolaborCostmap::PointCloud2Callback, this);
}

void AutolaborCostmap::PointCloud2Callback(const PointCloud::ConstPtr& msg)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  // // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  static bool first_time = true;
  static tf::TransformListener tf_listener_cloud;
  // Costmap pub msg
  static nav_msgs::OccupancyGrid costmap;
  // get image projection matrix
  if (first_time)
  {
    // init costmap
    costmap.header.seq++;
    costmap.header.frame_id = base_frame_;
    costmap.info.resolution = map_resolution_;
    costmap.info.width = map_size_width_ / map_resolution_;
    costmap.info.height = map_size_height_ / map_resolution_;
    costmap.info.origin.position.x = -map_size_width_ / 2.0;
    costmap.info.origin.position.y = -map_size_height_ / 2.0;
    costmap.info.origin.position.z = 0;
    costmap.info.origin.orientation.x = 0;
    costmap.info.origin.orientation.y = 0;
    costmap.info.origin.orientation.z = 0;
    costmap.info.origin.orientation.w = 1;
    for (int i = 0; i < costmap.info.width *costmap.info.height; i++)
    {
      costmap.data.push_back(0);
    }
    first_time = false;
  }
  // get point cloud transformed data
  PointCloudPtr cloud_out(new PointCloud);
  try
  {
    pcl_ros::transformPointCloud(base_frame_, *msg, *cloud_out, tf_listener_cloud);
  }
  catch (tf::TransformException& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
  // std::cout << "In: " << msg->points[0] << "\nOut: " << cloud_out.points[0] << std::endl;
  costmap.header.stamp = ros::Time::now();
  cropBoxFilter(cloud_out, cloud_out);
  PointCloudPtr cloud_filtered(new PointCloud);
  voxelGridFliter(cloud_out, cloud_filtered);
  if (g_pcl_viz)
  {
    g_cloud_viz_list.push_back(cloud_filtered);
  }
  toCostmap(cloud_filtered, costmap);
  // publish costmap
  costmap_pub_.publish(costmap);
}

void AutolaborCostmap::passThroughFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_obstacle_height_, max_obstacle_height_);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_out);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-map_size_width_/2.0, map_size_width_/2.0);
  pass.filter (*cloud_out);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-map_size_height_/2.0, map_size_height_/2.0);
  pass.filter (*cloud_out);
}

void AutolaborCostmap::cropBoxFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::CropBox<pcl::PointXYZ> box_filter(true);
  box_filter.setInputCloud (cloud_in);
  Eigen::Vector4f min_pt (-map_size_width_/2.0, -map_size_height_/2.0, min_obstacle_height_, 1.0f);
  Eigen::Vector4f max_pt ( map_size_width_/2.0,  map_size_height_/2.0, max_obstacle_height_, 1.0f);
  // Eigen::Vector4f min_pt (-100, -100, min_obstacle_height_, 1.0f);
  // Eigen::Vector4f max_pt ( 100,  100, max_obstacle_height_, 1.0f);
  box_filter.setMin (min_pt);
  box_filter.setMax (max_pt);
  box_filter.filter (*cloud_out);
  box_filter.setInputCloud (cloud_out);
  box_filter.setNegative(true);
  Eigen::Vector4f autolabor_min_pt (-0.35, -0.5, 0, 1);
  Eigen::Vector4f autolabor_max_pt ( 0.3,   0.5, 1.9, 1);
  box_filter.setMin (autolabor_min_pt);
  box_filter.setMax (autolabor_max_pt);
  box_filter.filter (*cloud_out);
}

void AutolaborCostmap::voxelGridFliter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> v_sor;
  v_sor.setInputCloud (cloud_in);
  v_sor.setLeafSize (map_resolution_, map_resolution_, map_resolution_);
  v_sor.filter (*cloud_out);
}

void AutolaborCostmap::toCostmap(PointCloudPtr &cloud_in, nav_msgs::OccupancyGrid &occupancy_grid)
{
  static double width = occupancy_grid.info.width;
  static double height = occupancy_grid.info.height;
  static double resolution = occupancy_grid.info.resolution;
  static double origin_x = occupancy_grid.info.origin.position.x;
  static double origin_y = occupancy_grid.info.origin.position.y;
  static double origin_z = occupancy_grid.info.origin.position.z;
  std::vector<Eigen::Vector2i> obstacle_index; 
  BOOST_FOREACH (const pcl::PointXYZ& pt, cloud_in->points)
  {
    int x = (pt.x - origin_x)/resolution;
    int y = (pt.y - origin_y)/resolution;
    obstacle_index.push_back(Eigen::Vector2i(x, y));
  }
  Eigen::MatrixXd map_matrix = this->getCostMat(obstacle_index, width, height, NUM_THRE1AD);
  occupancy_grid.data.clear();
  for(int i = 0; i < map_matrix.cols(); i++)
  {
    for(int j = 0; j < map_matrix.rows(); j++)
    {
      occupancy_grid.data.push_back((int)map_matrix(j, i));
    }
  }
}

Eigen::MatrixXd AutolaborCostmap::getCostMat(const std::vector<Eigen::Vector2i> obstacle_list, 
                                       int width, int height, int num_thread)
{
  Eigen::MatrixXd costmap_mat = Eigen::MatrixXd::Zero(width, height);
  std::vector<std::thread> threads(num_thread);
  std::vector<Eigen::MatrixXd> thread_mat(num_thread);
  double trunk = obstacle_list.size() / num_thread;
  for (int i = 0; i < num_thread; i++)
  {
    thread_mat[i] = Eigen::MatrixXd::Zero(width, height);
    threads[i] = std::thread(&AutolaborCostmap::computeMatDis, this, 
                             obstacle_list, 
                             std::ref(thread_mat[i]),
                             i*trunk, 
                             (i+1)*trunk);
  }
  for(int i = 0; i < threads.size() ; i++)
  {
    threads.at(i).join();
  }
  // merge matrixes
  for (int i = 0; i < costmap_mat.cols(); i++)
  {
    for (int j = 0; j < costmap_mat.rows(); j++)
    {
      double merg = thread_mat[0](j, i);
      for (int k = 0; k < num_thread; k++)
      {
        if (thread_mat[k](j, i) != 0)
        {
          if (thread_mat[k](j, i) == -1)
          {
            merg = -1;
            break;
          }
          else
          {
            if (merg < thread_mat[k](j, i))
            {
              merg = thread_mat[k](j, i);
            }
          }
        }
      }
      if (merg == -1)
      {
        costmap_mat(j, i) = 253;
      }
      else if(merg != 0)
      {
        // std::cout << merg << std::endl;
        // costmap_mat(j, i) = computeCost(merg*map_resolution_);
        costmap_mat(j, i) = 128;
      }
    }
  }
  return costmap_mat;
}

void AutolaborCostmap::computeMatDis(const std::vector<Eigen::Vector2i> obstacle_list, 
                               Eigen::MatrixXd &dst_mat,
                               int min_i, int max_i)
{
  double index_radius = inflation_radius_ / map_resolution_;
  int map_size_w = map_size_width_ / map_resolution_;
  int map_size_h = map_size_width_ / map_resolution_;
  for (int i = min_i; i < max_i; i++)
  {
    int x = obstacle_list[i](0);
    int y = obstacle_list[i](1);
    int min_x = std::max(0, (int)(x - index_radius) );
    int min_y = std::max(0, (int)(y - index_radius) );
    int max_x = std::min(map_size_w, (int)(x + index_radius) );
    int max_y = std::min(map_size_h, (int)(y + index_radius) );
    for (int i = min_x; i < max_x; i++)
    {
      for (int j = min_y; j < max_y; j++)
      {
        if (i==x && j==y)
        {
          dst_mat(i, j) = -1;
        }
        else
        {
          double dis = std::hypot(i-x, j-y);
          if (dis <= index_radius)
          {
            if (dst_mat(i, j) == 0)
            {
              dst_mat(i, j) = dis;
            }
            else
            {
              if (dis < dst_mat(i, j))
              {
                dst_mat(i, j) = dis;
              }
            }
          }
        }
      }
    }
  }
}

double AutolaborCostmap::computeCost(const double dis)
{
  return exp(-1.0 * cost_scaling_factor_ * 
                  (dis - inflation_radius_)) * 
                  (INSCRIBED_INFLATED_OBSTACLE - 1);
}