#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define M_PI 3.14159265358979323846  /* pi */
using namespace std;

ros::Subscriber global_path_sub_, costmap_sub_;
ros::Publisher local_path_pub_;

nav_msgs::OccupancyGrid costmap_data_;
geometry_msgs::PoseArray obstacle_poses_;
geometry_msgs::Pose robot_tf_pose_;
geometry_msgs::PoseStamped target_pose_;

struct NEIGHBORHOOD{
  int index;
  double dis;
};


struct CELL_DATA{
  double x;
  double y;
  int value;
  std::vector<NEIGHBORHOOD> neighborhoods;
};

std::vector<CELL_DATA> map_data_;

int status_msg_;
int source_id_;
int target_id_;
bool get_costmap_data_ = false;
bool get_globle_path_ = false;
bool local_path_detect_obstacle_ = false;
bool target_yaw_;
double threshold_occupied_;
double target_point_dis_;
double path_resolution_;
double robot_footptint_base_;
nav_msgs::Path local_path_;
nav_msgs::Path globle_path_;

string robot_frame_;
string path_frame_;

void LocalPathGenerate();
void CompareWithCostmap();
void FindShortestPath();
int minDistance(const std::vector<float> &dist, bool sptSet[]) ;
void getSolution(const std::vector<int> &parent, const std::vector<float> &dist, int source, int target) ;
void setIdPath(std::vector<int> &path, const std::vector<int> &parent, int j) ;
void setLocalPath(std::vector<int> &path);
//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("robot_frame", robot_frame_, "base_link");
  n_private.param<double>("threshold_occupied", threshold_occupied_, 10);
  n_private.param<double>("target_point_dis", target_point_dis_, 0.5);
  n_private.param<double>("path_resolution", path_resolution_, 0.05);
  n_private.param<double>("robot_footptint_base", robot_footptint_base_, 0.05);

}
//-----------------------------------------------------------------------------------------------
void UpdateCampusRoverPoseFromTF()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;

  try
  {
    transformStamped = tfBuffer.lookupTransform(path_frame_, robot_frame_, ros::Time(0), ros::Duration(2));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN(" %s. Can't update pose from TF, for that will be use the latest source point.", ex.what());
  }

  robot_tf_pose_.position.x = transformStamped.transform.translation.x;
  robot_tf_pose_.position.y = transformStamped.transform.translation.y;
  robot_tf_pose_.position.z = transformStamped.transform.translation.z;
  robot_tf_pose_.orientation.x = transformStamped.transform.rotation.x;
  robot_tf_pose_.orientation.y = transformStamped.transform.rotation.y;
  robot_tf_pose_.orientation.z = transformStamped.transform.rotation.z;
  robot_tf_pose_.orientation.w = transformStamped.transform.rotation.w;

  tf::Quaternion q( robot_tf_pose_.orientation.x,
                    robot_tf_pose_.orientation.y,
                    robot_tf_pose_.orientation.z,
                    robot_tf_pose_.orientation.w);
  tf::Matrix3x3 m(q);
  
  // std::cout << "arriving_end_point_ : " << arriving_end_point_<< " arriving_end_dir : "<< arriving_end_direction_<<'\n';
  
}
//-----------------------------------------------------------------------------------------------
void TimerCallback(const ros::TimerEvent &event)
{
  if(get_globle_path_)
  {
    if(get_costmap_data_)
    {
      LocalPathGenerate();
      CompareWithCostmap();
      if(local_path_detect_obstacle_)
      {
        local_path_.poses.clear();
        FindShortestPath();

      }
      local_path_pub_.publish(local_path_);

      // get_costmap_data_ = false;
      // get_globle_path_ = false;
    }
    else
    {
      status_msg_ = 2;
    }
  }
  else
  {
    status_msg_ = 1;
  }
  

}
//-----------------------------------------------------------------------------------------------
void msgs_timerCallback(const ros::TimerEvent &event)
{
  //std::cout << "parameter[arriving_range] :  " << arriving_range_<< '\n';
  if (status_msg_ == 1)
  {
    ROS_WARN("obstacle avoidance .cpp : Without Globle Path to follow, Waiting for the Path input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 2)
  {
    ROS_WARN("obstacle avoidance .cpp : Without costmap input , Waiting for the costmap input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 3)
  {
    ROS_INFO("obstacle avoidance .cpp : detect obstacle");
    status_msg_ = 0;
  }
  else if (status_msg_ == 4)
  {
    ROS_ERROR("obstacle avoidance .cpp : Arrival the destination");
    status_msg_ = 0;
  }
  else
  {
    //ROS_INFO("dp planner : PEV moving");
    status_msg_ = 0;
  }

}
//-----------------------------------------------------------------------------------------------
void LocalPathGenerate()
{
  static geometry_msgs::PoseStamped local_path_pose;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double x_p ,y_p;
  static double dist_fp_p;
  static double closest_dist;
  static int closest_id;
  static int target_point_id;
  static double x_fp, y_fp;
  static double looking_dist;
  static int local_path_step_count;
  static double step_x, step_y;
  static double target_yaw;

  // UpdateCampusRoverPoseFromTF();

  local_path_.header.frame_id = robot_frame_;
  local_path_pose.header.frame_id = robot_frame_;

  //push robot pose to the path as the first point 
  local_path_.poses.clear();
  local_path_pose.pose = robot_tf_pose_;
  local_path_.poses.push_back(local_path_pose);

  // find the cloest point from robot to path
  for (int cp = 0; cp < globle_path_.poses.size(); cp++)
  {
    x_p = globle_path_.poses[cp].pose.position.x;
    y_p = globle_path_.poses[cp].pose.position.y;
    dist_fp_p = sqrt(pow(robot_tf_pose_.position.x - x_p,2) + pow(robot_tf_pose_.position.y - y_p,2));
    if (cp == 0)
    {
      closest_dist = dist_fp_p;
      closest_id = cp;
    }
    else if (dist_fp_p < closest_dist)
    {
      closest_dist = dist_fp_p;
      closest_id = cp;
    }
  }
  //push the path pose from robot pose to closest path pose 
  local_path_step_count = int (sqrt(pow(globle_path_.poses[closest_id].pose.position.x 
                                      - robot_tf_pose_.position.x,2) 
                                  + pow(globle_path_.poses[closest_id].pose.position.y 
                                      - robot_tf_pose_.position.y, 2))/path_resolution_);
  step_x = double ((globle_path_.poses[closest_id].pose.position.x 
                                      - robot_tf_pose_.position.x)/local_path_step_count);
  step_y = double ((globle_path_.poses[closest_id].pose.position.y 
                                      - robot_tf_pose_.position.y)/local_path_step_count);
  
  for(int i = 0;i < local_path_step_count;i++)
  {
    if(i ==0)
    {
      local_path_pose.pose.position.x = robot_tf_pose_.position.x;
      local_path_pose.pose.position.y = robot_tf_pose_.position.y;
    }
    else
    {
      local_path_pose.pose.position.x += step_x;
      local_path_pose.pose.position.y += step_y;

      target_yaw = atan2(local_path_pose.pose.position.y - local_path_.poses[i-1].pose.position.y, 
                          local_path_pose.pose.position.x - local_path_.poses[i-1].pose.position.x);
      
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      local_path_pose.pose.orientation = path_pose_q_msg;
    }
    local_path_.poses.push_back(local_path_pose);
  }


  // find the target_point_id from path
  if (closest_id >= globle_path_.poses.size() - 1)
  {
    target_point_id = closest_id;
  }
  else
  {
    //find the target point from globle path for following
    for(int fp = closest_id;fp<globle_path_.poses.size();fp++)
    {
      target_point_id = fp;
      local_path_pose.pose.position.x = globle_path_.poses[fp].pose.position.x;
      local_path_pose.pose.position.y = globle_path_.poses[fp].pose.position.y;
      looking_dist = sqrt(pow(local_path_pose.pose.position.x - robot_tf_pose_.position.x, 2) 
                        + pow(local_path_pose.pose.position.y - robot_tf_pose_.position.y, 2));

      local_path_.poses.push_back(local_path_pose);

      if(looking_dist >= target_point_dis_)
      {
        break;
      }
    }
    // cout<< "closest_id : " << closest_id<<"target_point_id"<<target_point_id<<endl;
  }

  target_pose_.header.frame_id = globle_path_.header.frame_id;
  target_pose_.pose = globle_path_.poses[target_point_id].pose;
}
//-----------------------------------------------------------------------------------------------
void CompareWithCostmap()
{
  static double obstacle_distance;

  local_path_detect_obstacle_ = false;
  for (int path_size_count = 0; path_size_count < local_path_.poses.size(); path_size_count++)
    {
      for (int obstacle_count = 0; obstacle_count < obstacle_poses_.poses.size(); obstacle_count++)
      {
        obstacle_distance = sqrt(pow(local_path_.poses[path_size_count].pose.position.x - obstacle_poses_.poses[obstacle_count].position.x, 2) +
                                 pow(local_path_.poses[path_size_count].pose.position.y - obstacle_poses_.poses[obstacle_count].position.y, 2));
        // std::cout << "/obstacle_distance: " <<obstacle_distance<< "/costmap_data_: " <<costmap_data_.info.resolution<<'\n';
        // std::cout << "nPathDetectObstacle" <<local_path_detect_obstacle_<< "obstacle_distance "<<obstacle_distance<<'\n';
        if (obstacle_distance < robot_footptint_base_)
        {
          local_path_detect_obstacle_ = true;
          return;
        }
      }
    }
}
//-----------------------------------------------------------------------------------------------
void FindShortestPath()
{
  std::vector<int> parent(map_data_.size());     // Parent array to store shortest path tree
  std::vector<float> dist(map_data_.size());    // The output array. dist[i] will hold
  static double dist_ta;
  static double closest_dist;
  static int closest_id;

  // sptSet[i] will true if vertex i is included / in shortest
  // path tree or shortest distance from source to i is finalized
  bool sptSet[map_data_.size()];

  // Initialize all distances as INFINITE and stpSet[] as false
  for (int i = 0; i < map_data_.size(); i++) {
    //parent[0] = -1;
    parent[i] = -1;
    dist[i] = 1000.0; //INT_MAX
    sptSet[i] = false;
  }

  // Distance of source vertex from itself is always 0
  dist[source_id_] = 0;

  for (int j = 0; j < map_data_.size(); j++) {

    dist_ta = sqrt(pow(map_data_[j].x - target_pose_.pose.position.x ,2) + pow(map_data_[j].y - target_pose_.pose.position.y,2));
    if (j == 0)
    {
      closest_dist = dist_ta;
      closest_id = j;
    }
    else if (dist_ta < closest_dist)
    {
      closest_dist = dist_ta;
      closest_id = j;
    }
  }

  target_id_ = closest_id;

  // Find shortest path for all vertices
  for (int count = 0; count < map_data_.size() - 1; count++)
  {
    // Pick the minimum distance vertex from the set of
    // vertices not yet processed. u is always equal to source
    // in first iteration.
    int u = minDistance(dist, sptSet);
    // cout<< "minDistance_id " <<u<< " minDistance " <<dist[u]<<endl;
    
    // Mark the picked vertex as processed
    sptSet[u] = true;

    // Update dist value of the adjacent vertices of the
    // picked vertex.
    for (int v = 0; v < map_data_[u].neighborhoods.size(); v++)
    {
      // Update dist[v] only if is not in sptSet, there is
      // an edge from u to v, and total weight of path from
      // source to v through u is smaller than current value of
      // dist[v]
      // cout<< "neighborhoods_minDistance_id " <<map_data_[u].neighborhoods[v].dis<<endl;
      if (!sptSet[map_data_[u].neighborhoods[v].index] && dist[u] + map_data_[u].neighborhoods[v].dis < dist[map_data_[u].neighborhoods[v].index] 
                    && map_data_[map_data_[u].neighborhoods[v].index].value > 0 ) 
      {
          parent[map_data_[u].neighborhoods[v].index] = u;
          dist[map_data_[u].neighborhoods[v].index] = dist[u] + map_data_[u].neighborhoods[v].dis;
          // cout<< "parent " <<u<< " dist " <<dist[map_data_[u].neighborhoods[v].index]<< "target_id_ " <<target_id_<<endl;
      }
    }
    
    // print the constructed distance array
    // TODO toto je asi bug, algoritmus by mal pokracovat kym neprehlada cely priestor

    // ros::Duration(0.5).sleep();
    if (u == target_id_) 
    {
      getSolution(parent, dist, source_id_, target_id_);
      return;
    }
    
      
  }
  getSolution(parent, dist, source_id_, target_id_);

}
//-----------------------------------------------------------------------------------------------
int minDistance(const std::vector<float> &dist, bool sptSet[]) 
{

  // Initialize min value
  int min = 1000.0, min_index; //INT_MAX
  for (int v = 0; v < dist.size(); v++)
  { 
    if (sptSet[v] == false && dist[v] <= min)
    {
      min = dist[v], min_index = v;
    }
     
  }
    
  return min_index;
}
//-----------------------------------------------------------------------------------------------
void getSolution(const std::vector<int> &parent, const std::vector<float> &dist, int source, int target) 
{

  std::vector<int> path;
  path.clear();

  if (dist[target] >= 1000.0) {
    // ROS_ERROR("can't find safe path");
    status_msg_ = 4;
    return;
  }

  path.push_back(source);
  setIdPath(path, parent, target);
  setLocalPath(path);
}
//-----------------------------------------------------------------------------------------------
void setIdPath(std::vector<int> &path, const std::vector<int> &parent, int j) 
{
  // Base Case : If j is source
  if (parent[j] == -1)
    return;

  setIdPath(path, parent, parent[j]);
  path.push_back(j);
}
//-----------------------------------------------------------------------------------------------
void setLocalPath(std::vector<int> &path)
{
  static geometry_msgs::PoseStamped local_path_pose;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;

  local_path_.poses.clear();

  for (int v = 0; v < path.size(); v++)
  {
    local_path_pose.pose.position.x = map_data_[path[v]].x;
    local_path_pose.pose.position.y = map_data_[path[v]].y;

    local_path_.poses.push_back(local_path_pose);
  }

  

  // path_pose_q_tf.setRPY(0.0,0.0,target_yaw_);
  // path_pose_q_msg = tf2::toMsg(path_pose_q_tf);

  // local_path_pose.pose.orientation = path_pose_q_msg;
  // local_path_pose.pose.position = target_pose_.pose.position;

  // local_path_.poses.push_back(local_path_pose);

}

//-----------------------------------------------------------------------------------------------
void GlobalPathCallback(const nav_msgs::PathConstPtr &path)
{
  static geometry_msgs::PoseStamped global_pose;
  static geometry_msgs::PoseStamped base_pose;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::PoseStamped target_pose;
  static double roll, pitch, yaw;

  path_frame_ = path->header.frame_id;

  if(path->poses.size()==0)
  {
    return;
  }
  global_pose.header.frame_id = path->header.frame_id;
  globle_path_.header.frame_id = robot_frame_;

  globle_path_.poses.clear();

  for (int i = 0; i < path->poses.size(); i++)
  {
    global_pose.pose.position.x = path->poses[i].pose.position.x;
    global_pose.pose.position.y = path->poses[i].pose.position.y;
    global_pose.pose.position.z = path->poses[i].pose.position.z;
    global_pose.pose.orientation.x = path->poses[i].pose.orientation.x;
    global_pose.pose.orientation.y = path->poses[i].pose.orientation.y;
    global_pose.pose.orientation.z = path->poses[i].pose.orientation.z;
    global_pose.pose.orientation.w = path->poses[i].pose.orientation.w;

    try
    {
      tfBuffer.transform(global_pose, base_pose, robot_frame_, ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("ob : %s", ex.what());
      return;
    }

    globle_path_.poses.push_back(base_pose);

    if(i == path->poses.size()-1)
    {
      target_pose.header.frame_id = robot_frame_;
      target_pose.pose.position = base_pose.pose.position;
      target_pose.pose.orientation = base_pose.pose.orientation;
    }
  }

  tf::Quaternion q( target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  target_yaw_ = yaw;
  
  get_globle_path_ = true;
}
//-----------------------------------------------------------------------------------------------
void CostmapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
  geometry_msgs::Pose ob_pose;
  geometry_msgs::PoseStamped ob_posestamped;
  geometry_msgs::PoseStamped base_ob_pose;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;
  double data;
  double value;

  static bool first_time = true;
  static CELL_DATA cell_data;
  static std::vector<CELL_DATA> cell_datas;
  static NEIGHBORHOOD neighborhood;
  static geometry_msgs::PoseStamped base_map_pose;
  static geometry_msgs::PoseStamped map_posestamped;
  static double cell_dis;

  if(first_time)
  {
    cout<< "processing map data "<<endl;
    for (int i = 0; i < map->data.size(); i++)
    {
      if (map->header.frame_id == robot_frame_)
      {
        cell_data.y = (floor(i / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        cell_data.x = ((i - map->info.width * floor(i / map->info.width)) * map->info.resolution) + map->info.origin.position.x;
      }
      else
      {
        map_posestamped.pose.position.y = (floor(i / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        map_posestamped.pose.position.x = ((i - map->info.width * floor(i / map->info.width)) * map->info.resolution) + map->info.origin.position.x;

        try
        {
          tfBuffer.transform(map_posestamped, base_map_pose, robot_frame_, ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("ob : %s", ex.what());
          return;
        }

        cell_data.y = base_map_pose.pose.position.y;
        cell_data.x = base_map_pose.pose.position.x;
      }

      cell_data.value = 1;// free space

      // cout<< "cell_data x: " << cell_data.x<< "cell_data y: " << cell_data.y<< "cell_data value: " << cell_data.value<<endl;
      if(  (floor(i / map->info.width)) == (map->info.width/2) && 
           ((i - map->info.width * floor(i / map->info.width))) == (map->info.height/2))
      {
        source_id_ = i;
        // cout<< "source_id_  " << source_id_<<endl;
        // cout<< "x  " << ((i - map->info.width * floor(i / map->info.width)) )<< "y  " << (floor(i / map->info.width))<<endl;
      }

      cell_datas.push_back(cell_data);
    }

    //find the neighborhood points
    for(int u = 0; u < cell_datas.size(); u++)
    {
      cell_data.neighborhoods.clear();
      cell_data.x = cell_datas[u].x;
      cell_data.y = cell_datas[u].y;
      cell_data.value = cell_datas[u].value;

      for(int v = 0; v < cell_datas.size(); v++)
      {
        if(u != v)
        {
          cell_dis = sqrt(pow(cell_datas[u].x - cell_datas[v].x,2)
                          +pow(cell_datas[u].y - cell_datas[v].y,2));
          
          if(cell_dis <= 1.5*map->info.resolution)
          {
            // cout<< "find the neighborhood : "<<neighborhood.dis<<endl;
            neighborhood.index = v;
            neighborhood.dis = cell_dis;
            cell_data.neighborhoods.push_back(neighborhood);
          }
        }
      }
      map_data_.push_back(cell_data);
    }
    first_time = false;
    cout<< "get costmap data done!"<<endl;
  }

  obstacle_poses_.poses.clear();

  for (int j = 0; j < map->data.size(); j++)
  {
    if(abs(map->data[j]) > threshold_occupied_)
    {
      map_data_[j].value = -1; // obstacle space
      ob_pose.position.x = map_data_[j].x;
      ob_pose.position.y = map_data_[j].y;
      obstacle_poses_.poses.push_back(ob_pose);
    }
    else
    {
      map_data_[j].value = 1;// free space
    }
    

  }

  get_costmap_data_ = true;
}

//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_avoidance");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();
  
  global_path_sub_ = nh.subscribe("global_path", 1, GlobalPathCallback);
  costmap_sub_ = nh.subscribe("costmap", 1, CostmapCallback);
  local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 20);

  ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCallback);
  ros::Timer msgs_timer = nh.createTimer(ros::Duration(3), msgs_timerCallback);

  ros::spin();

  return 0;
}