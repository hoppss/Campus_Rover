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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define NO_PATH -1

using namespace std;

ros::Subscriber path_sub_, costmap_sub_, click_sub_;
ros::Publisher twist_pub_, twist_path_pub_, path_marker_pub_;

string robot_frame_;

nav_msgs::Path globle_path_;
nav_msgs::OccupancyGrid costmap_data_;

geometry_msgs::Pose robot_tf_pose_;
geometry_msgs::PoseArray obstacle_poses_;

//path switch stop timer
ros::Time stop_timer_last_;
ros::Time stop_timer_current_;

int status_msg_ = 0;
int swap_path_num_;

double max_vel_x_;
double min_vel_x_;
double max_rot_vel_;
double min_rot_vel_;
double acc_lim_x_;
double acc_lim_theta_;
double min_turning_radius_;

double step_linear_x_;
double step_angular_z_;
double profile_linear_x_;
double profile_angular_z_;
double footprint_max_x_;
double footprint_min_x_;
double footprint_max_y_;
double footprint_min_y_;
double arriving_range_;
double target_point_dis_;
double angular_velocity_p_;

double max_dist_;
double min_dist_;
double vel_x_step_;
double rot_x_step_;
double path_resolution_;

double threshold_occupied_;
double obstacle_stop_timer_;
double obstacle_path_switch_stop_time_;

double rot_x_zero_vel_angle_;

bool get_globle_path_ = false;
bool get_costmap_data_ = false;
bool find_available_path_ = false;
bool stop_cmd_ = false;
bool arriving_end_point_;
bool obstacle_path_switch_ = false;
bool enable_path_switch_stop_;
bool enble_costmap_obstacle_;
bool avoid_right_side_priority_;

bool twist_profile_;
bool PathVisualize_;

bool new_path_;

void make_path();
void VisualizePath(const std::vector<nav_msgs::Path> &nPath, int n);
void TwistPublish(double x, double z);
void make_twist_path(double twist_linear_x, double twist_angular_z);
//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("robot_frame", robot_frame_, "base_link");
  n_private.param<double>("arriving_range", arriving_range_, 2.0);
  //velocity configution
  n_private.param<double>("max_vel_x", max_vel_x_, 1.0);
  n_private.param<double>("max_rot_vel", max_rot_vel_, 2.0);
  n_private.param<double>("min_rot_vel", min_rot_vel_, -2.0);
  n_private.param<double>("min_turning_radius", min_turning_radius_, 3.0);
  n_private.param<double>("target_point_dis", target_point_dis_, 3.0);
  //velocity profile
  n_private.param<double>("max_dist", max_dist_, 3.0);
  n_private.param<double>("min_dist", min_dist_, 3.0);
  n_private.param<double>("step_linear_x", step_linear_x_, 0.02);
  n_private.param<double>("step_angular_z", step_angular_z_, 0.03);
  n_private.param<double>("angular_velocity_p", angular_velocity_p_, 1.0);
  //path step
  n_private.param<double>("vel_x_step", vel_x_step_, 0.15);
  n_private.param<double>("rot_x_step", rot_x_step_, 0.1);
  n_private.param<double>("rot_x_zero_vel_angle", rot_x_zero_vel_angle_, 1.0);
  n_private.param<double>("path_resolution", path_resolution_, 0.1);
  n_private.param<int>("swap_path_num", swap_path_num_, 8);

  n_private.param<double>("threshold_occupied", threshold_occupied_, 10);
  n_private.param<double>("footprint_max_x", footprint_max_x_, 3.5);
  n_private.param<double>("footprint_min_x", footprint_min_x_, -0.5);
  n_private.param<double>("footprint_max_y", footprint_max_y_, 0.6);
  n_private.param<double>("footprint_min_y", footprint_min_y_, -0.6);

  n_private.param<bool>("enable_path_switch_stop", enable_path_switch_stop_, false);
  n_private.param<double>("obstacle_path_switch_stop_time", obstacle_path_switch_stop_time_, 2.0);
  n_private.param<bool>("avoid_right_side_priority", avoid_right_side_priority_, true);

  n_private.param<bool>("enble_costmap_obstacle", enble_costmap_obstacle_, false);
  n_private.param<bool>("twist_profile", twist_profile_, true);
  n_private.param<bool>("PathVisualize", PathVisualize_, true);
}
//-----------------------------------------------------------------------------------------------
void UpdatePEVPoseFromTF()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped base_pose;

  try
  {
    transformStamped = tfBuffer.lookupTransform(globle_path_.header.frame_id, robot_frame_, ros::Time(0), ros::Duration(2));
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

}
//-----------------------------------------------------------------------------------------------
void check_arrive_goal()
{
  double dist;
  dist = sqrt(pow(globle_path_.poses[globle_path_.poses.size()-1].pose.position.x - robot_tf_pose_.position.x, 2)
              + pow(globle_path_.poses[globle_path_.poses.size()-1].pose.position.y - robot_tf_pose_.position.y, 2));

  if(dist < arriving_range_)
  {
    arriving_end_point_ = true;
  }else{
    arriving_end_point_ = false;
  }

}
//-----------------------------------------------------------------------------------------------
void TimerCallback(const ros::TimerEvent &event)
{
  if (stop_cmd_)
  {
    TwistPublish(0.0, 0.0);
  }
  else
  {
    if (get_globle_path_)
    {
      if (get_costmap_data_ || !enble_costmap_obstacle_)
      {
        check_arrive_goal();
        if(!arriving_end_point_)
        {
          if(!obstacle_path_switch_)
          {
            UpdatePEVPoseFromTF();
            make_path();
          }
          else
          {
            TwistPublish(0.0, 0.0);
            stop_timer_current_ = ros::Time::now();
            obstacle_stop_timer_ = stop_timer_current_.toSec() - stop_timer_last_.toSec();
            status_msg_ = 5;
            if(obstacle_stop_timer_ > obstacle_path_switch_stop_time_)
            {
              obstacle_path_switch_ = false;
            }
          }
        }
        else
        {
          status_msg_ = 4;
          TwistPublish(0.0, 0.0);
        }
      }
      else
      {
        status_msg_ = 2;
        TwistPublish(0.0, 0.0);
      }
    }
    else
    {
      status_msg_ = 1;
      TwistPublish(0.0, 0.0);
    }
  }
}
//-----------------------------------------------------------------------------------------------
void msgs_timerCallback(const ros::TimerEvent &event)
{
  //std::cout << "parameter[arriving_range] :  " << arriving_range_<< '\n';
  if (status_msg_ == 1)
  {
    ROS_WARN("adp planner : Without Globle Path to follow, Waiting for the Path input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 2)
  {
    ROS_WARN("adp planner : Without costmap input , Waiting for the costmap input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 3)
  {
    ROS_INFO("adp planner : detect obstacle");
    status_msg_ = 0;
  }
  else if (status_msg_ == 4)
  {
    ROS_INFO("adp planner : Arrival the destination");
    status_msg_ = 0;
  }
  else if (status_msg_ == 4)
  {
    ROS_WARN("adp planner : Path switch stop");
    status_msg_ = 0;
  }
  else
  {
    //ROS_INFO("dp planner : PEV moving");
    status_msg_ = 0;
  }
}
//-----------------------------------------------------------------------------------------------
void make_path()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;
  static bool first_time = true;

  double dec_time;
  double vel_x_count;
  double vel_rot_count;

  double vel_x_in;
  double vel_rot_in;

  double radius;
  geometry_msgs::Twist twist;
  static std::vector<geometry_msgs::Twist> twist_set;

  //////////make end point param//////////
  static std::vector<geometry_msgs::PoseStamped> twist_end_points;
  nav_msgs::Path twist_path;
  std::vector<nav_msgs::Path> twist_path_set;
  geometry_msgs::PoseStamped end_point;
  geometry_msgs::PoseStamped tw_pose;
  double x = 0;
  double y = 0;
  double dist_fp_tw = 0;
  double angle_tw = 0;
  double x_dist_path;
  double rot_rad_path;
  double x_dist_path_step;
  double rot_rad_path_step;
  static double step_count ;
  double circle_x, circle_y;
  double r;
  double angle;

  if(first_time)
  {
    step_count = max_dist_/path_resolution_;
    vel_rot_count = (max_rot_vel_ - min_rot_vel_) / rot_x_step_;
    twist_set.clear();
    vel_x_in = max_vel_x_;

    for (int vrc = 0; vrc <= vel_rot_count; vrc++)
    {
      if (vrc == 0)
      {
        vel_rot_in = min_rot_vel_;
      }
      else
      {
        vel_rot_in = vel_rot_in + rot_x_step_;
      }

      if (vel_rot_in == 0)
      {
        twist.linear.x = vel_x_in;
        twist.linear.y = 0.0;
        twist.angular.z = vel_rot_in;
        twist_set.push_back(twist);
        //std::cout << "vel_x_in : " << vel_x_in<<" vel_rot_in : " << vel_rot_in<<'\n';
      }
      else if (abs(max_dist_ / vel_rot_in) >= min_turning_radius_)
      {
        if(abs(vel_rot_in) > rot_x_zero_vel_angle_)
        {
          twist.linear.x = 0.0;
          twist.angular.x = max_dist_ / vel_rot_in;
          twist.angular.z = vel_rot_in;
          twist_set.push_back(twist);
        }
        else
        {
          twist.linear.x = vel_x_in * ((rot_x_zero_vel_angle_ - abs(vel_rot_in))/rot_x_zero_vel_angle_);
          twist.angular.x = max_dist_ / vel_rot_in;
          twist.angular.z = vel_rot_in;
          twist_set.push_back(twist);
        }
          
        
        //std::cout << "vel_x_in : " << twist.linear.x<<" vel_rot_in : " << twist.angular.z<<'\n';
      }
    }


    //make end point

    twist_end_points.clear();

    for (int pc = 0; pc < twist_set.size(); pc++)
    {
      if(twist_set[pc].angular.z == 0.0)
      {
        //store each twist end point to vector
        //end_point.header.stamp = ros::Time::now();
        end_point.header.frame_id = robot_frame_;
        end_point.pose.position.x = max_dist_;
        end_point.pose.position.y = 0;
        end_point.pose.position.z = 0;
        end_point.pose.orientation.x = 0;
        end_point.pose.orientation.y = 0;
        end_point.pose.orientation.z = 0;
        end_point.pose.orientation.w = 1;

      }else{

        end_point.header.frame_id = robot_frame_;
        end_point.pose.position.x = max_dist_ * cos(twist_set[pc].angular.z);
        end_point.pose.position.y = max_dist_ * sin(twist_set[pc].angular.z);
        end_point.pose.position.z = 0;
        end_point.pose.orientation.x = 0;
        end_point.pose.orientation.y = 0;
        end_point.pose.orientation.z = 0;
        end_point.pose.orientation.w = 1;
      }

      twist_end_points.push_back(end_point);
      //std::cout << "end_point x: " << end_point.pose.position.x<<" end_point y: " << end_point.pose.position.y<<'\n';
    }

    first_time = false;
  }
  //std::cout << "end_point : " << end_point<<'\n';
  //find close point from globle path

  static double x_p ,y_p;
  static double dist_fp_p;
  static double closest_dist;
  static int closest_id;

  double new_x_fp, new_y_fp, old_x_fp, old_y_fp;
  double looking_dist;
  int new_looking_id, old_looking_id, target_point_id;
  double dist_fp;
  geometry_msgs::PoseStamped target_pose;

  if(new_path_)
  {
    for (int cp = 0; cp < globle_path_.poses.size(); cp++)
    {
      x_p = globle_path_.poses[cp].pose.position.x;
      y_p = globle_path_.poses[cp].pose.position.y;
      dist_fp_p = sqrt((robot_tf_pose_.position.x - x_p) * (robot_tf_pose_.position.x - x_p) + (robot_tf_pose_.position.y - y_p) * (robot_tf_pose_.position.y - y_p));
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
    //new_path_ = false;
  }
  else
  {
    for (int cp = closest_id; cp < globle_path_.poses.size(); cp++)
    {
      x_p = globle_path_.poses[cp].pose.position.x;
      y_p = globle_path_.poses[cp].pose.position.y;
      dist_fp_p = sqrt((robot_tf_pose_.position.x - x_p) * (robot_tf_pose_.position.x - x_p) + (robot_tf_pose_.position.y - y_p) * (robot_tf_pose_.position.y - y_p));
      if (cp == closest_id)
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
  }
  //std::cout << "closest_id : " << closest_id<<" new_path_ : " << new_path_<<'\n';
  if (closest_id >= globle_path_.poses.size() - 1)
  {

    target_point_id = closest_id;

  }
  else
  {
    //find the target point from globle path for following

    new_looking_id = closest_id;
    dist_fp = 0;
    looking_dist = 0;

    while (looking_dist <= target_point_dis_)
    {
      new_looking_id = new_looking_id + 1;

      new_x_fp = globle_path_.poses[new_looking_id].pose.position.x;
      new_y_fp = globle_path_.poses[new_looking_id].pose.position.y;
      // old_x_fp = globle_path_.poses[old_looking_id].pose.position.x;
      // old_y_fp = globle_path_.poses[old_looking_id].pose.position.y;

      dist_fp = sqrt(pow(new_x_fp - robot_tf_pose_.position.x, 2) + pow(new_y_fp - robot_tf_pose_.position.y, 2));
      looking_dist = dist_fp;


      target_point_id = new_looking_id;

      if (new_looking_id >= globle_path_.poses.size() - 1)
      {
        break;
      }
      // else
      // {
      //   old_looking_id = new_looking_id;
      // }
    }
    //cout<< "closest_id : " << closest_id<<"target_point_id"<<target_point_id<<endl;
  }

  target_pose.header.frame_id = globle_path_.header.frame_id;
  target_pose.pose.position.x = globle_path_.poses[target_point_id].pose.position.x;
  target_pose.pose.position.y = globle_path_.poses[target_point_id].pose.position.y;
  target_pose.pose.position.z = globle_path_.poses[target_point_id].pose.position.z;

  

  //compare target point and twist end point
  static double com_dist;
  static int closest_path_id;
  static double cmp_min_dist;
  static geometry_msgs::PoseStamped comp_end_point;
  static geometry_msgs::Twist twist_swap;
  static std::vector<geometry_msgs::Twist> twist_swap_set;

  twist_swap_set.clear();

  for (int cmp = 0; cmp < twist_end_points.size(); cmp++)
  {
    //cout<<"twist_end_points[cmp]"<<twist_end_points[cmp]<<endl;
    try
    {
      tfBuffer.transform(twist_end_points[cmp], comp_end_point, globle_path_.header.frame_id, ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("CMP : %s", ex.what());
      return;
    }
    //std::cout << "twist_end_points[cmp] : " << twist_end_points[cmp]<<" comp_end_point : " << comp_end_point<<'\n';

    com_dist = sqrt(pow(comp_end_point.pose.position.x - target_pose.pose.position.x, 2) +
                    pow(comp_end_point.pose.position.y - target_pose.pose.position.y, 2));
    // cout<<" comp_end_point_x : "<<comp_end_point.pose.position.x<<" target_pose_x : "<<target_pose.pose.position.x<<" com_dist : "<<com_dist<<endl;
    // cout<<" comp_end_point_y : "<<comp_end_point.pose.position.y<<" target_pose_y : "<<target_pose.pose.position.y<<" com_dist : "<<com_dist<<endl;
    // twist_swap.linear.x = twist_set[cmp].linear.x;
    // twist_swap.linear.y = com_dist;
    // twist_swap.angular.x = twist_set[cmp].angular.x;
    // twist_swap.angular.z = twist_set[cmp].angular.z;
    //twist_swap_set.push_back(twist_swap);

    if(cmp == 0)
    {
      cmp_min_dist = com_dist;
      closest_path_id = cmp;
    }
    else if(cmp_min_dist > com_dist)
    {
      cmp_min_dist = com_dist;
      closest_path_id = cmp;
    }
  }
  //std::cout << "closest_path_id : " << closest_path_id<<" cmp_min_dist : " << cmp_min_dist<<'\n';
  //cout<<"--------------------------------------------- "<<endl;

  //swap twist commond set basis "cmp_min_dist" from close to far
  static int swap_count = 0;
  static int right_path_id;
  static int left_path_id;
  static bool push_right_path;
  swap_count = 0;
  if(avoid_right_side_priority_)
  {
    push_right_path =true;
  }
  for (int swap = 0; swap < twist_set.size(); swap++)
  {
    if(swap == 0)
    {
      right_path_id = closest_path_id - 1; 
      left_path_id = closest_path_id + 1;
      twist_swap.linear.x = twist_set[closest_path_id].linear.x;
      twist_swap.linear.y = com_dist;
      twist_swap.angular.x = twist_set[closest_path_id].angular.x;
      twist_swap.angular.z = twist_set[closest_path_id].angular.z;
      twist_swap_set.push_back(twist_swap);
    }else 
    {
      if(push_right_path && right_path_id >= 0)
      {
        twist_swap.linear.x = twist_set[right_path_id].linear.x;
        twist_swap.linear.y = com_dist;
        twist_swap.angular.x = twist_set[right_path_id].angular.x;
        twist_swap.angular.z = twist_set[right_path_id].angular.z;
        twist_swap_set.push_back(twist_swap);
        right_path_id--;

      }else if(left_path_id < twist_set.size())
      {
        twist_swap.linear.x = twist_set[left_path_id].linear.x;
        twist_swap.linear.y = com_dist;
        twist_swap.angular.x = twist_set[left_path_id].angular.x;
        twist_swap.angular.z = twist_set[left_path_id].angular.z;
        twist_swap_set.push_back(twist_swap);
        left_path_id++;
      }

      if(right_path_id < 0)
      {
        push_right_path = false;
      }else if(left_path_id > twist_set.size()-1)
      {
        push_right_path = true;
      }else
      {
        if(swap_count < swap_path_num_)
        {
          swap_count++;
        }
        else
        {
          if(push_right_path)
          {
            push_right_path = false;
          }else
          {
            push_right_path = true;
          }
          swap_count = 0;
        }
      }
    }
    //std::cout << "right_path_id : " << right_path_id<<" left_path_id : " << left_path_id<<" closest_path_id : " << closest_path_id<<'\n';
  }
  

  //swap twist commond set basis "com_dist" from close to far

  // for (int a = 0; a < twist_swap_set.size() - 1; a++)
  // {
  //   for (int b = a; b < twist_swap_set.size(); b++)
  //   {
  //     if (twist_swap_set[b].linear.y < twist_swap_set[a].linear.y)
  //     {
  //       // swap elements//
  //       swap(twist_swap_set[a].linear.x, twist_swap_set[b].linear.x);
  //       swap(twist_swap_set[a].linear.y, twist_swap_set[b].linear.y);
  //       swap(twist_swap_set[a].angular.x, twist_swap_set[b].angular.x);
  //       swap(twist_swap_set[a].angular.z, twist_swap_set[b].angular.z);
  //     }
  //   }
  // }

  // for (int p=0; p<twist_swap_set.size(); p++) {
  //   std::cout << "p : " <<p<<" " << '\n';
  //   std::cout << "linear x : " <<twist_swap_set[p].linear.x<<" " << '\n';
  //   std::cout << " angular z : " <<twist_swap_set[p].angular.z<<" " << '\n';
  //   std::cout << " com_dist : " <<twist_swap_set[p].linear.y<<" " << '\n';
  //   std::cout <<" " << '\n';
  // }

  //make twist path
  static bool check_path_switch = true;
  double p_circle_x, p_circle_y;
  double p_r;
  double p_angle;

  for (int sw_pc = 0; sw_pc < twist_swap_set.size(); sw_pc++)
  {

    twist_path.poses.clear();
    dist_fp_tw = 0;
    angle_tw = 0;
    x = y = 0;

    if(twist_swap_set[sw_pc].angular.z == 0)
    {
      x_dist_path = max_dist_;
      x_dist_path_step = x_dist_path / step_count;
    }else{

      p_angle = twist_swap_set[sw_pc].angular.z;
      x_dist_path = max_dist_* ((rot_x_zero_vel_angle_ - abs(p_angle))/rot_x_zero_vel_angle_);

      if(x_dist_path <= min_dist_)
      {
        x_dist_path = min_dist_;
      }

      x_dist_path_step = x_dist_path / step_count;

    }
    


    // dist_fp_tw = 0;
    // angle_tw = 0;
    // x = y = 0;
    //
    // x_dist_path = twist_swap_set[sw_pc].linear.x * dec_time;
    // rot_rad_path = twist_swap_set[sw_pc].angular.z * dec_time;
    //
    // x_dist_path_step = x_dist_path / step_count;
    // rot_rad_path_step = rot_rad_path / step_count;

    for (int tc = 0; tc < step_count; tc++)
    {
      if(twist_swap_set[sw_pc].angular.z == 0)
      {

        if (tc > 0)
        {
          dist_fp_tw = dist_fp_tw + x_dist_path_step;

          x = dist_fp_tw;
          y = 0.0;

        }
      }else{
        if (tc > 0)
        {
          dist_fp_tw = dist_fp_tw + x_dist_path_step;
          // angle_tw = angle_tw + rot_rad_path_step;

          x = dist_fp_tw * cos(p_angle);
          y = dist_fp_tw * sin(p_angle);

        }
      }

      // dist_fp_tw = dist_fp_tw + x_dist_path_step;
      // angle_tw = angle_tw + rot_rad_path_step;
      //
      // x = dist_fp_tw * cos(angle_tw);
      // y = dist_fp_tw * sin(angle_tw);

      tw_pose.pose.position.x = x;
      tw_pose.pose.position.y = y;
      tw_pose.pose.position.z = 0;
      tw_pose.pose.orientation.x = 0;
      tw_pose.pose.orientation.y = 0;
      tw_pose.pose.orientation.z = 0;
      tw_pose.pose.orientation.w = 1;

      twist_path.poses.push_back(tw_pose);
    }
    //std::cout<<"--------------------------" <<'\n';
    twist_path_set.push_back(twist_path);
  }

  //compare with costmap
  double obstacle_distance;
  std::vector<bool> nPathDetectObstacle(twist_path_set.size(), false);

  for (int npath_count = 0; npath_count < twist_path_set.size(); npath_count++)
  {
    for (int path_size_count = 0; path_size_count < twist_path_set[npath_count].poses.size(); path_size_count++)
    {
      for (int obstacle_count = 0; obstacle_count < obstacle_poses_.poses.size(); obstacle_count++)
      {
        obstacle_distance = sqrt(pow(twist_path_set[npath_count].poses[path_size_count].pose.position.x - obstacle_poses_.poses[obstacle_count].position.x, 2) +
                                 pow(twist_path_set[npath_count].poses[path_size_count].pose.position.y - obstacle_poses_.poses[obstacle_count].position.y, 2));
        //std::cout << "/obstacle_distance: " <<obstacle_distance<< "/costmap_data_: " <<costmap_data_.info.resolution<<'\n';
        if (obstacle_distance < costmap_data_.info.resolution * 2.0)
        {
          nPathDetectObstacle[npath_count] = true;

          if(enable_path_switch_stop_)
          {
            if(check_path_switch)
            {
              if(npath_count == 0)
              {
                obstacle_path_switch_ = false;
                stop_timer_last_ = ros::Time::now();
              }
              check_path_switch = false;
            }
          }

          //std::cout << "nPathDetectObstacle" << '\n';
          break;
        }
        else
        {
          nPathDetectObstacle[npath_count] = false;
        }
      }
      if (nPathDetectObstacle[npath_count])
      {
        break;
      }
    }
    if(obstacle_path_switch_)
    {
      break;
    }
    else if (!nPathDetectObstacle[npath_count]) //find the available path
    {
      //std::cout << "find the available path: " <<npath_count<< '\n';
      TwistPublish(twist_swap_set[npath_count].linear.x, twist_swap_set[npath_count].angular.z);
      find_available_path_ = true;
      if (PathVisualize_)
      {
        VisualizePath(twist_path_set, npath_count);
      }
      if(npath_count == 0)
      {
        check_path_switch = true;
      }
      break;
    }
    else
    {
      if (nPathDetectObstacle[twist_path_set.size() - 1]) //can't find safe path
      {
        ROS_ERROR("dp Planner : can't find safe path");
        //std::cout << "can't find safe path" << '\n';
        TwistPublish(0, 0);
        if (PathVisualize_)
        {
          VisualizePath(twist_path_set, NO_PATH);
        }
      }
      nPathDetectObstacle[npath_count] = false;
      find_available_path_ = false;
    }
  }
}
//-----------------------------------------------------------------------------------------------
void TwistProfile(double unprofile_linear_x, double unprofile_angular_z)
{
  static double twist_linear_step = 0;
  static double twist_angular_step = 0;

  if (unprofile_linear_x > twist_linear_step)
  {
    twist_linear_step = twist_linear_step + step_linear_x_;
  }
  else if (unprofile_linear_x < twist_linear_step)
  {
    twist_linear_step = unprofile_linear_x;
  }
  else
  {
    twist_linear_step = unprofile_linear_x;
  }

  profile_linear_x_ = twist_linear_step;

  if (unprofile_angular_z > twist_angular_step)
  {
    twist_angular_step = twist_angular_step + step_angular_z_;
  }
  else if (unprofile_angular_z < twist_angular_step)
  {
    twist_angular_step = twist_angular_step - step_angular_z_;
  }
  else
  {
    twist_angular_step = unprofile_angular_z;
  }

  profile_angular_z_ = unprofile_angular_z;
}
//-----------------------------------------------------------------------------------------------
void TwistPublish(double x, double z)
{
  static geometry_msgs::Twist pub_twist;

  if (twist_profile_)
  {
    TwistProfile(x, z);
    pub_twist.linear.x = profile_linear_x_ ;
    pub_twist.angular.z = profile_angular_z_ * angular_velocity_p_;
  }
  else
  {
    pub_twist.linear.x = x;
    pub_twist.angular.z = z * angular_velocity_p_;
  }
  //make_twist_path(pub_twist.linear.x, pub_twist.angular.z);
  twist_pub_.publish(pub_twist);
}
//-----------------------------------------------------------------------------------------------
void make_twist_path(double twist_linear_x, double twist_angular_z)
{
  geometry_msgs::PoseStamped pose;
  nav_msgs::Path twist_path;

  double dec_time;
  double max_dist;
  double x_dist_path;
  double rot_rad_path;
  double step_count = 50;
  double dist = 0;
  double angle = 0;
  double x = 0;
  double y = 0;
  double x_dist_path_step;
  double rot_rad_path_step;

  twist_path.header.stamp = ros::Time::now();
  twist_path.header.frame_id = robot_frame_;

  //calculate max dist
  max_dist = pow(max_vel_x_, 2) / (2 * acc_lim_x_);
  dec_time = max_dist / max_vel_x_;

  x_dist_path = twist_linear_x * dec_time;
  rot_rad_path = twist_angular_z * dec_time;

  x_dist_path_step = x_dist_path / step_count;
  rot_rad_path_step = rot_rad_path / step_count;

  twist_path.poses.clear();

  for (int tc = 0; tc < step_count; tc++)
  {

    if (tc > 0)
    {
      dist = dist + x_dist_path_step;
      angle = angle + rot_rad_path_step;

      x = dist * cos(angle);
      y = dist * sin(angle);
    }
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    twist_path.poses.push_back(pose);
  }
  twist_path_pub_.publish(twist_path);
}
//-----------------------------------------------------------------------------------------------
void PathCallback(const nav_msgs::PathConstPtr &path)
{
  geometry_msgs::PoseStamped pose;

  if(path->poses.size()==0)
  {
    return;
  }

  globle_path_.header.frame_id = path->header.frame_id;

  globle_path_.poses.clear();

  for (int i = 0; i < path->poses.size(); i++)
  {
    pose.pose.position.x = path->poses[i].pose.position.x;
    pose.pose.position.y = path->poses[i].pose.position.y;
    pose.pose.position.z = path->poses[i].pose.position.z;
    pose.pose.orientation.x = path->poses[i].pose.orientation.x;
    pose.pose.orientation.y = path->poses[i].pose.orientation.y;
    pose.pose.orientation.z = path->poses[i].pose.orientation.z;
    pose.pose.orientation.w = path->poses[i].pose.orientation.w;

    globle_path_.poses.push_back(pose);
  }

  get_globle_path_ = true;
  new_path_ = true;
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

  costmap_data_.info.resolution = map->info.resolution;
  obstacle_poses_.poses.clear();

  if (map->header.frame_id == robot_frame_)
  {
    for (int i = 0; i < map->data.size(); i++)
    {
      data = map->data[i];
      value = i;

      if (abs(data) > threshold_occupied_)
      {
        ob_pose.position.y = (floor(value / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        ob_pose.position.x = ((value - map->info.width * floor(value / map->info.width)) * map->info.resolution) + map->info.origin.position.x;

        if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ && ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
        {
          stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          stop_cmd_ = false;
        }
        obstacle_poses_.poses.push_back(ob_pose);
        //std::cout << "i : " << i<< " data : "<< data<< " x : " << ob_pose.position.x<<" y : " << ob_pose.position.y<<'\n';
      }
    }
  }
  else
  {
    ob_posestamped.header.frame_id = map->header.frame_id;

    for (int i = 0; i < map->data.size(); i++)
    {
      data = map->data[i];

      if (abs(data) > threshold_occupied_)
      {

        value = i;

        ob_posestamped.pose.position.y = (floor(value / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        ob_posestamped.pose.position.x = ((value - map->info.width * floor(value / map->info.width)) * map->info.resolution) + map->info.origin.position.x;

        try
        {
          tfBuffer.transform(ob_posestamped, base_ob_pose, robot_frame_, ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("ob : %s", ex.what());
          return;
        }

        ob_pose.position.y = base_ob_pose.pose.position.y;
        ob_pose.position.x = base_ob_pose.pose.position.x;

        if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ && ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
        {
          stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          stop_cmd_ = false;
        }
        obstacle_poses_.poses.push_back(ob_pose);

        //std::cout << "i : " << i<< " data : "<< data<< " x : " << ob_pose.position.x<<" y : " << ob_pose.position.y<<'\n';
      }
    }
  }

  get_costmap_data_ = true;
}
//-----------------------------------------------------------------------------------------------
void VisualizePath(const std::vector<nav_msgs::Path> &nPath, int n)
{
  visualization_msgs::MarkerArray all_rollOuts;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = robot_frame_;
  lane_waypoint_marker.header.stamp = ros::Time(0);
  lane_waypoint_marker.ns = "path_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.05;
  lane_waypoint_marker.scale.y = 0.05;
  lane_waypoint_marker.frame_locked = false;

  for (int i = 0; i < nPath.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = i;

    for (int j = 0; j < nPath[i].poses.size(); j++)
    {
      geometry_msgs::Point point;

      point.x = nPath[i].poses[j].pose.position.x;
      point.y = nPath[i].poses[j].pose.position.y;
      point.z = nPath[i].poses[j].pose.position.z;

      lane_waypoint_marker.points.push_back(point);
    }
    lane_waypoint_marker.color.a = 0.9;

    if (n == i)
    {
      lane_waypoint_marker.color.r = 0.0;
      lane_waypoint_marker.color.g = 1.0;
      lane_waypoint_marker.color.b = 0.0;
    }
    else
    {
      lane_waypoint_marker.color.r = 1.0;
      lane_waypoint_marker.color.g = 0.0;
      lane_waypoint_marker.color.b = 1.0;
    }

    all_rollOuts.markers.push_back(lane_waypoint_marker);
  }
  path_marker_pub_.publish(all_rollOuts);
}
//-----------------------------------------------------------------------------------------------
void ClickCallback(const geometry_msgs::PointStampedConstPtr &msgs)
{
  geometry_msgs::PoseStamped pose;

  globle_path_.header.frame_id = msgs->header.frame_id;

  globle_path_.poses.clear();

  pose.pose.position.x = msgs->point.x;
  pose.pose.position.y = msgs->point.y;
  pose.pose.position.z = msgs->point.z;

  globle_path_.poses.push_back(pose);

  get_globle_path_ = true;
}
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adp_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  path_sub_ = nh.subscribe("path", 10, PathCallback);
  costmap_sub_ = nh.subscribe("costmap", 10, CostmapCallback);

  click_sub_ = nh.subscribe("/clicked_point", 10, ClickCallback);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);

  twist_path_pub_ = nh.advertise<nav_msgs::Path>("twist_path", 20);
  path_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories", 10);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), TimerCallback);
  ros::Timer msgs_timer = nh.createTimer(ros::Duration(2), msgs_timerCallback);

  ros::spin();

  return 0;
}
