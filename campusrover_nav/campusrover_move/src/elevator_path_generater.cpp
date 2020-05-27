#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Path.h>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <campusrover_msgs/DoorStatus.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber door_status_sub_;
ros::Publisher path_pub_;

std::vector<geometry_msgs::PoseStamped> poses_ ;
geometry_msgs::PoseStamped pose_ ;

string path_frame_, robot_frame_;
double path_x1_dis;
double path_x2_dis;
double path_resolution_;
double min_pose_dis_;
double pose_range_threshold_;
double pose_yaw_threshold_;
int priority_;
int avg_pose_size_;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("path_frame", path_frame_, "map");
  n_private.param<string>("robot_frame", robot_frame_, "base_link");
  n_private.param<double>("path_x1_dis", path_x1_dis, 3.0);
  n_private.param<double>("path_x2_dis", path_x2_dis, 0.8);
  n_private.param<int>("elevator_priority", priority_, 0.0);
  n_private.param<double>("path_resolution", path_resolution_, 0.05);
  n_private.param<double>("min_pose_dis", min_pose_dis_, 0.7);
  n_private.param<double>("pose_range_threshold", pose_range_threshold_, 0.1);
  n_private.param<double>("pose_yaw_threshold", pose_yaw_threshold_, 0.1);
  n_private.param<int>("avg_pose_size", avg_pose_size_, 5.0);

}

void DoorStatusCallback(const campusrover_msgs::DoorStatusConstPtr &doors_status)
{
  static nav_msgs::Path path;
  static geometry_msgs::PoseStamped path_pose ;
  static geometry_msgs::PoseStamped before_pose ;
  static geometry_msgs::PoseStamped after_pose ;
  static geometry_msgs::PoseStamped last_pose ;
  static geometry_msgs::PoseStamped path_frame_pose ;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static double roll, pitch, yaw;
  static double roll2, pitch2, yaw2;
  static bool first_time = true;
  static int poses_count = 0;
  static bool door_open;

  if(doors_status->doors_pose.poses.size() == 0)
    return;

  path.poses.clear();
  door_open = false;
  
  before_pose.header.frame_id = doors_status->doors_pose.header.frame_id;

  if(priority_ > 0)
  {
    if(doors_status->doors_pose.poses.size() >= 2.0)
    {
      before_pose.pose = doors_status->doors_pose.poses[priority_-1];
    }
    else
    {
      before_pose.pose = doors_status->doors_pose.poses[0];
    }
  }
  else
  {
    for(int i =0;i<doors_status->doors_status.size();i++)
    {
      if(doors_status->doors_status[i].data == "open")
      {
        before_pose.pose = doors_status->doors_pose.poses[i];
        door_open = true;
        break;
      }
    }
    if(!door_open)
      return;
  }
  
  

  // try
  // {
  //   tfBuffer.transform(before_pose, robot_pose, robot_frame_);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("elevator_path_generater : %s",ex.what());
  //   ros::Duration(0.5).sleep();
  // }
  // std::cout << " dis: " <<  sqrt(pow(before_pose.pose.position.x,2)+pow(before_pose.pose.position.y,2)) <<'\n';
  // if(sqrt(pow(before_pose.pose.position.x,2)+pow(before_pose.pose.position.y,2)) < min_pose_dis_)
  // {
  //   std::cout << " out!! "  <<'\n';
  //   return;
  // }

  if(before_pose.header.frame_id != path_frame_)
  {
    try
    {
      tfBuffer.transform(before_pose, path_frame_pose, path_frame_);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("elevator_path_generater : %s",ex.what());
      ros::Duration(0.5).sleep();
      return;
    }
  }
  else
  {
    path_frame_pose.header.frame_id = before_pose.header.frame_id;
    path_frame_pose.pose = before_pose.pose;
  }

  tf::Quaternion q( path_frame_pose.pose.orientation.x,
                    path_frame_pose.pose.orientation.y,
                    path_frame_pose.pose.orientation.z,
                    path_frame_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  
  m.getRPY(roll, pitch, yaw);
  
  double sum_x=0,sum_y=0,sum_yaw=0;
  double avg_x=0,avg_y=0,avg_yaw=0;
  if(poses_.size() >= avg_pose_size_)
  {    
    for(int i = 0;i < poses_.size();i++)
    {
      sum_x += poses_[i].pose.position.x;
      sum_y += poses_[i].pose.position.y;

      tf::Quaternion q2( poses_[i].pose.orientation.x,
                        poses_[i].pose.orientation.y,
                        poses_[i].pose.orientation.z,
                        poses_[i].pose.orientation.w);
      tf::Matrix3x3 m2(q2);
  
      m2.getRPY(roll2, pitch2, yaw2);

      sum_yaw += yaw2;
    }
    avg_x = sum_x/poses_.size();
    avg_y = sum_y/poses_.size();
    avg_yaw = sum_yaw/poses_.size();
    //std::cout << " dis "  <<sqrt(pow(avg_x -path_frame_pose.pose.position.x,2)+pow(avg_y-path_frame_pose.pose.position.y,2))<<'\n';
    //std::cout << " avg_yaw "  <<avg_yaw<< " yaw "  <<yaw<<'\n';
    if(sqrt(pow(avg_x -path_frame_pose.pose.position.x,2)+pow(avg_y-path_frame_pose.pose.position.y,2))< pose_range_threshold_)
    {
      if(abs(avg_yaw - yaw) < pose_yaw_threshold_)
      {
        poses_[poses_count]=path_frame_pose ;
        poses_count++;
        if(poses_count > avg_pose_size_-1)
        {
          poses_count =0;
        }
      }
      
    }
    
  }else
  {
    poses_.push_back(path_frame_pose) ;
    return;
  }
  
  // std::cout << " size "  <<poses_.size()<< " sum_x "  <<sum_x<< " sum_y "  <<sum_x<<'\n';

  
  x1_point.x = avg_x + path_x1_dis*cos(avg_yaw-M_PI);
  x1_point.y = avg_y + path_x1_dis*sin(avg_yaw-M_PI);

  x2_point.x = avg_x + path_x2_dis*cos(avg_yaw);
  x2_point.y = avg_y + path_x2_dis*sin(avg_yaw);

  int step_count = int (sqrt(pow(x1_point.x - x2_point.x,2) + pow(x1_point.y - x2_point.y, 2))/path_resolution_);
  double s_x = double ((x2_point.x - x1_point.x)/step_count);
  double s_y = double ((x2_point.y - x1_point.y)/step_count);

  path_pose.header.frame_id = path_frame_;
  path.header.frame_id = path_frame_;
  for(int i = 0;i < step_count;i++)
  {
    if(i ==0)
    {
      path_pose.pose.position.x = x1_point.x;
      path_pose.pose.position.y = x1_point.y;
    }
    else
    {
      path_pose.pose.position.x += s_x;
      path_pose.pose.position.y += s_y;
    }
    path.poses.push_back(path_pose);
    
  }
  path_pub_.publish(path);
  

}


//--------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_path_generater");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  door_status_sub_ = nh.subscribe("doors_status", 10, DoorStatusCallback);
  path_pub_ = nh.advertise<nav_msgs::Path> ("elevator_path", 10);

  ros::spin();

  return 0;
}