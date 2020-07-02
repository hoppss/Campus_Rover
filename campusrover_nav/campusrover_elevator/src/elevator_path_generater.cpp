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
#include <campusrover_msgs/ElevatorControlStatus.h>
#include <campusrover_msgs/ElevatorStatusChecker.h>


using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber elevator_pose_sub_, control_status_sub_;
ros::Publisher path_pub_;
ros::ServiceClient status_check_client_;


geometry_msgs::PoseStamped pose_ ;

string path_frame_, robot_frame_;

double path_resolution_;
double pose_range_threshold_;
double pose_yaw_threshold_;
int avg_pose_size_;

double outside_button_pose_x_1_;
double outside_button_pose_y_1_;
double outside_button_pose_x_2_;
double outside_button_pose_y_2_;
double outside_button_pose_yaw_;

double outside_standby_pose_x_1_;
double outside_standby_pose_y_1_;
double outside_standby_pose_x_2_;
double outside_standby_pose_y_2_;
double outside_standby_pose_yaw_;

double enter_path_1_dis_;
double enter_path_2_dis_;
double enter_pose_yaw_;

double inside_button_pose_x_1_;
double inside_button_pose_y_1_;
double inside_button_pose_x_2_;
double inside_button_pose_y_2_;
double inside_button_pose_yaw_;

double inside_standby_pose_x_1_;
double inside_standby_pose_y_1_;
double inside_standby_pose_x_2_;
double inside_standby_pose_y_2_;
double inside_standby_pose_yaw_;

double leave_path_1_dis_;
double leave_path_2_dis_;
double leave_pose_yaw_;

int control_status_ ;

void MoveToBtnPath_Outside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);
void MoveToStandbyPositionPath_Outside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);;
void EnterElevatorPath(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);
void MoveToBtnPath_Inside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);
void MoveToStandbyPositionPath_Inside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);
void LeaveElevatorPath(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw);

void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv);

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("path_frame", path_frame_, "map");
  n_private.param<string>("robot_frame", robot_frame_, "base_link");
  n_private.param<double>("path_resolution", path_resolution_, 0.05);

  //filter
  n_private.param<double>("pose_range_threshold", pose_range_threshold_, 0.1);
  n_private.param<double>("pose_yaw_threshold", pose_yaw_threshold_, 0.1);
  n_private.param<int>("avg_pose_size", avg_pose_size_, 5.0);

  //MoveToBtnPath_Outside step 1
  n_private.param<double>("outside_button_pose_x_1", outside_button_pose_x_1_, -1.3);
  n_private.param<double>("outside_button_pose_y_1", outside_button_pose_y_1_, -0.6);

  n_private.param<double>("outside_button_pose_x_2", outside_button_pose_x_2_, -0.25);
  n_private.param<double>("outside_button_pose_y_2", outside_button_pose_y_2_, -0.6);
  n_private.param<double>("outside_button_pose_yaw", outside_button_pose_yaw_, 0.0);

  
  //MoveToStandbyPositionPath_Outside step 3
  n_private.param<double>("outside_standby_pose_x_1", outside_standby_pose_x_1_, 0.0);
  n_private.param<double>("outside_standby_pose_y_1", outside_standby_pose_y_1_, 0.0);

  n_private.param<double>("outside_standby_pose_x_2", outside_standby_pose_x_2_, -2.0);
  n_private.param<double>("outside_standby_pose_y_2", outside_standby_pose_y_2_, 0.0);
  n_private.param<double>("outside_standby_pose_yaw", outside_standby_pose_yaw_, 0.0);

  //EnterElevatorPath step 5
  n_private.param<double>("enter_path_1_dis", enter_path_1_dis_, 3.0);
  n_private.param<double>("enter_path_2_dis", enter_path_2_dis_, 0.8);
  n_private.param<double>("enter_pose_yaw", enter_pose_yaw_, -1.57);

  //MoveToBtnPath_Inside step 6
  n_private.param<double>("inside_button_pose_x_1", inside_button_pose_x_1_, 1.0);
  n_private.param<double>("inside_button_pose_y_1", inside_button_pose_y_1_, 0.8);
  n_private.param<double>("inside_button_pose_x_2", inside_button_pose_x_2_, 1.0);
  n_private.param<double>("inside_button_pose_y_2", inside_button_pose_y_2_, 0.8);
  n_private.param<double>("inside_button_pose_yaw", inside_button_pose_yaw_, 0.8);

  //MoveToStandbyPositionPath_Inside step 8
  n_private.param<double>("inside_standby_pose_x_1", inside_standby_pose_x_1_, 1.0);
  n_private.param<double>("inside_standby_pose_y_1", inside_standby_pose_y_1_, 0.8);
  n_private.param<double>("inside_standby_pose_x_2", inside_standby_pose_x_2_, 1.0);
  n_private.param<double>("inside_standby_pose_y_2", inside_standby_pose_y_2_, 0.8);
  n_private.param<double>("inside_standby_pose_yaw", inside_standby_pose_yaw_, 0.8);
  
  //LeaveElevatorPath step 7
  n_private.param<double>("leave_path_1_dis", leave_path_1_dis_, 0.8);
  n_private.param<double>("leave_path_2_dis", leave_path_2_dis_, 1.5);
  n_private.param<double>("leave_pose_yaw", leave_pose_yaw_, 0.0);
}

void PoseCallback(const geometry_msgs::PoseArrayConstPtr &elevator_poses)
{
  static campusrover_msgs::ElevatorStatusChecker status_msg;
  static int current_status;
  static int last_status =0;
  static nav_msgs::Path path;
  static geometry_msgs::PoseStamped path_frame_pose ;
  static geometry_msgs::PoseStamped before_pose ;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  
  static double roll, pitch, yaw;
  


  if(control_status_ != 1 && control_status_ != 3 &&control_status_ != 5 &&control_status_ != 6 && control_status_ != 8 && control_status_ != 11 )
    return;

  

  if(elevator_poses->poses.size() == 0)
    return;

  path.poses.clear();
  
  before_pose.header.frame_id = elevator_poses->header.frame_id;
  before_pose.pose = elevator_poses->poses[0];
  

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


  //std::cout << " control_status_ "  <<control_status_<<'\n';
  if(control_status_ == 1.0) /////step 1 /////////////////////////////////////////////////////////
  {
    MoveToBtnPath_Outside(path_frame_pose, path, yaw);
  }
  else if(control_status_ == 3.0) /////step 3 /////////////////////////////////////////////////////////
  {
    MoveToStandbyPositionPath_Outside(path_frame_pose, path, yaw);
  }
  else if(control_status_ == 5.0) /////step 5 /////////////////////////////////////////////////////////
  {
    EnterElevatorPath(path_frame_pose, path, yaw);
  }
  else if(control_status_ == 6.0) /////step 6 /////////////////////////////////////////////////////////
  {
    MoveToBtnPath_Inside(path_frame_pose, path, yaw);
  }
  else if(control_status_ == 8.0) /////step 8 /////////////////////////////////////////////////////////
  {
    MoveToStandbyPositionPath_Inside(path_frame_pose, path, yaw);
  }
  else if(control_status_ == 11.0) /////step 11 /////////////////////////////////////////////////////////
  {
    LeaveElevatorPath(path_frame_pose, path, yaw);
  }
  

  path_pub_.publish(path);

  current_status = control_status_;

  if(last_status != current_status)
  {
    status_msg.request.node_name.data = "path_generater";;
    status_msg.request.status.data = true;
    StatusCheckCallService(status_check_client_, status_msg);
    last_status = current_status;
  }

  
  

}
//--------------------------------------------------------------------
void ControlStatusCallback(const campusrover_msgs::ElevatorControlStatusConstPtr &con_status)
{
  control_status_ = con_status->control_status;
}
//////////////////////////////////step 1 /////////////////////////////////////////////////////////
void MoveToBtnPath_Outside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static geometry_msgs::PoseStamped path_pose ;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double target_yaw;

  x1_point.x = input_pose.pose.position.x + outside_button_pose_x_1_*cos(yaw) - outside_button_pose_y_1_*sin(yaw);
  x1_point.y = input_pose.pose.position.y + outside_button_pose_x_1_*sin(yaw) + outside_button_pose_y_1_*cos(yaw);

  x2_point.x = input_pose.pose.position.x + outside_button_pose_x_2_*cos(yaw) - outside_button_pose_y_2_*sin(yaw);
  x2_point.y = input_pose.pose.position.y + outside_button_pose_x_2_*sin(yaw) + outside_button_pose_y_2_*cos(yaw);

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

      if(i >= step_count-1)
      {
        target_yaw = outside_button_pose_yaw_ + yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }


}
//--------------------------------------------------------------------
void MoveToStandbyPositionPath_Outside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static geometry_msgs::PoseStamped path_pose ;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double target_yaw;

  x1_point.x = input_pose.pose.position.x + outside_standby_pose_x_1_*cos(yaw ) - outside_standby_pose_y_1_*sin(yaw );
  x1_point.y = input_pose.pose.position.y + outside_standby_pose_x_1_*sin(yaw ) + outside_standby_pose_y_1_*cos(yaw );

  x2_point.x = input_pose.pose.position.x + outside_standby_pose_x_2_*cos(yaw ) - outside_standby_pose_y_2_*sin(yaw );
  x2_point.y = input_pose.pose.position.y + outside_standby_pose_x_2_*sin(yaw ) + outside_standby_pose_y_2_*cos(yaw );

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

      if(i >= step_count-1)
      {
        target_yaw = outside_standby_pose_yaw_ + yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }

}

//--------------------------------------------------------------------
void EnterElevatorPath(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{
  static std::vector<geometry_msgs::PoseStamped> avg_poses ;
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static int poses_count = 0;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static geometry_msgs::PoseStamped path_pose ;
  static double target_yaw;
  double roll2, pitch2, yaw2;
  double sum_x=0,sum_y=0,sum_yaw=0;
  double avg_x=0,avg_y=0,avg_yaw=0;

  if(avg_poses.size() >= avg_pose_size_)
  {    
    for(int i = 0;i < avg_poses.size();i++)
    {
      sum_x += avg_poses[i].pose.position.x;
      sum_y += avg_poses[i].pose.position.y;

      tf::Quaternion q2( avg_poses[i].pose.orientation.x,
                        avg_poses[i].pose.orientation.y,
                        avg_poses[i].pose.orientation.z,
                        avg_poses[i].pose.orientation.w);
      tf::Matrix3x3 m2(q2);
  
      m2.getRPY(roll2, pitch2, yaw2);

      sum_yaw += yaw2;
    }
    avg_x = sum_x/avg_poses.size();
    avg_y = sum_y/avg_poses.size();
    avg_yaw = sum_yaw/avg_poses.size();
    //std::cout << " dis "  <<sqrt(pow(avg_x -input_pose.pose.position.x,2)+pow(avg_y-input_pose.pose.position.y,2))<<'\n';
    //std::cout << " avg_yaw "  <<avg_yaw<< " yaw "  <<yaw<<'\n';
    if(sqrt(pow(avg_x -input_pose.pose.position.x,2)+pow(avg_y-input_pose.pose.position.y,2))< pose_range_threshold_)
    {
      if(abs(avg_yaw - yaw) < pose_yaw_threshold_)
      {
        avg_poses[poses_count]=input_pose ;
        poses_count++;
        if(poses_count > avg_pose_size_-1)
        {
          poses_count =0;
        }
      }
      
    }
    
  }else
  {
    avg_poses.push_back(input_pose) ;
    return;
  }
  
  // std::cout << " size "  <<avg_poses.size()<< " sum_x "  <<sum_x<< " sum_y "  <<sum_x<<'\n';

  
  x1_point.x = avg_x + enter_path_1_dis_*cos(avg_yaw-M_PI);
  x1_point.y = avg_y + enter_path_1_dis_*sin(avg_yaw-M_PI);

  x2_point.x = avg_x + enter_path_2_dis_*cos(avg_yaw);
  x2_point.y = avg_y + enter_path_2_dis_*sin(avg_yaw);

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

      
      if(i >= step_count-1)
      {
        target_yaw = enter_pose_yaw_ + avg_yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }
}
//--------------------------------------------------------------------
void MoveToBtnPath_Inside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static geometry_msgs::PoseStamped path_pose ;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double target_yaw;

  x1_point.x = input_pose.pose.position.x + inside_button_pose_x_1_*cos(yaw) - inside_button_pose_y_1_*sin(yaw);
  x1_point.y = input_pose.pose.position.y + inside_button_pose_x_1_*sin(yaw) + inside_button_pose_y_1_*cos(yaw);

  x2_point.x = input_pose.pose.position.x + inside_button_pose_x_2_*cos(yaw) - inside_button_pose_y_2_*sin(yaw);
  x2_point.y = input_pose.pose.position.y + inside_button_pose_x_2_*sin(yaw) + inside_button_pose_y_2_*cos(yaw);

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

      if(i >= step_count-1)
      {
        target_yaw = inside_button_pose_yaw_ + yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }
  

}
//--------------------------------------------------------------------
void MoveToStandbyPositionPath_Inside(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static geometry_msgs::PoseStamped path_pose ;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double target_yaw;

  x1_point.x = input_pose.pose.position.x + inside_standby_pose_x_1_*cos(yaw ) - inside_standby_pose_y_1_*sin(yaw );
  x1_point.y = input_pose.pose.position.y + inside_standby_pose_x_1_*sin(yaw ) + inside_standby_pose_y_1_*cos(yaw );

  x2_point.x = input_pose.pose.position.x + inside_standby_pose_x_2_*cos(yaw ) - inside_standby_pose_y_2_*sin(yaw );
  x2_point.y = input_pose.pose.position.y + inside_standby_pose_x_2_*sin(yaw ) + inside_standby_pose_y_2_*cos(yaw );

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

      if(i >= step_count-1)
      {
        target_yaw = inside_standby_pose_yaw_ + yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }

}
//--------------------------------------------------------------------
void LeaveElevatorPath(geometry_msgs::PoseStamped &input_pose , nav_msgs::Path &path, double &yaw)
{

  static std::vector<geometry_msgs::PoseStamped> avg_poses ;
  static geometry_msgs::Point x1_point;
  static geometry_msgs::Point x2_point;
  static int poses_count = 0;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static geometry_msgs::PoseStamped path_pose ;
  static double target_yaw;
  double roll2, pitch2, yaw2;
  double sum_x=0,sum_y=0,sum_yaw=0;
  double avg_x=0,avg_y=0,avg_yaw=0;

  if(avg_poses.size() >= avg_pose_size_)
  {    
    for(int i = 0;i < avg_poses.size();i++)
    {
      sum_x += avg_poses[i].pose.position.x;
      sum_y += avg_poses[i].pose.position.y;

      tf::Quaternion q2( avg_poses[i].pose.orientation.x,
                        avg_poses[i].pose.orientation.y,
                        avg_poses[i].pose.orientation.z,
                        avg_poses[i].pose.orientation.w);
      tf::Matrix3x3 m2(q2);
  
      m2.getRPY(roll2, pitch2, yaw2);

      sum_yaw += yaw2;
    }
    avg_x = sum_x/avg_poses.size();
    avg_y = sum_y/avg_poses.size();
    avg_yaw = sum_yaw/avg_poses.size();
    //std::cout << " dis "  <<sqrt(pow(avg_x -input_pose.pose.position.x,2)+pow(avg_y-input_pose.pose.position.y,2))<<'\n';
    //std::cout << " avg_yaw "  <<avg_yaw<< " yaw "  <<yaw<<'\n';
    if(sqrt(pow(avg_x -input_pose.pose.position.x,2)+pow(avg_y-input_pose.pose.position.y,2))< pose_range_threshold_)
    {
      if(abs(avg_yaw - yaw) < pose_yaw_threshold_)
      {
        avg_poses[poses_count]=input_pose ;
        poses_count++;
        if(poses_count > avg_pose_size_-1)
        {
          poses_count =0;
        }
      }
      
    }
    
  }else
  {
    avg_poses.push_back(input_pose) ;
    return;
  }
  
  // std::cout << " size "  <<avg_poses.size()<< " sum_x "  <<sum_x<< " sum_y "  <<sum_x<<'\n';

  
  x1_point.x = avg_x + leave_path_1_dis_*cos(avg_yaw-M_PI);
  x1_point.y = avg_y + leave_path_1_dis_*sin(avg_yaw-M_PI);

  x2_point.x = avg_x + leave_path_2_dis_*cos(avg_yaw);
  x2_point.y = avg_y + leave_path_2_dis_*sin(avg_yaw);

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

      
      if(i >= step_count-1)
      {
        target_yaw = enter_pose_yaw_ + avg_yaw;
      }
      else
      {
        target_yaw = atan2(path_pose.pose.position.y - path.poses[i-1].pose.position.y, path_pose.pose.position.x - path.poses[i-1].pose.position.x);
      }
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      path_pose.pose.orientation = path_pose_q_msg;

    }
    path.poses.push_back(path_pose);
    
  }
  
}
//--------------------------------------------------------------------

void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv)
{
  string str = "===========path generater status check============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("path generater status check : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_path_generater");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  control_status_sub_ = nh.subscribe("control_status", 10, ControlStatusCallback);
  elevator_pose_sub_ = nh.subscribe("elevator_poses", 10, PoseCallback);

  path_pub_ = nh.advertise<nav_msgs::Path> ("elevator_path", 10);

  status_check_client_ = nh.serviceClient<campusrover_msgs::ElevatorStatusChecker>("elevator_status_checker");

  ros::spin();

  return 0;
}