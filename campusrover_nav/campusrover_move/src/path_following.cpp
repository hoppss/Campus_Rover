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

#include <campusrover_msgs/PlannerFunction.h>
#include <campusrover_msgs/ElevatorStatusChecker.h>

#define M_PI 3.14159265358979323846  /* pi */
using namespace std;



ros::Subscriber path_sub_, costmap_sub_, click_sub_;
ros::Publisher twist_pub_, twist_path_pub_, path_marker_pub_;
ros::ServiceClient status_check_client_;

nav_msgs::Path globle_path_;
geometry_msgs::PoseStamped target_pose_;
nav_msgs::OccupancyGrid costmap_data_;

geometry_msgs::Pose robot_tf_pose_;
geometry_msgs::PoseArray obstacle_poses_;

campusrover_msgs::ElevatorStatusChecker status_checker_msg_;



string robot_frame_;
int status_msg_;
double arriving_range_dis_;
double arriving_range_angle_;

bool action_flag_ = false;
bool get_globle_path_ = false;
bool get_costmap_data_ = false;
bool obstacle_stop_cmd_ = false;
bool arriving_end_point_= false;
bool arriving_end_direction_= false;
bool enble_costmap_obstacle_;
bool direction_inverse_= false;;

double threshold_occupied_;
double footprint_max_x_;
double footprint_min_x_;
double footprint_max_y_;
double footprint_min_y_;

double robot_yaw_;
double speed_pid_k_;
double target_yaw_;

double max_linear_velocity_;
double min_linear_velocity_;
double max_angular_velocity_;
double min_angular_velocity_;
double target_point_dis_;

void TwistPublish(double x, double z);
void moving_to_target_point();
void moving_to_target_direction();
void angle_normalize(double &angle);
void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv);

//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("robot_frame", robot_frame_, "base_link");
  n_private.param<double>("arriving_range_dis", arriving_range_dis_, 0.2);
  n_private.param<double>("arriving_range_angle", arriving_range_angle_, 0.1);

  n_private.param<double>("max_linear_velocity", max_linear_velocity_, 0.5);
  n_private.param<double>("max_angular_velocity", max_angular_velocity_, 0.3);
  n_private.param<double>("min_angular_velocity", min_angular_velocity_, 0.1);
  n_private.param<double>("target_point_dis", target_point_dis_, 0.5);

  n_private.param<double>("threshold_occupied", threshold_occupied_, 10);
  n_private.param<double>("footprint_max_x", footprint_max_x_, 3.5);
  n_private.param<double>("footprint_min_x", footprint_min_x_, -0.5);
  n_private.param<double>("footprint_max_y", footprint_max_y_, 0.6);
  n_private.param<double>("footprint_min_y", footprint_min_y_, -0.6);
  n_private.param<double>("speed_pid_k", speed_pid_k_, 0.06);

  n_private.param<bool>("enble_costmap_obstacle", enble_costmap_obstacle_, false);
  n_private.param<bool>("direction_inverse", direction_inverse_, false);
}

//-----------------------------------------------------------------------------------------------
void UpdateCampusRoverPoseFromTF()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;
  static double roll, pitch, yaw;
  static double pre_yaw;

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

  tf::Quaternion q( robot_tf_pose_.orientation.x,
                    robot_tf_pose_.orientation.y,
                    robot_tf_pose_.orientation.z,
                    robot_tf_pose_.orientation.w);
  tf::Matrix3x3 m(q);
  
  m.getRPY(roll, pitch, yaw);
  if(!direction_inverse_)
  {
    pre_yaw = yaw;
  }
  else
  {
    pre_yaw = yaw+M_PI;
    angle_normalize(pre_yaw);
  }
  robot_yaw_ = pre_yaw;

  std::cout << "arriving_end_point_ : " << arriving_end_point_<< " arriving_end_dir : "<< arriving_end_direction_<<'\n';
  
}
//-----------------------------------------------------------------------------------------------
void check_arrive_point()
{
  static double dist;

  dist = sqrt(pow(target_pose_.pose.position.x - robot_tf_pose_.position.x, 2)
              + pow(target_pose_.pose.position.y - robot_tf_pose_.position.y, 2));

  if(dist < arriving_range_dis_ )
  {
    arriving_end_point_ = true;
  }else{
    arriving_end_point_ = false;
  }
}
//-----------------------------------------------------------------------------------------------
void check_arrive_direction()
{
  static double angle_error;
  static double roll, pitch, yaw;
  
  angle_error = target_yaw_ - robot_yaw_;

  angle_normalize(angle_error);

  //cout<< "target_yaw_ : " << target_yaw_<<" robot_yaw_"<<robot_yaw_<<" angle_error"<<angle_error<<endl;

  if(abs(angle_error) < arriving_range_angle_)
  {
    arriving_end_direction_ = true;
    status_checker_msg_.request.node_name.data = "planner";
    status_checker_msg_.request.status.data = arriving_end_direction_;
    StatusCheckCallService(status_check_client_, status_checker_msg_);
  }else{
    arriving_end_direction_ = false;
  }
  

}
//----------------------------------------------------------------------------------------------

void TimerCallback(const ros::TimerEvent &event)
{
  if(!action_flag_)
    return;
    
  if (obstacle_stop_cmd_ )
  {
    TwistPublish(0.0, 0.0);
  }
  else
  {
    if (get_globle_path_)
    {
      if (get_costmap_data_ || !enble_costmap_obstacle_)
      {
        UpdateCampusRoverPoseFromTF();
        if(!arriving_end_direction_)
        {
          
          if(!arriving_end_point_)
          {
            check_arrive_point();
            if(arriving_end_point_)
            {
              moving_to_target_direction();
              return;
            }
            moving_to_target_point();
          }
          else
          {
            check_arrive_direction();
            if(arriving_end_direction_)
            {
              status_msg_ = 4;
              TwistPublish(0.0, 0.0);
              return;
            }

            moving_to_target_direction();
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
    ROS_WARN("path following : Without Globle Path to follow, Waiting for the Path input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 2)
  {
    ROS_WARN("path following : Without costmap input , Waiting for the costmap input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 3)
  {
    ROS_INFO("path following : detect obstacle");
    status_msg_ = 0;
  }
  else if (status_msg_ == 4)
  {
    ROS_INFO("path following : Arrival the destination");
    status_msg_ = 0;
  }
  else
  {
    //ROS_INFO("dp planner : PEV moving");
    status_msg_ = 0;
  }

}
//-----------------------------------------------------------------------------------------------
void moving_to_target_point()
{
  //find close point from globle path

  static double x_p ,y_p;
  static double dist_fp_p;
  static double closest_dist;
  static int closest_id;

  static double x_fp, y_fp;
  static double looking_dist;
  static int target_point_id;
  geometry_msgs::PoseStamped target_pose;

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
  
  //std::cout << "closest_id : " << closest_id<<" new_path_ : " << new_path_<<'\n';
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
      x_fp = globle_path_.poses[fp].pose.position.x;
      y_fp = globle_path_.poses[fp].pose.position.y;
      looking_dist = sqrt(pow(x_fp - robot_tf_pose_.position.x, 2) + pow(y_fp - robot_tf_pose_.position.y, 2));
    
      if(looking_dist >= target_point_dis_)
      {
        break;
      }
    }
    //cout<< "closest_id : " << closest_id<<"target_point_id"<<target_point_id<<endl;
  }

  target_pose.header.frame_id = globle_path_.header.frame_id;
  target_pose.pose.position.x = globle_path_.poses[target_point_id].pose.position.x;
  target_pose.pose.position.y = globle_path_.poses[target_point_id].pose.position.y;
  target_pose.pose.position.z = globle_path_.poses[target_point_id].pose.position.z;

  static double direction_yaw;
  static double yaw_error;
  static double ang_vel;
  static double len_vel;


  direction_yaw = atan2(target_pose.pose.position.y - robot_tf_pose_.position.y, target_pose.pose.position.x - robot_tf_pose_.position.x);
  
  yaw_error = direction_yaw - robot_yaw_;
  angle_normalize(yaw_error);

  ang_vel = yaw_error*speed_pid_k_;

  if(direction_inverse_)
  {
    len_vel = -max_linear_velocity_;
  }
  else
  {
    len_vel = max_linear_velocity_;
  }

  if(target_point_id == closest_id)
  {
    len_vel = len_vel*0.7;
  }
  
  
  
  //cout<< "direction_yaw : " <<direction_yaw<<" robot_yaw_"<<robot_yaw_ <<" robot_direction_yaw"<<robot_direction_yaw <<" yaw_error"<<yaw_error<<endl;
  
  
  TwistPublish(len_vel, ang_vel);
}
//-----------------------------------------------------------------------------------------------
void moving_to_target_direction()
{
  static double yaw_error;
  static double ang_vel;

  yaw_error = target_yaw_ - robot_yaw_;
  angle_normalize(yaw_error);

  ang_vel = yaw_error*speed_pid_k_;
  
  

  TwistPublish(0.0, ang_vel);
}
//-----------------------------------------------------------------------------------------------
void TwistPublish(double x, double z)
{
  static geometry_msgs::Twist pub_twist;

  if(z > max_angular_velocity_)
  {
    pub_twist.angular.z =max_angular_velocity_;
  }
  else if(z < -max_angular_velocity_)
  {
    pub_twist.angular.z = -max_angular_velocity_;
  }
  else if(z > 0 && z < min_angular_velocity_)
  {
    pub_twist.angular.z = min_angular_velocity_;
  }
  else if(z < 0 && z > -min_angular_velocity_)
  {
    pub_twist.angular.z = -min_angular_velocity_;
  }
  else
  {
    pub_twist.angular.z = z;
  }
  

    pub_twist.linear.x = x;
    
  
  //make_twist_path(pub_twist.linear.x, pub_twist.angular.z);
  twist_pub_.publish(pub_twist);
}
//-----------------------------------------------------------------------------------------------
void angle_normalize(double &angle)
{
  if(angle > M_PI)
  {
    angle = -2*M_PI + angle;
  }else if(angle < -M_PI)
  {
    angle = 2*M_PI + angle;
  }
}
//-----------------------------------------------------------------------------------------------
void PathCallback(const nav_msgs::PathConstPtr &path)
{
  static geometry_msgs::PoseStamped pose;
  static double roll, pitch, yaw;

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

    if(i == path->poses.size()-1)
    {
      target_pose_.header.frame_id = path->header.frame_id;
      target_pose_.pose.position = path->poses[i].pose.position;
      target_pose_.pose.orientation = path->poses[i].pose.orientation;
    }
  }
  tf::Quaternion q( target_pose_.pose.orientation.x,
                    target_pose_.pose.orientation.y,
                    target_pose_.pose.orientation.z,
                    target_pose_.pose.orientation.w);
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
          obstacle_stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          obstacle_stop_cmd_ = false;
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
          obstacle_stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          obstacle_stop_cmd_ = false;
        }
        obstacle_poses_.poses.push_back(ob_pose);

        //std::cout << "i : " << i<< " data : "<< data<< " x : " << ob_pose.position.x<<" y : " << ob_pose.position.y<<'\n';
      }
    }
  }

  get_costmap_data_ = true;
}
//-----------------------------------------------------------------------------------------------
bool ServiceCallback(campusrover_msgs::PlannerFunction::Request  &req, campusrover_msgs::PlannerFunction::Response &res)
{
  //ros::Duration(1.0).sleep();
  action_flag_ = req.action.data;
  direction_inverse_ = req.direction_inverse.data;
  max_linear_velocity_ = req.speed_parameter.linear.x;
  max_angular_velocity_ = req.speed_parameter.angular.z;

  cout << "recrvie planner fuction : " << endl;
  cout << "  action_flag : " <<action_flag_<< endl;
  cout << "  direction_inverse : " <<direction_inverse_<< endl;
  cout << "  speed fuction : " <<req.speed_parameter<< endl;
  //
  if(action_flag_)
  {
    arriving_end_point_ = false;
    arriving_end_direction_ = false;
    get_globle_path_ = false;
  }
  
  
  return true;

  
}
//-----------------------------------------------------------------------------------------------
void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv)
{
  ros::Duration(0.5).sleep();
  string str = "===========planner status check============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("planner status check : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_following");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  path_sub_ = nh.subscribe("path", 10, PathCallback);
  costmap_sub_ = nh.subscribe("costmap", 10, CostmapCallback);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);

  twist_path_pub_ = nh.advertise<nav_msgs::Path>("twist_path", 20);
  path_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories", 10);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), TimerCallback);
  ros::Timer msgs_timer = nh.createTimer(ros::Duration(2), msgs_timerCallback);

  ros::ServiceServer service = nh.advertiseService("planner_function", ServiceCallback);
  status_check_client_ = nh.serviceClient<campusrover_msgs::ElevatorStatusChecker>("elevator_status_checker");

  ros::spin();

  return 0;
}

