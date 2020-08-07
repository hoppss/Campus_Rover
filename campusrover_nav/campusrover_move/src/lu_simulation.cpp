#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

using namespace std;

ros::Subscriber twist_sub_, initial_pose_sub_, goal_pose_sub_;
ros::Publisher odom_pub_, global_path_pub_;

nav_msgs::Path global_path_;
string odom_frame_id_, robot_frame_id_;
bool publish_tf_;
double x_ = 0, y_ = 0, yaw_ = 0;
double current_speed_, current_angular_velocity_;
double path_resolution_;


void GetParameters(ros::NodeHandle n_private)
{
  n_private.param<string>("odom_frame_id", odom_frame_id_, "map");
  n_private.param<string>("robot_frame_id", robot_frame_id_, "base_link");
  n_private.param<bool>("publish_tf", publish_tf_, true);
  n_private.param<double>("path_resolution", path_resolution_, 0.05);
}

void timerCallback(const ros::TimerEvent& event)
{
  static tf::TransformBroadcaster tf_bc ;


  // publish odometry message
  nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
  odom->header.frame_id = odom_frame_id_;
  odom->header.stamp = ros::Time::now();
  odom->child_frame_id = robot_frame_id_;

  // Position
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation.x = 0.0;
  odom->pose.pose.orientation.y = 0.0;
  odom->pose.pose.orientation.z = sin(yaw_/2.0);
  odom->pose.pose.orientation.w = cos(yaw_/2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom->pose.covariance[0]  = 0.2; ///< x
  odom->pose.covariance[7]  = 0.2; ///< y
  odom->pose.covariance[35] = 0.4; ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom->twist.twist.linear.x = current_speed_;
  odom->twist.twist.linear.y = 0.0;
  odom->twist.twist.angular.z = current_angular_velocity_;

  odom_pub_.publish(odom);

  if (publish_tf_) {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = robot_frame_id_;
    tf.header.stamp = ros::Time::now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom->pose.pose.orientation;
    tf_bc.sendTransform(tf);
  }


}

void TwistCallback(const geometry_msgs::TwistConstPtr& twist_msg)
{
  static bool first_time = true;
  static ros::Time past_time, current_time;
  static tf::TransformBroadcaster br;
  // static nav_msgs::Odometry odom;
  static double dt;
  static double theta, d_theta;
  double x_dot, y_dot;

  current_speed_ = twist_msg->linear.x;
  current_angular_velocity_ = twist_msg->angular.z;

  if(first_time)
  {
    past_time = ros::Time::now();
    // x_ = y_ = yaw_ = 0;
    first_time = false;
    return;
  }


  current_time = ros::Time::now();
  dt =  current_time.toSec() - past_time.toSec();
  past_time = current_time;



  x_ += current_speed_ * cos(yaw_) * dt;
  y_ += current_speed_ * sin(yaw_) * dt;
  yaw_ += current_angular_velocity_ * dt;




}

void InitialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::PoseStamped initialpose;
  geometry_msgs::PoseStamped odompose;
  geometry_msgs::Pose pose;
  tf::Pose tf_pose;

  initialpose.header.stamp = ros::Time::now();
  initialpose.header.frame_id = pose_msg->header.frame_id;

  initialpose.pose.position.x = pose_msg->pose.pose.position.x;
  initialpose.pose.position.y = pose_msg->pose.pose.position.y;
  initialpose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
  initialpose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
  initialpose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
  initialpose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

  try{
    tfBuffer.transform(initialpose, odompose, odom_frame_id_, ros::Duration(1));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  x_ = odompose.pose.position.x;
  y_ = odompose.pose.position.y;

  pose.orientation.x = odompose.pose.orientation.x;
  pose.orientation.y = odompose.pose.orientation.y;
  pose.orientation.z = odompose.pose.orientation.z;
  pose.orientation.w = odompose.pose.orientation.w;


  tf::poseMsgToTF(pose, tf_pose);
  yaw_ = tf::getYaw(tf_pose.getRotation());


}

void GoalposeCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  static int global_path_step_count;
  static geometry_msgs::PoseStamped global_path_pose;
  static geometry_msgs::PoseStamped last_pose;
  static tf2::Quaternion path_pose_q_tf;
  static geometry_msgs::Quaternion path_pose_q_msg;
  static double target_yaw;
  static bool first_time = true;
  static double step_x, step_y;

  if(first_time)
  {
    last_pose.pose = pose_msg->pose;
    global_path_.header.frame_id = pose_msg->header.frame_id;
    first_time = false;
    return;
  }

  //push the path pose from robot pose to closest path pose 
  global_path_step_count = int (sqrt(pow(last_pose.pose.position.x 
                                      - pose_msg->pose.position.x,2) 
                                  + pow(last_pose.pose.position.y 
                                      - pose_msg->pose.position.y, 2))/path_resolution_);
  step_x = double ((pose_msg->pose.position.x 
                                      - last_pose.pose.position.x)/global_path_step_count);
  step_y = double ((pose_msg->pose.position.y 
                                      - last_pose.pose.position.y)/global_path_step_count);
  
  for(int i = 0;i < global_path_step_count;i++)
  {
    if(i ==0)
    {
      global_path_pose.pose.position.x = last_pose.pose.position.x;
      global_path_pose.pose.position.y = last_pose.pose.position.y;
    }
    else
    {
      global_path_pose.pose.position.x += step_x;
      global_path_pose.pose.position.y += step_y;

      target_yaw = atan2(global_path_pose.pose.position.y - global_path_.poses[i-1].pose.position.y, 
                          global_path_pose.pose.position.x - global_path_.poses[i-1].pose.position.x);
      
      path_pose_q_tf.setRPY(0.0,0.0,target_yaw);
      path_pose_q_msg = tf2::toMsg(path_pose_q_tf);
      global_path_pose.pose.orientation = path_pose_q_msg;
    }
    global_path_.poses.push_back(global_path_pose);
  }

  global_path_pub_.publish(global_path_);
  last_pose.pose = pose_msg->pose;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "lu_simulation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GetParameters(nh_private);
	ros::Time::init();

  twist_sub_ = nh.subscribe("/cmd_vel", 100, TwistCallback);
  initial_pose_sub_ = nh.subscribe("/initialpose", 10, InitialposeCallback);
  goal_pose_sub_ = nh.subscribe("/move_base_simple/goal", 10, GoalposeCallback);

  odom_pub_ = nh.advertise<nav_msgs::Odometry> ("odom", 10);
  global_path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 20);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

  ros::spin();

  return 0;

}
