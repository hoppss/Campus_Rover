#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber pose_sub_;
ros::Publisher odom_pub_;
string source_frame_id_, target_frame_id_;
bool pub_tf_, pub_odom_;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("source_frame_id", source_frame_id_, "odom");
  n_private.param<string>("target_frame_id", target_frame_id_, "base_link");
  n_private.param<bool>("pub_tf", pub_tf_, true);
  n_private.param<bool>("pub_odom", pub_odom_, false);
}

void PoseCallback(geometry_msgs::PoseArray poses)
{
  static tf::TransformBroadcaster br;
  static nav_msgs::Odometry odom;
  tf::Transform transform;
  static geometry_msgs::PoseStamped input_pose;
  static geometry_msgs::PoseStamped local_pose;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static ros::Time past_time = ros::Time::now();
  static ros::Time current_time;
  static double dt;

  if(poses.poses.size()== 0)
    return;

  input_pose.header.frame_id = poses.header.frame_id;
  input_pose.pose = poses.poses[0];

  if(poses.header.frame_id != target_frame_id_)
  {
    try
    {
      tfBuffer.transform(input_pose, local_pose, target_frame_id_);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.5).sleep();
    }
  }
  else
  {
    local_pose.header.frame_id = poses.header.frame_id;
    local_pose.pose.position = poses.poses[0].position;
    local_pose.pose.orientation = poses.poses[0].orientation;
  }

  tf::Quaternion q(local_pose.pose.orientation.x,
                   local_pose.pose.orientation.y,
                   local_pose.pose.orientation.z,
                   local_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  dt =  current_time.toSec() - past_time.toSec();
  past_time = current_time;

  double seta = atan2(local_pose.pose.position.y,local_pose.pose.position.x);
  double x,y,r=0.0;
  r = sqrt(pow(local_pose.pose.position.x,2)+pow(local_pose.pose.position.y,2));
  x = r * cos(seta - yaw);
  y = r * sin(seta - yaw);


  //Tf
  q.setRPY(0, 0,-yaw);
  transform.setOrigin( tf::Vector3(-x, -y, 0.0) );
  transform.setRotation(q);
  if (pub_tf_)
  {
    br.sendTransform(tf::StampedTransform(transform,
                        ros::Time::now(),
                        source_frame_id_,
                        target_frame_id_));
  }
  //Odom
  if (pub_odom_)
  {
    odom.header.seq++;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = source_frame_id_;
    odom.child_frame_id = target_frame_id_;
    odom.pose.pose.position.x = local_pose.pose.position.x;
    odom.pose.pose.position.y = local_pose.pose.position.y;
    odom.pose.pose.position.z = 0;
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.z = yaw / dt;
    odom_pub_.publish(odom);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_pose_tf");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();


  pose_sub_ = nh.subscribe("elevator_poses", 10, PoseCallback);
  odom_pub_ = nh.advertise<nav_msgs::Odometry> ("odom", 10);

  ros::spin();

  return 0;
}