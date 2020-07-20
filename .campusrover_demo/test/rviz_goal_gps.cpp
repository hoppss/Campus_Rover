#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

// LIBRARIES
#include <gps_common/conversions.h>

ros::Subscriber rviz_goal_sub_;
ros::Publisher goal_point_pub_;
std::string utm_zone_ = "51R";

void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::PoseStamped map_point;
  static geometry_msgs::PoseStamped world_point;
  static sensor_msgs::NavSatFix gps_msg;
  map_point.header.frame_id = msg->header.frame_id;
  map_point.header.seq++;
  map_point.header.stamp = ros::Time::now();
  map_point.pose.position.x =  msg->pose.position.x;
  map_point.pose.position.y =  msg->pose.position.y;
  map_point.pose.position.z =  msg->pose.position.z;
  map_point.pose.orientation.x = 0;
  map_point.pose.orientation.y = 0;
  map_point.pose.orientation.z = 0;
  map_point.pose.orientation.w = 1;
  bool transformed = false;
  // get destination transform
  while(!transformed)
  {
    try
    {
      tfBuffer.transform(map_point, world_point, "world");
      transformed = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      continue;
    }
  }
  double latitude, longitude;
  gps_common::UTMtoLL(world_point.pose.position.y, 
                      world_point.pose.position.x, 
                      utm_zone_, latitude, longitude);
  gps_msg.header.frame_id = "world";
  gps_msg.header.stamp = ros::Time::now();
  gps_msg.header.seq++;
  gps_msg.latitude = latitude;
  gps_msg.longitude = longitude;
  goal_point_pub_.publish(gps_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autolabor_rviz_goal");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("utm_zone", utm_zone_, "19T");
  rviz_goal_sub_ = n.subscribe("goal", 10, goalCallback);
  goal_point_pub_ = n.advertise<sensor_msgs::NavSatFix>( "autolabor_gps_destination", 10);
  ros::spin();
}