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
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/Image.h> 

#include <campusrover_msgs/ElevatorControlStatus.h>


using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber camera_1_sub_, camera_2_sub_, camera_depth_1_sub_, camera_depth_2_sub_, elevator_status_sub_;
ros::Publisher image_pub_, depth_image_pub_;

int control_status_;

sensor_msgs::Image image_1_data_;
sensor_msgs::Image image_2_data_;




void Image1Callback(sensor_msgs::Image image_data)
{
  if(control_status_ > 5 || control_status_ == 0)
    return;

  image_1_data_ = image_data;
}


void ImageDepth1Callback(const sensor_msgs::ImageConstPtr &image_depth_data)
{
  if(control_status_ > 5 || control_status_ == 0)
    return;

  image_pub_.publish(image_1_data_);
  depth_image_pub_.publish(image_depth_data);
}
//--------------------------------------------------------------------
void Image2Callback(sensor_msgs::Image image_data)
{
  if(control_status_ < 6 || control_status_ == 0)
    return;

  image_2_data_ = image_data;
}



void ImageDepth2Callback(const sensor_msgs::ImageConstPtr &image_depth_data)
{
  if(control_status_ < 6 || control_status_ == 0)
    return;

  image_pub_.publish(image_2_data_);
  depth_image_pub_.publish(image_depth_data);
}
//--------------------------------------------------------------------

void ControlStatusCallback(const campusrover_msgs::ElevatorControlStatusConstPtr &con_status)
{
  control_status_ = con_status->control_status;
}
//--------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_topic_mux");
  ros::NodeHandle nh;
  ros::Time::init();

  camera_1_sub_ = nh.subscribe("/right_camera/color/image_raw", 10, Image1Callback);
  camera_depth_1_sub_ = nh.subscribe("/right_camera/aligned_depth_to_color/image_raw", 10, ImageDepth1Callback);

  camera_2_sub_ = nh.subscribe("/left_camera/color/image_raw", 10, Image2Callback);
  camera_depth_2_sub_ = nh.subscribe("/left_camera/aligned_depth_to_color/image_raw", 10, ImageDepth2Callback);

  elevator_status_sub_= nh.subscribe("control_status", 10, ControlStatusCallback);

  image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 50);
  depth_image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 50);
  ros::spin();

  return 0;
}