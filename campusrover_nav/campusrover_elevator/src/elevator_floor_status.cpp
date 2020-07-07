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

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <campusrover_msgs/FloorStatus.h>
#include <campusrover_msgs/InitFloor.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber pressure_sub_;
ros::Publisher floor_status_pub_;

campusrover_msgs::FloorStatus floor_status_data_;

double d_floor_pressure_;
int current_pressure_;
int init_floor_;
bool init_floor_done_ = false;
bool get_current_pressure_data_ = false;



void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<double>("d_floor_pressure", d_floor_pressure_, 10.0);
}

void PressureCallback(std_msgs::Int32 pressure_data)
{
  static int d_pressure, d_floor;

  current_pressure_ = pressure_data.data;

  floor_status_data_.current_pressure = pressure_data.data;

  get_current_pressure_data_ = true;

  d_pressure = floor_status_data_.init_pressure - floor_status_data_.current_pressure;
  d_floor = d_pressure/d_floor_pressure_ ;

  floor_status_data_.current_floor = floor_status_data_.init_floor + d_floor;

  floor_status_pub_.publish(floor_status_data_);
}
//-----------------------------------------------------------------------------------------------
bool ServiceCallback(campusrover_msgs::InitFloor::Request  &req, campusrover_msgs::InitFloor::Response &res)
{
  init_floor_done_ = true;

  while (!get_current_pressure_data_)
  {
    ROS_ERROR("elevator floor handle : Not get the pressure data yet");
    ros::Duration(0.5).sleep();
  }
  
  floor_status_data_.init_floor = req.init_floor;
  floor_status_data_.init_pressure = current_pressure_;

  cout << "----  set init floor ---- " << endl;
  cout << "  init floor : " <<floor_status_data_.init_floor<< endl;
  cout << "  init pressure : " <<floor_status_data_.init_pressure<< endl;

  
  return true;

  
}

//--------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_floor_status");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  pressure_sub_ = nh.subscribe("pressure_data", 10, PressureCallback);
  floor_status_pub_ = nh.advertise<campusrover_msgs::FloorStatus>("floor_status", 50);

  ros::ServiceServer service = nh.advertiseService("init_floor", ServiceCallback);

  ros::spin();

  return 0;
}