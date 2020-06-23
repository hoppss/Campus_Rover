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

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <campusrover_msgs/FloorStatus.h>
#include <campusrover_msgs/InitFloor.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber pressure_sub_;
ros::Publisher floor_status_pub_;

int init_floor_;



void get_parameters(ros::NodeHandle n_private)
{


}

void PressureCallback(std_msgs::Float32 data)
{

}
//-----------------------------------------------------------------------------------------------
bool ServiceCallback(campusrover_msgs::InitFloor::Request  &req, campusrover_msgs::InitFloor::Response &res)
{
  init_floor_ = req.init_floor;

  cout << "----  set init floor ---- " << endl;
  cout << "  init floor : " <<init_floor_<< endl;
  
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

  pressure_sub_ = nh.subscribe("pressure", 10, PressureCallback);
  floor_status_pub_ = nh.advertise<campusrover_msgs::FloorStatus>("floor_status", 50);

  ros::ServiceServer service = nh.advertiseService("init_floor", ServiceCallback);

  ros::spin();

  return 0;
}