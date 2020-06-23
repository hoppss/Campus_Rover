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

//campus rover srvs
#include <campusrover_msgs/InitFloor.h>
#include <campusrover_msgs/PlannerFunction.h>
#include <campusrover_msgs/PressButton.h> 
#include <campusrover_msgs/ElevatorControlInterface.h>
//campus rover msgs
#include <campusrover_msgs/ElevatorControlStatus.h>
#include <campusrover_msgs/DoorStatus.h>
#include <campusrover_msgs/FloorStatus.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber door_status_sub_,floor_status_sub_;
ros::Publisher control_status_pub_;
ros::ServiceClient init_floor_srv_client_, button_srv_client_, planner_srv_client_;

geometry_msgs::Twist twist_param_1_;

int control_status_ = 0;
int current_floor_;
int init_floor_;
int target_floor_;
string door_status_;

void InitFloorCallService(ros::ServiceClient &client,campusrover_msgs::InitFloor &srv) ;
bool PressButtonCallService(ros::ServiceClient &client,campusrover_msgs::PressButton &srv) ;
bool PlannerFunctionCallService(ros::ServiceClient &client,campusrover_msgs::PlannerFunction &srv) ;
//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  twist_param_1_.linear.x = 0.4;
  twist_param_1_.angular.z = 2.0;


}
//----------------------------------------------------------------------------------------------
void TimerCallback(const ros::TimerEvent &event)
{
  static campusrover_msgs::ElevatorControlStatus status;
  static campusrover_msgs::PlannerFunction planner_param;

  if(control_status_ == 1)
  {
    status.control_status = control_status_;

    planner_param.request.action.data = true;
    planner_param.request.direction_inverse.data = false;
    planner_param.request.speed_parameter = twist_param_1_;
    
    if(PlannerFunctionCallService(planner_srv_client_, planner_param))
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_1_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      control_status_++;
    }


  }else if(control_status_ == 2)
  {
    status.control_status = control_status_;

  }else if(control_status_ == 3)
  {
    status.control_status = control_status_;
    
  }else if(control_status_ == 4)
  {
    status.control_status = control_status_;
    
  }else if(control_status_ == 5)
  {
    status.control_status = control_status_;
    
  }else if(control_status_ == 6)
  {
    
  }else if(control_status_ == 7)
  {
    
  }else if(control_status_ == 8)
  {
    
  }else if(control_status_ == 9)
  {
    
  }else if(control_status_ == 10)
  {
    
  }else if(control_status_ == 11)
  {
    
  }

  control_status_pub_.publish(status);

}
//-----------------------------------------------------------------------------------------------
void DoorStatusCallback(const campusrover_msgs::DoorStatusConstPtr &door_status )
{
  door_status_ = door_status->doors_status[0].data;

}
//-----------------------------------------------------------------------------------------------
void FloorStatusCallback(const campusrover_msgs::FloorStatusConstPtr &floor_status )
{
  current_floor_ = floor_status->current_floor;
}
//-----------------------------------------------------------------------------------------------
void InitFloorCallService(ros::ServiceClient &client,campusrover_msgs::InitFloor &srv)
{
  string str = "===========init floor============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("init floor : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
bool PressButtonCallService(ros::ServiceClient &client,campusrover_msgs::PressButton &srv)
{
  string str = "===========press button============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("press button : Failed to call service");
    ros::Duration(1.0).sleep();
  }
  return srv.response.execution_done.data;

}
//-----------------------------------------------------------------------------------------------
bool PlannerFunctionCallService(ros::ServiceClient &client,campusrover_msgs::PlannerFunction &srv)
{
  string str = "===========planner function============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("planner function : Failed to call service");
    ros::Duration(1.0).sleep();
  }
  
  return srv.response.execution_done.data;

}

//-----------------------------------------------------------------------------------------------
bool ControlServiceCallback(campusrover_msgs::ElevatorControlInterface::Request  &req, campusrover_msgs::ElevatorControlInterface::Response &res)
{
  init_floor_ = req.init_floor;
  target_floor_ = req.target_floor;

  cout << "recrvie elevator control command : " << endl;
  cout << "  init floor : " <<init_floor_<< endl;
  cout << "  target floor : " <<target_floor_<< endl;
  control_status_ = 1;
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

  door_status_sub_ = nh.subscribe("door_status", 10, DoorStatusCallback);
  floor_status_sub_ = nh.subscribe("floor_status", 10, FloorStatusCallback);

  control_status_pub_ = nh.advertise<campusrover_msgs::ElevatorControlStatus>("control_status", 50);

  init_floor_srv_client_ = nh.serviceClient<campusrover_msgs::InitFloor>("init_floor");
  button_srv_client_ = nh.serviceClient<campusrover_msgs::PressButton>("button_press");
  planner_srv_client_ = nh.serviceClient<campusrover_msgs::PlannerFunction>("planner_function");

  ros::ServiceServer control_service = nh.advertiseService("elevator_controller", ControlServiceCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), TimerCallback);


  ros::spin();

  return 0;
}