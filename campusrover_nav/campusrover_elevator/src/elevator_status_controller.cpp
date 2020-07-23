#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string>

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
#include <campusrover_msgs/ElevatorStatusChecker.h>
#include <campusrover_msgs/ArmStandby.h>
//campus rover msgs
#include <campusrover_msgs/ElevatorControlStatus.h>
#include <campusrover_msgs/DoorStatus.h>
#include <campusrover_msgs/FloorStatus.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber door_status_sub_,floor_status_sub_;
ros::Publisher control_status_pub_, finish_pub_;
ros::ServiceClient init_floor_srv_client_, button_srv_client_, planner_srv_client_, arm_standby_srv_client_;

geometry_msgs::Twist twist_param_0_, twist_param_1_, twist_param_2_;

int control_status_ = 0;
int current_floor_;
int init_floor_;
int target_floor_;
string door_status_;

bool planner_check_done_ = false;
bool arm_return_status_ = false;
bool arm_return_checker_ = false;
bool path_generater_check_done_ = false;
bool elevator_pose_ckeck_done_ = false;
bool arrive_target_floor_ = false;
bool control_status_first_time_ = true;

void InitFloorCallService(ros::ServiceClient &client,campusrover_msgs::InitFloor &srv) ;
void PressButtonCallService(ros::ServiceClient &client,campusrover_msgs::PressButton &srv) ;
void PlannerFunctionCallService(ros::ServiceClient &client,campusrover_msgs::PlannerFunction &srv) ;
void ArmStandybyFunctionCallService(ros::ServiceClient &client,campusrover_msgs::ArmStandby &srv);

//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  twist_param_0_.linear.x = 0.0;
  twist_param_0_.angular.z = 0.0;

  twist_param_1_.linear.x = 0.2;
  twist_param_1_.angular.z = 0.4;

  twist_param_2_.linear.x = 0.3;
  twist_param_2_.angular.z = 0.5;


}
//----------------------------------------------------------------------------------------------
void TimerCallback(const ros::TimerEvent &event)
{
  static campusrover_msgs::ElevatorControlStatus status;
  static campusrover_msgs::PlannerFunction planner_param;
  static campusrover_msgs::InitFloor init_param;
  static campusrover_msgs::PressButton button_param;
  static campusrover_msgs::ArmStandby arm_standby_msg;
  static std_msgs::Empty finish_msgs;

  status.control_status = control_status_;
  control_status_pub_.publish(status);
  // cout << "  control_status_first_time_ : " <<control_status_first_time_<< endl;
  // cout << "  path_generater_check_done_ : " <<path_generater_check_done_<< endl;
  // cout << "  elevator_pose_ckeck_done_ : " <<elevator_pose_ckeck_done_<< endl;
  // cout << "  -------------------------------------- " << endl;

  if(control_status_ == 1)// move to front of button (outside)
  {
  
    if(control_status_first_time_ && path_generater_check_done_ && elevator_pose_ckeck_done_)
    {

      //init floor 
      init_param.request.init_floor = init_floor_;
      InitFloorCallService(init_floor_srv_client_, init_param);

      planner_param.request.action.data = true;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_1_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      //Arm Standby pose 
      arm_standby_msg.request.status.data = true;
      ArmStandybyFunctionCallService(arm_standby_srv_client_, arm_standby_msg);
      control_status_first_time_ = false;
    }
  
    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      elevator_pose_ckeck_done_ = false;
      control_status_first_time_ = true;
      control_status_++;
    }


  }else if(control_status_ == 2) //press button (outside)
  {
    // 
    if(control_status_first_time_)
    {
      if(target_floor_ - init_floor_> 0)
      {
        button_param.request.button_type.data="(";
      }
      else
      {
        button_param.request.button_type.data=")";
      }

      PressButtonCallService(button_srv_client_,button_param);
      control_status_first_time_ = false;
    }
  
    if(arm_return_checker_)
    {
      
      control_status_first_time_ = true;
      arm_return_checker_ = false;
      control_status_++;

    }
    

  }else if(control_status_ == 3)// move to standby position
  {
    if(control_status_first_time_ && path_generater_check_done_)
    {
      planner_param.request.action.data = true;
      planner_param.request.direction_inverse.data = true;
      planner_param.request.speed_parameter = twist_param_1_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      
      //
      control_status_first_time_ = false;
    }
  
    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = true;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      control_status_first_time_ = true;

      if(arm_return_status_)
      {
        control_status_++;
      }
      else
      {
        control_status_ = control_status_- 2.0 ;
      }
      
      
    }

  }else if(control_status_ == 4)// waiting door open
  {
    if(door_status_ == "open")
    {
      control_status_++;
    }
    
  }else if(control_status_ == 5)//enter the elevator
  {
    if(door_status_ == "open")
    {
      if(control_status_first_time_ && path_generater_check_done_)
      {
        planner_param.request.action.data = true;
        planner_param.request.direction_inverse.data = false;
        planner_param.request.speed_parameter = twist_param_2_;
        PlannerFunctionCallService(planner_srv_client_, planner_param);
        control_status_first_time_ = false;
      }
    }
    else if(!control_status_first_time_ && door_status_ == "close")
    {
      planner_param.request.action.data = true;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      control_status_first_time_ = true;
    }
    
    
  
    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      control_status_first_time_ = true;
      control_status_ ++;
      
    }
    
  }else if(control_status_ == 6)//move to front of button (inside)
  {
    if(control_status_first_time_ && path_generater_check_done_ )
    {

      planner_param.request.action.data = true;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_1_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      //Arm Standby pose 
      arm_standby_msg.request.status.data = true;
      ArmStandybyFunctionCallService(arm_standby_srv_client_, arm_standby_msg);
      control_status_first_time_ = false;
    }

    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      control_status_first_time_ = true;
      control_status_++;
    }
    
  }else if(control_status_ == 7) //press button (inside)
  {
    if(control_status_first_time_)
    {
      //Arm Standby pose 
      arm_standby_msg.request.status.data = true;
      ArmStandybyFunctionCallService(arm_standby_srv_client_, arm_standby_msg);

      button_param.request.button_type.data = to_string(target_floor_);
      PressButtonCallService(button_srv_client_,button_param);
      control_status_first_time_ = false;
    }


    if(arm_return_checker_)
    {
      
      control_status_first_time_ = true;
      arm_return_checker_ = false;
      control_status_++;

    }
    
  }
  else if(control_status_ == 8)//move to standby position
  {
    if(control_status_first_time_ && path_generater_check_done_)
    {
      planner_param.request.action.data = true;
      planner_param.request.direction_inverse.data = true;
      planner_param.request.speed_parameter = twist_param_1_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      control_status_first_time_ = false;
    }
  
    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      control_status_first_time_ = true;

      if(arm_return_status_)
      {
        control_status_++;
      }
      else
      {
        control_status_ = control_status_- 2.0 ;
      }
    }
  }
  else if(control_status_ == 9)//waiting arrive target floor
  {
    if(arrive_target_floor_)
    {
      control_status_++;
    }
  }
  else if(control_status_ == 10)//waiting door open
  {
    if(door_status_ == "open")
    {
      control_status_++;
    }
  }
  else if(control_status_ == 11)//go out of the elevator
  {
    if(door_status_ == "open")
    {
      if(control_status_first_time_ && path_generater_check_done_)
      {
        planner_param.request.action.data = true;
        planner_param.request.direction_inverse.data = false;
        planner_param.request.speed_parameter = twist_param_1_;
        PlannerFunctionCallService(planner_srv_client_, planner_param);
        control_status_first_time_ = false;
      }
    }
    else if(door_status_ == "close")
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      control_status_first_time_ = true;
      control_status_ = 9;
    }

    if(planner_check_done_)
    {
      planner_param.request.action.data = false;
      planner_param.request.direction_inverse.data = false;
      planner_param.request.speed_parameter = twist_param_0_;
      PlannerFunctionCallService(planner_srv_client_, planner_param);
      planner_check_done_ = false;
      path_generater_check_done_ = false;
      control_status_first_time_ = true;
      control_status_ = 0;
      finish_pub_.publish(finish_msgs);
    }
  }
  

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

  if(target_floor_ ==  current_floor_)
  {
     arrive_target_floor_ = true;
  }
  else
  {
    arrive_target_floor_ = false;
  }
  
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
void PressButtonCallService(ros::ServiceClient &client,campusrover_msgs::PressButton &srv)
{
  string str = "===========press button============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("press button : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
void PlannerFunctionCallService(ros::ServiceClient &client,campusrover_msgs::PlannerFunction &srv)
{
  string str = "===========planner function============= " ;
  srv.request.mode = campusrover_msgs::PlannerFunction::Request::MODE_ELEVATOR_PATH;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("planner function : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
void ArmStandybyFunctionCallService(ros::ServiceClient &client,campusrover_msgs::ArmStandby &srv)
{
  string str = "===========arm_move_to_standby_pose function============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("arm_move_to_standby_pose function : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}

//-----------------------------------------------------------------------------------------------
bool ControlServiceCallback(campusrover_msgs::ElevatorControlInterface::Request  &req, campusrover_msgs::ElevatorControlInterface::Response &res)
{
  init_floor_ = req.init_floor;
  target_floor_ = req.target_floor;

  if(init_floor_ != target_floor_)
  {
    cout << "recrvie elevator control command : " << endl;
    cout << "  init floor : " <<init_floor_<< endl;
    cout << "  target floor : " <<target_floor_<< endl;
    control_status_ = 1;
    control_status_first_time_ = true;
  }
  else
  {
    ROS_WARN("recrvie elevator floor but init floor & target floor should not the same ");
    //cout << "recrvie elevator floor but init floor & target floor should not same : " << endl;
    cout << "  init floor : " <<init_floor_<< endl;
    cout << "  target floor : " <<target_floor_<< endl;
  }

  
  return true;

}
//-----------------------------------------------------------------------------------------------
bool StatusCheckServiceCallback(campusrover_msgs::ElevatorStatusChecker::Request  &req, campusrover_msgs::ElevatorStatusChecker::Response &res)
{
  cout << "  ==============" << endl;
  cout << "  node_name : " <<req.node_name.data<< endl; 
  cout << "  data : " <<req.status.data<< endl;
  if(req.node_name.data == "planner")
  {
    planner_check_done_ = req.status.data;
    return true;
  }
  else if(req.node_name.data == "arm")
  {
    arm_return_status_ = req.status.data;
    arm_return_checker_ = true;
    return true;
  }
  else if(req.node_name.data == "path_generater")
  {
    path_generater_check_done_ = req.status.data;
    return true;
  }

  else if(req.node_name.data == "position_finder")
  {
    elevator_pose_ckeck_done_ = req.status.data;
    return true;
  }
  else
  {
    return false;
  }
  

}
//--------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_floor_status");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  door_status_sub_ = nh.subscribe("doors_status", 10, DoorStatusCallback);
  floor_status_sub_ = nh.subscribe("floor_status", 10, FloorStatusCallback);

  control_status_pub_ = nh.advertise<campusrover_msgs::ElevatorControlStatus>("control_status", 50);
  finish_pub_ = nh.advertise<std_msgs::Empty>("elevator_completed", 50);

  init_floor_srv_client_ = nh.serviceClient<campusrover_msgs::InitFloor>("init_floor");
  button_srv_client_ = nh.serviceClient<campusrover_msgs::PressButton>("button_press");
  planner_srv_client_ = nh.serviceClient<campusrover_msgs::PlannerFunction>("planner_function");
  arm_standby_srv_client_ = nh.serviceClient<campusrover_msgs::ArmStandby>("arm_move_to_standby_pose");

  ros::ServiceServer control_service = nh.advertiseService("elevator_controller", ControlServiceCallback);
  ros::ServiceServer status_check_service = nh.advertiseService("elevator_status_checker", StatusCheckServiceCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), TimerCallback);


  ros::spin();

  return 0;
}