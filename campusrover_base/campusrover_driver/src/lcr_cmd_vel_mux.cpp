#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

ros::Subscriber joy_sub_, joy_cmd_vel_sub_, nav_cmd_vel_sub_;
ros::Publisher cmd_vel_pub_;

int joy_mode_button_, nav_mode_button_, stop_mode_button_, release_mode_button_;
int mode_; //{release = 0, stop = 1, manual = 2, auto_mode = 3 };

//----------------------------------------------------------------------------------------------
void GetParameters(ros::NodeHandle n_private)
{
  n_private.param<int>("joy_mode_button", joy_mode_button_, 7.0);
  n_private.param<int>("nav_mode_button", nav_mode_button_, 0.0);
  n_private.param<int>("stop_mode_button", stop_mode_button_, 1.0);
  n_private.param<int>("release_mode_button", release_mode_button_, 3.0);
  
}
//----------------------------------------------------------------------------------------------
void TimerCallback(const ros::TimerEvent &event)
{
  static geometry_msgs::Twist twist_msg;

  if(mode_ == 1)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    cmd_vel_pub_.publish(twist_msg);

  }

}
//-----------------------------------------------------------------------------------------------//
void JoyCallback(sensor_msgs::Joy joy)
{
  if(joy.buttons[stop_mode_button_] == 1.0 )
  {
    mode_ = 1.0;
  }
  else if(joy.buttons[release_mode_button_] == 1.0)
  {
    mode_ = 0.0;
  }
  else if(joy.buttons[joy_mode_button_] == 1.0)
  {
    mode_ = 2.0;
  }
  else if(joy.buttons[nav_mode_button_] == 1.0)
  {
    mode_ = 3.0;
  }

}
//-----------------------------------------------------------------------------------------------//
void JoyTwistCallback(geometry_msgs::Twist joy_twist)
{
  if(mode_ != 2.0)
    return;

  cmd_vel_pub_.publish(joy_twist);



}
//-----------------------------------------------------------------------------------------------//
void NavTwistCallback(geometry_msgs::Twist nav_twist)
{
  if(mode_ != 3.0)
    return;

  cmd_vel_pub_.publish(nav_twist);

}
//-----------------------------------------------------------------------------------------------//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lcr_cmd_vel_mux");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GetParameters(nh_private);
  ros::Time::init();

  joy_sub_ = nh.subscribe("joy", 10, JoyCallback);
  joy_cmd_vel_sub_ = nh.subscribe("input/joy_cmd_vel", 10, JoyTwistCallback);
  nav_cmd_vel_sub_ = nh.subscribe("input/nav_cmd_vel", 10, NavTwistCallback);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("output/cmd_vel", 0.1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), TimerCallback);

  ros::spin();

  return 0;
}