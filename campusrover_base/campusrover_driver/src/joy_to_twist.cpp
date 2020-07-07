#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

ros::Subscriber joy_sub_;
ros::Publisher twist_pub_;

double scale_linear_, scale_angular_;
int axis_linear_, axis_angular_;

void GetParameters(ros::NodeHandle n_private)
{
  n_private.param<double>("scale_linear", scale_linear_, 1.0);
  n_private.param<double>("scale_angular", scale_angular_, 1.0);
  n_private.param<int>("axis_linear", axis_linear_, 0.0);
  n_private.param<int>("axis_angular", axis_angular_, 1.0);
}
//-----------------------------------------------------------------------------------------------//
void JoyCallback(sensor_msgs::Joy joy)
{
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = joy.axes[axis_linear_];
  cmd_vel_msg.linear.y = 0.0;
  cmd_vel_msg.linear.z = 0.0;
  cmd_vel_msg.angular.z = joy.axes[axis_angular_];
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.x = 0.0;

  twist_pub_.publish(cmd_vel_msg);

}
//-----------------------------------------------------------------------------------------------//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_to_twist");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GetParameters(nh_private);
  ros::Time::init();

  joy_sub_ = nh.subscribe("joy", 10, JoyCallback);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 0.1);

  ros::spin();

  return 0;
}