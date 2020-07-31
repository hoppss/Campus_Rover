#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h> 
#include <campusrover_msgs/Encode.h>

#include <string>
#include <vector>

#define M_PI 3.14159265358979323846  /* pi */

using namespace std;



ros::Subscriber encoder_data_sub_;
ros::Publisher odom_pub_;



bool start_flag_;
double delta_time_;
double accumulation_x_, accumulation_y_, accumulation_th_;
int cur_left_, cur_right_, rev_left_, rev_right_, delta_left_, delta_right_;

bool pub_tf_;
bool pub_odom_;
string source_frame_id_;
string target_frame_id_;
int encoder_resolution_;
double wheel_diameter_;
double wheel_base_;
double reduction_ratio_;

void cal_pulse(int &current, int &receive, int &delta);

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<bool>("pub_tf", pub_tf_, true);
  n_private.param<bool>("pub_odom", pub_odom_, true);
  n_private.param<string>("source_frame_id", source_frame_id_, "odom");
  n_private.param<string>("target_frame_id", target_frame_id_, "base_link");
  n_private.param<int>("encoder_resolution", encoder_resolution_, 1600);
  n_private.param<double>("wheel_diameter", wheel_diameter_, 0.25);
  n_private.param<double>("wheel_base", wheel_base_, 0.78);
  n_private.param<double>("reduction_ratio", reduction_ratio_, 1.0);
  cur_left_ = 0;
  cur_right_ = 0;
  start_flag_ = true;
}

void EncoderDataCallback(campusrover_msgs::Encode encode_data)
{
  static tf2_ros::TransformBroadcaster br_;
  static geometry_msgs::TransformStamped transformStamped_;
  static nav_msgs::Odometry odom_;
  static ros::Time last_time_, now_; 

  rev_left_ = encode_data.left;
  rev_right_ = encode_data.right;

  cal_pulse(cur_left_, rev_left_, delta_left_);
  cal_pulse(cur_right_, rev_right_, delta_right_);

  now_ = ros::Time::now();

  if (start_flag_){
    accumulation_x_ = accumulation_y_ = accumulation_th_ = 0.0;
    last_time_ = now_;
    start_flag_ = false;
    return;
  }

  delta_time_ = (now_ - last_time_).toSec();

  double pulse_per_cycle = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_);

  double delta_theta = (delta_right_ - delta_left_)/ (pulse_per_cycle * wheel_base_);
  double v_theta = delta_theta / delta_time_;

  double delta_dis = (delta_right_ + delta_left_) / (pulse_per_cycle * 2.0);
  double v_dis = delta_dis / delta_time_;

  double delta_x, delta_y;
  if (delta_theta == 0){
    delta_x = delta_dis;
    delta_y = 0.0;
  }else{
    delta_x = delta_dis * (sin(delta_theta) / delta_theta);
    delta_y = delta_dis * ( (1 - cos(delta_theta)) / delta_theta );
  }

  accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
  accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
  accumulation_th_ += delta_theta;

  tf2::Quaternion q;
  if(pub_tf_)
  {
    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = source_frame_id_;
    transformStamped_.child_frame_id = target_frame_id_;
    transformStamped_.transform.translation.x = accumulation_x_;
    transformStamped_.transform.translation.y = accumulation_y_;
    transformStamped_.transform.translation.z = 0.0;
    
    q.setRPY(0, 0, accumulation_th_);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();

    br_.sendTransform(transformStamped_);
  }

  
  if(pub_odom_)
  {
    odom_.header.stamp = ros::Time::now();
    odom_.header.frame_id = source_frame_id_;
    odom_.child_frame_id = target_frame_id_;
    odom_.pose.pose.position.x = accumulation_x_;
    odom_.pose.pose.position.y = accumulation_y_;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation.x = q.getX();
    odom_.pose.pose.orientation.y = q.getY();
    odom_.pose.pose.orientation.z = q.getZ();
    odom_.pose.pose.orientation.w = q.getW();
    odom_.twist.twist.linear.x = v_dis;
    odom_.twist.twist.linear.y = 0;
    odom_.twist.twist.angular.z = v_theta;

    odom_pub_.publish(odom_);
  }

  last_time_ = now_;
}

void cal_pulse(int &current, int &receive, int &delta){
  if (receive > current){
    delta = (receive - current) < (current - receive + 65535) ? (receive - current) : (receive - current - 65535);
  }else{
    delta = (current - receive) < (receive - current + 65535) ? (receive - current) : (receive - current + 65535);
  }
  current = receive;
}


//-----------------------------------------------------------------------------------------------//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "campusrover_odom");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);

  encoder_data_sub_ = nh.subscribe("encoder_data", 10, EncoderDataCallback);

  ros::spin();

  return 0;
}