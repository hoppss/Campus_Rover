#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;

double pose_x,pose_y,pose_z,orientation_x,orientation_y,orientation_z,orientation_w;
string frame_id_,yaml_file_,folder_path_,load_file_,map_name_,subscribe_topic_,pose_;

void getParameters(ros::NodeHandle n_private)
{
  n_private.param<string>("map_frame", frame_id_, "map");
  n_private.param<string>("folder_path", folder_path_, ".yaml");
  n_private.param<string>("subscribe_topic", subscribe_topic_, "initialpose");
}

 void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("Loading [%s] position", msg->data.c_str());
    getfile(folder_path_,msg->data.c_str());
  }

void getfile (string folder_path_,string map_name_)
{     
  load_file_= folder_path_ + map_name_;
  load_file_ +=".yaml";
  YAML::Node pose = YAML::LoadFile(folder_path_);
  try
  {
    pose["pose"]["position"]["x"] = pose_x;
    pose["pose"]["position"]["y"] = pose_y;
    pose["pose"]["position"]["z"] = pose_z;
    pose["pose"]["orientation"]["x"] = orientation_x;
    pose["pose"]["orientation"]["y"] = orientation_y;
    pose["pose"]["orientation"]["z"] = orientation_z;
    pose["pose"]["orientation"]["w"] = orientation_w;
  }
  catch(YAML::InvalidScalar)
  {
     ROS_ERROR("YAML load paramerters failed");
    exit(-1);
  }
  ROS_INFO("[%s] position loaded", load_file_.c_str());
}

void pose_publish(ros::Publisher &pub)
{
    geometry_msgs::PoseWithCovarianceStamped pose_publish;
    pose_publish.pose.pose.position.x = pose_x;
    pose_publish.pose.pose.position.y = pose_y;
    pose_publish.pose.pose.position.z = pose_z;
    pose_publish.pose.pose.orientation.x = orientation_x;
    pose_publish.pose.pose.orientation.y = orientation_y;
    pose_publish.pose.pose.orientation.z = orientation_z;
    pose_publish.pose.pose.orientation.w = orientation_w;
    ROS_INFO("get pose from [%s] ", load_file_);
    pub.publish(pose_publish);
    ROS_INFO("Publishing pose to initial pose", load_file_);

}
int main (int argc,char **argv)
{
    ros::init(argc, argv, "point_publisher");
    
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");   
    getParameters(n_private);

    ros::Subscriber map_name_= n.subscribe(subscribe_topic_, 1000,chatterCallback);
    
    ros::Publisher pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped >("initialpose", 1000, true);
    pose_publish(pose_pub_);
}