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

#include <autolabor_msgs/DoorStatus.h>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber scan_sub_, pose_sub_;
ros::Publisher door_status_pub_, elevator_marker_pub_;

std::vector<double> scan_range_;
std::vector<double> scan_angle_;
string scan_frame_;
bool get_scan_data = false;

int elevator_num_ = 0;
double door_t1_;
double door_gap_dis_;
double door_x1_;
double door_y1_;
double door_y2_;
double door_open_threshold_;

void VisualizeMarker(std::vector<geometry_msgs::Point> points);
void VisualizeElevatorEnterPoint(autolabor_msgs::DoorStatus msgs) ;
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<double>("door_t1", door_t1_, 0.2);
  n_private.param<double>("door_gap_dis", door_gap_dis_, 0.91);
  n_private.param<double>("door_x1", door_x1_, 0.34);
  n_private.param<double>("door_y1", door_y1_, 0.94);
  n_private.param<double>("door_y2", door_y2_, 0.8);
  n_private.param<double>("door_open_threshold", door_open_threshold_, 0.0);

}

void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  static double angle;
  if(elevator_num_ == 0)
  {
    get_scan_data = false;
    return;
  }
  scan_frame_ = scan->header.frame_id;
  
  scan_range_.clear();
  scan_angle_.clear();

  for (int i = 0; i < scan->ranges.size(); i++)
  {
    if(scan->ranges[i] > 0.001)
    {
      angle = scan->angle_min + (i * scan->angle_increment);
      scan_range_.push_back(scan->ranges[i]);
      scan_angle_.push_back(angle);
      //cout<<"scan_poses x = "<<pose.position.x<<" scan_poses y = "<<pose.position.y<<'\n';
      //cout<<"angle = "<<angle<<'\n';
      
    }
  }

  get_scan_data = true;
  
  
}

void PoseCallback(const geometry_msgs::PoseArrayConstPtr &poses)
{
  static geometry_msgs::Pose pose;
  static std_msgs::String door_status;
  static autolabor_msgs::DoorStatus doors_status_msg;
  static geometry_msgs::Point point_1;
  static geometry_msgs::Point point_2;
  static geometry_msgs::PoseStamped before_pose ;
  static geometry_msgs::PoseStamped after_pose ;
  static std::vector<geometry_msgs::Point> points;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static double angle_1, angle_2, angle_3;
  static double roll, pitch, yaw, yaw_v;
  static double d1, d2, v;

  doors_status_msg.doors_status.clear();
  doors_status_msg.doors_pose.poses.clear();

  elevator_num_ = poses->poses.size();

  if(elevator_num_ == 0)
    return;

  if(get_scan_data)
  {
    
    points.clear();
    for(int i = 0;i <elevator_num_;i++)
    {

      if(scan_frame_ != poses->header.frame_id)
      {
        before_pose.header = poses->header;
        before_pose.pose.position = poses->poses[i].position;
        before_pose.pose.orientation = poses->poses[i].orientation;
        try
        {
          tfBuffer.transform(before_pose, after_pose, scan_frame_);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("elevator_path_generater : %s",ex.what());
          ros::Duration(0.5).sleep();
          return;
        }
        //cout<<"scan_frame_ "<<scan_frame_<<" pose_frame "<<poses->header.frame_id<<"after_pose frame "<<after_pose.header.frame_id<<'\n';
      }
      else
      {
        after_pose.header = poses->header;
        after_pose.pose.position = poses->poses[i].position;
        after_pose.pose.orientation = poses->poses[i].orientation;
      }
      

      pose.position = after_pose.pose.position;
      pose.orientation = after_pose.pose.orientation;
      doors_status_msg.doors_pose.poses.push_back(pose);

      tf::Quaternion q( after_pose.pose.orientation.x,
                        after_pose.pose.orientation.y,
                        after_pose.pose.orientation.z,
                        after_pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      
      m.getRPY(roll, pitch, yaw_v);

      yaw = yaw_v - M_PI/2.0;

      point_1.x = after_pose.pose.position.x + (door_y1_/2.0)*cos(yaw);
      point_1.y = after_pose.pose.position.y + (door_y1_/2.0)*sin(yaw);

      point_2.x = after_pose.pose.position.x - (door_y1_/2.0)*cos(yaw);
      point_2.y = after_pose.pose.position.y - (door_y1_/2.0)*sin(yaw);

      // points.push_back(point_1);
      // points.push_back(point_2);
      d1 = sqrt(pow(point_1.x,2)+pow(point_1.y,2));
      d2 = sqrt(pow(point_2.x,2)+pow(point_2.y,2));
      angle_1 = atan2(point_1.y, point_1.x);
      angle_2 = atan2(point_2.y, point_2.x);

      v = abs(sqrt(pow(after_pose.pose.position.x,2)+pow(after_pose.pose.position.y,2)) * cos(yaw_v));

      //cout<<"angle_1 "<<angle_1<<" angle_2 "<<angle_2<<" v "<<v<<'\n';
      int count = 0;
      double open_rate_sgn = 0;
      double open_rate = 0;
      for(int s = 0;s <scan_range_.size();s++)
      {
        if(scan_angle_[s] > angle_1 && scan_angle_[s] < angle_2)
        {
          double l = abs(scan_range_[s] * cos(yaw_v));
          if(l - v - door_x1_ - door_t1_ < 0)
          {
            //open_rate_sgn--;
          }
          else if(l - v - door_x1_ - door_t1_ > 0)
          {
            open_rate_sgn++;
          }
          
          count++;
          //cout<<"l "<<l<<" scan_range "<<scan_range_[s]<<'\n';
        }
        //cout<<" scan_range "<<scan_range_[s]<<'\n';
      }
      open_rate = open_rate_sgn / count;
      //cout<<"open_rate "<<open_rate<<'\n';

      if(open_rate > door_open_threshold_)
      {
        door_status.data = "open";
      }else
      {
        door_status.data = "close";
      }
      doors_status_msg.doors_status.push_back(door_status);
      
    }
    //VisualizeMarker(points);
    doors_status_msg.doors_pose.header.frame_id = scan_frame_;
    door_status_pub_.publish(doors_status_msg);
    VisualizeElevatorEnterPoint(doors_status_msg);
  }
  


}
//-------------------------------------------------------------
void VisualizeMarker(std::vector<geometry_msgs::Point> points) 
{
  static visualization_msgs::MarkerArray point_markers;
  static visualization_msgs::Marker point_marker;

  point_markers.markers.clear();

  if(points.size() == 0)
    return;

  point_marker.header.frame_id = scan_frame_;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "corner_position_Orignal";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.position.z = 0;
  point_marker.pose.orientation.x = 0.0;
  point_marker.pose.orientation.y = 0.0;
  point_marker.pose.orientation.z = 0.0;
  point_marker.pose.orientation.w = 1.0;
  point_marker.scale.y = 0.15;
  point_marker.scale.x = 0.15;
  point_marker.scale.z = 0.01;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.color.a = 0.5f;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 0.0f;

  for(int i=0; i < points.size(); i++)
  {
    point_marker.id = i;
    point_marker.pose.position.x = points[i].x;
    point_marker.pose.position.y = points[i].y;
    point_markers.markers.push_back(point_marker);
  }
  

  elevator_marker_pub_.publish(point_markers);
}
//--------------------------------------------------------------------

void VisualizeElevatorEnterPoint(autolabor_msgs::DoorStatus msgs) 
{
  static visualization_msgs::MarkerArray all_rollOuts;
  static visualization_msgs::Marker lane_waypoint_marker;
  static geometry_msgs::Point point;
  lane_waypoint_marker.header.frame_id = scan_frame_;
  lane_waypoint_marker.header.stamp = ros::Time(0);
  lane_waypoint_marker.ns = "elevator_door_status_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.05;
  lane_waypoint_marker.scale.y = 0.05;
  lane_waypoint_marker.frame_locked = false;
  static int max_count = 0;
  all_rollOuts.markers.clear();
  if(msgs.doors_pose.poses.size() > max_count)
  {
    max_count = msgs.doors_pose.poses.size();
  }
  
  for (int i = 0; i < msgs.doors_pose.poses.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    if(msgs.doors_status[i].data == "open")
    {
      lane_waypoint_marker.id = i;

      lane_waypoint_marker.color.r = 0.0;
      lane_waypoint_marker.color.g = 1.0;
      lane_waypoint_marker.color.b = 0.0;
      lane_waypoint_marker.color.a = 0.9;
    }
    else if(msgs.doors_status[i].data == "close")
    {
      lane_waypoint_marker.id = i;

      lane_waypoint_marker.color.r = 1.0;
      lane_waypoint_marker.color.g = 0.0;
      lane_waypoint_marker.color.b = 0.0;
      lane_waypoint_marker.color.a = 0.9;
    }
    // else if(i ==2)
    // {
    //   lane_waypoint_marker.id = i;

    //   lane_waypoint_marker.color.r = 0.0;
    //   lane_waypoint_marker.color.g = 0.0;
    //   lane_waypoint_marker.color.b = 1.0;
    //   lane_waypoint_marker.color.a = 0.9;
    // }



    tf::Quaternion q( msgs.doors_pose.poses[i].orientation.x,
                      msgs.doors_pose.poses[i].orientation.y,
                      msgs.doors_pose.poses[i].orientation.z,
                      msgs.doors_pose.poses[i].orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_v;
    m.getRPY(roll, pitch, yaw_v);

    yaw = yaw_v - M_PI/2.0;
    
    point.x = msgs.doors_pose.poses[i].position.x + (door_y1_/2.0)*cos(yaw);
    point.y = msgs.doors_pose.poses[i].position.y + (door_y1_/2.0)*sin(yaw);

    lane_waypoint_marker.points.push_back(point);

    point.x = msgs.doors_pose.poses[i].position.x - (door_y1_/2.0)*cos(yaw);
    point.y = msgs.doors_pose.poses[i].position.y - (door_y1_/2.0)*sin(yaw);
      
    lane_waypoint_marker.points.push_back(point);

  
    all_rollOuts.markers.push_back(lane_waypoint_marker);
  }
  if(msgs.doors_pose.poses.size()<max_count)
  {
    for(int f = 0;f <max_count-msgs.doors_pose.poses.size();f++)
    {
      lane_waypoint_marker.id = all_rollOuts.markers.size()+f;
      lane_waypoint_marker.color.a = 0.01;
      //lane_waypoint_marker.points.push_back(point);
      all_rollOuts.markers.push_back(lane_waypoint_marker);
    }
  }

  elevator_marker_pub_.publish(all_rollOuts);

}
//--------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_door_status");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  scan_sub_ = nh.subscribe("scan", 10, ScanCallback);
  pose_sub_ = nh.subscribe("elevator_poses", 10, PoseCallback);
  door_status_pub_ = nh.advertise<autolabor_msgs::DoorStatus> ("doors_status", 10);
  elevator_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("doors_status_marker", 50);

  ros::spin();

  return 0;
}