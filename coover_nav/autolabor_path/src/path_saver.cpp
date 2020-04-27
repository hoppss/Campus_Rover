#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <autolabor_path/SavePath.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Subscriber path_sub_, clicked_point_sub_, goal_point_sub_;
ros::Publisher vis_pub_, path_pub_, path_marker_pub_;
nav_msgs::Path sub_path_;
std::vector<nav_msgs::Path> save_paths;
string file_path_;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("file_path", file_path_, " ");
}

int getMiniIndex(geometry_msgs::PointStamped point, nav_msgs::Path &path)
{
  double min_dis = 100;
  double dis;
  int index = -1;
  for(int i = 0; i < path.poses.size(); i++)
  {
    dis = pow(path.poses[i].pose.position.x - point.point.x, 2)
          + pow(path.poses[i].pose.position.y - point.point.y, 2);
    if(min_dis > dis)
    {
      min_dis = dis;
      index = i;
    }
  }
  return index;
}

int getMiniIndex(geometry_msgs::PoseStamped pose, nav_msgs::Path &path)
{
  double min_dis = 99999;
  double dis;
  int index = -1;
  for(int i = 0; i < path.poses.size(); i++)
  {
    dis = pow(path.poses[i].pose.position.x - pose.pose.position.x, 2)
          + pow(path.poses[i].pose.position.y - pose.pose.position.y, 2);
    if(min_dis > dis)
    {
      min_dis = dis;
      index = i;
    }
  }
  return index;
}

void putMarkers(bool push_back, nav_msgs::Path &path, int indexA, int indexB)
{
  static visualization_msgs::MarkerArray marker_array;
  static visualization_msgs::Marker marker;
  marker.header.frame_id = path.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.ns = "path_points";
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id++;
  if(!push_back)
  {
    marker.pose = path.poses[indexA].pose;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  else
  {
    marker.pose = path.poses[indexB].pose;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  marker_array.markers.push_back(marker);
  vis_pub_.publish(marker_array);
  // vis_pub.publish( marker );
}

void PathCallback(const nav_msgs::Path::ConstPtr& path)
{
  sub_path_ = *path;
}

void pushBackNewPath(int A, int B, nav_msgs::Path &path_in, std::vector<nav_msgs::Path> &paths)
{
  static nav_msgs::Path myPath;
  myPath.header.seq++;
  myPath.header.stamp = ros::Time::now();
  myPath.header.frame_id = path_in.header.frame_id;
  myPath.poses.clear();
  for(int i = A; i <= B; i++)
  {
    myPath.poses.push_back(path_in.poses[i]);
  }
  paths.push_back(myPath);
}

void ClickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& point)
{
  static bool push_back = false;
  static int nearest_point_index;
  static int indexA, indexB;
  nearest_point_index = getMiniIndex(*point, sub_path_);
  if(nearest_point_index != -1)
  {
    if(!push_back)
    {
      indexA = nearest_point_index;
    }
    else
    {
      indexB = nearest_point_index;
      pushBackNewPath(indexA, indexB, sub_path_, save_paths);
      path_pub_.publish(save_paths[save_paths.size()-1]);
    }
    putMarkers(push_back, sub_path_, indexA, indexB);
    push_back = !push_back;
  }
}

void goalPointCallback(const geometry_msgs::PoseStamped::ConstPtr& PoseStamped)
{
  static bool push_back = false;
  static int nearest_point_index;
  static int indexA, indexB;
  nearest_point_index = getMiniIndex(*PoseStamped, sub_path_);
  if(nearest_point_index != -1)
  {
    if(!push_back)
    {
      indexA = nearest_point_index;
    }
    else
    {
      indexB = nearest_point_index;
      pushBackNewPath(indexA, indexB, sub_path_, save_paths);
      path_pub_.publish(save_paths[save_paths.size()-1]);
    }
    putMarkers(push_back, sub_path_, indexA, indexB);
    push_back = !push_back;
  }
}

bool SavePath(autolabor_path::SavePath::Request  &req,
              autolabor_path::SavePath::Response &res)
{
  ofstream myfile;
  res.path_number = save_paths.size();
  string file = file_path_ + req.file_name + ".path";
  myfile.open (file);
  myfile << save_paths.size() << endl;
  for(int i = 0; i < save_paths.size(); i++)
  {
    myfile << save_paths[i].poses.size() << endl;
    for(int j = 0; j < save_paths[i].poses.size(); j++)
    {
      myfile << save_paths[i].poses[j].pose.position.x << '\t';
      myfile << save_paths[i].poses[j].pose.position.y << '\t';
      myfile << save_paths[i].poses[j].pose.position.z << '\t';
      myfile << save_paths[i].poses[j].pose.orientation.x << '\t';
      myfile << save_paths[i].poses[j].pose.orientation.y << '\t';
      myfile << save_paths[i].poses[j].pose.orientation.z << '\t';
      myfile << save_paths[i].poses[j].pose.orientation.w << '\t' << endl;
    }
  }
  myfile.close();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_saver");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  get_parameters(n_private);
  path_sub_ = n.subscribe("path_in", 100, PathCallback);
  clicked_point_sub_ = n.subscribe("clicked_point", 10, ClickedPointCallback);
  goal_point_sub_ = n.subscribe("/move_base_simple/goal", 10, goalPointCallback);
  vis_pub_ = n.advertise<visualization_msgs::MarkerArray>( "path_saver_marker", 10);
  // path_marker_pub_ = n.advertise<visualization_msgs::Marker>( "save_path_marker", 10);
  path_pub_ = n.advertise<nav_msgs::Path>( "save_path", 10);
  ros::ServiceServer service = n.advertiseService("save_path", SavePath);
  ros::spin();
}
