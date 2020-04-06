#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

using namespace std;

std::vector<nav_msgs::Path> paths;
string file_path_, map_frame_id_;
ros::Publisher path_pub_;
ros::Timer timer_;

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("file_path", file_path_, "/home/justin/");
  n_private.param<string>("map_frame_id", map_frame_id_, "map");
}

void loadPath()
{
  string line;
  ifstream myfile(file_path_);
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  cout << "Loading path ..." << endl;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  pose.header = path.header;
  if (myfile.is_open())
  {
    getline(myfile,line);
    int path_number = std::stoi(line);
    for(int i = 0; i < path_number; i++)
    {
      path.poses.clear();
      getline(myfile,line);
      int points_number = std::stoi(line);
      for(int j = 0; j < points_number; j++)
      {
        getline(myfile,line);
        istringstream iss(line);
        vector<string> pos{istream_iterator<string>{iss},
                       istream_iterator<string>{}};
        pose.pose.position.x = stof(pos[0]);
        pose.pose.position.y = stof(pos[1]);
        pose.pose.position.z = stof(pos[2]);
        pose.pose.orientation.x = stof(pos[3]);
        pose.pose.orientation.y = stof(pos[4]);
        pose.pose.orientation.z = stof(pos[5]);
        pose.pose.orientation.w = stof(pos[6]);
        path.poses.push_back(pose);
      }
      paths.push_back(path);
    }
    myfile.close();
  }
  else
  {
    cout << "Unable to open file";
  }
  cout << "Publishing " <<   paths.size() << " path(s)" << endl;
}

void Timercallback(const ros::TimerEvent& event)
{
  if(paths.size() != 0)
  {
    path_pub_.publish(paths[0]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_loader");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  get_parameters(n_private);
  loadPath();
  path_pub_ = n.advertise<nav_msgs::Path>( "path_loader", 10);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), Timercallback);
  ros::spin();
  return 0;
}
