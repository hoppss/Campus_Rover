#include <autolabor_demo/demo_gps.hpp>

void getParameters(ros::NodeHandle n_private)
{
  string path_name, get_path_name;
  n_private.param<string>("map_frame", map_frame_, "map");
  n_private.param<double>("pending_dis", pending_dis_, 2);
  n_private.param<string>("path_file", file_path_, " " );
  n_private.param<bool>("pub_autolabor_gps_position", pub_autolabor_gps_position_, false );
  n_private.param<bool>("use_osrm", use_osrm_, false);
}


void gpsGoalCallcack(const sensor_msgs::NavSatFixConstPtr &gps)
{
  double UTM_North, UTM_East;
  std::string UTM_zone;
  static geometry_msgs::PoseStamped global_point;
  static geometry_msgs::PoseStamped local_point;
  static geometry_msgs::PoseStamped base_point;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  nav_msgs::Path path_msg;
  // if(mode_ == functions && function_ == pending)
  if(mode_ == functions && ~use_osrm_)
  {
    gps_common::LLtoUTM(gps->latitude, gps->longitude, UTM_North, UTM_East, UTM_zone);
    // Transform Point
    global_point.header.frame_id = "world";
    global_point.header.seq++;
    global_point.header.stamp = ros::Time::now();
    global_point.pose.position.x = UTM_East;
    global_point.pose.position.y = UTM_North;
    global_point.pose.position.z = 0;
    global_point.pose.orientation.x = 0;
    global_point.pose.orientation.y = 0;
    global_point.pose.orientation.z = 0;
    global_point.pose.orientation.w = 1;
    bool transformed = false;
    // get destination transform
    while(!transformed)
    {
      try
      {
        tfBuffer.transform(global_point, local_point, map_frame_);
        transformed = true;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        continue;
      }
    }
    global_point.header.frame_id = "base_link";
    global_point.header.seq++;
    global_point.header.stamp = ros::Time::now();
    global_point.pose.position.x = 0;
    global_point.pose.position.y = 0;
    global_point.pose.position.z = 0;
    transformed = false;
    // get destination transform
    while(!transformed)
    {
      try
      {
        tfBuffer.transform(global_point, base_point, map_frame_);
        transformed = true;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        continue;
      }
    }
    // get path from O/D
    path_end_nearest_index_ = getNearestPointIndex(slam_paths_[0], local_point);
    int current_path_nearest_index_ = getNearestPointIndex(slam_paths_[0], base_point);
    path_msg.header.frame_id = map_frame_;
    path_msg.header.seq++;
    path_msg.header.stamp = ros::Time::now();
    for (int i = current_path_nearest_index_; i <= path_end_nearest_index_; i++)
    {
      path_msg.poses.push_back(slam_paths_[0].poses[i]);
    }
    function_ = delivering;
    std::cout << "Start Delivery" << std::endl;
    path_pub_.publish(path_msg);
  }
}

void updatePosition(const ros::TimerEvent&)
{
  static geometry_msgs::TransformStamped transformStamped;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped autolabor_transform;
  static sensor_msgs::NavSatFix autolabor_gps_msg;
  if(pub_autolabor_gps_position_)
  {
    try{
      autolabor_transform = tfBuffer.lookupTransform( "world",
                                                   "base_link",
                                                   ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    autolabor_gps_msg.header.seq++;
    autolabor_gps_msg.header.stamp = ros::Time::now();
    autolabor_gps_msg.header.frame_id = "world";
    ros::NodeHandle n;
    n.getParam("UTM_zone", UTM_zone_);
    UTM_zone_ = "51R";
    gps_common::UTMtoLL(autolabor_transform.transform.translation.y,
                        autolabor_transform.transform.translation.x,
                        UTM_zone_,
                        autolabor_gps_msg.latitude,
                        autolabor_gps_msg.longitude);
    autolabor_position_pub_.publish(autolabor_gps_msg);
    // std::cout << autolabor_gps_msg << std::endl;
  }
  if(mode_ == functions)
  {
    try{
      transformStamped = tfBuffer.lookupTransform( map_frame_,
                                                  "base_link",
                                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    if(function_ == delivering)
    {
      double r = pow(slam_paths_[0].poses[path_end_nearest_index_].pose.position.x - transformStamped.transform.translation.x, 2)
                +pow(slam_paths_[0].poses[path_end_nearest_index_].pose.position.y - transformStamped.transform.translation.y, 2);
      // std::cout << r << std::endl;
      if(r < pending_dis_ * pending_dis_)
      {
        function_ = pending;
      }
    }
  }
}


void loadPath(std::string file_path)
{
  string line;
  ifstream myfile(file_path);
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  cout << "Loading path ..." << endl;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = map_frame_;
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
      slam_paths_.push_back(path);
    }
    myfile.close();
  }
  else
  {
    cout << "Unable to open file";
  }
  cout << "Loaded " <<   slam_paths_.size() << " path(s)" << endl;
}

int getNearestPointIndex(nav_msgs::Path &path, geometry_msgs::PoseStamped &point)
{
  double r;
  double mini_r = 10000; // default 100m
  int index = 0;
  for(int i = 0; i < path.poses.size(); i++)
  {
    r = pow(path.poses[i].pose.position.x - point.pose.position.x, 2)
       +pow(path.poses[i].pose.position.y - point.pose.position.y, 2);
    if(r < mini_r)
    {
      mini_r = r;
      index = i;
    }
  }
  return index;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autolabor_demo_gps");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  getParameters(nh_private);
  loadPath(file_path_);
  path_pub_ = n.advertise<nav_msgs::Path>( "autolabor_route_path", 10, true);
  if (pub_autolabor_gps_position_)
  {
    autolabor_position_pub_ = n.advertise<sensor_msgs::NavSatFix>( "autolabor_gps_location", 10);
  }
  gps_sub_ = n.subscribe("autolabor_gps_destination", 10, gpsGoalCallcack);
  ros::Timer timer = n.createTimer(ros::Duration(0.05), updatePosition);
  ros::spin();
}
