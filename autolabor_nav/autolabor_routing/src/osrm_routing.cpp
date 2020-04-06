#include <autolabor_routing/osrm_routing.hpp>

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<std::string>("path_frame_id", path_frame_id_, "world");
  n_private.param<std::string>("osrm_file", osrm_file_, " ");
  n_private.param<double>("route_resolution", route_resolution_, 0.3);
}

void upSampling(bool new_route, nav_msgs::Path &path,
               double new_poiunt_x, double new_poiunt_y, double resolution_dxy)
{
  static geometry_msgs::PoseStamped last_pose;
  double nubmer_of_inster_points ;
  double dx, dy;
  double x = 0;
  double y = 0;
  if(new_route)
  {
    path.poses.clear();
    path.header.seq = 0;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_frame_id_;
    last_pose.header.seq = 0;
    last_pose.header.stamp = ros::Time::now();
    last_pose.header.frame_id = path_frame_id_;
    last_pose.pose.position.x = new_poiunt_x;
    last_pose.pose.position.y = new_poiunt_y;
    last_pose.pose.position.z = 0;
    last_pose.pose.orientation.x = 0;
    last_pose.pose.orientation.y = 0;
    last_pose.pose.orientation.z = 0;
    last_pose.pose.orientation.w = 1;
    path.poses.push_back(last_pose);
  }
  else
  {
    int nubmer_of_inster_points = sqrt(pow((new_poiunt_x - last_pose.pose.position.x), 2.0) +
                                       pow((new_poiunt_y - last_pose.pose.position.y), 2.0)) / resolution_dxy;
    dx = (new_poiunt_x - last_pose.pose.position.x) / (double) nubmer_of_inster_points;
    dy = (new_poiunt_y - last_pose.pose.position.y) / (double) nubmer_of_inster_points;
    for(int i=1; i <= nubmer_of_inster_points; i++ )
    {
      last_pose.pose.position.x += dx;
      last_pose.pose.position.y += dy;
      last_pose.pose.position.z = 0;
      last_pose.pose.orientation.x = 0;
      last_pose.pose.orientation.y = 0;
      last_pose.pose.orientation.z = 0;
      last_pose.pose.orientation.w = 1;
      path.poses.push_back(last_pose);
    }
  }
  //std::cout << new_route << std::endl;
}

void pushPointToPath(bool new_route,
                    nav_msgs::Path &path,
                    double new_poiunt_x,
                    double new_poiunt_y)
{
  static bool first_time = true;
  static geometry_msgs::PoseStamped pose;
  if(new_route)
  {
    path.poses.clear();
    path.header.seq = 0;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_frame_id_;
    pose.header.seq = 0;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = path_frame_id_;
    pose.pose.position.x = new_poiunt_x;
    pose.pose.position.y = new_poiunt_y;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
  }
  else
  {
    pose.pose.position.x = new_poiunt_x;
    pose.pose.position.y = new_poiunt_y;
  }
  path.poses.push_back(pose);
}

void publishResaultPath(ros::Publisher &pub, std::vector<RouteCoordinate> &route)
{
  static nav_msgs::Path path;
  double UTM_North, UTM_East;
  path.header.seq++;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  path.poses.clear();
  for(unsigned i = 0; i < route.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    gps_common::UTM( route.at(i).lat,  route.at(i).lon, &UTM_North, &UTM_East);
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = UTM_North;
    pose.pose.position.y = UTM_East;
    pose.pose.position.z = 5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
  // std::cout << path << std::endl;
  pub.publish(path);
}

void publishResaultPath(ros::Publisher &pub, json::Array &coordinates)
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::PoseStamped local_point;
  static geometry_msgs::PoseStamped global_point;
  static nav_msgs::Path path;
  static bool first_time = true;
  double UTM_North, UTM_East;
  std::string UTM_zone;
  RouteCoordinate coordinate;
  bool new_route = true;
  global_point.header.seq++;
  global_point.header.stamp = ros::Time::now();
  global_point.header.frame_id = "world";

  // if (first_time)
  // {
  //   gps_common::LLtoUTM( origin.lat,  origin.lon, UTM_North, UTM_East, UTM_zone);
  //   global_point.pose.position.x = UTM_East;
  //   global_point.pose.position.y = UTM_North;
  //   global_point.pose.position.z = 0;
  //   global_point.pose.orientation.x = 0;
  //   global_point.pose.orientation.y = 0;
  //   global_point.pose.orientation.z = 0;
  //   global_point.pose.orientation.w = 1;
  //   try
  //   {
  //     tfBuffer.transform(global_point, local_point, "map");
  //   }
  //   catch (tf2::TransformException &ex)
  //   {
  //     ROS_WARN("OSRM1: %s",ex.what());
  //     ros::Duration(1.0).sleep();
  //   }
  //   upSampling(new_route, path, local_point.pose.position.x, local_point.pose.position.y, route_resolution_);

  //   first_time = false;
  //   new_route = false;
  // }
  
  for (auto rnode : coordinates.values)
  {
    auto &latlong = rnode.get<json::Array>();
    coordinate.lon = latlong.values.at(0).get<json::Number>().value;
    coordinate.lat = latlong.values.at(1).get<json::Number>().value;
    gps_common::LLtoUTM( coordinate.lat,  coordinate.lon, UTM_North, UTM_East, UTM_zone);
    global_point.pose.position.x = UTM_East;
    global_point.pose.position.y = UTM_North;
    global_point.pose.position.z = 0;
    global_point.pose.orientation.x = 0;
    global_point.pose.orientation.y = 0;
    global_point.pose.orientation.z = 0;
    global_point.pose.orientation.w = 1;
    try
    {
      tfBuffer.transform(global_point, local_point, "map");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("OSRM2: %s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    upSampling(new_route, path, local_point.pose.position.x, local_point.pose.position.y, route_resolution_);
    //std::cout << std::fixed << coordinate.lat << ", " << coordinate.lon << std::endl;
    // pushPointToPath(new_route, path, UTM_East, UTM_North);
    new_route = false;
  }
  pub.publish(path);
}

void OsrmRouting::getRoute(RouteParameters &params)
{
  json::Object result;
  const auto status = osrm->Route(params, result);
  if (status == Status::Ok)
  {
      auto &routes = result.values["routes"].get<json::Array>();

      // Let's just use the first route
      auto &route = routes.values.at(0).get<json::Object>();
      const auto distance = route.values["distance"].get<json::Number>().value;
      const auto duration = route.values["duration"].get<json::Number>().value;
      auto &geometry = route.values["geometry"].get<json::Object>();
      json::Array &coordinates = geometry.values["coordinates"].get<json::Array>();
      publishResaultPath(route_path_pub_, coordinates);
      // Warn users if extract does not contain the default coordinates from above
      if (distance == 0 || duration == 0)
      {
          std::cout << "Note: distance or duration is zero. ";
          std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
      }
      std::cout << "Distance: " << distance << " meter\n";
      std::cout << "Duration: " << duration << " seconds\n";
  }
  else if (status == Status::Error)
  {
      const auto code = result.values["code"].get<json::String>().value;
      const auto message = result.values["message"].get<json::String>().value;

      std::cout << "Code: " << code << "\n";
      std::cout << "Message: " << code << "\n";
      // return EXIT_FAILURE;
  }
}

OsrmRouting::OsrmRouting(std::string osrm_file)
{
  EngineConfig config;
  config.storage_config = {osrm_file};
  config.use_shared_memory = false;
  config.algorithm = EngineConfig::Algorithm::CH;
  osrm = new OSRM{config};
  // params.coordinates.push_back({util::FloatLongitude{-71.086799}, util::FloatLatitude{42.361404}});
  // params.coordinates.push_back({util::FloatLongitude{-71.087345}, util::FloatLatitude{42.361250}});
  params.overview = engine::api::RouteParameters::OverviewType::Full;
  params.geometries = engine::api::RouteParameters::GeometriesType::GeoJSON;
  params.continue_straight = false;
};

void OsrmRouting::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ros::NodeHandle n;
  static geometry_msgs::PoseStamped local_point;
  static geometry_msgs::PoseStamped global_point;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);

  n.getParam("UTM_zone", UTM_zone_);

  local_point.header.seq++;
  local_point.header.stamp = ros::Time::now();
  local_point.header.frame_id = msg->header.frame_id;
  local_point.pose.orientation.x = 0;
  local_point.pose.orientation.y = 0;
  local_point.pose.orientation.z = 0;
  local_point.pose.orientation.w = 1;
  local_point.pose.position.x =  msg->pose.pose.position.x;
  local_point.pose.position.y =  msg->pose.pose.position.y;
  local_point.pose.position.z =  msg->pose.pose.position.z;
  bool transformed = false;
  while(!transformed)
  {
    try
    {
      tfBuffer.transform(local_point, global_point, "world");
      transformed = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }
  }
  gps_common::UTMtoLL(global_point.pose.position.y,
                      global_point.pose.position.x,
                      UTM_zone_,
                      origin.lat,
                      origin.lon);
  std::cout << std::fixed << origin.lat << ", " << origin.lon << std::endl;
}

void OsrmRouting::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static geometry_msgs::PoseStamped local_point;
  static geometry_msgs::PoseStamped global_point;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  local_point.header.seq++;
  local_point.header.stamp = ros::Time::now();
  local_point.header.frame_id = msg->header.frame_id;
  local_point.pose.orientation.x = 0;
  local_point.pose.orientation.y = 0;
  local_point.pose.orientation.z = 0;
  local_point.pose.orientation.w = 1;
  local_point.pose.position.x =  msg->pose.position.x;
  local_point.pose.position.y =  msg->pose.position.y;
  local_point.pose.position.z =  msg->pose.position.z;
  bool transformed = false;
  UTM_zone_ = "51R";//Taipei UTM zone
  while(!transformed)
    {
    try
    {
     tfBuffer.transform(local_point, global_point, "world");
     transformed = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("OSRMGOAL: %s",ex.what());
      continue;
    }
  }
  gps_common::UTMtoLL(global_point.pose.position.y,
                      global_point.pose.position.x,
                      UTM_zone_,
                      destination.lat,
                      destination.lon);
  std::cout << "OD: " << std::endl;
    std::cout << std::fixed << origin.lat << ", " << origin.lon << std::endl;
  std::cout << std::fixed << destination.lat << ", " << destination.lon << std::endl;
  params.coordinates.clear();
  params.coordinates.push_back({util::FloatLongitude{origin.lon}, util::FloatLatitude{origin.lat}});
  params.coordinates.push_back({util::FloatLongitude{destination.lon}, util::FloatLatitude{destination.lat}});
  OsrmRouting::getRoute(params);
}

void OsrmRouting::gpsDestinationCallback(const sensor_msgs::NavSatFix::ConstPtr& gps)
{
  if(gps->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED)
  {
    params.coordinates.clear();
    std::cout << "OD: " << std::endl;
    std::cout << std::fixed << origin.lat << ", " << origin.lon << std::endl;
    std::cout << std::fixed << gps->latitude << ", " << gps->longitude << std::endl;
    params.coordinates.push_back({util::FloatLongitude{ origin.lon}, util::FloatLatitude{origin.lat}});
    params.coordinates.push_back({util::FloatLongitude{ gps->longitude}, util::FloatLatitude{gps->latitude}});
    OsrmRouting::getRoute(params);
  }
}

void OsrmRouting::updateAutolaborGpsLocationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
   origin.lon = msg->longitude;
   origin.lat = msg->latitude;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_osrm_node");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  n.getParam("UTM_zone", UTM_zone_);
  OsrmRouting osrm_routing(osrm_file_);
  ros::Subscriber init_pose_sub = n.subscribe("/initialpose", 10,
                                &OsrmRouting::initialPoseCallback, &osrm_routing);
  ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 10,
                                &OsrmRouting::goalCallback, &osrm_routing);
  ros::Subscriber gps_goal_sub = n.subscribe("/autolabor_gps_destination", 10,
                                &OsrmRouting::gpsDestinationCallback, &osrm_routing);
  ros::Subscriber autolabor_gps_sub = n.subscribe("/autolabor_gps_location", 10,
                                &OsrmRouting::updateAutolaborGpsLocationCallback, &osrm_routing);
  route_path_pub_ =  n.advertise<nav_msgs::Path>("osrm_route_path", 10, true);
  ros::spin();
}
