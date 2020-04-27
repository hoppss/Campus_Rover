#include <map_tool.hpp>

MapTool::MapTool(int argc, char **pArgv)
        : m_gps_table(), m_Init_argc(argc), m_pInit_argv(pArgv), m_stop_flag(false)
{ }

MapTool::~MapTool()
{
  if (ros::isStarted())
  {
      ros::shutdown();
      ros::waitForShutdown();
  }//end if

  m_stop_flag = true;
  if (m_thread.joinable())
    m_thread.join();
}

bool MapTool::init()
{
  ros::init(m_Init_argc, m_pInit_argv, "autolabor_map_tool");

  if (!ros::master::check())
      return false;//do not start without ros.

  ros::start();
  ros::Time::init();
  ros::NodeHandle nh;
  nh.getParam("UTM_zone", m_gps_table.UTM_zone);
  points_sub = nh.subscribe("autolabor_points", 10, &MapTool::pointsCallback, this);
  path_sub = nh.subscribe("path_loader", 10, &MapTool::pathCallback, this);
  marks_pub  = nh.advertise<visualization_msgs::MarkerArray>("/autolabor_markers", 100);
  // ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback);
  m_thread = std::thread(std::bind(&MapTool::run, std::ref(*this)));
  return true;
}

void MapTool::run()
{
  // ros::spin();
  while (!m_stop_flag.load() && ros::ok())
  {
    ros::spinOnce();
  }
}

void MapTool::pointsCallback(const geometry_msgs::PointStamped &point)
{
  m_gps_table.addSubPoints(point.point.x, point.point.y, point.point.z);
}

void MapTool::pathCallback(const nav_msgs::Path &path)
{
  static geometry_msgs::PoseStamped local_point;
  static geometry_msgs::PoseStamped global_point;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  local_point.header.seq = path.header.seq;
  local_point.header.stamp = path.header.stamp;
  local_point.header.frame_id = path.header.frame_id;
  local_point.pose.orientation.x = 0;
  local_point.pose.orientation.y = 0;
  local_point.pose.orientation.z = 0;
  local_point.pose.orientation.w = 1;
  for (int i = 0; i < path.poses.size(); i++)
  {
    local_point.pose.position.x =  path.poses[i].pose.position.x;
    local_point.pose.position.y =  path.poses[i].pose.position.y;
    local_point.pose.position.z =  path.poses[i].pose.position.z;
    try
     {
       tfBuffer.transform(local_point, global_point, "world");
     }
     catch (tf2::TransformException &ex)
     {
       ROS_WARN("%s",ex.what());
       return;
     }
    m_gps_table.addSubPoints(global_point.pose.position.x,
                             global_point.pose.position.y,
                             global_point.pose.position.z );
  }
}

void MapTool::setWayointMode()
{
  m_gps_table.setItemTypeWaypoint();
}

void MapTool::setLaneMode()
{
  m_gps_table.setItemTypeLane();
}

void MapTool::setObjectMode()
{
  m_gps_table.setItemTypeObject();
}

void MapTool::delectLastPoint()
{
  m_gps_table.deleteLastPoint();
}

void MapTool::inserItem()
{
  m_gps_table.insertCurrentItem();
}

void MapTool::deleteLastItem()
{
  m_gps_table.deleteLastItem();
}

void MapTool::clearAllItem()
{
  m_gps_table.clearAllItem();
}

void MapTool::Reset_table()
{
  m_gps_table.resetAll();
}

void MapTool::getItemsNumber(int &lane_num, int &way_num, int &object_num)
{
  m_gps_table.getItemsNumber(lane_num, way_num, object_num);
}

void MapTool::saveToFile(std::string path)
{
  m_gps_table.saveToFile(path);
}
