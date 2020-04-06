#ifndef __MAPTOOL_H__
#define __MAPTOOL_H__
#include <QtCore>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>

#include <iostream>
#include <thread>
#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gpx_table.hpp"

class MapTool : public QObject
{
  Q_OBJECT
public:
  MapTool(int argc, char **pArgv);
  virtual ~MapTool();
  bool init();
  void run();

  void pointsCallback(const geometry_msgs::PointStamped &point);
  void pathCallback(const nav_msgs::Path &path);

  void setWayointMode();
  void setLaneMode();
  void setObjectMode();
  void delectLastPoint();
  void inserItem();
  void deleteLastItem();
  void clearAllItem();
  void Reset_table();
  void getItemsNumber(int &lane_num, int &way_num, int &object_num);
  void saveToFile(std::string path);
private:
  int m_Init_argc;
  char** m_pInit_argv;
  std::thread m_thread;
  std::atomic<bool> m_stop_flag;

  ros::Subscriber points_sub, path_sub;
  ros::Publisher marks_pub;
  GpxTable m_gps_table;

};
#endif
