/*
1. Click point A on openstreet map.
2. Click point B on openstreet map.
3. Click point A on laser 2D map.
4. Click point B on laser 2D map. (calculate theta and make rotate)
5. Click translate point on laser 2D map.
6. Click translate point on openstreet map. (make translate)
7. Choose ROI point A.
8. Choose ROI point B. (plot ROI and show results)
*/

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <campusrover_map_server/DatabaseWrite.h>

#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

using namespace std;
using namespace sql;

struct Point{
  double x;
  double y;
};

double tanslation_x_, tanslation_y_, tf_x_, tf_y_, tf_yaw_;
Point map_roi[2]; // 0: min, 1: max
ros::Publisher ROI_path_pub_;

void get_parameters(ros::NodeHandle n_private);
double getTheta(Point &v, Point &w);
void get_tf_callback(const geometry_msgs::PointStampedConstPtr &msg);
void addPathPoint(double x, double y, nav_msgs::Path &path);
bool database_Write(campusrover_map_server::DatabaseWrite::Request  &req,
                    campusrover_map_server::DatabaseWrite::Response &res);
