// MySQL: https://dev.mysql.com/doc/connector-cpp/en/connector-cpp-examples-complete-example-2.html
#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <autolabor_map_server/MapUpdate.h>

#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <ros/console.h>
#include <map_server/image_loader.h>
#include <nav_msgs/MapMetaData.h>
#include <yaml-cpp/yaml.h>

using namespace std;

struct MapInfo {
  int map_id;
  double max_utm_x;
  double min_utm_x;
  double max_utm_y;
  double min_utm_y;
  double tf_x;
  double tf_y;
  double tf_yaw;
  string path;
};

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

string table_name_;
string file_path_;

sql::Driver *sql_driver_;
sql::Connection *sql_con_;
sql::Statement *sql_stmt;
sql::ResultSet *sql_res_;

void get_parameters(ros::NodeHandle n_private);
bool update_map(autolabor_map_server::MapUpdate::Request  &req,
                autolabor_map_server::MapUpdate::Response &res);
void connect_database();
