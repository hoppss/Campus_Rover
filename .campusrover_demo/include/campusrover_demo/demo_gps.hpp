#include <math.h>
#include <fstream>
//ROS
#include <ros/ros.h>
#include <string>

//PKG
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

// Tf
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
// LIBRARIES
#include <gps_common/conversions.h>

static const int BREAK_SERVICE = 1;
static const int STREERING_CENTER_SERVICE = 2;
static const int RESET_STREERING_CENTER_SERVICE = 3;
static const int RELEASE_SERVICE = 4;
static const int MANUAL_SERVICE = 5;
static const int AUTO_SERVICE = 6;

static const double L = 1.3;

using namespace std;

enum ModeId {stop, joy_control, release, functions, none};
enum FunctionId {pending = 0x01, delivering = 0x02};
enum LedCmd {solid=0x00, breath=0x01, top_LED=0x02, left_LED=0x03, right_LED=0x04};

struct Point{
  double x;
  double y;
};

int path_num_;
ModeId mode_ = functions;
FunctionId function_;
ros::Subscriber gps_sub_;
ros::Publisher front_led_pub_;
ros::Publisher path_pub_;
ros::Publisher position_pub_;
ros::Publisher hmi_led_pub_;
std::vector<nav_msgs::Path> slam_paths_;

double pending_dis_;
string map_frame_;
std::vector<string> path_name_;
// geometry_msgs::PoseStamped goal_point_;
bool pub_gps_position_;
bool use_osrm_;
string file_path_;
int path_end_nearest_index_;
std::string UTM_zone_;


void getParameters(ros::NodeHandle n_private);
void gpsGoalCallcack(const sensor_msgs::NavSatFix::Ptr gps);
void updatePosition(const ros::TimerEvent&);
void pubSystemStatus(ModeId mode, FunctionId function);
// path follow
void updatePath(const ros::TimerEvent& event);
void loadPath(std::string file_path);
int getNearestPointIndex(nav_msgs::Path &path, geometry_msgs::PoseStamped &point);
