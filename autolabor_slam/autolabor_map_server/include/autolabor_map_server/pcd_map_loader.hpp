// C++
#include <pwd.h>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

//tf
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


// LIBRARIES
#include <gps_common/conversions.h>

using namespace std;

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

struct Mytf
{
  double x;
  double y;
  double z;
  double r_x;
  double r_y;
  double r_z;
  double r_w;
};

string config_file_, pcd_file_, gps_topic_name_, world_frame_, local_frame_,
       map_frame_;
double latitude_, longitude_;
Mytf pcd_tf_;
ros::Publisher fake_gps_pub_, cloud_pub_;

void getParameters(ros::NodeHandle n_private);
void publishFakegps(ros::Publisher &pub, double lat, double lon);
void loadPcdMap(ros::NodeHandle &nh,
                string file_name,
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void publishCloud(ros::Publisher &pub,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
