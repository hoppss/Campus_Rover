#include <campusrover_map_server/pcd_map_loader.hpp>

void getParameters(ros::NodeHandle n_private)
{
  n_private.param<string>("config_file", config_file_, ".yaml");
}

void publishFakegps(ros::Publisher &pub, double lat, double lon)
{
  sensor_msgs::NavSatFix gps;
  gps.header.frame_id = "gps";
  gps.status.status = 0;
  gps.status.service = 1;
  gps.latitude = lat;
  gps.longitude = lon;
  gps.altitude = 10;
  gps.position_covariance[0] = 3.9561210000000004;
  gps.position_covariance[1] = 0.0;
  gps.position_covariance[2] = 0.0;
  gps.position_covariance[3] = 0.0;
  gps.position_covariance[4] = 3.9561210000000004;
  gps.position_covariance[5] = 0.0;
  gps.position_covariance[6] = 0.0;
  gps.position_covariance[7] = 0.0;
  gps.position_covariance[8] = 7.650756;
  gps.position_covariance_type = 2;
  pub.publish(gps);
}

void loadPcdMap(ros::NodeHandle &nh,
                string file_name,
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  YAML::Node config = YAML::LoadFile(file_name);
  const char* home = getenv("HOME");
  std::string path(home);
  std::string pcd_file = "";
  try {
    config["pcd_file"] >> pcd_file;
    pcd_file_ = path + "/pcd/" + pcd_file;
    // cout << pcd_file_ << endl;
    config["gps_topic"]["topic_name"] >> gps_topic_name_;
    config["gps_topic"]["parent_frame"] >> world_frame_;
    config["gps_topic"]["child_frame"] >> local_frame_;
    config["gps_topic"]["latitude"] >> latitude_;
    config["gps_topic"]["longitude"] >> longitude_;
    config["map_fix_tf"]["child_frame"] >> map_frame_;
    config["map_fix_tf"]["tf"][0] >> pcd_tf_.x;
    config["map_fix_tf"]["tf"][1] >> pcd_tf_.y;
    config["map_fix_tf"]["tf"][2] >> pcd_tf_.z;
    config["map_fix_tf"]["tf"][3] >> pcd_tf_.r_x;
    config["map_fix_tf"]["tf"][4] >> pcd_tf_.r_y;
    config["map_fix_tf"]["tf"][5] >> pcd_tf_.r_z;
    config["map_fix_tf"]["tf"][6] >> pcd_tf_.r_w;
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("YAML load paramerters failed");
    exit(-1);
  }
  // read pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_file_, *cloud) == -1)
  {
    ROS_ERROR("Load %s failed", pcd_file_.c_str());
    exit(-1);
  }
  else
  {
    cout << "Loaded file: " << pcd_file_ << endl;
  }
  // Set world to Local statellite map static transform
  static tf2_ros::StaticTransformBroadcaster static_broadcaster_local;
  geometry_msgs::TransformStamped static_transformStamped;
  double UTM_North, UTM_East;
  std::string UTM_zone;
  gps_common::LLtoUTM(latitude_, longitude_, UTM_North, UTM_East, UTM_zone);
  nh.setParam("UTM_zone", UTM_zone);
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = world_frame_;
  static_transformStamped.child_frame_id = local_frame_;
  static_transformStamped.transform.translation.x = UTM_East;
  static_transformStamped.transform.translation.y = UTM_North;
  static_transformStamped.transform.translation.z = 0;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;
  static_broadcaster_local.sendTransform(static_transformStamped);
  // Set Local statellite map to pcd map static transform
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = local_frame_;
  static_transformStamped.child_frame_id = map_frame_;
  static_transformStamped.transform.translation.x = pcd_tf_.x;
  static_transformStamped.transform.translation.y = pcd_tf_.y;
  static_transformStamped.transform.translation.z = pcd_tf_.z;
  static_transformStamped.transform.rotation.x = pcd_tf_.r_x;
  static_transformStamped.transform.rotation.y = pcd_tf_.r_y;
  static_transformStamped.transform.rotation.z = pcd_tf_.r_z;
  static_transformStamped.transform.rotation.w = pcd_tf_.r_w;
  static_broadcaster_local.sendTransform(static_transformStamped);
  // Set fake base frame static transform
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped base_pose;

  try
  {
    transformStamped = tfBuffer.lookupTransform(local_frame_, "base_link", ros::Time(0), ros::Duration(2));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN(" %s. Can't update pose from TF, for that will be use the latest source point.", ex.what());
  }

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = local_frame_;
  static_transformStamped.child_frame_id = "fake_base_link";
  static_transformStamped.transform.translation.x = transformStamped.transform.translation.x;
  static_transformStamped.transform.translation.y = transformStamped.transform.translation.y;
  static_transformStamped.transform.translation.z = 0;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;
  static_broadcaster_local.sendTransform(static_transformStamped);
}

void publishCloud(ros::Publisher &pub,  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  static sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.seq++;
  ros_cloud.header.stamp = ros::Time::now();
  ros_cloud.header.frame_id = map_frame_;
  pub.publish(ros_cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_tf_finder_node");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  getParameters(nh_private);
  loadPcdMap(n, config_file_, cloud);
  // publish latch fake_gps
  fake_gps_pub_ = n.advertise<sensor_msgs::NavSatFix>(gps_topic_name_, 10, true);
  publishFakegps(fake_gps_pub_, latitude_, longitude_);
  cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("map_cloud", 10, true);
  publishCloud(cloud_pub_, cloud);
  ros::spin();
}
