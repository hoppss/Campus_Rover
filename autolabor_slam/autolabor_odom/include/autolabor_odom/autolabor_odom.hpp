#include <ros/ros.h>
#include <math.h>
#include <queue>
#include <mutex>
#include <list>
#include <vector>
#include <numeric>
#include <tr1/functional>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gps_common/conversions.h>


using namespace std;

ros::Publisher gps_init_pose_pub_;
ros::Publisher flow_imu_pub_;
ros::Subscriber imu_sub_;
ros::Subscriber encoder_sub_;
bool pub_tf_, pub_odom_, pub_imu_;
string source_frame_id_, target_frame_id_;
double imu_yaw_, encorder_diameter_;
int encorder_ppr_;
bool imu_ready_;
int num_data_init_pose_;
std::deque<std::list<double> > *gps_mag_queue_;
std::mutex gps_mag_queue_mutex_;

void get_parameters(ros::NodeHandle n_private);
void imu_update_yaw(const sensor_msgs::ImuConstPtr& imu_msg);
void GpsMagSynCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg, 
                       const sensor_msgs::MagneticFieldConstPtr& mag_msg);
