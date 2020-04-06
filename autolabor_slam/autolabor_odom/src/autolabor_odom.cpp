#include <autolabor_odom/autolabor_odom.hpp>

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<bool>("pub_tf", pub_tf_, true);
  n_private.param<bool>("pub_odom", pub_odom_, true);
  n_private.param<string>("source_frame_id", source_frame_id_, "odom");
  n_private.param<string>("target_frame_id", target_frame_id_, "base_link");
  n_private.param<int>("encorder_ppr", encorder_ppr_, 10000);
  n_private.param<double>("encorder_diameter", encorder_diameter_, 0.0484);
  n_private.param<int>("num_data_init_pose", num_data_init_pose_, 10);
}

void imu_update_yaw(const sensor_msgs::ImuConstPtr& imu_msg)
{
  static bool first_time = true;
  sensor_msgs::Imu imu_pub_msg;
  tf::Quaternion q(imu_msg->orientation.x,
                   imu_msg->orientation.y,
                   imu_msg->orientation.z,
                   imu_msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (!first_time)
  {
    first_time = false;
  }
  else
  {
    imu_yaw_ = yaw;
    imu_ready_ = true;
  }
}


void GpsMagSynCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg, 
                       const sensor_msgs::MagneticFieldConstPtr& mag_msg)
{
  // list[0]: latitude, list[1]: longitude, list[2]:altitude,  list[3]:rotation
  std::list<double> mylist;
  mylist.push_back(gps_msg->latitude);
  mylist.push_back(gps_msg->longitude);
  mylist.push_back(gps_msg->altitude);
  mylist.push_back(atan2( mag_msg->magnetic_field.x, mag_msg->magnetic_field.y));
  gps_mag_queue_mutex_.lock();
  if (gps_mag_queue_->size() <= num_data_init_pose_)
  {
    gps_mag_queue_->push_back(mylist);
  }
  else
  {
    gps_mag_queue_->pop_front();
    gps_mag_queue_->push_back(mylist);
  }
  gps_mag_queue_mutex_.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_lanemark_extraction");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  gps_mag_queue_ = new std::deque<std::list<double> >();
  imu_yaw_ = 0;
  imu_ready_ = false;
  flow_imu_pub_ = n.advertise<nav_msgs::Odometry> ("odom", 10);
  imu_sub_ = n.subscribe("imu", 10, imu_update_yaw);
  // GPS and imu magatic sensor synchronize
  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(n, "gps", 10);
  message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub(n, "magnetic", 10);
  typedef message_filters::sync_policies::ApproximateTime
                      <sensor_msgs::NavSatFix, sensor_msgs::MagneticField> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, mag_sub);
  sync.registerCallback(std::tr1::bind(&GpsMagSynCallback, 
                                       std::tr1::placeholders::_1, 
                                       std::tr1::placeholders::_2));
  ros::spin();
  return 0;
}
