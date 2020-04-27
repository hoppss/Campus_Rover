#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_;
ros::Publisher cloud_pub_;
ros::Subscriber scan_sub_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  static bool tf_get = false;
  static tf::TransformListener listener_;
  if(!tf_get)
  {
    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
   }
   tf_get = true;
 }
 else
 {
   sensor_msgs::PointCloud2 cloud;
   projector_.transformLaserScanToPointCloud("base_link",*scan_in,
           cloud,listener_);
   cloud_pub_.publish(cloud);
 }
  // Do something with cloud.
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_pointcloud2");
  ros::NodeHandle n;
  cloud_pub_ = n.advertise<sensor_msgs::PointCloud2> ("cloud2", 10);
  scan_sub_= n.subscribe("scan", 10, scanCallback);
  ros::spin();
  return 0;
}
