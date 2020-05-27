#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

laser_geometry::LaserProjection projector_;
ros::Publisher cloud_pub_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  static int div_time = 0;
  //if (div_time++ > 4)
  {
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud);
    cloud_pub_.publish(cloud);
    div_time = 0;
  }
  
  // Do something with cloud.
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "campusrover_costmap_node");
  ros::NodeHandle nh;
  ros::Subscriber laser_sub_ = nh.subscribe
                               <sensor_msgs::LaserScan>("laser", 10, &scanCallback);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
  ros::spin();
  return 0;
}