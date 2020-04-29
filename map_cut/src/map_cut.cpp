#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


main (int argc, char **argv)
{
    ros::init (argc, argv, "map_combine");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("point2", 100);
    pcl::PointCloud<pcl::PointXYZ> cloud_1, cloud_2, cloud_3, cloud_4, cloud_5;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 out;

    ros::Rate r(100);
    while (nh.ok())
    {
        ros::spinOnce();
        pcl::io::loadPCDFile("/home/eric/Desktop/c.pcd",cloud_1);
        pcl::io::loadPCDFile("/home/eric/Desktop/f.pcd",cloud_2);
        pcl::io::loadPCDFile("/home/eric/Desktop/b.pcd",cloud_3);
        pcl::io::loadPCDFile("/home/eric/Desktop/l.pcd",cloud_4);
        pcl::io::loadPCDFile("/home/eric/Desktop/r.pcd",cloud_5);
        cloud = cloud_1 + cloud_2;
        cloud += cloud_3;
        cloud += cloud_4;
        cloud += cloud_5;
        pcl::toROSMsg(cloud,out);
        out.header.stamp = ros::Time::now();
        out.header.frame_id = "odom";
        point_pub.publish(out);
        r.sleep();
    }
    
}