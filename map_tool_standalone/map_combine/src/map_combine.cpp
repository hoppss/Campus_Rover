#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <stdio.h>

#include <map_load/load.h>

float dis, leafs;
float pose_x = 0.0, pose_y = 0.0, initial_pose_x, initial_pose_y;
std::string dir, floors, c_status;
char cstr[9][50];

ros::Subscriber controller_sub;
ros::Subscriber floor_sub;
ros::Subscriber ndt_sub;
ros::Subscriber initial_pose;

ros::Publisher point_pub;

ros::ServiceClient client;

pcl::VoxelGrid<pcl::PointXYZI> voxel;

pcl::PointCloud<pcl::PointXYZI> cloud_n;
pcl::PointCloud<pcl::PointXYZI> filter_cloud[9];

sensor_msgs::PointCloud2 out;

map_load::load srv;

void getParameters(ros::NodeHandle nh_p)
{
    nh_p.param<float>("grid_distant", dis, 7.0);
    nh_p.param<float>("leafs", leafs, 0.2);
    nh_p.param<std::string>("dir", dir, "/home/eric/Desktop/map2/");
}

void controller_callback(const std_msgs::String& c)
{
    c_status = c.data;
}

void floor_callback(const std_msgs::String& f)
{
    floors = f.data;
}

void callback(const geometry_msgs::PoseStamped& p)
{
    pose_x = p.pose.position.x;
    pose_y = p.pose.position.y;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& i)
{
    initial_pose_x = i.pose.pose.position.x;
    initial_pose_y = i.pose.pose.position.y;
}

std::string* service_call(float x, float y)
{
    srv.request.x = x;
    srv.request.y = y;
    srv.request.dis = dis;
    client.call(srv);

    std::string* str = new std::string[9];
    str[0] = srv.response.a;
    str[1] = srv.response.b;
    str[2] = srv.response.c;
    str[3] = srv.response.d;
    str[4] = srv.response.e;
    str[5] = srv.response.f;
    str[6] = srv.response.g;
    str[7] = srv.response.h;
    str[8] = srv.response.i;
    return str;
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "map_combine");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    floor_sub = nh.subscribe("floor", 100, floor_callback);
    ndt_sub = nh.subscribe("ndt_pose", 100, callback);
    controller_sub = nh.subscribe("controller", 100, controller_callback);
    initial_pose = nh.subscribe("initialpose", 100, pose_callback);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("points_map", 100);
    client = nh.serviceClient<map_load::load>("map_load");

    getParameters(nh_p);

    ros::Rate r(100);
    while (nh.ok())
    {
        ros::spinOnce();
        pcl::PointCloud<pcl::PointXYZI> cloud;
        if (c_status == "elevator")
        {
            std::string* str = service_call(initial_pose_x, initial_pose_y);

            sprintf(cstr[0], (dir + "%s").c_str(), (floors + "/" +*(str + 0)).c_str());
            pcl::io::loadPCDFile(cstr[0],cloud_n);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            *cloud_ptr = cloud_n;
            voxel.setInputCloud(cloud_ptr);
            voxel.setLeafSize(leafs, leafs, leafs);
            voxel.filter(filter_cloud[0]);

            cloud = filter_cloud[0];
        }
        else if (c_status == "planner")
        {
            std::string* str = service_call(pose_x, pose_y);

            for (int i = 0; i < 9; i++)
            {
                sprintf(cstr[i], (dir + "%s").c_str(), (floors + "/" +*(str + i)).c_str());
                pcl::io::loadPCDFile(cstr[i],cloud_n);

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                *cloud_ptr = cloud_n;
                voxel.setInputCloud(cloud_ptr);
                voxel.setLeafSize(leafs, leafs, leafs);
                voxel.filter(filter_cloud[i]);
                
                cloud += filter_cloud[i];
            }
        }
        
        pcl::toROSMsg(cloud,out);
        out.header.stamp = ros::Time::now();
        out.header.frame_id = "map";
        point_pub.publish(out);
        r.sleep();
    }
    
}