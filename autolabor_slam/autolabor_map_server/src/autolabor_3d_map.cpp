#include <autolabor_map_server/autolabor_3d_map.hpp>

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<bool>("show_pc", show_pc_, true);
  n_private.param<string>("map_frame", map_frame_, std::string("map"));
  n_private.param<string>("file_path", file_path_, std::string("/home"));
  n_private.param<bool>("use_scan", use_scan_, true);
  n_private.param<bool>("use_pointcloud2", use_pointcloud2_, false);
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  static tf::TransformListener listener_;
  //static CloudPtr cloud_acc (new Cloud());
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        map_frame_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
  {
    return;
  }
  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud(map_frame_,*scan_in, cloud,listener_);
  CloudPtr cloud_in (new Cloud());
  pcl::fromROSMsg(cloud,*cloud_in);
  // cout << cloud_in->points[100].intensity << endl;
  *cloud_acc_ += *cloud_in;
  cloud_show_ = cloud_acc_;
  // Do something with cloud.
}

void pointcloud2Callback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  static sensor_msgs::PointCloud2 cloud_out;
  static tf::TransformListener listener_;
  if(!listener_.waitForTransform(
        cloud_in->header.frame_id,
        map_frame_,
        ros::Time::now(),
        ros::Duration(1.0)))
  {
    return;
  }
  pcl_ros::transformPointCloud(map_frame_, *cloud_in, cloud_out, listener_);
  CloudPtr cloud_pcl (new Cloud());
  pcl::fromROSMsg(cloud_out,*cloud_pcl);
  *cloud_acc_ += *cloud_pcl;
  cloud_show_ = cloud_acc_;
}

//visualization's callback function
void viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  static bool first_time = true;
  if (first_time)
  {
    viz.addCoordinateSystem (1.0);
    first_time = false;
    viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  }
  //Draw downsampled point cloud from sensor
  CloudPtr cloud_pass;
  cloud_pass = cloud_show_;

    if(cloud_pass){

      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>intensity_distribution(cloud_pass);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud_pass, 100, 100, 100);
      if (!viz.updatePointCloud<pcl::PointXYZI>(cloud_pass,single_color, "cloud"))
      {
        viz.addPointCloud<pcl::PointXYZI>(cloud_pass,single_color, "cloud");
        viz.resetCameraViewpoint("cloud");
      }
    }
}

bool save_3d_cloud(autolabor_map_server::SavePcd::Request  &req,
                   autolabor_map_server::SavePcd::Response &res)
{
  string path = file_path_ + req.file_name + ".pcd";
  double resolution = req.resolution;
  cout << "Save pointcloud to " << path << endl;
  if(resolution <= 0)
  {
    pcl::io::savePCDFileASCII ( path, *cloud_acc_);
    res.could_size = cloud_acc_->width * cloud_acc_->height;
  }
  else
  {
    CloudPtr cloud_filter (new Cloud());
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_acc_);
    sor.setLeafSize (resolution, resolution, resolution);
    sor.filter (*cloud_filter);
    pcl::io::savePCDFileASCII ( path, *cloud_filter);
    res.could_size = cloud_filter->width * cloud_filter->height;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autolabor_3d_map");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  get_parameters(n_private);
  CloudPtr cloud_acc (new Cloud());
  cloud_acc_ = cloud_acc;
  if(show_pc_)
  {
    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL Viewer");
    viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");
  }
  if(use_scan_)
  {
    scan_sub_= n.subscribe("scan", 10, scanCallback);
    use_pointcloud2_ = false;
  }
  if(use_pointcloud2_)
  {
    scan_sub_= n.subscribe("pointcloud2", 10, pointcloud2Callback);
  }
  ros::ServiceServer service = n.advertiseService("save_pcd", save_3d_cloud);
  ros::spin();
}
