#include "campusrover_costmap.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

void PclVizCallback(pcl::visualization::PCLVisualizer& viz)
{
  // TODO
  static bool first_time = true;
  if(first_time)
  {
    pcl::PointXYZ p1(0, 0, 0);
    pcl::PointXYZ p2(5, 0, 0);
    viz.addCube(0, 1.6, -0.45, 0.45, 0, 1.1);
    viz.addLine (p1, p2, 255, 0,0);
    first_time = false;
  }
  for (int i = 0; i < g_cloud_viz_list.size(); i++)
  {
    // std::cout << "Cloud size: " << g_cloud_viz_list[i].size() << std::endl;
    PointCloudPtr cloud_pass = g_cloud_viz_list[i];
    viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (cloud_pass, 255, 255, 255);
    if(cloud_pass == NULL) return;
    if (!viz.updatePointCloud (cloud_pass, color, "cloud"))
    {
      viz.addPointCloud (cloud_pass,color, "cloud");
      viz.resetCameraViewpoint ("cloud");
    }
  }
  g_cloud_viz_list.clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "campusrover_costmap_node");
  ros::NodeHandle nh;
  CampusroverCostmap campusrover_costmap(nh);
  if(g_pcl_viz)
  {
    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL Viewer");
    viewer_->runOnVisualizationThread (boost::bind(&PclVizCallback, _1), "g_pcl_vizcb");
  }
  ros::spin();
  return 0;
}
