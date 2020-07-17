#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char *argv[])
{
  if(argc < 2) {
    std::cout << "Too a few arg" << std::endl;
    return 0;
  } else {
    std::cout << "Open file: " << argv[1] << std::endl;
  }
  // pcl::visualization::CloudViewer viewer_src ("Source");
  // pcl::visualization::CloudViewer viewer ("Filtered");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  // Fill in the cloud data
  // pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  // reader.read<pcl::PointXYZI> (argv[1], *cloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud) == -1) //* load the file
  {
    std::cout << "Couldn't read file: " << argv[1] << std::endl;
    return (-1);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  // OutlierRemoval
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (100);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);


  // VoxelGrid filter
  pcl::VoxelGrid<pcl::PointXYZI> v_sor;
  v_sor.setInputCloud (cloud_filtered);
  v_sor.setLeafSize (0.2, 0.2, 0.2);
  v_sor.filter (*cloud_filtered);

  // viewer.showCloud (cloud_filtered);
  // viewer_src.showCloud(cloud);
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  if(argc > 2)
  {
    std::cout << "Save pcd file as " << argv[2]<< std::endl;
    std::string file_name = argv[2];
    pcl::io::savePCDFileASCII (file_name, *cloud_filtered);

  }
  // // while (!viewer.wasStopped ())
  // // {
  // // }
  return 0;
}
