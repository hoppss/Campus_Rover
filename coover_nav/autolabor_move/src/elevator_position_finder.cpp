#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Path.h>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber scan_sub_;
ros::Publisher pose_pub_, break_point_marker_pub_, elevator_corner_marker_pub_, featuer_point_marker_pub_, elevator_marker_pub_;

string scan_frame_;
string map_frame_;
string window_type_;
double window_size_;
double max_scan_range_;
double corner_feature_threshold;

double enter_elevator_feature_dis_range_;
double enter_elevator_feature_d_angle_range_;
int elevator_num_;
double door_gap_dis_;
double door_x1_;
double door_y1_;
double door_y2_;
double elevator_wd_;
double filter_range_dis_;
double filter_range_angle_;

bool elevator_pose_filter_;

std::vector<geometry_msgs::Point> detect_feature_point;
std::vector<geometry_msgs::Point> enter_elevator_feature;
std::vector<geometry_msgs::Point> exit_elevator_feature;
std::vector<geometry_msgs::Point> sample_elevator_feature;

void VisualizeCorner(std::vector<geometry_msgs::Point> points);
void VisualizeBreakPoint(std::vector<geometry_msgs::Point> points);
void VisualizefeaturePoint(std::vector<geometry_msgs::Point> points) ;
void FeatureMatching(std::vector<geometry_msgs::Point> points);
void VisualizeElevatorEnterPoint(std::vector<nav_msgs::Path> paths);
void initial_elevator_feature();
void pose_filter(geometry_msgs::PoseArray &poses);

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("map_frame", map_frame_, "map");
  n_private.param<string>("window_type", window_type_, "circular");
  n_private.param<bool>("elevator_pose_filter", elevator_pose_filter_, true); 
  n_private.param<double>("window_size", window_size_, 0.15); 
  n_private.param<double>("max_scan_range", max_scan_range_, 4.0);
  n_private.param<double>("corner_feature_threshold", corner_feature_threshold, 0.000002);
  n_private.param<double>("enter_elevator_feature_dis_range", enter_elevator_feature_dis_range_, 0.1);
  n_private.param<double>("enter_elevator_feature_d_angle_range", enter_elevator_feature_d_angle_range_, 0.1);

  n_private.param<double>("filter_range_dis", filter_range_dis_, 0.1);
  n_private.param<double>("filter_range_angle", filter_range_angle_, 0.3);

  n_private.param<int>("elevator_num", elevator_num_, 2.0);
  n_private.param<double>("door_gap_dis", door_gap_dis_, 0.91);
  n_private.param<double>("door_x1", door_x1_, 0.34);
  n_private.param<double>("door_y1", door_y1_, 0.94);
  n_private.param<double>("door_y2", door_y2_, 0.8);
  n_private.param<double>("elevator_wd", elevator_wd_, 0.8);

  initial_elevator_feature();
}

void initial_elevator_feature()
{
  static double dis, angle, angle2, angle3, dangle;
  static double vx,vy; 
  static double last_vx,last_vy,last_angle;
  geometry_msgs::Point feature;

  //---enter feature--------//
  for(int i=0; i < 7;i++)
  {
    if(i == 0)
    {
      dis = 0.0;
      angle = angle2 =angle3=0.0;
      vx = vy =0.0;
    }
    else if(i == 1 )    
    {
      vx = 0.0;
      vy = -door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 2)
    {
      vx = -(door_x1_*cos(asin(((door_y1_-door_y2_)/2.0)/door_x1_)));
      vy = -((door_y1_-door_y2_)/2.0);
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle2 = angle;
    }
    else if(i == 3)
    {
      vx = 0.0;
      vy = -door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      

    }else if(i == 4)
    {
      vx = door_x1_*cos(asin(((door_y1_-door_y2_)/2.0)/door_x1_));
      vy = -((door_y1_-door_y2_)/2.0);
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle3 = angle;

    }else if(i == 5)
    {
      vx = 0.0;
      vy = -door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 6)
    {
      vx = 0.0;
      vy = 0.0;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      last_angle = angle2;
      angle = angle3;
    }
    dangle = last_angle - angle;
    last_angle = angle;

    feature.x = dis;
    feature.y = angle;
    feature.z = dangle;

    enter_elevator_feature.push_back(feature);
    
    cout<<"Enter Feature = "<<feature<<'\n';
  }

  //---exit feature--------//
  for(int i=0; i < 4;i++)
  {
    if(i == 0)
    {
      dis = 0.0;
      angle =0.0;
      dangle = 0.0;
      vx = vy =0.0;
    }
    else if(i == 1 )    
    {
      vx = -elevator_wd_;
      vy = 0.0;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 2)
    {
      vx = 0.0;
      vy = -elevator_wd_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 3)
    {
      vx = elevator_wd_;
      vy = 0.0;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    dangle = last_angle - angle;
    last_angle = angle;

    if(dangle > M_PI)
    {
      dangle = -2*M_PI + dangle;
    }else if(dangle < -M_PI)
    {
      dangle = 2*M_PI + dangle;
    }

    feature.x = dis;
    feature.y = angle;
    feature.z = dangle;

    exit_elevator_feature.push_back(feature);
    
    cout<<"Exit Feature = "<<feature<<'\n';
  }
}

void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  static geometry_msgs::Pose pose;
  static geometry_msgs::PoseArray scan_poses;
  static double angle;
  static bool first_time = true;

  std::vector<double> range;
  std::vector<double> intensities;

  //cout<<"=================================="<<'\n';

  if(scan->ranges.size()==0)
    return;
  
  scan_poses.header.stamp = ros::Time::now();
  scan_frame_ = scan->header.frame_id;
  scan_poses.header.frame_id = scan_frame_;
  scan_poses.poses.clear();
  detect_feature_point.clear();

  for (int i = 0; i < scan->ranges.size(); i++)
  {
    if(scan->ranges[i] > 0.001)
    {
      angle = scan->angle_min + (i * scan->angle_increment);
      if(scan->ranges[i] < max_scan_range_)
      {
        pose.position.x = scan->ranges[i]*cos(angle);
        pose.position.y = scan->ranges[i]*sin(angle);
        scan_poses.poses.push_back(pose);
        //cout<<"scan_poses x = "<<pose.position.x<<" scan_poses y = "<<pose.position.y<<'\n';
        //cout<<"angle = "<<angle<<'\n';
      }
    }
  }
  // if(first_time)
  // {
  //   pose_pub_.publish(scan_poses);
  //   first_time = false;
  // }
  
  

  static std::vector<geometry_msgs::Point> window_points ;
  static std::vector<double> v_x;
  static std::vector<double> v_y;
  static std::vector<double> delta_vx;
  static std::vector<double> delta_vy;
  static double dis;
  static double vx_sum;
  static double vy_sum;
  static double vx_average;
  static double vy_average;
  geometry_msgs::Point point;
  std::vector<geometry_msgs::Point> up_th_d_lambdas;
  geometry_msgs::Point corner_point;
  std::vector<geometry_msgs::Point> corner_points;
  geometry_msgs::Point break_last_point;
  std::vector<geometry_msgs::Point> break_points;
  static geometry_msgs::Point avg_last_point;
  static std::vector<geometry_msgs::Point> avg_lambdas;
  bool group_emtry = true;

  avg_lambdas.clear();
  avg_last_point.x =avg_last_point.y =0;

  for (int j = 0; j < scan_poses.poses.size(); j++)
  {
    v_x.clear(); v_y.clear();
    vx_sum = vy_sum = 0;
    v_x.push_back(scan_poses.poses[j].position.x);
    v_y.push_back(scan_poses.poses[j].position.y);

    //de find point//
    for(int de_count = j; de_count > 0; de_count--) 
    {
      dis = sqrt( pow(scan_poses.poses[j].position.x - scan_poses.poses[de_count].position.x,2) 
                  + pow(scan_poses.poses[j].position.y - scan_poses.poses[de_count].position.y,2) );

      if(dis > window_size_)
      {
        break;
      }
      //cout<<"dis = "<<dis<<'\n';
      v_x.push_back(scan_poses.poses[de_count].position.x);
      v_y.push_back(scan_poses.poses[de_count].position.y);
    }

    //in find point
    for(int in_count = j; in_count < scan_poses.poses.size(); in_count++) 
    {
      dis = sqrt( pow(scan_poses.poses[j].position.x - scan_poses.poses[in_count].position.x,2) 
                  + pow(scan_poses.poses[j].position.y - scan_poses.poses[in_count].position.y,2) );

      if(dis > window_size_)
      {
        break;
      }
        
      v_x.push_back(scan_poses.poses[in_count].position.x);
      v_y.push_back(scan_poses.poses[in_count].position.y);
    }

    for(int s=0; s<v_x.size();s++)
    {
      vx_sum += v_x[s];
      vy_sum += v_y[s];
    }
    // vx_sum = accumulate( v_x.begin(), v_x.end(), 0);
    // vy_sum = accumulate( v_y.begin(), v_y.end(), 0);
    vx_average = vx_sum / v_x.size();
    vy_average = vy_sum / v_y.size();

    //cout<<"vx_sum = "<<vx_sum<<" vy_sum = "<<vy_sum<<'\n';
    // cout<<"vx_average = "<<vx_average<<" vy_average = "<<vy_average<<'\n';

    delta_vx.clear();
    delta_vy.clear();

    Eigen::Vector2d delta_vxy;
    Eigen::Vector2d delta_vxy_sum;
    Eigen::Matrix2d delta_mat;
    Eigen::Matrix2d cov_mat;
    double cov_xx = 0;
    double cov_yy = 0;
    double cov_xy = 0;    
    double dx;
    double dy;
    for(int v = 0; v < v_x.size(); v++)
    {
      dx = v_x[v]-vx_average;
      dy = v_y[v]-vy_average;
      cov_xx += dx*dx;
      cov_xy += dx*dy;
      cov_yy += dy*dy;
    }
    cov_xx /= v_x.size();
    cov_xy /= v_x.size();
    cov_yy /= v_x.size();
    
    cov_mat(0,0) = cov_xx;
    cov_mat(0,1) = cov_xy;
    cov_mat(1,0) = cov_xy;
    cov_mat(1,1) = cov_yy;
    

    // cout<<"delta_vxy "<<delta_vxy<<'\n';
    //cout<<"delta_mat "<<delta_mat<<'\n';
    // cout<<"cov_mat "<<cov_mat<<'\n';
    // Eigen::Matrix2d delta_mat = delta_vxy*delta_vxy.transpose();
    Eigen::EigenSolver<Eigen::MatrixXd> eig(cov_mat);

    //cout<<"eig "<<eig.eigenvalues()<<'\n';
    //std::complex<double> e_1 = eig.eigenvalues()[0];
    double lambda0 = eig.eigenvalues()[0].real();
    double lambda1 = eig.eigenvalues()[1].real();

    //cout<< lambda0 << ", " << lambda1 <<'\n';
    if(lambda0 < lambda1)
    {
      swap(lambda0,lambda1);
    }
    
    double d_lambda = lambda0*lambda1;
    
    
    //cout << "d_lambda " << d_lambda <<'\n';

    if(d_lambda > corner_feature_threshold)
    {
      avg_last_point.x = scan_poses.poses[j].position.x;
      avg_last_point.y = scan_poses.poses[j].position.y;
      avg_lambdas.push_back(avg_last_point);
      group_emtry = false;
      // point.x = scan_poses.poses[j].position.x;
      // point.y = scan_poses.poses[j].position.y;
      // point.z = d_lambda;
      // up_th_d_lambdas.push_back(point);
    }else if(!group_emtry)
    {
      if( sqrt( pow(avg_last_point.x - scan_poses.poses[j].position.x,2) + pow(avg_last_point.y - scan_poses.poses[j].position.y,2)) > window_size_*0.5)
      {
        point.x = avg_lambdas[int (avg_lambdas.size()/2)].x;
        point.y = avg_lambdas[int (avg_lambdas.size()/2)].y;
        //up_th_d_lambdas.push_back(point);
        detect_feature_point.push_back(point);//feature using
        avg_lambdas.clear();
        group_emtry = true;
        //cout << "----------push-up_th_d_lambdas--------------- " <<'\n';
      }
    }
    

    
    //////////break point drtectopn/////////////
    
    if(j == 0)
    {
      //break_points.clear();
      break_last_point.x = scan_poses.poses[j].position.x;
      break_last_point.y = scan_poses.poses[j].position.y;
    }
    else
    {
      if( sqrt( pow(break_last_point.x - scan_poses.poses[j].position.x,2) + pow(break_last_point.y - scan_poses.poses[j].position.y,2)) > window_size_*1.5)
      {
        //break_points.push_back(break_last_point);
        detect_feature_point.push_back(break_last_point);//feature using
        break_last_point.x = scan_poses.poses[j].position.x;
        break_last_point.y = scan_poses.poses[j].position.y;
        //break_points.push_back(break_last_point);
        detect_feature_point.push_back(break_last_point);//feature using
        //cout << "----------push-break_points--------------- " <<'\n';
        //cout << "avg_last_point.x " << avg_last_point.x<< " avg_last_point.y " << avg_last_point.y  <<'\n';
      }
      else
      {
        break_last_point.x = scan_poses.poses[j].position.x;
        break_last_point.y = scan_poses.poses[j].position.y;
      }
      
    }
    

  }
  // VisualizeCorner(up_th_d_lambdas);
  // VisualizeBreakPoint(break_points);
  VisualizefeaturePoint(detect_feature_point);

  FeatureMatching(detect_feature_point);
}

// void PoseCallback(geometry_msgs::PoseArray scan_poses)
// {
//   static std::vector<geometry_msgs::Point> window_points ;
//   static std::vector<double> v_x;
//   static std::vector<double> v_y;
//   static std::vector<double> delta_vx;
//   static std::vector<double> delta_vy;
//   static double dis;
//   static double vx_sum;
//   static double vy_sum;
//   static double vx_average;
//   static double vy_average;
//   geometry_msgs::Point point;
//   std::vector<geometry_msgs::Point> up_th_d_lambdas;
//   geometry_msgs::Point corner_point;
//   std::vector<geometry_msgs::Point> corner_points;
//   geometry_msgs::Point break_last_point;
//   std::vector<geometry_msgs::Point> break_points;

//   for (int j = 0; j < scan_poses.poses.size(); j++)
//   {
//     v_x.clear(); v_y.clear();
//     vx_sum = vy_sum = 0;
//     v_x.push_back(scan_poses.poses[j].position.x);
//     v_y.push_back(scan_poses.poses[j].position.y);

//     //de find point//
//     for(int de_count = j; de_count > 0; de_count--) 
//     {
//       dis = sqrt( pow(scan_poses.poses[j].position.x - scan_poses.poses[de_count].position.x,2) 
//                   + pow(scan_poses.poses[j].position.y - scan_poses.poses[de_count].position.y,2) );

//       if(dis > window_size_)
//       {
//         break;
//       }
//       //cout<<"dis = "<<dis<<'\n';
//       v_x.push_back(scan_poses.poses[de_count].position.x);
//       v_y.push_back(scan_poses.poses[de_count].position.y);
//     }

//     //in find point
//     for(int in_count = j; in_count < scan_poses.poses.size(); in_count++) 
//     {
//       dis = sqrt( pow(scan_poses.poses[j].position.x - scan_poses.poses[in_count].position.x,2) 
//                   + pow(scan_poses.poses[j].position.y - scan_poses.poses[in_count].position.y,2) );

//       if(dis > window_size_)
//       {
//         break;
//       }
        
//       v_x.push_back(scan_poses.poses[in_count].position.x);
//       v_y.push_back(scan_poses.poses[in_count].position.y);
//     }

//     for(int s=0; s<v_x.size();s++)
//     {
//       vx_sum += v_x[s];
//       vy_sum += v_y[s];
//     }
//     // vx_sum = accumulate( v_x.begin(), v_x.end(), 0);
//     // vy_sum = accumulate( v_y.begin(), v_y.end(), 0);
//     vx_average = vx_sum / v_x.size();
//     vy_average = vy_sum / v_y.size();

//     //cout<<"vx_sum = "<<vx_sum<<" vy_sum = "<<vy_sum<<'\n';
//     // cout<<"vx_average = "<<vx_average<<" vy_average = "<<vy_average<<'\n';

//     delta_vx.clear();
//     delta_vy.clear();

//     Eigen::Vector2d delta_vxy;
//     Eigen::Vector2d delta_vxy_sum;
//     Eigen::Matrix2d delta_mat;
//     Eigen::Matrix2d cov_mat;
//     double cov_xx = 0;
//     double cov_yy = 0;
//     double cov_xy = 0;    
//     double dx;
//     double dy;
//     for(int v = 0; v < v_x.size(); v++)
//     {
//       dx = v_x[v]-vx_average;
//       dy = v_y[v]-vy_average;
//       cov_xx += dx*dx;
//       cov_xy += dx*dy;
//       cov_yy += dy*dy;
//     }
//     cov_xx /= v_x.size();
//     cov_xy /= v_x.size();
//     cov_yy /= v_x.size();
    
//     cov_mat(0,0) = cov_xx;
//     cov_mat(0,1) = cov_xy;
//     cov_mat(1,0) = cov_xy;
//     cov_mat(1,1) = cov_yy;
    

//     // cout<<"delta_vxy "<<delta_vxy<<'\n';
//     //cout<<"delta_mat "<<delta_mat<<'\n';
//     // cout<<"cov_mat "<<cov_mat<<'\n';
//     // Eigen::Matrix2d delta_mat = delta_vxy*delta_vxy.transpose();
//     Eigen::EigenSolver<Eigen::MatrixXd> eig(cov_mat);

//     //cout<<"eig "<<eig.eigenvalues()<<'\n';
//     //std::complex<double> e_1 = eig.eigenvalues()[0];
//     double lambda0 = eig.eigenvalues()[0].real();
//     double lambda1 = eig.eigenvalues()[1].real();

//     //cout<< lambda0 << ", " << lambda1 <<'\n';
//     if(lambda0 < lambda1)
//     {
//       swap(lambda0,lambda1);
//     }
//     static geometry_msgs::Point avg_last_point;
//     static std::vector<geometry_msgs::Point> avg_lambdas;
//     double d_lambda = lambda0*lambda1;
//     static bool group_emtry = true;
    
//     //cout << "d_lambda " << d_lambda <<'\n';

//     if(d_lambda > corner_feature_threshold)
//     {
//       if(group_emtry)
//       {
//         avg_last_point.x = scan_poses.poses[j].position.x;
//         avg_last_point.y = scan_poses.poses[j].position.y;
//         avg_lambdas.push_back(avg_last_point);

//         group_emtry = false;
//       }else
//       {
//         if( sqrt( pow(avg_last_point.x - scan_poses.poses[j].position.x,2) + pow(avg_last_point.y - scan_poses.poses[j].position.y,2)) > window_size_*0.7)
//         {
//           point.x = avg_lambdas[int (avg_lambdas.size()/2)].x;
//           point.y = avg_lambdas[int (avg_lambdas.size()/2)].y;
//           up_th_d_lambdas.push_back(point);
//           detect_feature_point.push_back(point);//feature using
//           avg_lambdas.clear();
//           group_emtry = true;
//         }
//         else
//         {
//           avg_last_point.x = scan_poses.poses[j].position.x;
//           avg_last_point.y = scan_poses.poses[j].position.y;
//           avg_lambdas.push_back(avg_last_point);
//         }
        
        
//       }
      
//       // point.x = scan_poses.poses[j].position.x;
//       // point.y = scan_poses.poses[j].position.y;
//       // point.z = d_lambda;
//       // up_th_d_lambdas.push_back(point);
//     }
//     //////////break point drtectopn/////////////
    
//     if(j == 0)
//     {
//       //break_points.clear();
//       break_last_point.x = scan_poses.poses[j].position.x;
//       break_last_point.y = scan_poses.poses[j].position.y;
//     }
//     else
//     {
//       if( sqrt( pow(break_last_point.x - scan_poses.poses[j].position.x,2) + pow(break_last_point.y - scan_poses.poses[j].position.y,2)) > window_size_*1.5)
//       {
//         break_points.push_back(break_last_point);
//         detect_feature_point.push_back(break_last_point);//feature using
//         break_last_point.x = scan_poses.poses[j].position.x;
//         break_last_point.y = scan_poses.poses[j].position.y;
//         break_points.push_back(break_last_point);
//         detect_feature_point.push_back(break_last_point);//feature using
//       }
//       else
//       {
//         break_last_point.x = scan_poses.poses[j].position.x;
//         break_last_point.y = scan_poses.poses[j].position.y;
//       }
      
//     }
    

//   }
//   VisualizeCorner(up_th_d_lambdas);
//   VisualizeBreakPoint(break_points);
//   //VisualizefeaturePoint(detect_feature_point);
// }
//-----------------------------------------------------------------------------------------------
void FeatureMatching(std::vector<geometry_msgs::Point> points)
{
  static double last_point_x,last_point_y;
  static geometry_msgs::Point find_feature_point;
  static std::vector<geometry_msgs::Point> enter_feature_points;
  static std::vector<geometry_msgs::Point> exit_feature_points;
  static double dis, angle, dangle;
  static double last_angle;
  static bool feature_1, feature_2, feature_3;
  static geometry_msgs::PoseStamped feature_pose;
  static nav_msgs::Path path;
  static std::vector<nav_msgs::Path> paths;
  static std::vector<bool> features;
  static geometry_msgs::Pose elevator_pose;
  static geometry_msgs::PoseArray elevator_poses;
  static tf2::Quaternion elevator_pose_q_tf;
  static geometry_msgs::Quaternion elevator_pose_q_msg;

  paths.clear();
  path.poses.clear();
  enter_feature_points.clear();
  exit_feature_points.clear();
  elevator_poses.poses.clear();
  elevator_poses.header.frame_id = scan_frame_;
  elevator_poses.header.stamp =ros::Time::now();

  //std::cout<<"points_size"<<points.size()<<'\n';

  if(points.size()<3)
    return;
  

  for(int i = 0; i < points.size();i++)
  {
    
    dis = angle = 0;
    for(int f = i; f < i+3;f++)
    {
      if(f == points.size()-1)
      {
        break;
      }
      if(f == i)
      {
        last_point_x = points[f].x;
        last_point_y = points[f].y;
        last_angle = 0.0;
      }else
      {
        dis = sqrt(pow(last_point_x - points[f].x,2)+pow(last_point_y - points[f].y,2));
        angle = atan2(last_point_y - points[f].y, last_point_x - points[f].x);
        dangle = last_angle - angle;
        last_point_x = points[f].x;
        last_point_y = points[f].y;
        last_angle = angle;

        if(dangle > M_PI)
        {
          dangle = -2*M_PI + dangle;
        }else if(dangle < -M_PI)
        {
          dangle = 2*M_PI + dangle;
        }
      } 

      if(f == i+2)
      {
        if(abs(dangle - enter_elevator_feature[2].z ) <= enter_elevator_feature_d_angle_range_)
        {
          find_feature_point.x = points[i+1].x;
          find_feature_point.y = points[i+1].y;
          find_feature_point.z = i+1;
          enter_feature_points.push_back(find_feature_point);
        }

        if(abs(dangle - exit_elevator_feature[2].z ) <= enter_elevator_feature_d_angle_range_)
        {
          find_feature_point.x = points[i+1].x;
          find_feature_point.y = points[i+1].y;
          find_feature_point.z = i+1;
          exit_feature_points.push_back(find_feature_point);
        }
        
      }
      //std::cout << "dis : " << dis << " dangle " <<  dangle <<'\n';
    }
    // std::cout<<"---------------------------------"<<'\n';
  }
  VisualizeCorner(exit_feature_points);

  if(enter_feature_points.size()<2)
    return;
  
  for(int f_1 = 0;f_1 < enter_feature_points.size()-1;f_1++)
  {
    for(int f_2 = f_1+1;f_2 < enter_feature_points.size();f_2++)
    { 
      if(abs(sqrt(pow(enter_feature_points[f_1].x -enter_feature_points[f_2].x,2)+
                  pow(enter_feature_points[f_1].y -enter_feature_points[f_2].y,2))-door_y1_) <= enter_elevator_feature_dis_range_)
      {

        double angle1 = atan2(points[enter_feature_points[f_1].z].y - points[enter_feature_points[f_1].z+1].y, 
                              points[enter_feature_points[f_1].z].x - points[enter_feature_points[f_1].z+1].x);

        double angle2 = atan2(points[enter_feature_points[f_2].z-1].y - points[enter_feature_points[f_2].z].y, 
                              points[enter_feature_points[f_2].z-1].x - points[enter_feature_points[f_2].z].x);
        dangle = angle1 - angle2;
        if(dangle > M_PI)
        {
          dangle = -2*M_PI + dangle;
        }else if(dangle < -M_PI)
        {
          dangle = 2*M_PI + dangle;
        }
        //std::cout << " dangle " <<  dangle <<'\n';
        if(abs(dangle -enter_elevator_feature[6].z) <= enter_elevator_feature_d_angle_range_)
        {
          for(int p = 0; p < 2;p++)
          {
            if(p == 0)
            {
              feature_pose.pose.position.x = enter_feature_points[f_1].x;
              feature_pose.pose.position.y = enter_feature_points[f_1].y;
            }
            else if(p == 1)
            {
              feature_pose.pose.position.x = enter_feature_points[f_2].x;
              feature_pose.pose.position.y = enter_feature_points[f_2].y;
            }
            
            path.poses.push_back(feature_pose);
          }
          paths.push_back(path);
          path.poses.clear();

          elevator_pose.position.x = (enter_feature_points[f_1].x +enter_feature_points[f_2].x)/2.0;
          elevator_pose.position.y = (enter_feature_points[f_1].y +enter_feature_points[f_2].y)/2.0;
          double yaw = atan2(enter_feature_points[f_1].y - enter_feature_points[f_2].y, 
                              enter_feature_points[f_1].x - enter_feature_points[f_2].x) + M_PI/2.0;
          elevator_pose_q_tf.setRPY(0.0,0.0,yaw);
          elevator_pose_q_msg = tf2::toMsg(elevator_pose_q_tf);
          elevator_pose.orientation = elevator_pose_q_msg;

          elevator_poses.poses.push_back(elevator_pose);
        }


        
        
      }
    }
  }

  //std::cout << " elevator_num: " <<  paths.size() <<'\n';
  //VisualizeElevatorEnterPoint(paths);

  if(elevator_pose_filter_)
  {
    pose_filter(elevator_poses);
  }
 
  pose_pub_.publish(elevator_poses);
  
  
  //std::cout<<"=================================="<<'\n';


}
//-----------------------------------------------------------------------------------------------
void pose_filter(geometry_msgs::PoseArray &filter_poses)
{
  static bool first_time = true;
  static geometry_msgs::PoseStamped before_pose;
  static geometry_msgs::PoseStamped map_pose;
  static geometry_msgs::Pose pose;
  static std::vector<geometry_msgs::PoseStamped> poses;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static double m_roll, m_pitch, m_yaw;
  static double roll, pitch, yaw;
  static bool matching;

  if(first_time)
  {
    for(int i =0;i<filter_poses.poses.size();i++)
    {
      before_pose.header.frame_id = filter_poses.header.frame_id;
      before_pose.pose.position = filter_poses.poses[i].position;
      before_pose.pose.orientation = filter_poses.poses[i].orientation;

      if(filter_poses.header.frame_id != map_frame_)
      {
        try
        {
          tfBuffer.transform(before_pose, map_pose, map_frame_);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("elevator point finder : %s",ex.what());
          ros::Duration(0.5).sleep();
          return;
        }
      }
      else
      {
        map_pose.header.frame_id = filter_poses.header.frame_id;
        map_pose.pose = before_pose.pose;
      }

      map_pose.pose.position.z = 2.0;
      poses.push_back(map_pose);
    }
    first_time = false;
  }
  else
  {
    matching = false;

    for(int i =0;i<filter_poses.poses.size();i++)
    {
      before_pose.header.frame_id = filter_poses.header.frame_id;
      before_pose.pose.position = filter_poses.poses[i].position;
      before_pose.pose.orientation = filter_poses.poses[i].orientation;

      if(filter_poses.header.frame_id != map_frame_)
      {
        try
        {
          tfBuffer.transform(before_pose, map_pose, map_frame_);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("elevator point finder2 : %s",ex.what());
          ros::Duration(0.5).sleep();
          return;
        }
      }
      else
      {
        map_pose.header.frame_id = filter_poses.header.frame_id;
        map_pose.pose = before_pose.pose;
      }
      tf::Quaternion q( map_pose.pose.orientation.x,
                        map_pose.pose.orientation.y,
                        map_pose.pose.orientation.z,
                        map_pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      
      m.getRPY(m_roll, m_pitch, m_yaw);

      for(int j= 0;j<poses.size();j++)
      {
        tf::Quaternion q2( poses[j].pose.orientation.x,
                          poses[j].pose.orientation.y,
                          poses[j].pose.orientation.z,
                          poses[j].pose.orientation.w);
        tf::Matrix3x3 m2(q2);
        m2.getRPY(roll, pitch, yaw);
        
        if(abs(sqrt(pow(map_pose.pose.position.x - poses[j].pose.position.x,2)+
                    pow(map_pose.pose.position.y - poses[j].pose.position.y,2)))< filter_range_dis_
                    && abs(m_yaw - yaw)< filter_range_angle_)
        {
          poses[j].pose.position.x = map_pose.pose.position.x;
          poses[j].pose.position.y = map_pose.pose.position.y;
          if(poses[j].pose.position.z <20.0)
          {
            poses[j].pose.position.z = poses[j].pose.position.z+2;
            
          }
          matching =true;
        }else
        {
          poses[j].pose.position.z--;
        
          // std::cout << " dis: " <<  sqrt(pow(map_pose.pose.position.x - poses[j].pose.position.x,2)+
          //           pow(map_pose.pose.position.y - poses[j].pose.position.y,2)) <<'\n';
          // std::cout << " abgle: " <<  abs(m_yaw - yaw) <<'\n';
          // std::cout << " --------------- " << '\n';
        }
      }
      if(!matching)
      {
        //std::cout << " add new pose!! " << '\n';
        map_pose.pose.position.z = 2.0;
        poses.push_back(map_pose);
      }
    }
    //std::cout << " =============== " << '\n';
  }

  filter_poses.poses.clear();
  filter_poses.header.frame_id = map_frame_;
  for(int k =0;k<poses.size();k++)
  {
    if(poses[k].pose.position.z > 5.0)
    {
      pose.position.x = poses[k].pose.position.x;
      pose.position.y = poses[k].pose.position.y;
      pose.position.z = poses[k].pose.position.z;
      pose.orientation = poses[k].pose.orientation;
      filter_poses.poses.push_back(pose);
    }
    if(poses[k].pose.position.z <= 0)
    {
      poses.erase(poses.begin()+k);
      //break;
    }
    
  }
  
  
}
//-----------------------------------------------------------------------------------------------
void VisualizeCorner(std::vector<geometry_msgs::Point> points) 
{
  static visualization_msgs::MarkerArray point_markers;
  static visualization_msgs::Marker point_marker;

  point_markers.markers.clear();

  if(points.size() == 0)
    return;

  point_marker.header.frame_id = scan_frame_;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "corner_position_Orignal";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.position.z = 0;
  point_marker.pose.orientation.x = 0.0;
  point_marker.pose.orientation.y = 0.0;
  point_marker.pose.orientation.z = 0.0;
  point_marker.pose.orientation.w = 1.0;
  point_marker.scale.y = window_size_;
  point_marker.scale.x = window_size_;
  point_marker.scale.z = 0.01;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.color.a = 0.5f;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 0.0f;

  for(int i=0; i < points.size(); i++)
  {
    point_marker.id = i;
    point_marker.pose.position.x = points[i].x;
    point_marker.pose.position.y = points[i].y;
    point_markers.markers.push_back(point_marker);
  }
  

  elevator_corner_marker_pub_.publish(point_markers);
}
//-----------------------------------------------------------------------------------------------
void VisualizeBreakPoint(std::vector<geometry_msgs::Point> points) 
{
  static visualization_msgs::MarkerArray point_markers;
  static visualization_msgs::Marker point_marker;

  point_markers.markers.clear();

  if(points.size() == 0)
    return;

  point_marker.header.frame_id = scan_frame_;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "breakpoint_position_Orignal";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.position.z = 0;
  point_marker.pose.orientation.x = 0.0;
  point_marker.pose.orientation.y = 0.0;
  point_marker.pose.orientation.z = 0.0;
  point_marker.pose.orientation.w = 1.0;
  point_marker.scale.y = window_size_;
  point_marker.scale.x = window_size_;
  point_marker.scale.z = 0.01;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.color.a = 0.5f;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;

  for(int i=0; i < points.size(); i++)
  {
    point_marker.id = i;
    point_marker.pose.position.x = points[i].x;
    point_marker.pose.position.y = points[i].y;
    point_markers.markers.push_back(point_marker);
  }
  

  break_point_marker_pub_.publish(point_markers);
}
//-----------------------------------------------------------------------------------------------
void VisualizefeaturePoint(std::vector<geometry_msgs::Point> points) 
{
  static visualization_msgs::MarkerArray point_markers;
  static visualization_msgs::Marker point_marker;

  point_markers.markers.clear();

  if(points.size() == 0)
    return;

  point_marker.header.frame_id = scan_frame_;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "breakpoint_position_Orignal";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.position.z = 0;
  point_marker.pose.orientation.x = 0.0;
  point_marker.pose.orientation.y = 0.0;
  point_marker.pose.orientation.z = 0.0;
  point_marker.pose.orientation.w = 1.0;
  point_marker.scale.y = window_size_;
  point_marker.scale.x = window_size_;
  point_marker.scale.z = 0.01;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.color.a = 0.5f;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 1.0f;

  for(int i=0; i < points.size(); i++)
  {
    point_marker.id = i;
    point_marker.pose.position.x = points[i].x;
    point_marker.pose.position.y = points[i].y;
    point_markers.markers.push_back(point_marker);
  }
  

  featuer_point_marker_pub_.publish(point_markers);
}

//-----------------------------------------------------------------------------------------------
void VisualizeElevatorEnterPoint(std::vector<nav_msgs::Path> paths) 
{
  static visualization_msgs::MarkerArray all_rollOuts;
  static visualization_msgs::Marker lane_waypoint_marker;
  static geometry_msgs::Point point;
  lane_waypoint_marker.header.frame_id = scan_frame_;
  lane_waypoint_marker.header.stamp = ros::Time(0);
  lane_waypoint_marker.ns = "elevator_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.05;
  lane_waypoint_marker.scale.y = 0.05;
  lane_waypoint_marker.frame_locked = false;
  static int max_count = 0;
  all_rollOuts.markers.clear();
  if(paths.size() > max_count)
  {
    max_count = paths.size();
  }
  
  for (int i = 0; i < paths.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    //if(i == 0)
    {
      lane_waypoint_marker.id = i;

      lane_waypoint_marker.color.r = 0.0;
      lane_waypoint_marker.color.g = 0.0;
      lane_waypoint_marker.color.b = 1.0;
      lane_waypoint_marker.color.a = 0.9;
    }
    // else if(i ==1)
    // {
    //   lane_waypoint_marker.id = i;

    //   lane_waypoint_marker.color.r = 1.0;
    //   lane_waypoint_marker.color.g = 0.0;
    //   lane_waypoint_marker.color.b = 0.0;
    //   lane_waypoint_marker.color.a = 0.9;
    // }
    // else if(i ==2)
    // {
    //   lane_waypoint_marker.id = i;

    //   lane_waypoint_marker.color.r = 0.0;
    //   lane_waypoint_marker.color.g = 0.0;
    //   lane_waypoint_marker.color.b = 1.0;
    //   lane_waypoint_marker.color.a = 0.9;
    // }

    for (int j = 0; j < paths[i].poses.size(); j++)
    {
        point.x = paths[i].poses[j].pose.position.x;
        point.y = paths[i].poses[j].pose.position.y;
        point.z = paths[i].poses[j].pose.position.z;

      lane_waypoint_marker.points.push_back(point);
    }
  
    all_rollOuts.markers.push_back(lane_waypoint_marker);
  }
  if(paths.size()<max_count)
  {
    for(int f = 0;f <max_count-paths.size();f++)
    {
      lane_waypoint_marker.id = all_rollOuts.markers.size()+f;
      lane_waypoint_marker.color.a = 0.01;
      //lane_waypoint_marker.points.push_back(point);
      all_rollOuts.markers.push_back(lane_waypoint_marker);
    }
  }

  elevator_marker_pub_.publish(all_rollOuts);

}

//-----------------------------------------------------------------------------------------------//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_position_finder");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  get_parameters(nh_private);
  ros::Time::init();

  scan_sub_ = nh.subscribe("scan", 10, ScanCallback);
  //pose_sub_ = nh.subscribe("pose_topic", 10, PoseCallback);

  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("elevator_poses", 0.1);
  elevator_corner_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("elevator_corner_marker", 50);
  break_point_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("break_point_marker", 50);
  featuer_point_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("feature_point_marker", 50);
  elevator_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("elevator_marker", 10);

  ros::spin();

  return 0;
}
