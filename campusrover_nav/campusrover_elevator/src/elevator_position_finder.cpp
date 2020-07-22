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

#include <campusrover_msgs/ElevatorControlStatus.h>
#include <campusrover_msgs/ElevatorStatusChecker.h>

#include <Eigen/Dense>

using namespace std;

#define M_PI 3.14159265358979323846  /* pi */

ros::Subscriber scan_sub_, scan2_sub_, control_status_sub_;
ros::Publisher pose_pub_, break_point_marker_pub_, elevator_corner_marker_pub_, featuer_point_marker_pub_, elevator_marker_pub_;
ros::ServiceClient status_check_client_;

string status_;
string scan_frame_;
string map_frame_;
string window_type_;

double max_scan_range_;
double corner_feature_threshold;
double window_size_;

double enter_feature_point_neighborhood_dis_;
double enter_break_point_neighborhood_dis_;
double enter_window_size_;
double enter_corner_feature_threshold_;
double exit_feature_point_neighborhood_dis_;
double exit_break_point_neighborhood_dis_;
double exit_window_size_;
double exit_corner_feature_threshold_;

double elevator_feature_dis_range_;
double elevator_feature_d_angle_range_;
int elevator_num_;
double door_gap_dis_;
double enter_door_x1_;
double enter_door_y1_;
double enter_door_y2_;

double exit_door_x1_;
double exit_door_y1_;
double exit_door_y2_;

double elevator_wd_;
double filter_range_dis_;
double filter_range_angle_;

double control_status_;

double feature_point_neighborhood_dis_;
double break_point_neighborhood_dis_;

double shift_x_,shift_y_;

bool elevator_pose_filter_;
bool avg_pose_filter_;
bool enable_mode_ = false;
bool enter_done_ = false;
bool exit_done_ = false;
bool position_find_done_ = false;

bool pose_filter_first_time_ = true;
bool matching_success_;

int sub_mode_;

int avg_pose_size_ ;

std::vector<geometry_msgs::Point> detect_feature_point;
std::vector<geometry_msgs::Point> enter_elevator_feature;
std::vector<geometry_msgs::Point> exit_elevator_feature;
std::vector<geometry_msgs::Point> sample_elevator_feature;

geometry_msgs::PoseArray hold_poses_;
geometry_msgs::PoseArray prosses_poses_;

void VisualizeCorner(std::vector<geometry_msgs::Point> points);
void VisualizeBreakPoint(std::vector<geometry_msgs::Point> points);
void VisualizefeaturePoint(std::vector<geometry_msgs::Point> points) ;
void FeatureMatching(std::vector<geometry_msgs::Point> points);
void VisualizeElevatorEnterPoint(std::vector<nav_msgs::Path> paths);
void initial_elevator_feature();
void pose_filter(geometry_msgs::PoseArray &poses);
void FeatureExtract(geometry_msgs::PoseArray &scan_poses);
void avg_pose(geometry_msgs::Pose &input_pose);
void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv);

void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<string>("map_frame", map_frame_, "map");
  n_private.param<string>("window_type", window_type_, "circular");
  n_private.param<bool>("elevator_pose_filter", elevator_pose_filter_, true); 
  n_private.param<bool>("elevator_avg_pose_filter", avg_pose_filter_, true); 
  n_private.param<double>("max_scan_range", max_scan_range_, 4.0);
  n_private.param<int>("avg_pose_size", avg_pose_size_, 8.0);

  n_private.param<double>("enter_feature_point_neighborhood_dis", enter_feature_point_neighborhood_dis_, 0.065);
  n_private.param<double>("enter_break_point_neighborhood_dis", enter_break_point_neighborhood_dis_, 0.25);
  n_private.param<double>("enter_window_size", enter_window_size_, 0.15);
  n_private.param<double>("enter_corner_feature_threshold", enter_corner_feature_threshold_, 0.0000025);

  n_private.param<double>("exit_feature_point_neighborhood_dis", exit_feature_point_neighborhood_dis_, 0.02);
  n_private.param<double>("exit_break_point_neighborhood_dis", exit_break_point_neighborhood_dis_, 0.05);
  n_private.param<double>("exit_window_size", exit_window_size_, 0.05);
  n_private.param<double>("exit_corner_feature_threshold", exit_corner_feature_threshold_, 0.00000005);

  n_private.param<double>("elevator_feature_dis_range", elevator_feature_dis_range_, 0.1);
  n_private.param<double>("elevator_feature_d_angle_range", elevator_feature_d_angle_range_, 0.1);

  n_private.param<double>("filter_range_dis", filter_range_dis_, 0.1);
  n_private.param<double>("filter_range_angle", filter_range_angle_, 0.3);

  n_private.param<int>("elevator_num", elevator_num_, 2.0);
  n_private.param<double>("door_gap_dis", door_gap_dis_, 0.91);

  n_private.param<double>("enter_door_x1", enter_door_x1_, 0.34);
  n_private.param<double>("enter_door_y1", enter_door_y1_, 0.94);
  n_private.param<double>("enter_door_y2", enter_door_y2_, 0.8);
  
  n_private.param<double>("exit_door_x1", exit_door_x1_, 0.34);
  n_private.param<double>("exit_door_y1", exit_door_y1_, 0.94);
  n_private.param<double>("exit_door_y2", exit_door_y2_, 0.8);
  n_private.param<double>("elevator_wd", elevator_wd_, 0.8);

  n_private.param<double>("shift_x", shift_x_, 0.4);
  n_private.param<double>("shift_y", shift_y_, 0.0);

  n_private.param<int>("sub_mode", sub_mode_, 2.0);

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
      vy = -enter_door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 2)
    {
      vx = -(enter_door_x1_*cos(asin(((enter_door_y1_- enter_door_y2_)/2.0)/enter_door_x1_)));
      vy = -((enter_door_y1_- enter_door_y2_)/2.0);
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle2 = angle;
    }
    else if(i == 3)
    {
      vx = 0.0;
      vy = -enter_door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      

    }else if(i == 4)
    {
      vx = enter_door_x1_*cos(asin(((enter_door_y1_-enter_door_y2_)/2.0)/enter_door_x1_));
      vy = -((enter_door_y1_-enter_door_y2_)/2.0);
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle3 = angle;

    }else if(i == 5)
    {
      vx = 0.0;
      vy = -enter_door_y2_;
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
      vy = -exit_door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
    }
    else if(i == 2)
    {
      vx = -exit_door_x1_;
      vy = 0.0;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle2 = angle;
    }
    else if(i == 3)
    {
      vx = 0.0;
      vy = -exit_door_y2_;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      

    }else if(i == 4)
    {
      vx = exit_door_x1_;
      vy = 0.0;
      dis = sqrt(pow(vx,2)+pow(vy,2));
      angle = atan2(vy,vx);
      angle3 = angle;

    }else if(i == 5)
    {
      vx = 0.0;
      vy = -exit_door_y2_;
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

  std::vector<double> range;
  std::vector<double> intensities;

  if(sub_mode_ != 1)
    return;

  if(!enable_mode_)
  {
    pose_pub_.publish(prosses_poses_);
    return;
  }

  if(scan->ranges.size()==0)
    return;
  
  scan_poses.header.stamp = ros::Time::now();
  scan_frame_ = scan->header.frame_id;
  scan_poses.header.frame_id = scan_frame_;
  scan_poses.poses.clear();
  

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
  matching_success_ = true;
  FeatureExtract(scan_poses);
  if(!matching_success_)
  {
    pose_pub_.publish(hold_poses_);
  }
}
//---------------------Scan2-------------------------//
void Scan2Callback(const sensor_msgs::LaserScanConstPtr &scan)
{
  static geometry_msgs::Pose pose;
  static geometry_msgs::PoseArray scan_poses;
  static double angle;

  std::vector<double> range;
  std::vector<double> intensities;

  if(sub_mode_ != 2)
    return;

  if(!enable_mode_)
  {
    pose_pub_.publish(prosses_poses_);
    return;
  }

  if(scan->ranges.size()==0)
    return;
  
  scan_poses.header.stamp = ros::Time::now();
  scan_frame_ = scan->header.frame_id;
  scan_poses.header.frame_id = scan_frame_;
  scan_poses.poses.clear();
  

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
  matching_success_ = true;
  FeatureExtract(scan_poses);
  if(!matching_success_)
  {
    pose_pub_.publish(hold_poses_);
  }
}
//--------------------------------------------------------------------

void ControlStatusCallback(const campusrover_msgs::ElevatorControlStatusConstPtr &con_status)
{
  geometry_msgs::Pose prosses_pose;
  
  static campusrover_msgs::ElevatorStatusChecker status_msg;

  

  control_status_ = con_status->control_status;

  if(control_status_ == 1 && !position_find_done_ && !enter_done_)
  {
    enable_mode_ = true;
    sub_mode_ = 2.0; //PointCloud to LaserScan
    feature_point_neighborhood_dis_ = enter_feature_point_neighborhood_dis_; //
    break_point_neighborhood_dis_ = enter_break_point_neighborhood_dis_; //
    window_size_ = enter_window_size_; //
    corner_feature_threshold = enter_corner_feature_threshold_; //
    status_ = "enter";
    exit_done_ = false;
    if(hold_poses_.poses.size() != 0)
    {
      if(hold_poses_.poses[0].position.z >= 30.0 )
      {
        status_msg.request.node_name.data = "position_finder";
        status_msg.request.status.data = true;
        // StatusCheckCallService(status_check_client_, status_msg);
        position_find_done_ = true;
      }
    }
    
  }
  else if(control_status_ == 1 && position_find_done_ && !enter_done_)
  {
    enable_mode_ = false;
    prosses_pose = hold_poses_.poses[0];
    prosses_poses_.poses.push_back(prosses_pose);
    
  }
  else if(control_status_ == 5  && !enter_done_)
  {
    enable_mode_ = true;
    sub_mode_ = 2.0; //PointCloud to LaserScan
    feature_point_neighborhood_dis_ = enter_feature_point_neighborhood_dis_; //
    break_point_neighborhood_dis_ = enter_break_point_neighborhood_dis_; //
    window_size_ = enter_window_size_; //
    corner_feature_threshold = enter_corner_feature_threshold_; //
    status_ = "enter";
    exit_done_ = false;
  }

  else if(control_status_ > 1 && control_status_ < 6 && !enter_done_)
  {
    enable_mode_ = false;
    prosses_pose = hold_poses_.poses[0];
    prosses_poses_.poses.push_back(prosses_pose);
    position_find_done_ = false;
  }
  else if(control_status_ > 0 && control_status_ < 6 && enter_done_)
  {
    enable_mode_ = false;
    prosses_pose = hold_poses_.poses[0];
    prosses_pose.position.z = -1;
    prosses_poses_.poses.push_back(prosses_pose);
  }
  else if (control_status_ > 5 && control_status_ < 12 && !exit_done_)
  {
    status_ = "exit";
    enable_mode_ = false;
    enter_done_ = false;

    double roll, pitch, yaw;

    tf::Quaternion q( hold_poses_.poses[0].orientation.x,
                      hold_poses_.poses[0].orientation.y,
                      hold_poses_.poses[0].orientation.z,
                      hold_poses_.poses[0].orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);

    tf2::Quaternion elevator_pose_q_tf;
    geometry_msgs::Quaternion elevator_pose_q_msg;

    elevator_pose_q_tf.setRPY(0.0,0.0,yaw+M_PI);
    elevator_pose_q_msg = tf2::toMsg(elevator_pose_q_tf);

    prosses_pose.position = hold_poses_.poses[0].position;
    prosses_pose.orientation = elevator_pose_q_msg;

    prosses_poses_.poses.push_back(prosses_pose);
  
    
  //   enable_mode_ = true;
  //   sub_mode_ = 1.0; //LaserScan
  //   feature_point_neighborhood_dis_ = exit_feature_point_neighborhood_dis_; //
  //   break_point_neighborhood_dis_ = exit_break_point_neighborhood_dis_; //
  //   window_size_ = exit_window_size_; //
  //   corner_feature_threshold= exit_corner_feature_threshold_; //
  }
  else
  {
    enable_mode_ = false;
    pose_filter_first_time_ = true;
  }

  // if(enter_done_ || exit_done_)
  // {
  //   enable_mode_ = false;
  // }
  
  
}

//--------------------------------------------------------------------
void FeatureExtract(geometry_msgs::PoseArray &scan_poses)
{
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

  detect_feature_point.clear();
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
      if( sqrt( pow(avg_last_point.x - scan_poses.poses[j].position.x,2) + 
                pow(avg_last_point.y - scan_poses.poses[j].position.y,2)) > feature_point_neighborhood_dis_) //feature_point_neighborhood_dis_
      {
        point.x = avg_lambdas[int (avg_lambdas.size()/2)].x;
        point.y = avg_lambdas[int (avg_lambdas.size()/2)].y;
        up_th_d_lambdas.push_back(point);
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
      if( sqrt( pow(break_last_point.x - scan_poses.poses[j].position.x,2) + 
                pow(break_last_point.y - scan_poses.poses[j].position.y,2)) > break_point_neighborhood_dis_)//window_size_*1.5 break_point_neighborhood_dis_
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
  //VisualizeCorner(up_th_d_lambdas);
  // VisualizeBreakPoint(break_points);
  VisualizefeaturePoint(detect_feature_point);

  FeatureMatching(detect_feature_point);
}

// void PoseCallback(geometry_msgs::PoseArray scan_poses)
// {

// }
//-----------------------------------------------------------------------------------------------
void FeatureMatching(std::vector<geometry_msgs::Point> points)
{
  static double last_point_x,last_point_y;
  static geometry_msgs::Point find_feature_point;
  static std::vector<geometry_msgs::Point> elevator_feature_points;
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
  static double angle1, angle2;
  static bool find_shortest_dis_first_time = true;
  static double shortest_dis, s_dis;
  static double dangle_featrue;
  static double door_corner_dis;
  static double door_corner_dangle;
  static double error_dangle;

  paths.clear();
  path.poses.clear();
  elevator_feature_points.clear();
  elevator_poses.poses.clear();
  elevator_poses.header.frame_id = scan_frame_;
  elevator_poses.header.stamp =ros::Time::now();

  //std::cout<<"points_size"<<points.size()<<'\n';

  if(points.size()<3)
  {
    matching_success_ = false;
    return;
  }
    
  
  if(status_ == "enter")
  {
    dangle_featrue = enter_elevator_feature[2].z;
    door_corner_dis = enter_door_y1_;
    door_corner_dangle =  enter_elevator_feature[6].z;
  }
  else if(status_ == "exit")
  {
    dangle_featrue = exit_elevator_feature[2].z;
    door_corner_dis = exit_door_y1_;
    door_corner_dangle = exit_elevator_feature[6].z;
  }

  for(int i = 0; i < points.size()-2;i++)
  {
    
    dis = angle = dangle =0;
    for(int f = i; f < i+3;f++)
    {
      // if(f >= points.size()-3)
      // {
      //   break;
      // }
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

        error_dangle = dangle - dangle_featrue;

        if(error_dangle > M_PI)
        {
          error_dangle = -2*M_PI + error_dangle;
        }
        else if(error_dangle < -M_PI)
        {
          error_dangle = 2*M_PI + error_dangle;
        }

        if(abs(error_dangle) <= elevator_feature_d_angle_range_)
        //if(abs(dangle - dangle_featrue ) <= 0.5)
        {
          find_feature_point.x = points[i+1].x;
          find_feature_point.y = points[i+1].y;
          find_feature_point.z = i+1;
          elevator_feature_points.push_back(find_feature_point);
        }
        
      }
      
    }
    // std::cout << " dangle " <<  dangle <<'\n';
    
  }
  // std::cout<<"---------------------------------"<<'\n';
  VisualizeCorner(elevator_feature_points);

  if(elevator_feature_points.size()<2)
  {
    matching_success_ = false;
    return;
  }
  
  

  for(int f_1 = 0;f_1 < elevator_feature_points.size();f_1++)
  {
    for(int f_2 = 0;f_2 < elevator_feature_points.size();f_2++)
    { 
      if(abs(sqrt(pow(elevator_feature_points[f_1].x -elevator_feature_points[f_2].x,2)+
                  pow(elevator_feature_points[f_1].y -elevator_feature_points[f_2].y,2))-door_corner_dis) <= elevator_feature_dis_range_)
      {

        angle1 = atan2(points[elevator_feature_points[f_1].z].y - points[elevator_feature_points[f_1].z+1].y, 
                              points[elevator_feature_points[f_1].z].x - points[elevator_feature_points[f_1].z+1].x);

        angle2 = atan2(points[elevator_feature_points[f_2].z-1].y - points[elevator_feature_points[f_2].z].y, 
                              points[elevator_feature_points[f_2].z-1].x - points[elevator_feature_points[f_2].z].x);
        dangle = angle1 - angle2;
        if(dangle > M_PI)
        {
          dangle = -2*M_PI + dangle;
        }
        else if(dangle < -M_PI)
        {
          dangle = 2*M_PI + dangle;
        }

        //std::cout << " dangle " <<  dangle <<'\n';
        

        error_dangle = dangle - door_corner_dangle;

        if(error_dangle > M_PI)
        {
          error_dangle = -2*M_PI + error_dangle;
        }
        else if(error_dangle < -M_PI)
        {
          error_dangle = 2*M_PI + error_dangle;
        }
        
        if(abs(error_dangle) <= elevator_feature_d_angle_range_)
        {
          // for(int p = 0; p < 2;p++)
          // {
          //   if(p == 0)
          //   {
          //     feature_pose.pose.position.x = elevator_feature_points[f_1].x;
          //     feature_pose.pose.position.y = elevator_feature_points[f_1].y;
          //   }
          //   else if(p == 1)
          //   {
          //     feature_pose.pose.position.x = elevator_feature_points[f_2].x;
          //     feature_pose.pose.position.y = elevator_feature_points[f_2].y;
          //   }
            
          //   path.poses.push_back(feature_pose);
          // }
          // paths.push_back(path);
          // path.poses.clear();

          elevator_pose.position.x = ((elevator_feature_points[f_1].x +elevator_feature_points[f_2].x)/2.0);
          elevator_pose.position.y = (elevator_feature_points[f_1].y +elevator_feature_points[f_2].y)/2.0;
          double yaw = atan2(elevator_feature_points[f_1].y - elevator_feature_points[f_2].y, 
                              elevator_feature_points[f_1].x - elevator_feature_points[f_2].x);
          elevator_pose_q_tf.setRPY(0.0,0.0,yaw);
          elevator_pose_q_msg = tf2::toMsg(elevator_pose_q_tf);
          elevator_pose.orientation = elevator_pose_q_msg;

          elevator_poses.poses.push_back(elevator_pose);

          
        }
      }
    }
  }
  // std::cout << " ==================== "  <<'\n';
  // std::cout << " elevator_poses.poses: " <<  elevator_poses.poses.size() <<'\n';
  //VisualizeElevatorEnterPoint(paths);

  if(elevator_poses.poses.size() < 1)
  {
    matching_success_ = false;
    return;
  }

  shortest_dis = -1.0;
  find_shortest_dis_first_time = true;

  for(int f = 0;f<elevator_poses.poses.size();f++)
  {
    if(find_shortest_dis_first_time)
    {
      shortest_dis = sqrt(pow(elevator_poses.poses[f].position.x,2)+pow(elevator_poses.poses[f].position.y,2));
      find_shortest_dis_first_time = false;
    }
    else
    {
      s_dis = sqrt(pow(elevator_poses.poses[f].position.x,2)+pow(elevator_poses.poses[f].position.y,2));
      if(shortest_dis > s_dis)
      {
        shortest_dis = s_dis;
      }
    }
  }
// //
  if(shortest_dis != -1.0 && status_ == "enter")
  {
    if(shortest_dis <  0.5)
    {
      enter_done_ = true;
    }

  }
  // else if(shortest_dis != -1.0 && status_ == "exit")
  // {
  //   if(shortest_dis <  0.1)
  //   {
  //     exit_done_ = true;
  //   }
  // }

  // std::cout << " shortest_dis: " <<  shortest_dis << " enter_done_: " <<  enter_done_ << " exit_done_: " <<  exit_done_ << " enable_mode_: " <<  enable_mode_ <<'\n';

  if(elevator_pose_filter_ )
  {
    pose_filter(elevator_poses);
  }

  double shift_roll, shift_pitch, shift_yaw;

  tf::Quaternion sq( elevator_poses.poses[0].orientation.x,
                    elevator_poses.poses[0].orientation.y,
                    elevator_poses.poses[0].orientation.z,
                    elevator_poses.poses[0].orientation.w);
  tf::Matrix3x3 sm(sq);
  
  sm.getRPY(shift_roll, shift_pitch, shift_yaw);

  elevator_poses.poses[0].position.x = elevator_poses.poses[0].position.x + shift_x_*cos(shift_yaw+M_PI/2.0) - shift_y_*sin(shift_yaw+M_PI/2.0);
  elevator_poses.poses[0].position.y = elevator_poses.poses[0].position.y + shift_x_*sin(shift_yaw+M_PI/2.0) + shift_y_*cos(shift_yaw+M_PI/2.0);

  elevator_pose_q_tf.setRPY(0.0,0.0,shift_yaw+M_PI/2.0);
  elevator_pose_q_msg = tf2::toMsg(elevator_pose_q_tf);
  elevator_poses.poses[0].orientation = elevator_pose_q_msg;

  hold_poses_.poses.clear();
  hold_poses_.header.frame_id = map_frame_;
  hold_poses_.poses.push_back(elevator_poses.poses[0]);

  pose_pub_.publish(elevator_poses);
  // 
  
  
 
  

  
  

  // if(shortest_dis != -1.0)
  // {
  //   if(shortest_dis < 0.5)
  //   {
  //     sub_mode_ = 1.0; //LaserScan
  //     feature_point_neighborhood_dis_ = 0.03;
  //     break_point_neighborhood_dis_ = 0.05;
  //     status_ = "exit";
  //   }
  //   else if (shortest_dis > 1.5)
  //   {
  //     sub_mode_ = 2.0; //PointCloud to LaserScan
  //     feature_point_neighborhood_dis_ = 0.075;
  //     break_point_neighborhood_dis_ = 0.25;
  //     status_ = "enter";
  //   }
  // }
  
  //
  
  //
  //std::cout<<"=================================="<<'\n';/


}
//-----------------------------------------------------------------------------------------------
void pose_filter(geometry_msgs::PoseArray &filter_poses)
{
  static geometry_msgs::PoseStamped before_pose;
  static geometry_msgs::PoseStamped map_pose;
  static geometry_msgs::Pose pose;
  static std::vector<geometry_msgs::PoseStamped> poses;
  static geometry_msgs::PoseStamped local_pose;
  static std::vector<geometry_msgs::PoseStamped> local_poses;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  static double m_roll, m_pitch, m_yaw;
  static double roll, pitch, yaw;
  static bool matching;

  

  if(pose_filter_first_time_)
  {
    if(filter_poses.poses.size() == 0)
      return;

    poses.clear();

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
    pose_filter_first_time_ = false;
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
          poses[j].pose.orientation = map_pose.pose.orientation;

          if(poses[j].pose.position.z <=30.0)
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

  int max_matching_pose_id;
  double max_matching_score;

  filter_poses.poses.clear();
  filter_poses.header.frame_id = map_frame_;
  for(int k =0;k<poses.size();k++)
  {
    if(k==0)
    {
      max_matching_score = poses[k].pose.position.z;
      max_matching_pose_id = k;
    }
    else
    {
      if(max_matching_score < poses[k].pose.position.z)
      {
        max_matching_pose_id = k;
      }
    }
    
    //if(poses[k].pose.position.z > 5.0)
    // {
    //   pose.position.x = poses[k].pose.position.x;
    //   pose.position.y = poses[k].pose.position.y;
    //   pose.position.z = poses[k].pose.position.z;
    //   pose.orientation = poses[k].pose.orientation;
      
    //   filter_poses.poses.push_back(pose);
    // }
    

    if(poses[k].pose.position.z <= 0)
    {
      if(poses.size() > 1)
      {
        poses.erase(poses.begin()+k);
        //break;
      }
    }
  }

  
  pose.position.x = poses[max_matching_pose_id].pose.position.x;
  pose.position.y = poses[max_matching_pose_id].pose.position.y;
  pose.position.z = poses[max_matching_pose_id].pose.position.z;
  pose.orientation = poses[max_matching_pose_id].pose.orientation;
  if(avg_pose_filter_)
  {
    avg_pose(pose);
  }
  
  filter_poses.poses.push_back(pose);

  
}
//-----------------------------------------------------------------------------------------------
void avg_pose(geometry_msgs::Pose &input_pose)
{
  static std::vector<geometry_msgs::Pose> avg_poses ;
  static tf2::Quaternion avg_pose_q_tf;
  static geometry_msgs::Quaternion avg_pose_q_msg;
  
  static int poses_count = 0;
  static double roll, pitch, yaw;
  double sum_x=0,sum_y=0,sum_yaw=0;
  double avg_x=0,avg_y=0,avg_yaw=0;

  if(avg_poses.size() >= avg_pose_size_)
  {    
    for(int i = 0;i < avg_poses.size();i++)
    {
      sum_x += avg_poses[i].position.x;
      sum_y += avg_poses[i].position.y;

      tf::Quaternion q2( avg_poses[i].orientation.x,
                        avg_poses[i].orientation.y,
                        avg_poses[i].orientation.z,
                        avg_poses[i].orientation.w);
      tf::Matrix3x3 m2(q2);
  
      m2.getRPY(roll, pitch, yaw);

      sum_yaw += yaw;
    }
    avg_x = sum_x/avg_poses.size();
    avg_y = sum_y/avg_poses.size();
    avg_yaw = sum_yaw/avg_poses.size();

    avg_poses[poses_count]=input_pose ;
    poses_count++;
    if(poses_count > avg_pose_size_-1)
    {
      poses_count =0;
    }

    
  }else
  {
    avg_poses.push_back(input_pose) ;
    return;
  }
//
  avg_pose_q_tf.setRPY(0.0,0.0,avg_yaw);
  avg_pose_q_msg = tf2::toMsg(avg_pose_q_tf);

  input_pose.position.x = avg_x;
  input_pose.position.y = avg_y;
  input_pose.orientation = avg_pose_q_msg;

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
//--------------------------------------------------------------------

void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv)
{
  string str = "===========position_finder status check============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("position_finder status check : Failed to call service");
    ros::Duration(1.0).sleep();
  }
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
  scan2_sub_ = nh.subscribe("scan2", 10, Scan2Callback);
  control_status_sub_ = nh.subscribe("control_status", 10, ControlStatusCallback);
  //pose_sub_ = nh.subscribe("pose_topic", 10, PoseCallback);

  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("elevator_poses", 0.1);
  elevator_corner_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("elevator_corner_marker", 50);
  break_point_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("break_point_marker", 50);
  featuer_point_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("feature_point_marker", 50);
  elevator_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("elevator_marker", 10);

  status_check_client_ = nh.serviceClient<campusrover_msgs::ElevatorStatusChecker>("elevator_status_checker");

  ros::spin();

  return 0;
}
