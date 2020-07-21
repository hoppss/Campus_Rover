#include <fstream>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/NavSatFix.h>

// YAML usage website: https://github.com/jbeder/yaml-cpp/wiki/How-To-Emit-YAML
#include <yaml-cpp/yaml.h>

#include <math.h>

// LIBRARIES
#include <gps_common/conversions.h>

using namespace visualization_msgs;
using namespace std;

struct Mytf
{
  double x;
  double y;
  double z;
  double r_x;
  double r_y;
  double r_z;
  double r_w;
};

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

ros::Subscriber fake_gps_sub_;

string world_frame_, local_frame_, cloud_frame_, save_file_, pcd_file_;
double latitude_, longitude_;

Mytf tf_;

void getParameters(ros::NodeHandle n_private)
{
  n_private.param<string>("world_frame", world_frame_, "world");
  n_private.param<string>("local_frame", local_frame_, "map");
  n_private.param<string>("cloud_frame", cloud_frame_, "cloud");
  n_private.param<string>("save_file", save_file_, "");
  n_private.param<string>("pcd_file", pcd_file_, "test.pcd");
}

void writeFile(string save_path)
{
  YAML::Emitter out;
  out.SetOutputCharset(YAML::EscapeNonAscii);
  out << YAML::BeginMap;
    out << YAML::Key << "pcd_file";
    out << YAML::Value << pcd_file_;
    out << YAML::Key << "gps_topic";
    out << YAML::Value;
    out << YAML::BeginMap;
      out << YAML::Key << "topic_name" << YAML::Value << "/fake_gps/fix";
      out << YAML::Key << "parent_frame" << YAML::Value << world_frame_;
      out << YAML::Key << "child_frame" << YAML::Value << local_frame_;
      out << YAML::Key << "latitude" << YAML::Value << latitude_;
      out << YAML::Key << "longitude" << YAML::Value << longitude_;
    out << YAML::EndMap;
    out << YAML::Key << "map_fix_tf";
    out << YAML::Value;
    out << YAML::BeginMap;
      out << YAML::Key << "child_frame" << YAML::Value << cloud_frame_;
      out << YAML::Key << "tf";
      out << YAML::Value;
      out << YAML::Flow;
      out << YAML::BeginSeq << tf_.x << tf_.y << tf_.z << tf_.r_x << tf_.r_y << tf_.r_z << tf_.r_w << YAML::EndSeq;
    out << YAML::EndMap;
  out << YAML::EndMap;
  std::ofstream fout(save_path);
  fout << out.c_str();
  fout.close();
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      switch(feedback->menu_entry_id)
      {
        case 1:
          writeFile(save_file_);
          break;
        case 3:
          tf_.x = 0;
          tf_.y = 0;
          tf_.z = 0;
          break;
        case 4:
          tf_.r_x = 0;
          tf_.r_y = 0;
          tf_.r_z = 0;
          tf_.r_w = 1;
          break;
        default:
          break;
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      tf_.x = feedback->pose.position.x;
      tf_.y = feedback->pose.position.y;
      tf_.z = feedback->pose.position.z;
      tf_.r_x = feedback->pose.orientation.x;
      tf_.r_y = feedback->pose.orientation.y;
      tf_.r_z = feedback->pose.orientation.z;
      tf_.r_w = feedback->pose.orientation.w;
      // cout << fixed << "update position: ["
      //      << tf_.x << ", "
      //      << tf_.y << ", "
      //      << tf_.z << ", "
      //      << tf_.r_x << ", "
      //      << tf_.r_y << ", "
      //      << tf_.r_z << ", "
      //      << tf_.r_w << "]"<< endl;
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = local_frame_;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 20;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}

void frameCallback(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;
  tf::Transform t;
  ros::Time time = ros::Time::now();
  t.setOrigin(tf::Vector3(tf_.x, tf_.y, tf_.z));
  tf::Quaternion q(tf_.r_x, tf_.r_y, tf_.r_z, tf_.r_w);
  t.setRotation(q);
  br.sendTransform(tf::StampedTransform(t, time, local_frame_, cloud_frame_));
}

void fakeGpsCallback(sensor_msgs::NavSatFix::ConstPtr msg)
{
  // Intial static tf
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  double UTM_North, UTM_East;
  std::string UTM_zone;
  latitude_ = msg->latitude;
  longitude_ = msg->longitude;
  gps_common::LLtoUTM(msg->latitude, msg->longitude, UTM_North, UTM_East, UTM_zone);
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = world_frame_;
  static_transformStamped.child_frame_id = local_frame_;
  static_transformStamped.transform.translation.x = UTM_East;
  static_transformStamped.transform.translation.y = UTM_North;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  cout << "Static TF: " << static_transformStamped << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_tf_finder_node");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  getParameters(nh_private);
  tf_.x = 0;
  tf_.y = 0;
  tf_.z = 0;
  tf_.r_x = 0;
  tf_.r_y = 0;
  tf_.r_z = 0;
  tf_.r_w = 1;
  fake_gps_sub_ = n.subscribe("fake_gps", 1000, fakeGpsCallback);
  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.1), frameCallback);
  server.reset( new interactive_markers::InteractiveMarkerServer("pcd_controls","",false) );
  ros::Duration(0.1).sleep();
  menu_handler.insert( "Save tf parameters", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Poses Reset" );
  menu_handler.insert(sub_menu_handle, "Reset Position", &processFeedback );
  menu_handler.insert(sub_menu_handle, "Reset Rotation", &processFeedback );
  tf::Vector3 position = tf::Vector3( 0, 0, 0);
  cout << fixed << "Intial Point: "
       << position.getX() << ", "
       << position.getY() << ", "
       << position.getZ() << endl;
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
  server->applyChanges();
  ros::spin();
  server.reset();
  return 0;
}
