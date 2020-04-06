#include <fstream>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/NavSatFix.h>

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

string local_frame_, base_frame_, save_file_;
double latitude_, longitude_;

Mytf tf_;

void getParameters(ros::NodeHandle n_private)
{
  n_private.param<string>("local_frame", local_frame_, "map");
  n_private.param<string>("base_frame", base_frame_, "base_link");
  n_private.param<string>("save_file", save_file_, "");
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
          // writeFile(save_file_);
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
      cout << fixed << "update position: ["
           << tf_.x << ", "
           << tf_.y << ", "
           << tf_.z << ", "
           << tf_.r_x << ", "
           << tf_.r_y << ", "
           << tf_.r_z << ", "
           << tf_.r_w << "]"<< endl;
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
  int_marker.scale = 1;

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
  br.sendTransform(tf::StampedTransform(t, time, local_frame_, base_frame_));
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
