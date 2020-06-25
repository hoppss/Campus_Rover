#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <campusrover_msgs/ArmAction.h>
#include <campusrover_msgs/PressButton.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Bool.h>

using namespace std;

ros::Subscriber tag_sub_, action_sub_;
ros::ServiceClient arm_srv_client_;

bool arm_action_enable_ = false;

void callService(ros::ServiceClient &client,campusrover_msgs::ArmAction &srv) ;
void ActionEnableCallback(std_msgs::Bool flag);

void get_parameters(ros::NodeHandle n_private)
{
    //....//
}

void ActionEnableCallback(std_msgs::Bool flag)
{
    arm_action_enable_ = flag.data;
}

void TagPoseCallback(apriltag_ros::AprilTagDetectionArray tag_pose)
{
    static campusrover_msgs::ArmAction pose_srv;
    static geometry_msgs::PoseStamped target_pose;
    if(tag_pose.detections.size()>0)
    {
        target_pose.header.frame_id = tag_pose.detections[0].pose.header.frame_id;
        target_pose.pose.position.x = tag_pose.detections[0].pose.pose.pose.position.x;
        target_pose.pose.position.y = tag_pose.detections[0].pose.pose.pose.position.y;
        target_pose.pose.position.z = tag_pose.detections[0].pose.pose.pose.position.z;
        target_pose.pose.orientation = tag_pose.detections[0].pose.pose.pose.orientation;
    }

    if(arm_action_enable_ && tag_pose.detections.size()>0)
    {
        pose_srv.request.button_pose = target_pose;
        callService(arm_srv_client_, pose_srv);
        arm_action_enable_ = false;
    }

    
}

void callService(ros::ServiceClient &client,campusrover_msgs::ArmAction &srv)
{
    string str = "======================== " ;
    cout << "Request massage: \n" << srv.request;
    while (!client.call(srv))
    {
    ROS_ERROR("Failed to call service");
    ros::Duration(1.0).sleep();
    }
}

//----------------------------------------------------------------------------------------------------------------------
bool ButtonServiceCallback(campusrover_msgs::PressButton::Request  &req, campusrover_msgs::PressButton::Response &res)
{
    // static campusrover_msgs::PressButton button_srv;
    // button_srv.request.button_type = req.button_type;
    arm_action_enable_ = true;
    
    //
    return true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_to_press");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    ros::AsyncSpinner spinner(2); // Use 4 threads
    get_parameters(n_private);
    tag_sub_ = n.subscribe("/tag_detections", 1, TagPoseCallback);
    action_sub_ = n.subscribe("/action_enable", 1, ActionEnableCallback);
    ros::ServiceServer button_service = n.advertiseService("button_info", ButtonServiceCallback);
    arm_srv_client_ = n.serviceClient<campusrover_msgs::ArmAction>("arm_action");
    spinner.start();
    ros::waitForShutdown();
    return 0;
}