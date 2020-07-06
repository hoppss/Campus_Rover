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
#include <campusrover_msgs/ElevatorStatusChecker.h> 
#include <campusrover_msgs/ArmStandby.h> 
#include <dynamixel_controllers/SetComplianceSlope.h>

using namespace std;

ros::ServiceClient button_srv_client_, status_check_client_, joint_1_slope_client_, joint_2_slope_client_, joint_3_slope_client_;

ros::Subscriber pose_sub_;
ros::Publisher button_info_pub_;
geometry_msgs::PoseStamped pose_;

string planning_frame_id_;
string planning_group_name_;
string standby_pose_name_;
string release_pose_name_;

double planning_time_;
double num_planning_attempts_;
double press_dis_;
double gap_dis_;
double jump_threshold_;
double eef_step_;

double shift_x_;
double shift_y_;
double shift_z_;

bool allow_replanning_;
bool Visualization_;
bool arm_execution_done_= false;
bool standby_pose_ready_ = false;

void initialization();
void ButtonPoseCallback(geometry_msgs::Pose Pose);
void BtnCallService(ros::ServiceClient &client,campusrover_msgs::PressButton &srv);
void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv);
void SetComplianceSlopeCallService(ros::ServiceClient &client,dynamixel_controllers::SetComplianceSlope &srv);

void get_parameters(ros::NodeHandle n_private)
{
    n_private.param<string>("planning_frame_id", planning_frame_id_, "link_0");
    n_private.param<string>("planning_group_name", planning_group_name_, "arm");
    n_private.param<string>("standby_pose_name", standby_pose_name_, "standby_pose");
    n_private.param<string>("release_pose_name", release_pose_name_, "standby_pose");
    n_private.param<double>("planning_time", planning_time_, 5.0);
    n_private.param<double>("num_planning_attempts", num_planning_attempts_, 10.0);
    n_private.param<double>("shift_x", shift_x_, 0.00);
    n_private.param<double>("shift_y", shift_y_, 0.00);
    n_private.param<double>("shift_z", shift_z_, 0.00);
    n_private.param<double>("press_dis", press_dis_, 0.05);
    n_private.param<double>("jump_threshold", jump_threshold_, 0.0);
    n_private.param<double>("eef_step", eef_step_, 0.01);
    n_private.param<bool>("allow_replanning", allow_replanning_, true);
    n_private.param<bool>("Visualization", Visualization_, true);
    initialization();
}

void initialization()
{

}

//void ButtonPoseCallback(geometry_msgs::Pose pose)
bool ArmServiceCallback(campusrover_msgs::ArmAction::Request  &req, campusrover_msgs::ArmAction::Response &res)
{
    static campusrover_msgs::ElevatorStatusChecker status_msg;
    static dynamixel_controllers::SetComplianceSlope slope_msg;

    pose_ = req.button_pose;
    cout << "recrvie pose : " <<pose_<< endl;
    static const std::string PLANNING_GROUP = planning_group_name_;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    move_group.setGoalOrientationTolerance(0.3); 
    move_group.setGoalJointTolerance(0.01);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(planning_time_);
    move_group.setNumPlanningAttempts(num_planning_attempts_);
    move_group.allowReplanning(allow_replanning_);

    slope_msg.request.slope = 35;
    SetComplianceSlopeCallService(joint_1_slope_client_, slope_msg);
    SetComplianceSlopeCallService(joint_2_slope_client_, slope_msg);
    SetComplianceSlopeCallService(joint_3_slope_client_, slope_msg);
    
    cout << "GoalJointTolerance : " <<move_group.getGoalJointTolerance()<< endl;
    cout << "GoalPositionTolerance : " <<move_group.getGoalPositionTolerance()<< endl;
    cout << "GoalOrientationTolerance : " <<move_group.getGoalOrientationTolerance()<< endl;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const robot_state::JointModelGroup* joint_model_group =
    //   move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    

    //Visualization_//
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(planning_frame_id_);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    visual_tools.trigger();

    //move to standby_pose//
    bool success;
    if(!standby_pose_ready_)
    {
      cout << "move to standby_pose" << endl;
      move_group.setMaxVelocityScalingFactor(1.0);
      success = (move_group.setNamedTarget(standby_pose_name_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group.move();
      standby_pose_ready_ = true;
    }


    //ros::Duration(1.0).sleep();
    //move_group.clearPoseTarget();

    //move to button pose
    cout << "move to button pose" << endl;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    static geometry_msgs::Pose target_pose;
    static geometry_msgs::PoseStamped local_pose;
    static tf2::Quaternion quat_tf;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    cout << "Planning Frame : " <<move_group.getPlanningFrame()<<" input frame : "<<pose_.header.frame_id<< endl;

    bool transformed = false;
    if (planning_frame_id_ != pose_.header.frame_id)
    {
        while(!transformed)
        {
            try
            {
            tfBuffer.transform(pose_, local_pose, planning_frame_id_);
            transformed = true;
            }
            catch (tf2::TransformException &ex)
            {
            ROS_WARN("position_move: %s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }
        }
        
        pose_ = local_pose;
    }
    
    

    double yaw = atan2(pose_.pose.position.y,pose_.pose.position.x);
    quat_tf.setRPY( 0, 0, yaw ); 
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    target_pose.position = pose_.pose.position;
    target_pose.orientation = quat_msg;
    target_pose.position.x += shift_x_;
    target_pose.position.y += shift_y_;
    target_pose.position.z += shift_z_;

    cout << "target pose : " <<target_pose<< endl;

    move_group.setPoseTarget(target_pose);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
    ros::Duration(0.5).sleep();

    static std::vector<geometry_msgs::Pose> waypoints;
    waypoints.clear();

    waypoints.push_back(target_pose);

    target_pose.position.x += press_dis_;
    //target_pose.position.x += press_dis_*cos(yaw);
    //target_pose.position.y += press_dis_*sin(yaw);
    waypoints.push_back(target_pose);  // right

    target_pose.position.x -= press_dis_;
    //target_pose.position.x -= press_dis_*cos(yaw);
    //target_pose.position.y -= press_dis_*sin(yaw);
    waypoints.push_back(target_pose);  // up and left

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(1.0);

    static moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step_, jump_threshold_, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    plan.trajectory_ = trajectory;
    move_group.execute(plan);

    ros::Duration(0.5).sleep();

    slope_msg.request.slope = 70;
    SetComplianceSlopeCallService(joint_1_slope_client_, slope_msg);
    SetComplianceSlopeCallService(joint_2_slope_client_, slope_msg);
    SetComplianceSlopeCallService(joint_3_slope_client_, slope_msg);

    status_msg.request.node_name.data = "arm";
    status_msg.request.status.data = true;
    StatusCheckCallService(status_check_client_, status_msg);

    cout << "move to standby_pose  " << endl;
    move_group.setMaxVelocityScalingFactor(1.0);
    success = (move_group.setNamedTarget("standby_pose") == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    ros::Duration(0.5).sleep();
    cout << "move to release_pose " << endl;
    move_group.setMaxVelocityScalingFactor(1.0);
    success = (move_group.setNamedTarget(release_pose_name_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    standby_pose_ready_ = false;
    
    cout << "done" << endl;
    return true;
    
}
//----------------------------------------------------------------------------------------------------------------------
bool ButtonServiceCallback(campusrover_msgs::PressButton::Request  &req, campusrover_msgs::PressButton::Response &res)
{
    static std_msgs::String button_info;
    button_info.data = req.button_type.data;

    button_info_pub_.publish(button_info);

    
    //
    return true;

}
//----------------------------------------------------------------------------------------------------------------------
bool MoveToStandbyPoseServiceCallback(campusrover_msgs::ArmStandby::Request  &req, campusrover_msgs::ArmStandby::Response &res)
{
  static dynamixel_controllers::SetComplianceSlope slope_msg;
  static moveit::planning_interface::MoveGroupInterface standby_pose_move_group(planning_group_name_);
  
  standby_pose_move_group.setGoalOrientationTolerance(0.3); 
  standby_pose_move_group.setGoalJointTolerance(0.01);
  standby_pose_move_group.setGoalPositionTolerance(0.01);
  standby_pose_move_group.setMaxVelocityScalingFactor(1.0);
  standby_pose_move_group.setMaxAccelerationScalingFactor(1.0);
  standby_pose_move_group.setPlanningTime(planning_time_);
  standby_pose_move_group.setNumPlanningAttempts(num_planning_attempts_);
  standby_pose_move_group.allowReplanning(allow_replanning_);

  slope_msg.request.slope = 35;
  SetComplianceSlopeCallService(joint_1_slope_client_, slope_msg);
  SetComplianceSlopeCallService(joint_2_slope_client_, slope_msg);
  SetComplianceSlopeCallService(joint_3_slope_client_, slope_msg);

  cout << "move to standby_pose" << endl;
  bool success = (standby_pose_move_group.setNamedTarget(standby_pose_name_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  standby_pose_move_group.move();
  
  standby_pose_ready_ = success;
  
  //
  return success;

}
//-----------------------------------------------------------------------------------------------
void StatusCheckCallService(ros::ServiceClient &client,campusrover_msgs::ElevatorStatusChecker &srv)
{
  string str = "===========arm status check============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("arm status check : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}
//-----------------------------------------------------------------------------------------------
void SetComplianceSlopeCallService(ros::ServiceClient &client,dynamixel_controllers::SetComplianceSlope &srv)
{
  string str = "===========set slope============= " ;
  cout << "Request massage: \n" << srv.request;
  while (!client.call(srv))
  {
    ROS_ERROR("joint set slope : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_moveit");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    get_parameters(n_private);
    //pose_sub_ = n.subscribe("/button_pose", 1, ButtonPoseCallback);
    ros::ServiceServer arm_service = n.advertiseService("arm_action", ArmServiceCallback);
    ros::ServiceServer button_service = n.advertiseService("button_press", ButtonServiceCallback);
    ros::ServiceServer arm_move_to_standby_pose_button_service = n.advertiseService("arm_move_to_standby_pose", MoveToStandbyPoseServiceCallback);

    button_info_pub_ = n.advertise<std_msgs::String>("button_info", 50);

    button_srv_client_ = n.serviceClient<campusrover_msgs::PressButton>("button_info");
    status_check_client_ = n.serviceClient<campusrover_msgs::ElevatorStatusChecker>("elevator_status_checker");

    joint_1_slope_client_ = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("/joint_1/set_compliance_slope");
    joint_2_slope_client_ = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("/joint_2/set_compliance_slope");
    joint_3_slope_client_ = n.serviceClient<dynamixel_controllers::SetComplianceSlope>("/joint_3/set_compliance_slope");

    spinner.start();
    ros::waitForShutdown();
    return 0;
}
