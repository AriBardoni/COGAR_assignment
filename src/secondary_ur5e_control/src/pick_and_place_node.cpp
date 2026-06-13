#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create the node options allowing MoveIt to load the robot description
  auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("pick_and_place_node", node_options);
  
  // We need a multi-threaded executor to handle MoveIt internal callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the MoveIt MoveGroupInterface for the UR5e arm
  // Usually the group name for UR robots in MoveIt configs is "ur_manipulator"
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Set reference frame and end effector link
  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink("tool0");

  RCLCPP_INFO(move_group_node->get_logger(), "MoveGroupInterface initialized for group: %s", PLANNING_GROUP.c_str());

  // Define the target pose for the gripper (directly above our object)
  // Object position from launch file: X=0.4, Y=0.0, Z=0.05
  geometry_msgs::msg::Pose target_pose;
  
  // Keep the gripper pointing downwards (orientation quaternions)
  target_pose.orientation.w = 0.0;
  target_pose.orientation.x = 1.0; // Facing down
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  
  // Position the end effector 15cm above the object center for approach
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.20; 

  // Set the target pose in MoveIt
  move_group.setPoseTarget(target_pose);

  // Plan the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(move_group_node->get_logger(), "Planning successful! Executing trajectory...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(move_group_node->get_logger(), "Planning failed!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}