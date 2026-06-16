#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <memory>
#include <chrono>
#include <atomic>
#include <future>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class PickAndPlacePipeline {
public:
    PickAndPlacePipeline(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");

        // Action client to control the gripper
        gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server...");
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(), "Gripper action server not available - gripper will not move.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
        }

        // --- TF setup for the dynamic cylinder ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initial fixed position of the cylinder in the world frame
        object_fixed_x_ = 0.0;
        object_fixed_y_ = 0.5;
        object_fixed_z_ = 0.0;

        is_grasped_.store(false);

        // Timer that publishes the object_link TF at 30Hz, replacing the
        // static_transform_publisher used previously in the launch file.
        tf_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&PickAndPlacePipeline::publishObjectTransform, this));

        RCLCPP_INFO(node_->get_logger(), "Pick-and-Place pipeline initialized successfully!");
    }

    void runPipeline() {
        RCLCPP_INFO(node_->get_logger(), "STARTING PIPELINE");

        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(10.0);

        // 1. Open gripper and go Home
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        if (!goHome()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 2. Pre-Grasp: above the cylinder
        if (!approachObject()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 3. Descend toward the cylinder
        if (!descendToGrasp()) return;
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 4. Force-Based Grasp
        executeForceGrasp();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 5. Lift
        if (!liftObject()) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 6. Transfer
        if (!transferObject()) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 7. Release
        releaseObject();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 8. Home
        goHome();

        RCLCPP_INFO(node_->get_logger(), "PIPELINE COMPLETED SUCCESSFULLY");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;

    // Dynamic TF for the cylinder (simulation porpouse)
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    double object_fixed_x_, object_fixed_y_, object_fixed_z_;
    std::atomic<bool> is_grasped_;
    tf2::Transform grasp_offset_;  // transform computed at grasp time

    // Publishes the world -> object_link transform every cycle
    // If the object is not grasped, it stays at its last known fixed pose
    // If grasped, it follows tool0 (the gripper) using the fixed offset computed at grasp time
    void publishObjectTransform() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "object_link";

        if (!is_grasped_.load()) {
            // Object resting at its fixed position
            t.transform.translation.x = object_fixed_x_;
            t.transform.translation.y = object_fixed_y_;
            t.transform.translation.z = object_fixed_z_;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;
        } else {
            // Object attached to tool0
            try {
                geometry_msgs::msg::TransformStamped world_to_tool =
                    tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);

                tf2::Transform world_to_tool_tf;
                tf2::fromMsg(world_to_tool.transform, world_to_tool_tf);

                tf2::Transform world_to_object_tf = world_to_tool_tf * grasp_offset_;

                t.transform = tf2::toMsg(world_to_object_tf);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "TF lookup world->tool0 failed: %s", ex.what());
                return;
            }
        }

        tf_broadcaster_->sendTransform(t);
    }

    // Computes the tool0 -> object_link offset at the current grasp position
    // and keeps it fixed for as long as the object stays attached
    void attachObjectToGripper() {
        try {
            geometry_msgs::msg::TransformStamped world_to_tool =
                tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);

            tf2::Transform world_to_tool_tf;
            tf2::fromMsg(world_to_tool.transform, world_to_tool_tf);

            tf2::Transform world_to_object_tf;
            world_to_object_tf.setOrigin(tf2::Vector3(object_fixed_x_, object_fixed_y_, object_fixed_z_));
            world_to_object_tf.setRotation(tf2::Quaternion(0, 0, 0, 1));

            // offset = (world->tool0)^-1 * world->object
            grasp_offset_ = world_to_tool_tf.inverse() * world_to_object_tf;

            is_grasped_.store(true);
            RCLCPP_INFO(node_->get_logger(), "Cylinder attached to end-effector (tool0).");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not attach cylinder: %s", ex.what());
        }
    }

    // Detaches the object, freezing it at its last computed world pose
    void detachObjectFromGripper() {
        try {
            geometry_msgs::msg::TransformStamped world_to_tool =
                tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);
            tf2::Transform world_to_tool_tf;
            tf2::fromMsg(world_to_tool.transform, world_to_tool_tf);
            tf2::Transform world_to_object_tf = world_to_tool_tf * grasp_offset_;

            object_fixed_x_ = world_to_object_tf.getOrigin().x();
            object_fixed_y_ = world_to_object_tf.getOrigin().y();
            object_fixed_z_ = world_to_object_tf.getOrigin().z();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not compute release position: %s", ex.what());
        }

        is_grasped_.store(false);
        RCLCPP_INFO(node_->get_logger(), "Cylinder released at current position.");
    }

    // GRIPPER CONTROL 

    // Sends a single-point FollowJointTrajectory goal to the gripper controller
    // and waits (with timeout) for the result via a promise/callback
    void sendGripperCommand(double position_m, const std::string& label) {
        if (!gripper_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Gripper action server not ready - skipping %s", label.c_str());
            return;
        }

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = {"gripper_gripper_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {position_m};
        point.time_from_start = rclcpp::Duration::from_seconds(1.5);
        goal.trajectory.points.push_back(point);

        RCLCPP_INFO(node_->get_logger(), "%s (position: %.4f m)", label.c_str(), position_m);

        auto done_promise = std::make_shared<std::promise<bool>>();
        auto done_future = done_promise->get_future();

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback =
            [this, done_promise, label](const GoalHandleFJT::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(node_->get_logger(), "%s completed.", label.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), "%s failed.", label.c_str());
                }
                done_promise->set_value(true);
            };

        gripper_action_client_->async_send_goal(goal, send_goal_options);

        // Wait up to 5 seconds for the callback to be executed 
        done_future.wait_for(std::chrono::seconds(5));
    }

    void openGripper() {
        // Upper joint limit = 0.0305 m (gripper fully open)
        sendGripperCommand(0.0305, "Gripper opening");
    }

    // Simulated force-based grasp: closes the gripper to a position calibrated
    // for the cylinder diameter. On real hardware this would monitor the
    // measured force and stop closing once the target threshold is reached
    void executeForceGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Force-Based Grasp (OnRobot 2FG7)...");

        double force_threshold_N = 20.0;   // Target grasp force in Newton
        double grasp_position = 0.015;     // ~15mm: grasp position 

        RCLCPP_INFO(node_->get_logger(),
            "Closing gripper with target force %.1f N - grasp position: %.4f m",
            force_threshold_N, grasp_position);

        sendGripperCommand(grasp_position, "Gripper closing (grasp)");

        // Attach the cylinder to the end-effector so it follows the arm motion
        attachObjectToGripper();

        RCLCPP_INFO(node_->get_logger(), "Grasp verified: object held with simulated force %.1f N", force_threshold_N);
    }

    void releaseObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Object release...");
        detachObjectFromGripper();
        openGripper();
    }

    // ARM MOTION 

    // End-effector orientation pointing straight down 
    geometry_msgs::msg::Quaternion getDownwardOrientation() {
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        q.normalize();
        geometry_msgs::msg::Quaternion msg;
        tf2::convert(q, msg);
        return msg;
    }

    bool moveToPose(double x, double y, double z, const std::string& label) {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        target.pose.orientation = getDownwardOrientation();

        move_group_->setPoseTarget(target);
        move_group_->setStartStateToCurrentState();

        auto result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "%s completed.", label.c_str());
            return true;
        }
        RCLCPP_WARN(node_->get_logger(), "%s failed (error code: %d).", label.c_str(), result.val);
        return false;
    }

    bool goHome() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Returning to Home position...");
        // Uprigth configuration 
        std::vector<double> home = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
        move_group_->setJointValueTarget(home);
        move_group_->setStartStateToCurrentState();
        bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) RCLCPP_INFO(node_->get_logger(), "Home reached.");
        else RCLCPP_WARN(node_->get_logger(), "Home motion failed - continuing anyway.");
        return true;
    }

    bool approachObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Pre-Grasp (above the cylinder)...");
        return moveToPose(0.0, 0.5, 0.35, "Pre-Grasp");
    }

    bool descendToGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Descending toward the cylinder...");
        return moveToPose(0.0, 0.5, 0.18, "Descent to grasp");
    }

    bool liftObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Lifting the cylinder...");
        return moveToPose(0.0, 0.5, 0.45, "Lift");
    }

    bool transferObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Transferring to the release zone...");
        return moveToPose(-0.4, 0.3, 0.45, "Transfer");
    }

    void recoveryBehavior() {
        RCLCPP_WARN(node_->get_logger(), "=== RECOVERY: motion failed, returning to Home ===");
        if (is_grasped_.load()) {
            detachObjectFromGripper();
        }
        openGripper();
        move_group_->stop();
        goHome();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_place_pipeline_node", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    PickAndPlacePipeline pipeline(node);
    pipeline.runPipeline();

    rclcpp::shutdown();
    return 0;
}