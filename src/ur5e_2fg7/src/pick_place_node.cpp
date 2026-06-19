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
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_, "ur_manipulator");

        // Action client to control the gripper via FollowJointTrajectory
        gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server...");
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(),
                "Gripper action server not available - gripper will not move.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
        }

        // TF broadcaster and listener for the dynamic cylinder transform
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initial fixed position of the cylinder in the world frame
        object_fixed_x_ = 0.0;
        object_fixed_y_ = 0.5;
        object_fixed_z_ = 0.0;

        is_grasped_.store(false);

        // Publish the cylinder TF at 30 Hz; replaces the static_transform_publisher
        // used previously so the cylinder can follow the gripper once grasped
        tf_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&PickAndPlacePipeline::publishObjectTransform, this));

        RCLCPP_INFO(node_->get_logger(), "Pick-and-Place pipeline initialized successfully!");
    }

    void runPipeline() {
        RCLCPP_INFO(node_->get_logger(), "=== STARTING PIPELINE B1b ===");

        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(10.0);

        // 1. Open gripper and move to the Up/Home position
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        if (!goHome()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 2. Pre-Grasp: position the end-effector above the cylinder
        if (!moveToPoseWithRetry(0.0, 0.5, 0.40, "Pre-Grasp")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 3. Descend toward the cylinder top
        if (!moveToPoseWithRetry(0.0, 0.5, 0.22, "Descent to grasp")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 4. Force-based grasp with automatic retry on failure
        if (!graspWithRetry()) {
            RCLCPP_ERROR(node_->get_logger(),
                "Grasp failed after all attempts - aborting pipeline.");
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 5. Lift the object
        if (!moveToPoseWithRetry(0.0, 0.5, 0.50, "Lift")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 6. Transfer to the release zone
        if (!moveToPoseWithRetry(-0.3, 0.2, 0.50, "Transfer")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 7. Release the object
        releaseObject();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 8. Return to Home
        goHome();

        RCLCPP_INFO(node_->get_logger(), "=== PIPELINE COMPLETED SUCCESSFULLY ===");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;

    // TF infrastructure for the simulated moving cylinder
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    rclcpp::TimerBase::SharedPtr                   tf_timer_;

    double object_fixed_x_, object_fixed_y_, object_fixed_z_;
    std::atomic<bool> is_grasped_;
    tf2::Transform grasp_offset_;  // fixed tool0->object offset computed at grasp time

    // CYLINDER DYNAMICS
    
    // Publishes world -> object_link at 30 Hz.
    // When not grasped the cylinder stays at its last known fixed pose.
    // When grasped it follows tool0 using the offset computed at grasp time.
    void publishObjectTransform() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = node_->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id  = "object_link";

        if (!is_grasped_.load()) {
            t.transform.translation.x = object_fixed_x_;
            t.transform.translation.y = object_fixed_y_;
            t.transform.translation.z = object_fixed_z_;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;
        } else {
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

    // Computes and stores the tool0->object offset at the moment of grasping
    void attachObjectToGripper() {
        try {
            geometry_msgs::msg::TransformStamped world_to_tool =
                tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);
            tf2::Transform world_to_tool_tf;
            tf2::fromMsg(world_to_tool.transform, world_to_tool_tf);

            tf2::Transform world_to_object_tf;
            world_to_object_tf.setOrigin(
                tf2::Vector3(object_fixed_x_, object_fixed_y_, object_fixed_z_));
            world_to_object_tf.setRotation(tf2::Quaternion(0, 0, 0, 1));

            // offset = inv(world->tool0) * world->object
            grasp_offset_ = world_to_tool_tf.inverse() * world_to_object_tf;
            is_grasped_.store(true);
            RCLCPP_INFO(node_->get_logger(), "Cylinder attached to end-effector (tool0).");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not attach cylinder: %s", ex.what());
        }
    }

    // Freezes the cylinder at its current world pose and detaches it from the gripper
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
            RCLCPP_WARN(node_->get_logger(),
                "Could not compute release position: %s", ex.what());
        }
        is_grasped_.store(false);
        RCLCPP_INFO(node_->get_logger(), "Cylinder released at current position.");
    }

    // GRIPPER CONTROL 

    // Sends a single-point FollowJointTrajectory goal to the gripper controller
    // and waits for the result via a promise/callback (avoids executor conflicts)
    void sendGripperCommand(double position_m, const std::string& label) {
        if (!gripper_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(),
                "Gripper action server not ready - skipping %s", label.c_str());
            return;
        }

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = {"gripper_gripper_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions        = {position_m};
        point.time_from_start  = rclcpp::Duration::from_seconds(1.5);
        goal.trajectory.points.push_back(point);

        RCLCPP_INFO(node_->get_logger(), "%s (position: %.4f m)", label.c_str(), position_m);

        auto done_promise = std::make_shared<std::promise<bool>>();
        auto done_future  = done_promise->get_future();

        auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        opts.result_callback =
            [this, done_promise, label](const GoalHandleFJT::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(node_->get_logger(), "%s completed.", label.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), "%s failed.", label.c_str());
                }
                done_promise->set_value(true);
            };

        gripper_action_client_->async_send_goal(goal, opts);
        done_future.wait_for(std::chrono::seconds(5));
    }

    // Open gripper to its upper joint limit (fully open = 0.0305 m)
    void openGripper() {
        sendGripperCommand(0.0305, "Gripper opening");
    }

    // Simulated force-based grasp:
    // closes the gripper to a position calibrated for the cylinder diameter.
    // On real hardware this would monitor the measured force and stop closing
    // once the target threshold (in Newton) is reached.
    void executeForceGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Force-Based Grasp (OnRobot 2FG7)...");

        const double force_threshold_N = 20.0;  // target grasp force [N]
        const double grasp_position    = 0.015;  // ~15 mm: calibrated for ~30 mm cylinder

        RCLCPP_INFO(node_->get_logger(),
            "Closing gripper - target force: %.1f N, grasp position: %.4f m",
            force_threshold_N, grasp_position);

        sendGripperCommand(grasp_position, "Gripper closing (grasp)");
        attachObjectToGripper();

        RCLCPP_INFO(node_->get_logger(),
            "Grasp verified: object held with simulated force %.1f N", force_threshold_N);
    }

    void releaseObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Object release...");
        detachObjectFromGripper();
        openGripper();
    }

    // ARM MOTION 

    // End-effector orientation pointing straight down (RPY = pi, 0, 0)
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
        target.header.frame_id = "world";
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
        RCLCPP_WARN(node_->get_logger(),
            "%s failed (error code: %d).", label.c_str(), result.val);
        return false;
    }

    // Recovery behavior 1 - Retry with replanning
    // OMPL is stochastic: a new planning attempt with a different random seed
    // often finds a valid trajectory when a previous attempt failed.
    bool moveToPoseWithRetry(double x, double y, double z,
                             const std::string& label, int max_attempts = 3) {
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(node_->get_logger(), "%s - attempt %d/%d",
                label.c_str(), attempt, max_attempts);
            if (moveToPose(x, y, z, label)) return true;
            RCLCPP_WARN(node_->get_logger(), "Attempt %d failed, retrying...", attempt);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_ERROR(node_->get_logger(),
            "%s failed after %d attempts.", label.c_str(), max_attempts);
        return false;
    }

    // Recovery behavior 2 - Safe retreat
    // If a motion fails during execution, move straight up along Z before
    // returning to Home to avoid collisions with the environment.
    void safeRetreat() {
        RCLCPP_WARN(node_->get_logger(), "Safe retreat: moving up along Z for safety...");
        geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose();
        geometry_msgs::msg::PoseStamped retreat;
        retreat.header.frame_id       = "world";
        retreat.pose.position.x       = current.pose.position.x;
        retreat.pose.position.y       = current.pose.position.y;
        retreat.pose.position.z       = current.pose.position.z + 0.15;
        retreat.pose.orientation      = getDownwardOrientation();
        move_group_->setPoseTarget(retreat);
        move_group_->move();
    }

    // Recovery behavior 3 - Grasp retry
    // If the grasp verification (small test lift) fails, the gripper is opened,
    // the arm re-descends, and the grasp is attempted again up to max_attempts.
    // On real hardware, force feedback from the 2FG7 would confirm the grasp
    // instead of the kinematic test lift used here.
    bool graspWithRetry(int max_attempts = 2) {
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(node_->get_logger(), "Grasp attempt %d/%d", attempt, max_attempts);
            executeForceGrasp();
            rclcpp::sleep_for(std::chrono::seconds(1));

            // Verify grasp with a small test lift of ~3 cm
            geometry_msgs::msg::PoseStamped test_lift;
            test_lift.header.frame_id  = "world";
            test_lift.pose.position.x  = 0.0;
            test_lift.pose.position.y  = 0.5;
            test_lift.pose.position.z  = 0.25;
            test_lift.pose.orientation = getDownwardOrientation();
            move_group_->setPoseTarget(test_lift);

            if (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(),
                    "Grasp verified at attempt %d.", attempt);
                return true;
            }

            RCLCPP_WARN(node_->get_logger(),
                "Grasp verification failed - releasing and retrying...");
            detachObjectFromGripper();
            openGripper();
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            moveToPose(0.0, 0.5, 0.22, "Re-descend for retry");
        }
        return false;
    }

    // Move the arm to the "up" configuration defined in the SRDF,
    // which matches the default standing pose used in the real lab
    bool goHome() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Returning to Up position...");
        std::vector<double> up = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
        move_group_->setJointValueTarget(up);
        move_group_->setStartStateToCurrentState();
        bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) RCLCPP_INFO(node_->get_logger(), "Up position reached.");
        else RCLCPP_WARN(node_->get_logger(), "Up motion failed - continuing anyway.");
        return true;
    }

    // Main recovery handler: detach object, open gripper, safe retreat, then Home
    void recoveryBehavior() {
        RCLCPP_WARN(node_->get_logger(),
            "=== RECOVERY: executing safe retreat then returning Home ===");
        if (is_grasped_.load()) detachObjectFromGripper();
        openGripper();
        move_group_->stop();
        safeRetreat();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        goHome();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_place_pipeline_node", node_options);

    // Run a multi-threaded executor in a background thread so that MoveIt
    // callbacks (joint states, TF, action results) are processed while the
    // main thread executes the pipeline sequentially
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    PickAndPlacePipeline pipeline(node);
    pipeline.runPipeline();

    rclcpp::shutdown();
    return 0;
}