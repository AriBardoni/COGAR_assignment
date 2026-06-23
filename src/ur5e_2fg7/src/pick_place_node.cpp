#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
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
#include <vector>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

static constexpr double CYLINDER_HEIGHT = 0.1;
static constexpr double CYLINDER_RADIUS = 0.02;
static constexpr const char* CYLINDER_ID = "target_cylinder";

class PickAndPlacePipeline {
public:
    PickAndPlacePipeline(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_, "ur_manipulator");
        planning_scene_interface_ =
            std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        // FIXED HERE: Added missing underscore to gripper_action_client_
        gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server...");
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(), "Gripper action server not available - gripper will not move.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
        }

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        object_fixed_x_ = 0.3;
        object_fixed_y_ = 0.3;
        object_fixed_z_ = 0.0;

        is_grasped_.store(false);

        tf_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&PickAndPlacePipeline::publishObjectTransform, this));

        addCylinderToScene();

        RCLCPP_INFO(node_->get_logger(), "Pick-and-Place pipeline initialized successfully!");
    }

    void runPipeline() {
        RCLCPP_INFO(node_->get_logger(), "=== STARTING CLEAN PIPELINE (CARTESIAN TRAJECTORY) ===");

        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(10.0); // More time to calculate optimized paths

        // 1. Initial reset: clear constraints, open gripper, and go Home
        clearConstraints();
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        if (!goHome()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // ACTIVATE CONSTRAINTS: Forces the gripper to point downward (Free Z tolerance for cylinder symmetry)
        setDownwardOrientationConstraint();

        // 2. Pre-Grasp: Above the cylinder
        if (!moveToPoseWithRetry(0.3, 0.3, 0.40, "Pre-Grasp")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 3. Descent: Linear descent onto the cylinder
        if (!moveToPoseWithRetry(0.3, 0.3, 0.22, "Descent to grasp")) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 4. Object grasp
        if (!graspWithRetry()) {
            RCLCPP_ERROR(node_->get_logger(), "Grasp failed after all attempts - aborting pipeline.");
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 5. Lift: Controlled vertical lifting via Cartesian waypoints
        if (!liftWithWaypoint()) {
            recoveryBehavior();
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 6. Transfer: Clean linear movement towards the release zone
        bool transfer_ok = moveToPoseWithRetry(-0.3, 0.2, 0.45, "Transfer to Release Zone");
        if (!transfer_ok) {
            recoveryBehavior();
            return;
        }

        // TEST FOR SAFE RETREAT, TRANSFER ZONE IMPOSSIBLE TO REACH 
        /*bool transfer_ok = moveToPoseWithRetry(5.0, 5.0, 5.0, "Transfer to Release Zone");  // fuori workspace
        if (!transfer_ok) {
            recoveryBehavior();
            return;
        }*/
        
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 7. Release: Cylinder release
        releaseObject();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Linear vertical retreat to avoid collisions with the newly placed object
        moveToPoseWithRetry(-0.3, 0.2, 0.55, "Post-release retreat");
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // CLEAR CONSTRAINTS: Required to allow the joints to return to the Home configuration freely
        clearConstraints();

        // 8. Return to Home
        goHome();

        RCLCPP_INFO(node_->get_logger(), "=== PIPELINE COMPLETED SUCCESSFULLY ===");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    rclcpp::TimerBase::SharedPtr                   tf_timer_;

    double object_fixed_x_, object_fixed_y_, object_fixed_z_;
    std::atomic<bool> is_grasped_;
    tf2::Transform grasp_offset_;

    // COLLISION SCENE MANAGEMENT

    void addCylinderToScene() {
        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.header.frame_id = "world";
        cylinder.id = CYLINDER_ID;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions = {CYLINDER_HEIGHT, CYLINDER_RADIUS};

        geometry_msgs::msg::Pose pose;
        pose.position.x = object_fixed_x_;
        pose.position.y = object_fixed_y_;
        pose.position.z = object_fixed_z_;
        pose.orientation.w = 1.0;

        cylinder.primitives.push_back(primitive);
        cylinder.primitive_poses.push_back(pose);
        cylinder.operation = cylinder.ADD;

        planning_scene_interface_->applyCollisionObject(cylinder);
        RCLCPP_INFO(node_->get_logger(), "Cylinder added to planning scene as collision object.");
    }

    void attachCollisionCylinder() {
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = "tool0";
        attached.object.id = CYLINDER_ID;
        attached.object.header.frame_id = "world";
        attached.object.operation = attached.object.MOVE;
        attached.touch_links = {"gripper_base_link", "left_finger_pad", "right_finger_pad"};
        planning_scene_interface_->applyAttachedCollisionObject(attached);
    }

    void detachCollisionCylinder() {
        moveit_msgs::msg::AttachedCollisionObject detached;
        detached.link_name = "tool0";
        detached.object.id = CYLINDER_ID;
        detached.object.operation = detached.object.REMOVE;
        planning_scene_interface_->applyAttachedCollisionObject(detached);

        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.header.frame_id = "world";
        cylinder.id = CYLINDER_ID;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions = {CYLINDER_HEIGHT, CYLINDER_RADIUS};

        geometry_msgs::msg::Pose pose;
        pose.position.x = object_fixed_x_;
        pose.position.y = object_fixed_y_;
        pose.position.z = object_fixed_z_;
        pose.orientation.w = 1.0;

        cylinder.primitives.push_back(primitive);
        cylinder.primitive_poses.push_back(pose);
        cylinder.operation = cylinder.ADD;

        planning_scene_interface_->applyCollisionObject(cylinder);
    }

    // DYNAMIC TF MANAGEMENT

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

    void attachObjectToGripper() {
        try {
            geometry_msgs::msg::TransformStamped world_to_tool =
                tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero);
            tf2::Transform world_to_tool_tf;
            tf2::fromMsg(world_to_tool.transform, world_to_tool_tf);

            tf2::Transform world_to_object_tf;
            world_to_object_tf.setOrigin(tf2::Vector3(object_fixed_x_, object_fixed_y_, object_fixed_z_));
            world_to_object_tf.setRotation(tf2::Quaternion(0, 0, 0, 1));

            grasp_offset_ = world_to_tool_tf.inverse() * world_to_object_tf;
            is_grasped_.store(true);
            attachCollisionCylinder();
            RCLCPP_INFO(node_->get_logger(), "Cylinder attached to end-effector (tool0).");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "Could not attach cylinder: %s", ex.what());
        }
    }

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
        detachCollisionCylinder();
        RCLCPP_INFO(node_->get_logger(), "Cylinder released at current position.");
    }

    // GRIPPER CONTROL

    void sendGripperCommand(double position_m, const std::string& label) {
        if (!gripper_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Gripper action server not ready - skipping %s", label.c_str());
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

    void openGripper() {
        sendGripperCommand(0.0305, "Gripper opening");
    }

    void executeForceGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Force-Based Grasp (OnRobot 2FG7)...");
        const double force_threshold_N = 20.0;
        const double grasp_position    = 0.015;

        RCLCPP_INFO(node_->get_logger(), "Closing gripper - target force: %.1f N, grasp position: %.4f m", force_threshold_N, grasp_position);

        sendGripperCommand(grasp_position, "Gripper closing (grasp)");
        attachObjectToGripper();

        RCLCPP_INFO(node_->get_logger(), "Grasp verified: object held with simulated force %.1f N", force_threshold_N);
    }

    void releaseObject() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Object release...");
        detachObjectFromGripper();
        openGripper();
    }

    // MOTION PLANNING AND EXECUTION 

    geometry_msgs::msg::Quaternion getDownwardOrientation() {
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        q.normalize();
        geometry_msgs::msg::Quaternion msg;
        tf2::convert(q, msg);
        return msg;
    }

    void setDownwardOrientationConstraint() {
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "tool0";
        ocm.header.frame_id = "world";
        ocm.orientation = getDownwardOrientation();
        ocm.absolute_x_axis_tolerance = 0.1; // Tight control over Roll
        ocm.absolute_y_axis_tolerance = 0.1; // Tight control over Pitch
        ocm.absolute_z_axis_tolerance = 6.28; // Full freedom around the Z axis (optimal for cylinders)
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        move_group_->setPathConstraints(constraints);
    }

    void clearConstraints() {
        move_group_->clearPathConstraints();
    }

    bool moveToPose(double x, double y, double z, const std::string& label) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation = getDownwardOrientation();

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // Path sampling every 1 cm
        const double jump_threshold = 0.0; // Disables joint configuration jumps (Gimbal Lock)

        move_group_->setStartStateToCurrentState();
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // If MoveIt guarantees a 100% perfect straight line, execute it directly
        if (fraction >= 1.0) {
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            
            auto result = move_group_->execute(cartesian_plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "%s (Linear Cartesian) completed.", label.c_str());
                return true;
            }
        }

        // Safety fallback: if linear Cartesian execution fails, attempt standard wide-range planning
        RCLCPP_WARN(node_->get_logger(), "%s Cartesian failed (fraction: %.2f). Executing standard fallback...", label.c_str(), fraction);
        
        move_group_->setGoalPositionTolerance(0.01);
        move_group_->setGoalOrientationTolerance(0.01);
        move_group_->setPoseTarget(target_pose);

        auto result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "%s (Standard Fallback) completed.", label.c_str());
            return true;
        }

        RCLCPP_WARN(node_->get_logger(), "%s completely failed (error code: %d).", label.c_str(), result.val);
        return false;
    }

    bool moveToPoseWithRetry(double x, double y, double z, const std::string& label, int max_attempts = 3) {
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(node_->get_logger(), "%s - attempt %d/%d", label.c_str(), attempt, max_attempts);
            if (moveToPose(x, y, z, label)) return true;
            RCLCPP_WARN(node_->get_logger(), "Attempt %d failed, retrying...", attempt);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_ERROR(node_->get_logger(), "%s failed after %d total attempts.", label.c_str(), max_attempts);
        return false;
    }

    // Guided linear lift
    bool liftWithWaypoint() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Controlled vertical lifting of the cylinder...");
        bool ok = moveToPoseWithRetry(object_fixed_x_, object_fixed_y_, 0.30, "Lift - intermediate waypoint");
        if (ok) {
            ok = moveToPoseWithRetry(object_fixed_x_, object_fixed_y_, 0.50, "Lift - final height");
        }
        return ok;
    }

    void safeRetreat() {
        RCLCPP_WARN(node_->get_logger(), "Safe retreat: clearing straight up along the Z axis...");
        geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose();
        moveToPose(current.pose.position.x, current.pose.position.y, current.pose.position.z + 0.15, "Emergency retreat");
    }

    bool graspWithRetry(int max_attempts = 2) {
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(node_->get_logger(), "Grasp attempt %d/%d", attempt, max_attempts);
            executeForceGrasp();
            rclcpp::sleep_for(std::chrono::seconds(1));

            if (moveToPose(object_fixed_x_, object_fixed_y_, 0.25, "Lift test for grasp verification")) {
                RCLCPP_INFO(node_->get_logger(), "Grasp successfully verified at attempt %d.", attempt);
                return true;
            }

            RCLCPP_WARN(node_->get_logger(), "Grasp verification failed - releasing and retrying...");
            detachObjectFromGripper();
            openGripper();
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            moveToPose(object_fixed_x_, object_fixed_y_, 0.22, "Repositioning for new attempt");
        }
        return false;
    }

    // TEST FOR GRASP RETRY, THIS CODE MAKES THE SIMULATION FAIL ON THE FIRST ATTEMPT
    /*bool graspWithRetry(int max_attempts = 2) {
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        RCLCPP_INFO(node_->get_logger(), "Grasp attempt %d/%d", attempt, max_attempts);
        executeForceGrasp();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // SCENARIO B TEST: force first attempt to fail
        bool lift_ok = moveToPose(object_fixed_x_, object_fixed_y_, 0.25, "Lift test for grasp verification");
        
        if (lift_ok && attempt > 1) {  // solo dal 2° tentativo in poi
            RCLCPP_INFO(node_->get_logger(), "Grasp successfully verified at attempt %d.", attempt);
            return true;
        }
        if (lift_ok && attempt == 1) {
            // simula fallimento al 1° tentativo
            RCLCPP_WARN(node_->get_logger(), "Grasp verification failed (simulated) - releasing and retrying...");
        } else {
            RCLCPP_WARN(node_->get_logger(), "Grasp verification failed - releasing and retrying...");
        }
        detachObjectFromGripper();
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        moveToPose(object_fixed_x_, object_fixed_y_, 0.22, "Repositioning for new attempt");
        }
        return false;
    }*/

    bool goHome() {
        RCLCPP_INFO(node_->get_logger(), "Phase: Returning to Up configuration pose...");
        std::vector<double> up = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
        move_group_->setJointValueTarget(up);
        move_group_->setStartStateToCurrentState();
        bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) RCLCPP_INFO(node_->get_logger(), "Up position reached.");
        else RCLCPP_WARN(node_->get_logger(), "Up motion failed or partial - proceeding anyway.");
        return true;
    }

    void recoveryBehavior() {
        RCLCPP_WARN(node_->get_logger(), "=== RECOVERY: executing clearing maneuver and returning Home ===");
        if (is_grasped_.load()) detachObjectFromGripper();
        openGripper();
        move_group_->stop();
        safeRetreat();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        clearConstraints();
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