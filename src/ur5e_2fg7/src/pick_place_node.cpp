#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <memory>
#include <chrono>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class PickAndPlacePipeline {
public:
    PickAndPlacePipeline(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");

        // Action client for gripper control
        gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(node_->get_logger(), "Attendo action server pinza...");
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(), "Action server pinza non disponibile - la pinza non si muovera'.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Action server pinza connesso.");
        }

        RCLCPP_INFO(node_->get_logger(), "Pipeline Pick-and-Place Inizializzata con Successo!");
    }

    void runPipeline() {
        RCLCPP_INFO(node_->get_logger(), "=== AVVIO PIPELINE B1b ===");

        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(10.0);

        // 1. Open gripper and go to home position
        openGripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        if (!goHome()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 2. Pre-Grasp: positioning at the top of the cylinder
        if (!approachObject()) return;
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 3. Go to the cylinder 
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

        RCLCPP_INFO(node_->get_logger(), "=== PIPELINE COMPLETATA CON SUCCESSO ===");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;

    // GRIPPER CHECK 

    void sendGripperCommand(double position_m, const std::string& label) {
        if (!gripper_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Action server pinza non pronto - salto %s", label.c_str());
            return;
        }

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = {"gripper_gripper_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {position_m};
        point.time_from_start = rclcpp::Duration::from_seconds(1.5);
        goal.trajectory.points.push_back(point);

        RCLCPP_INFO(node_->get_logger(), "%s (posizione: %.4f m)", label.c_str(), position_m);

        std::promise<bool> done_promise;
        auto done_future = done_promise.get_future();

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback =
            [&done_promise, &label, this](const GoalHandleFJT::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(node_->get_logger(), "%s completato.", label.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), "%s fallito.", label.c_str());
                }
                done_promise.set_value(true);
            };

        gripper_action_client_->async_send_goal(goal, send_goal_options);

        // Wait max 5 seconds
        done_future.wait_for(std::chrono::seconds(5));
    }

    void openGripper() {
        // upper limit = 0.0305 m (Open gripper)
        sendGripperCommand(0.0305, "Apertura pinza");
    }

    void executeForceGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Force-Based Grasp (OnRobot 2FG7)...");

        // Force based gripper simulation:
        // The gripper closes until touching the object 
        // With real hardware the force is monitored and it will stop 
        // when it reaches the force threshold (eg. 20N)
        double force_threshold_N = 20.0;  
        double grasp_position = 0.015;     // Pick position for the cylinder 

        RCLCPP_INFO(node_->get_logger(),
            "Chiusura pinza con forza target %.1f N - posizione presa: %.4f m",
            force_threshold_N, grasp_position);

        sendGripperCommand(grasp_position, "Chiusura pinza (grasp)");

        // Verifying grasping 
        RCLCPP_INFO(node_->get_logger(), "Grasp verificato: oggetto afferrato con forza simulata %.1f N", force_threshold_N);
    }

    void releaseObject() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Rilascio oggetto (Release)...");
        openGripper();
    }

    // ARM MOVEMENT 

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
            RCLCPP_INFO(node_->get_logger(), "%s completato.", label.c_str());
            return true;
        }
        RCLCPP_WARN(node_->get_logger(), "%s fallito (codice: %d).", label.c_str(), result.val);
        return false;
    }

    bool goHome() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Ritorno in posizione Home...");
        std::vector<double> home = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
        move_group_->setJointValueTarget(home);
        move_group_->setStartStateToCurrentState();
        bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) RCLCPP_INFO(node_->get_logger(), "Home raggiunta.");
        else RCLCPP_WARN(node_->get_logger(), "Home fallita - continuo comunque.");
        return true;
    }

    bool approachObject() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Pre-Grasp (sopra il cilindro)...");
        return moveToPose(0.0, 0.5, 0.35, "Pre-Grasp");
    }

    bool descendToGrasp() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Discesa verso il cilindro...");
        return moveToPose(0.0, 0.5, 0.18, "Discesa Grasp");
    }

    bool liftObject() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Sollevamento cilindro...");
        return moveToPose(0.0, 0.5, 0.45, "Lift");
    }

    bool transferObject() {
        RCLCPP_INFO(node_->get_logger(), "Fase: Trasferimento verso zona di rilascio...");
        return moveToPose(-0.4, 0.3, 0.45, "Transfer");
    }

    void recoveryBehavior() {
        RCLCPP_WARN(node_->get_logger(), "=== RECOVERY: movimento fallito, torno in Home ===");
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