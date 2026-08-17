#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using FollowJointTraj = control_msgs::action::FollowJointTrajectory;

rclcpp::Node::SharedPtr g_node;

// ── GRIPPER ───────────────────────────────────────────────────────────────────
void set_gripper(double position, double duration_sec = 1.5)
{
    auto client = rclcpp_action::create_client<FollowJointTraj>(
        g_node, "/fr3_gripper/follow_joint_trajectory");
    if (!client->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_WARN(g_node->get_logger(), "Gripper action server non trovato");
        return;
    }
    FollowJointTraj::Goal goal;
    goal.trajectory.joint_names = {"fr3_finger_joint1", "fr3_finger_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {position, position};
    point.velocities = {0.0, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(point);
    client->async_send_goal(goal);
    rclcpp::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_sec * 1000) + 300));
}

// ── OBSTACLE ──────────────────────────────────────────────────────────────────
void add_obstacle_to_scene(const geometry_msgs::msg::Point &obs,
                            double size_x = 0.08,
                            double size_y = 0.08,
                            double size_z = 0.15)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject co;
    co.id              = "blue_obstacle";
    co.header.frame_id = "world";
    co.operation       = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size_x, size_y, size_z};
    geometry_msgs::msg::Pose pose;
    pose.position      = obs;
    pose.orientation.w = 1.0;
    co.primitives.push_back(box);
    co.primitive_poses.push_back(pose);
    psi.applyCollisionObject(co);
    RCLCPP_INFO(g_node->get_logger(),
                "Ostacolo aggiunto @ (%.2f, %.2f, %.2f)", obs.x, obs.y, obs.z);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}

// ── MOVE TO (pianificazione libera) ───────────────────────────────────────────
bool move_to(moveit_cpp::MoveItCpp        &moveit_cpp_inst,
             moveit_cpp::PlanningComponent &arm,
             double x, double y, double z,
             double qx = 0.0, double qy = 1.0,
             double qz = 0.0, double qw = 0.0)
{
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id    = "world";
    target.pose.position.x    = x;
    target.pose.position.y    = y;
    target.pose.position.z    = z;
    target.pose.orientation.x = qx;
    target.pose.orientation.y = qy;
    target.pose.orientation.z = qz;
    target.pose.orientation.w = qw;

    arm.setGoal(target, "fr3_hand_tcp");

    moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
    plan_params.planning_pipeline               = "move_group";
    plan_params.planner_id                      = "RRTConnect";
    plan_params.planning_attempts               = 3;
    plan_params.planning_time                   = 5.0;
    plan_params.max_velocity_scaling_factor     = 0.4;
    plan_params.max_acceleration_scaling_factor = 0.15;

    auto plan = arm.plan(plan_params);
    if (!plan) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "Pianificazione FALLITA verso (%.3f, %.3f, %.3f)", x, y, z);
        return false;
    }
    bool ok = moveit_cpp_inst.execute("fr3_arm", plan.trajectory, true);
    if (!ok) {
        RCLCPP_ERROR(g_node->get_logger(), "Esecuzione FALLITA");
        return false;
    }
    return true;
}

// ── MOVE CARTESIAN (lista waypoints) ─────────────────────────────────────────
bool move_cartesian(moveit::planning_interface::MoveGroupInterface &mgi,
                    const std::vector<geometry_msgs::msg::Pose>   &waypoints)
{
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step    = 0.003;
    const double jump_thresh = 0.0;
    double fraction = mgi.computeCartesianPath(waypoints, eef_step,
                                               jump_thresh, trajectory);
    if (fraction < 0.9) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "Cartesiano: percorso incompleto (%.1f%%)", fraction * 100.0);
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto result = mgi.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Esecuzione cartesiana FALLITA");
        return false;
    }
    return true;
}

// ── MOVE CARTESIAN (singolo punto) ────────────────────────────────────────────
bool move_cartesian(moveit::planning_interface::MoveGroupInterface &mgi,
                    double x, double y, double z,
                    double qx = 0.0, double qy = 1.0,
                    double qz = 0.0, double qw = 0.0)
{
    geometry_msgs::msg::Pose target;
    target.position.x    = x;
    target.position.y    = y;
    target.position.z    = z;
    target.orientation.x = qx;
    target.orientation.y = qy;
    target.orientation.z = qz;
    target.orientation.w = qw;
    return move_cartesian(mgi, std::vector<geometry_msgs::msg::Pose>{target});
}

// ── PICK AND PLACE ────────────────────────────────────────────────────────────
void pick_and_place(moveit_cpp::MoveItCpp           &moveit_cpp_inst,
                    moveit_cpp::PlanningComponent   &arm,
                    moveit::planning_interface::MoveGroupInterface &mgi,
                    const geometry_msgs::msg::Point &cube,
                    const geometry_msgs::msg::Point              &goal,
                    const geometry_msgs::msg::Point              &obstacle)
{
    constexpr double HOVER      = 0.20;
    constexpr double CUBE_HALF  = 0.03;
    constexpr double TARGET_TOP = 0.02;

    RCLCPP_INFO(g_node->get_logger(),
        "cube=(%.3f,%.3f,%.3f) goal=(%.3f,%.3f,%.3f) obs=(%.3f,%.3f,%.3f)",
        cube.x, cube.y, cube.z,
        goal.x, goal.y, goal.z,
        obstacle.x, obstacle.y, obstacle.z);

    add_obstacle_to_scene(obstacle, 0.42, 0.12, 0.27);

    RCLCPP_INFO(g_node->get_logger(), "[1/9] Apertura gripper");
    set_gripper(0.04);

    RCLCPP_INFO(g_node->get_logger(), "[2/9] Pre-grasp sopra il cubo");
    if (!move_to(moveit_cpp_inst, arm, cube.x, cube.y, cube.z + HOVER)) return;

    RCLCPP_INFO(g_node->get_logger(), "[3/9] Discesa cartesiana sul cubo");
    if (!move_cartesian(mgi, cube.x, cube.y, cube.z)) return;

    RCLCPP_INFO(g_node->get_logger(), "[4/9] Chiusura gripper");
    set_gripper(0.017, 3.0);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(g_node->get_logger(), "[5/9] Sollevamento cubo (cartesiano)");
    if (!move_cartesian(mgi, cube.x, cube.y, cube.z + HOVER)) return;

    RCLCPP_INFO(g_node->get_logger(), "[6/9] Trasporto verso goal");
    if (!move_to(moveit_cpp_inst, arm,
                 goal.x, goal.y, goal.z + TARGET_TOP + CUBE_HALF + HOVER)) return;

    RCLCPP_INFO(g_node->get_logger(), "[7/9] Deposito al goal (cartesiano)");
    if (!move_cartesian(mgi, goal.x, goal.y, goal.z + TARGET_TOP + CUBE_HALF)) return;

    RCLCPP_INFO(g_node->get_logger(), "[8/9] Apertura gripper");
    set_gripper(0.04);

    RCLCPP_INFO(g_node->get_logger(), "[9/9] Ritiro braccio");
    move_cartesian(mgi, goal.x, goal.y, goal.z + TARGET_TOP + CUBE_HALF + HOVER);

    RCLCPP_INFO(g_node->get_logger(), "Missione completata!");
}