#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "myworkcell_core/perception_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>

extern rclcpp::Node::SharedPtr g_node;

void pick_and_place(moveit_cpp::MoveItCpp           &moveit_cpp_inst,
                    moveit_cpp::PlanningComponent   &arm,
                    moveit::planning_interface::MoveGroupInterface &mgi,
                    const geometry_msgs::msg::Point &cube,
                    const geometry_msgs::msg::Point &goal,
                    const geometry_msgs::msg::Point &obstacle);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PerceptionNode>(node_options);
    g_node = node;

    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    RCLCPP_INFO(node->get_logger(), "Attendo stabilizzazione sistema (10s)...");
    rclcpp::sleep_for(std::chrono::seconds(10));

    RCLCPP_INFO(node->get_logger(), "Inizializzo MoveItCpp...");
    moveit_cpp::MoveItCpp::Options moveit_options(node);
    auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_options);
    moveit_cpp->getPlanningSceneMonitor()->requestPlanningSceneState();
    moveit_cpp::PlanningComponent arm("fr3_arm", moveit_cpp);
    moveit::planning_interface::MoveGroupInterface mgi(node, "fr3_arm");
    RCLCPP_INFO(node->get_logger(), "MoveItCpp OK");

    RCLCPP_INFO(node->get_logger(),
                "Aspetto che la perception individui tutti e 3 gli oggetti...");
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !node->positions_ready_)
        rate.sleep();

    if (rclcpp::ok() && !node->mission_done_) {
        RCLCPP_INFO(node->get_logger(), "Oggetti trovati → avvio pick and place");

        geometry_msgs::msg::Point cube_snapshot     = node->cube_pos_;
        geometry_msgs::msg::Point goal_snapshot     = node->goal_pos_;
        geometry_msgs::msg::Point obstacle_snapshot = node->obstacle_pos_;
        node->mission_done_ = true;

        pick_and_place(*moveit_cpp, arm, mgi,
                       cube_snapshot, goal_snapshot, obstacle_snapshot);
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
