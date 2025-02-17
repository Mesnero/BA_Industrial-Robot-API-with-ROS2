#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <vector>
#include <cmath>

geometry_msgs::msg::Pose computeCirclePoint(double radius, double angle) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = radius * cos(angle);
    pose.position.y = radius * sin(angle);
    pose.position.z = 0.8; // Set a fixed Z-axis height

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  // Maintain a fixed orientation
    pose.orientation = tf2::toMsg(q);

    return pose;
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "circle_trajectory_follower", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("circle_trajectory_follower");

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = std::make_shared<MoveGroupInterface>(node, "arm");

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface->getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Set up circular trajectory parameters
    double radius = 0.1; // Radius of the circle
    double angle_increment = 0.001; // Angle increment, adjust to achieve smooth motion

    // Container for storing waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (double angle = 0.0; angle <= 2 * M_PI; angle += angle_increment) {
        waypoints.push_back(computeCirclePoint(radius, angle));
    }

    while (rclcpp::ok()) {
        // Ensure the starting position is correct
        move_group_interface->setStartStateToCurrentState();

        // Compute Cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.1;  // Control the end-effector step size
        double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.99) {
            RCLCPP_WARN(logger, "Could only compute %f of the path.", fraction);
        } else {
            // Execute the planned path
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            if (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(logger, "Path execution complete.");
            } else {
                RCLCPP_ERROR(logger, "Path execution failed.");
            }
        }

    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
