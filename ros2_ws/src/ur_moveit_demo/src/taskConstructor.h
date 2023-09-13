#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "vision.h"
#include "utils.h"

#pragma once

namespace mtc = moveit::task_constructor;

namespace TaskConstructor
{

    constexpr float BoxHeight = 0.3f;
    const Eigen::Vector3d TableDimension{ 0.950, 0.950, 0.411 };
    const Eigen::Vector3d ConveyorDimensions{ 2.0, 1., 0.15 };
    const Eigen::Vector3d PickupLocation{ 0.890, 0, 0.049 };
    const Eigen::Vector3d BoxDimension{ 0.3, 0.3, 0.3 };
    const Eigen::Vector3d PalletDimensions{ 1.2, 0.769, 2.0*0.111 };

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");

    class MTCController
    {
    public:
        MTCController(rclcpp::Node::SharedPtr node, std::string ns);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        bool doTask(mtc::Task& task);

        mtc::Task createTaskGrab(const geometry_msgs::msg::Pose& boxPose, std::string boxname, std::string ns);

        mtc::Task createTaskDrop(
            const Eigen::Vector3f address,
            std::string boxname,
            geometry_msgs::msg::Pose& palletPose,
            geometry_msgs::msg::Pose& boxPose,
            std::vector<geometry_msgs::msg::Pose> boxes,
            std::string ns);

        mtc::Task createTaskPark(std::string ns);

    private:
        std::map<std::string, double> PickupConfig;

        std::map<std::string, double> LiftConfig;

        constexpr static float change = 0.4f;
        std::map<std::string, double> DropConfig;

        // Compose an MTC task from a series of stages.
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;

        std::vector<std::string> namesOfBoxes;

        std::string ns;
    };
} // namespace TaskConstructor
