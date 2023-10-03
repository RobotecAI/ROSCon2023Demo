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
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "constants.h"
#pragma once

namespace mtc = moveit::task_constructor;

namespace TaskConstructor
{



    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");

    class MTCController
    {
    public:
        static constexpr char PickupPoseName[] = "pickup";
        static constexpr char LiftPoseName[] = "lift";
        static constexpr char DropPoseName[] = "drop";

        MTCController(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface, std::string ns);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        bool setPosePIP(const std::string& poseName);

        bool setPosePIP(const Eigen::Vector3d &tcp_position, const Eigen::Quaterniond& tcp_orientation, float speed = 0.75f, const std::string& interpolation="PTP");

        Eigen::Quaterniond getCurrentOrientation();


    private:
        using PredefinePoseJointSpace = std::map<std::string, double>;
        std::unordered_map<std::string, PredefinePoseJointSpace> m_predefinedPoses;

        constexpr static float change = 0.35f;
        std::map<std::string, double> DropConfig;

        // Compose an MTC task from a series of stages.
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;

        std::vector<std::string> namesOfBoxes;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_groupIterface;
        std::string ns;
    };
} // namespace TaskConstructor
