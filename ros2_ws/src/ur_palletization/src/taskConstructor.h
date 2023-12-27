#pragma once

#include "constants.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "utils.h"
#include "vision.h"

#include <memory>
#include <vector>

namespace Palletization
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");

    class RoboticArmController
    {
    public:
        static constexpr char PickupPoseName[] = "pickup";
        static constexpr char LiftPoseName[] = "lift";
        static constexpr char DropPoseName[] = "drop";

        RoboticArmController(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface, const std::string& ns);

        bool SetPosePIP(const std::string& poseName);

        bool SetPosePIP(
            const Eigen::Vector3d &tcpPosition,
            const Eigen::Quaterniond &tcpOrientation,
            float speed = 0.75f,
            const std::string& interpolation = "PTP");

        Eigen::Quaterniond GetCurrentOrientation() const;

    private:
        using PredefinePoseJointSpace = std::map<std::string, double>;

        // Compose an MTC task from a series of stages.
        rclcpp::Node::SharedPtr m_node;
        std::unordered_map<std::string, PredefinePoseJointSpace> m_predefinedPoses;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroupInterface;
        std::string m_ns;
    };
} // namespace TaskConstructor
