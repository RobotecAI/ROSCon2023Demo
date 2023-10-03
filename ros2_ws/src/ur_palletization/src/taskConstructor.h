#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include "vision.h"
#include "utils.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "constants.h"

#pragma once

namespace Palletization
{

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");

    class RoboticArmController
    {
    public:
        static constexpr char PickupPoseName[] = "pickup";
        static constexpr char LiftPoseName[] = "lift";
        static constexpr char DropPoseName[] = "drop";

        RoboticArmController(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface, std::string ns);

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
        rclcpp::Node::SharedPtr node_;

        std::vector<std::string> namesOfBoxes;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_groupIterface;
        std::string ns;
    };
} // namespace TaskConstructor
