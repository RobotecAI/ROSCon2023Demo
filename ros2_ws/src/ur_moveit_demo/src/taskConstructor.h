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
    const std::map<std::string, double> PickupConfig{
        { "wrist_1_joint", -0.8385483622550964 },      { "wrist_2_joint", 1.5643877983093262 },
        { "elbow_joint", -1.550349235534668 },         { "shoulder_pan_joint", -2.7139534950256348 },
        { "shoulder_lift_joint", -2.314471483230591 }, { "wrist_3_joint", -5.752989768981934 }
    };

    const std::map<std::string, double> LiftConfig{
        { "wrist_1_joint", -1.6993759870529175 },       { "wrist_2_joint", 1.5634684562683105 },
        { "elbow_joint", -1.0284128189086914 },         { "shoulder_pan_joint", -2.644500255584717 },
        { "shoulder_lift_joint", -1.9712634086608887 }, { "wrist_3_joint", -4.113039656 }
    };

    constexpr float change = 0.5f;
    const std::map<std::string, double> DropConfig{
        { "wrist_1_joint", -1.5789473056793213 - change }, { "wrist_2_joint", 1.5672444105148315 },
        { "elbow_joint", -0.9531064033508301 + change },   { "shoulder_pan_joint", -0.15679530799388885 },
        { "shoulder_lift_joint", -2.199610710144043 },     { "wrist_3_joint", -3.1959822177886963 }
    };

    constexpr float BoxHeight = 0.3f;
    const Eigen::Vector3d TableDimension{ 0.950, 0.950, 0.411 };
    const Eigen::Vector3d ConveyorDimensions{ 2.0, 1., 0.15 };
    const Eigen::Vector3d PickupLocation{ 0.890, 0, 0.049 };
    const Eigen::Vector3d BoxDimension{ 0.2, 0.2, 0.2 };
    const Eigen::Vector3d PalletDimensions{ 1.2, 0.769, 0.111 };

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");

    class MTCController
    {
    public:
        MTCController(rclcpp::Node::SharedPtr node);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        void doTask(mtc::Task& task);

        mtc::Task createTaskGrab(const geometry_msgs::msg::Pose& boxPose, std::string boxname);

        mtc::Task createTaskDrop(
            const Eigen::Vector3f address,
            std::string boxname,
            geometry_msgs::msg::Pose& palletPose,
            geometry_msgs::msg::Pose& boxPose,
            std::vector< geometry_msgs::msg::Pose> boxes);

        mtc::Task createTaskPark();

    private:
        // Compose an MTC task from a series of stages.
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;

        std::vector<std::string> namesOfBoxes;

    };
} // namespace TaskConstructor
