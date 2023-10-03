#pragma once
#include <chrono>

#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <utility>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include "gripper.h"
#include "taskConstructor.h"
#include "utils.h"
#include "vision.h"


class PalletizationNode {

public:
    PalletizationNode(rclcpp::Node::SharedPtr node, std::string ns) ;


private:
    //! Reports status to ROS2
    void SendStatus(const char *format, ...);

    //! Check if the robot is stationary
    bool CheckIfAmrIsStationary(const geometry_msgs::msg::Pose& robotPose);

    //! Main loop of the node
    void TimerCallback();


    //!
    std::vector<Eigen::Vector3f> putBoxesInPlaces(std::shared_ptr<Palletization::RoboticArmController> mtc_task_node,
                                                  std::shared_ptr<Gripper::GripperController> gripperController,
                                                  const std::vector<Eigen::Vector3f> &targets);

    //! Executes the palletization task
    void execute(const std::string &amrName) ;

    rclcpp::Node::SharedPtr m_node;
    std::shared_ptr<Camera::GroundTruthCamera> m_visionSystem; //!< Wrapper for vision system
    std::chrono::seconds m_waitTime{3}; //!< Initial wait time for AMR to enter the scene
    std::unique_ptr<std::thread> m_threadPalletization; //!< Thread for palletization task
    moveit::core::RobotModelPtr m_kinematic_model; //!< Robotic arm model
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_groupIterface; //!< Move group interface
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> m_statusDescriptionPublisher; //!< Publisher for cargo status

    rclcpp::TimerBase::SharedPtr m_timer;
    const std::string m_ns; //!< Namespace of the node

    Eigen::Vector3d m_lastRobotPose{0, 0, 0};
    std::string m_robotName;
    std::atomic<bool> m_isExecuting{false}; //!< Flag indicating if task is executing

    double m_tolerance = 0.01; //!< Tolerance for checking if AMR moved
    size_t m_numOfBoxes = 1; //!< Number of boxes to be placed on pallet (parameter - set)


    std::chrono::time_point<std::chrono::system_clock> lastUpdate{std::chrono::system_clock::now()};
};


/// Entry point for the palletization node
int main(int argc, char **argv);