#pragma once

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit_msgs/msg/collision_object.hpp>

namespace Utils
{
    Eigen::Quaterniond FromMsgQuaternion(const geometry_msgs::msg::Quaternion& msg);

    Eigen::Vector3d FromMsgPosition(const geometry_msgs::msg::Point& msg);

    Eigen::Vector3d FromMsgPosition(const geometry_msgs::msg::Vector3& msg);

    geometry_msgs::msg::Point ToMsgPoint(const Eigen::Vector3d& v);

    geometry_msgs::msg::Quaternion ToMsgQuaternion(const Eigen::Quaterniond& q);

    Eigen::Quaterniond GetClosestQuaternionFromList(const Eigen::Quaterniond& q1, const std::vector<Eigen::Quaterniond>& qList);

    moveit_msgs::msg::CollisionObject CreateBoxCollision(
        const std::string& name,
        const Eigen::Vector3d& dimension,
        const Eigen::Vector3d& location,
        const Eigen::Quaterniond& rot = Eigen::Quaterniond::Identity(),
        const std::string& ns = "");

    geometry_msgs::msg::Pose GetBoxTargetPose(
        const Eigen::Vector3f& adress,
        const geometry_msgs::msg::Pose& palletPose,
        const Eigen::Vector3d& boxDimension,
        float separation = 1.05f);
} // namespace Utils
