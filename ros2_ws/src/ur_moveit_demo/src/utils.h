#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit_msgs/msg/collision_object.hpp>

#pragma once

namespace Utils
{

    Eigen::Quaterniond fromMsgQuaternion(const geometry_msgs::msg::Quaternion& msg);

    Eigen::Vector3d fromMsgPosition(const geometry_msgs::msg::Point msg);

    Eigen::Vector3d fromMsgPosition(const geometry_msgs::msg::Vector3 msg);

    const geometry_msgs::msg::Point toMsgPoint(Eigen::Vector3d v);

    const geometry_msgs::msg::Quaternion toMsgQuaternion(const Eigen::Quaterniond& q);

    moveit_msgs::msg::CollisionObject CreateBoxCollision(
        const std::string& name,
        const Eigen::Vector3d dimension,
        const Eigen::Vector3d location,
        const Eigen::Quaterniond& rot = Eigen::Quaterniond::Identity(),
        std::string ns = "");

    geometry_msgs::msg::Pose getBoxTargetPose(
        const Eigen::Vector3f& adress,
        const geometry_msgs::msg::Pose& palletPose,
        const Eigen::Vector3d& boxDimension,
        float separation = 1.1f);

} // namespace Utils
