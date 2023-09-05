#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

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

#pragma once

namespace Camera
{
    class GroundTruthCamera
    {
    public:
        GroundTruthCamera(std::shared_ptr<rclcpp::Node> node, const std::string& topicPickup, const std::string& topicDrop, std::string ns);
        ~GroundTruthCamera() = default;

        std::optional<geometry_msgs::msg::Pose> getObjectPose(std::string objectName);

        std::optional<geometry_msgs::msg::Pose> getClosestBox();
        std::vector<geometry_msgs::msg::Pose> getAllBoxesOnPallet();

    private:
        void UpdateObjectPoses(
            std::map<std::string, geometry_msgs::msg::Pose>& objectPoses,
            const std::string& targetFrame,
            vision_msgs::msg::Detection3DArray::SharedPtr msg,
            std::vector<std::string> interestingObjectsPrefixes);

        std::optional<geometry_msgs::msg::Pose> getClosestBox(
            const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses, Eigen::Vector3d origin = Eigen::Vector3d::Zero());

        std::vector<geometry_msgs::msg::Pose> getAllBoxesOnPallet(const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses);

        std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection3DArray>> m_subscriberPickup;
        std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection3DArray>> m_subscriberDrop;

        std::mutex m_locationsMutex;
        std::optional<geometry_msgs::msg::Pose> m_closestBox;
        std::map<std::string, geometry_msgs::msg::Pose> m_objectPoses;
        std::vector<geometry_msgs::msg::Pose> m_allBoxesOnPallet;

        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

        static constexpr char BoxNamePrefix[] = "Box";
        static constexpr char PalletNamePrefix[] = "EuroPallet";
        static constexpr char PickedBoxName[] = "PickedBox";
    };
} // namespace Camera
