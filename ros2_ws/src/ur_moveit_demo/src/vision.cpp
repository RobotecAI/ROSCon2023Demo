#include "vision.h"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace Camera
{
    GroundTruthCamera::GroundTruthCamera(
        std::shared_ptr<rclcpp::Node> node, const std::string& topicPickup, const std::string& topicDrop, std::string ns)
    {
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_subscriberDrop = node->create_subscription<vision_msgs::msg::Detection3DArray>(
            topicDrop,
            10,
            [&, ns](vision_msgs::msg::Detection3DArray::SharedPtr msg)
            {
                UpdateObjectPoses(m_objectPoses, ns + "/world", msg, { PalletNamePrefix });
                std::map<std::string, geometry_msgs::msg::Pose> boxesPoses;
                UpdateObjectPoses(boxesPoses, ns + "/world", msg, { BoxNamePrefix });
                std::lock_guard<std::mutex> lock(m_locationsMutex);
                m_allBoxesOnPallet = getAllBoxesOnPallet(boxesPoses);
            });
        m_subscriberPickup = node->create_subscription<vision_msgs::msg::Detection3DArray>(
            topicPickup,
            10,
            [&, ns](vision_msgs::msg::Detection3DArray::SharedPtr msg)
            {
                UpdateObjectPoses(m_objectPoses, ns + "/world", msg, { BoxNamePrefix });
                std::map<std::string, geometry_msgs::msg::Pose> poses;
                UpdateObjectPoses(poses, ns + "/world", msg, { BoxNamePrefix });
                std::lock_guard<std::mutex> lock(m_locationsMutex);
                m_closestBox = getClosestBox(poses);
            });
    }

    void GroundTruthCamera::UpdateObjectPoses(
        std::map<std::string, geometry_msgs::msg::Pose>& objectPoses,
        const std::string& targetFrame,
        vision_msgs::msg::Detection3DArray::SharedPtr msg,
        std::vector<std::string> interestingObjectsPrefixes)
    {
        for (auto& detection : msg->detections)
        {
            const std::string name{ detection.id };
            bool interesting = false;
            for (const auto& interestingObject : interestingObjectsPrefixes)
            {
                if (name.find(interestingObject) != std::string::npos)
                {
                    interesting = true;
                    break;
                }
            }
            if (!interesting || detection.results.empty())
            {
                continue;
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header = detection.header;
            pose.pose = detection.results[0].pose.pose;
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = m_tf_buffer->lookupTransform(targetFrame, pose.header.frame_id, tf2::TimePointZero, tf2::Duration(0));
            } catch (tf2::TransformException& ex)
            {
                std::cerr << ex.what() << std::endl;
                return;
            }
            geometry_msgs::msg::PoseStamped transformedPose;
            tf2::doTransform(pose, transformedPose, transform);
            objectPoses[detection.id] = transformedPose.pose;
        }
    }

    std::optional<geometry_msgs::msg::Pose> GroundTruthCamera::getClosestBox(
        const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses, Eigen::Vector3d origin)
    {
        std::map<float, geometry_msgs::msg::Pose> box_locations;
        auto it = objectPoses.begin();
        if (it == objectPoses.end())
        {
            std::cerr << "Box not found" << std::endl;
            return std::optional<geometry_msgs::msg::Pose>();
        }
        for (; it != objectPoses.end(); ++it)
        {
            if (it->first.find(BoxNamePrefix) == std::string::npos)
            {
                continue;
            }
            Eigen::Vector3d box_location{ it->second.position.x, it->second.position.y, it->second.position.z };
            float distance = std::abs(origin.x() - box_location.x());
            box_locations[distance] = it->second;
        }
        return std::optional<geometry_msgs::msg::Pose>(box_locations.begin()->second);
    }

    std::vector<geometry_msgs::msg::Pose> GroundTruthCamera::getAllBoxesOnPallet(const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses)
    {
        std::vector<geometry_msgs::msg::Pose> boxes;
        auto it = objectPoses.begin();
        if (it == objectPoses.end())
        {
            return boxes;
        }
        for (; it != objectPoses.end(); ++it)
        {
            if (it->first.find(BoxNamePrefix) == std::string::npos)
            {
                continue;
            }
            boxes.push_back(it->second);
        }

        return boxes;
    }

    geometry_msgs::msg::Pose GroundTruthCamera::getObjectPose(std::string objectName)
    {
        auto it = std::lower_bound(
            m_objectPoses.begin(),
            m_objectPoses.end(),
            PalletNamePrefix,
            [](const auto& lhs, const auto& rhs)
            {
                return lhs.first < rhs;
            });
        if (it == m_objectPoses.end() || it->first.find(objectName) == std::string::npos)
        {
            std::cerr << objectName << " not found, detected objects: " << std::endl;
            for (auto it = m_objectPoses.begin(); it != m_objectPoses.end(); ++it)
            {
                std::cerr << it->first << std::endl;
            }
            std::abort();
        }
        return it->second;
    }

    std::optional<geometry_msgs::msg::Pose> GroundTruthCamera::getClosestBox()
    {
        std::lock_guard<std::mutex> lock(m_locationsMutex);
        return m_closestBox;
    }

    std::vector<geometry_msgs::msg::Pose> GroundTruthCamera::getAllBoxesOnPallet()
    {
        std::lock_guard<std::mutex> lock(m_locationsMutex);
        return m_allBoxesOnPallet;
    }

} // namespace Camera
