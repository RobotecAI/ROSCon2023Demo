#include "vision.h"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace Camera
{
    GroundTruthCamera::GroundTruthCamera(
        std::shared_ptr<rclcpp::Node> node, const std::string& topicPickup, const std::string& topicDrop, const std::string& ns)
    {
      m_tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        m_subscriberDrop = node->create_subscription<vision_msgs::msg::Detection3DArray>(
            topicDrop,
            10,
            [&, ns](vision_msgs::msg::Detection3DArray::SharedPtr msg)
            {
                UpdateObjectPoses(m_objectPoses, ns + "/world", msg, { PalletNamePrefix, RobotPrefix });
                UpdateObjectPresence(msg, { RobotPrefix }, m_isRobotPresent, m_robotName);
                std::map<std::string, geometry_msgs::msg::Pose> boxesPoses;
                UpdateObjectPoses(boxesPoses, ns + "/world", msg, { BoxNamePrefix });
                std::lock_guard<std::mutex> lock(m_locationsMutex);
                m_allBoxesOnPallet = GetAllBoxesOnPallet(boxesPoses);
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
                m_closestBox = GetClosestBox(poses);
            });
    }

    bool GroundTruthCamera::IsRobotPresent() const
    {
        return m_isRobotPresent;
    }

    std::string GroundTruthCamera::GetRobotName() const
    {
        return m_robotName;
    }

    void GroundTruthCamera::UpdateObjectPresence(
        vision_msgs::msg::Detection3DArray::SharedPtr msg,
        const std::vector<std::string>& interestingObjectsPrefixes,
        bool& objectPresence,
        std::string& objectName)
    {
        bool interesting = false;
        for (auto& detection : msg->detections)
        {
            const std::string name{ detection.id };
            for (const auto& interestingObject : interestingObjectsPrefixes)
            {
                if (name.find(interestingObject) != std::string::npos)
                {
                    interesting = true;
                    objectName = name;
                    objectPresence = true;
                    return;
                }
            }
            if (!interesting || detection.results.empty())
            {
                continue;
            }
        }

        objectPresence = false;
        objectName = "";
    }

    void GroundTruthCamera::UpdateObjectPoses(
        std::map<std::string, geometry_msgs::msg::Pose>& objectPoses,
        const std::string& targetFrame,
        vision_msgs::msg::Detection3DArray::SharedPtr msg,
        const std::vector<std::string>& interestingObjectsPrefixes)
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
                transform = m_tfBuffer->lookupTransform(targetFrame, pose.header.frame_id, tf2::TimePointZero, tf2::Duration(0));
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

    std::optional<geometry_msgs::msg::Pose> GroundTruthCamera::GetClosestBox(
        const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses, const Eigen::Vector3d& origin)
    {
        std::map<float, geometry_msgs::msg::Pose> boxLocations;
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
            Eigen::Vector3d boxLocation{ it->second.position.x, it->second.position.y, it->second.position.z };
            float distance = static_cast<float>(std::abs(origin.x() - boxLocation.x()));
            boxLocations[distance] = it->second;
        }
        return std::optional<geometry_msgs::msg::Pose>(boxLocations.begin()->second);
    }

    std::vector<geometry_msgs::msg::Pose> GroundTruthCamera::GetAllBoxesOnPallet(
        const std::map<std::string, geometry_msgs::msg::Pose>& objectPoses)
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

    std::optional<geometry_msgs::msg::Pose> GroundTruthCamera::GetObjectPose(const std::string& objectName) const
    {
        auto it = std::lower_bound(
            m_objectPoses.begin(),
            m_objectPoses.end(),
            objectName,
            [](const auto& lhs, const auto& rhs)
            {
                return lhs.first < rhs;
            });
        if (it == m_objectPoses.end() || it->first.find(objectName) == std::string::npos)
        {
            std::cerr << objectName << " not found, detected objects: " << std::endl;
            for (it = m_objectPoses.begin(); it != m_objectPoses.end(); ++it)
            {
                std::cerr << it->first << std::endl;
            }
            return std::optional<geometry_msgs::msg::Pose>();
        }
        return std::optional<geometry_msgs::msg::Pose>(it->second);
    }

    std::optional<geometry_msgs::msg::Pose> GroundTruthCamera::GetClosestBox()
    {
        std::lock_guard<std::mutex> lock(m_locationsMutex);
        return m_closestBox;
    }

    std::vector<geometry_msgs::msg::Pose> GroundTruthCamera::GetAllBoxesOnPallet()
    {
        std::lock_guard<std::mutex> lock(m_locationsMutex);
        return m_allBoxesOnPallet;
    }
} // namespace Camera
