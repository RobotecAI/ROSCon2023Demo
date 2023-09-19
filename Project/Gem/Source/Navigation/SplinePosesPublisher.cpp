/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "SplinePosesPublisher.h"
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <rclcpp/qos.hpp>

namespace ROS2::Demo
{

    void SplinePosesPublisher::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SplinePosesPublisher>()
                ->Version(1)
                ->Field("ReverseDirection", &SplinePosesPublisher::m_reverseDirection)
                ->Field("PathTopicConfig", &SplinePosesPublisher::m_topicName)
                ->Field("PoseCount", &SplinePosesPublisher::m_poseCount)
                ->Field("GlobalFrame", &SplinePosesPublisher::m_globalFrame);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SplinePosesPublisher>("SplinePosesPublisher", "SplinePosesPublisher")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplinePosesPublisher::m_reverseDirection,
                        "Reverse Direction",
                        "Reverse the robot movement direction")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SplinePosesPublisher::m_topicName, "Topic Name", "Topic name to publish poses")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplinePosesPublisher::m_poseCount,
                        "Pose Count",
                        "Number of poses to create and publish")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplinePosesPublisher::m_globalFrame, "Global Frame", "Global Frame");
            }
        }
    }

    void SplinePosesPublisher::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void SplinePosesPublisher::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_path = CalculatePoses();

        rclcpp::QoS qos(5);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);

        m_pathPublisher = ros2Node->create_publisher<nav_msgs::msg::Path>(m_topicName.data(), qos);
        m_pathPublisher->publish(m_path);
    }

    void SplinePosesPublisher::Deactivate()
    {
        m_pathPublisher.reset();
    }

    nav_msgs::msg::Path SplinePosesPublisher::CalculatePoses()
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        const size_t steps = m_poseCount - 1;

        nav_msgs::msg::Path path;
        path.header.frame_id = m_globalFrame.data();
        path.poses.reserve(m_poseCount);

        for (size_t i = 0; i <= steps; i++)
        {
            double fraction = float(i) / steps;
            if (m_reverseDirection)
            {
                fraction = 1 - fraction;
            }

            auto address = splinePtr->GetAddressByFraction(fraction);
            const AZ::Vector3 position_ = splinePtr->GetPosition(address);
            const AZ::Vector3 tangent_ = splinePtr->GetTangent(address);
            const AZ::Vector3 normal_ = splinePtr->GetNormal(address);

            const AZ::Matrix3x3 rot_ = AZ::Matrix3x3::CreateFromColumns(tangent_, normal_, tangent_.Cross(normal_));
            const AZ::Transform goalTransform_ = AZ::Transform::CreateFromMatrix3x3AndTranslation(rot_, position_);
            auto m_idealGoal_ = splineTransform * goalTransform_;

            auto pose = geometry_msgs::msg::PoseStamped();

            pose.header.frame_id = m_globalFrame.data();

            auto translation = m_idealGoal_.GetTranslation();

            pose.pose.position.x = translation.GetX();
            pose.pose.position.y = translation.GetY();
            pose.pose.position.z = translation.GetZ();

            auto rotation = m_idealGoal_.GetRotation();

            pose.pose.orientation.x = rotation.GetX();
            pose.pose.orientation.y = rotation.GetY();
            pose.pose.orientation.z = rotation.GetZ();
            pose.pose.orientation.w = rotation.GetW();

            AZ_Trace(
                "SplinePosesPublisher",
                "Calculated point at fraction %f:  position: %f %f %f rotation: %f %f %f %f",
                float(i) / steps,
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);

            path.poses.push_back(pose);
        }

        return path;
    }

} // namespace ROS2::Demo
