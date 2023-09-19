/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LaneComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/string/string.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <lane_provider_msgs/msg/detail/lane_paths__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>

namespace ROS2::Demo
{

    void LaneComponent::Reflect(AZ::ReflectContext* context)
    {
        PathInfo::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LaneComponent>()->Version(1)->Field("PathsInLane", &LaneComponent::m_paths);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LaneComponent>("LaneComponent", "LaneComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LaneComponent::m_paths,
                        "Set of paths associated with one working lane",
                        "Set of paths associated with one working lane");
            }
        }
    }

    void LaneComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void LaneComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        for (auto const path : m_paths)
        {
            m_pathsMsgs[path.m_orderNumber] = CalculatePoses(path);
        }

        AZ::TickBus::Handler::BusDisconnect();
    }

    nav_msgs::msg::Path LaneComponent::CalculatePoses(const PathInfo& pathInfo)
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(
            splinePtr, pathInfo.m_entityId, &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(splineTransform, pathInfo.m_entityId, &AZ::TransformBus::Events::GetWorldTM);

        const size_t steps = pathInfo.m_poseCount - 1;

        nav_msgs::msg::Path path;
        path.header.frame_id = m_globalFrame.data();
        path.poses.reserve(pathInfo.m_poseCount);

        for (size_t i = 0; i <= steps; i++)
        {
            double fraction = float(i) / steps;
            if (pathInfo.m_reverseDirection)
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

            path.poses.push_back(pose);
        }

        return path;
    }

    void LaneComponent::Deactivate()
    {
    }

    AZStd::string LaneComponent::GetLaneName()
    {
        return GetEntity()->GetName();
    }

    lane_provider_msgs::msg::LanePaths LaneComponent::GetLanePathMsgs()
    {
        lane_provider_msgs::msg::LanePaths lanePaths;
        lanePaths.lane_name = GetLaneName().c_str();

        for (auto path : m_paths)
        {
            AZ::Entity* pathEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(pathEntity, &AZ::ComponentApplicationRequests::FindEntity, path.m_entityId);
            lanePaths.path_names.push_back(pathEntity->GetName().c_str());
            lanePaths.lane_paths.push_back(m_pathsMsgs[path.m_orderNumber]);
        }

        return lanePaths;
    }

} // namespace ROS2::Demo
