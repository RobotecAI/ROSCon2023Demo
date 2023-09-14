/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LanesService.h"

#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/Debug/Trace.h"
#include "AzCore/Math/Matrix3x3.h"
#include "AzCore/Math/Spline.h"
#include "AzCore/std/string/string.h"
#include "LmbrCentral/Shape/SplineComponentBus.h"
#include "Navigation/LaneComponent.h"
#include "ROS2/ROS2Bus.h"
#include <AzCore/Serialization/EditContext.h>
#include <lane_provider_msgs/msg/detail/lane_paths__struct.hpp>
#include <lane_provider_msgs/srv/detail/list_tracks__struct.hpp>

#include <memory>
#include <rclcpp/node.hpp>

namespace ROS2::Demo
{

    void LanesService::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LanesService>()
                ->Version(1)
                ->Field("LanesInWarehouse", &LanesService::m_laneEntities)
                ->Field("PoseCount", &LanesService::m_poseCount)
                ->Field("GlobalFrame", &LanesService::m_globalFrame);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LanesService>("LanesService", "LanesService")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LanesService::m_laneEntities,
                        "Set of lanes in the warehouse",
                        "Set of lanes in the warehouse")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LanesService::m_poseCount, "Pose Count", "Number of poses to create and publish")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LanesService::m_globalFrame, "Global Frame", "Global Frame");
            }
        }
    }

    void LanesService::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void LanesService::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_listTracksService = ros2Node->create_service<lane_provider_msgs::srv::ListTracks>(
            "get_available_lanes_and_its_paths",
            [this](const ListTracksRequest request, const ListTracksResponse response)
            {
                ListTracks(request, response);
            });
    }

    void LanesService::Deactivate()
    {
        m_listTracksService.reset();
    }

    void LanesService::ListTracks(const ListTracksRequest request, const ListTracksResponse response)
    {
        AZStd::string requestedLaneName(request->lane_name.c_str());

        if (requestedLaneName == "") // returns all lanes and their paths
        {
            for (const auto entityId : m_laneEntities)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                LaneComponent* laneComponent = entity->FindComponent<LaneComponent>();
                auto laneName = laneComponent->GetLaneName();
                response->names.push_back(laneName.c_str());
                lane_provider_msgs::msg::LanePaths lanePaths;
                lanePaths.lane_name = laneName.c_str();

                for (auto pathEntityId : laneComponent->GetPaths())
                {
                    AZ::ConstSplinePtr splinePtr{ nullptr };
                    LmbrCentral::SplineComponentRequestBus::EventResult(
                        splinePtr, entityId, &LmbrCentral::SplineComponentRequests::GetSpline);
                    AZ_Assert(splinePtr, "Spline pointer is null");

                    AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
                    AZ::TransformBus::EventResult(splineTransform, pathEntityId, &AZ::TransformBus::Events::GetWorldTM);

                    const size_t steps = m_poseCount - 1;

                    nav_msgs::msg::Path path;
                    path.header.frame_id = m_globalFrame.data();
                    path.poses.reserve(m_poseCount);

                    for (size_t i = 0; i <= steps; i++)
                    {
                        double fraction = float(i) / steps;

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

                        AZ_Info(
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

                    AZ::Entity* pathEntity = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(pathEntity, &AZ::ComponentApplicationRequests::FindEntity, pathEntityId);
                    lanePaths.path_names.push_back(pathEntity->GetName().c_str());
                    lanePaths.lane_paths.push_back(path);
                }
                AZ_Printf("lane_service: %s", laneName.c_str());
            }
        }
        else // returns paths with given lane name
        {
            for (const auto entityId : m_laneEntities)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                LaneComponent* laneComponent = entity->FindComponent<LaneComponent>();
                auto laneName = laneComponent->GetLaneName();
                if (requestedLaneName == laneName)
                {
                    
                }
            }

            // response->names
            // if there's no such lane returns empty msg
        }
    }

} // namespace ROS2::Demo
