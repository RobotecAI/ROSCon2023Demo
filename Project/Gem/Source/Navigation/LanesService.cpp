/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LanesService.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/string/string.h>
#include <Navigation/LaneComponent.h>
#include <ROS2/ROS2Bus.h>
#include <lane_provider_msgs/srv/detail/list_tracks__struct.hpp>
#include <rclcpp/node.hpp>

namespace ROS2::Demo
{

    void LanesService::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LanesService>()
                ->Version(1)
                ->Field("LanesInWarehouse", &LanesService::m_lanesEntities)
                ->Field("GlobalFrame", &LanesService::m_globalFrame);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LanesService>("LanesService", "LanesService")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LanesService::m_lanesEntities,
                        "Set of lanes in the warehouse",
                        "Set of lanes in the warehouse")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LanesService::m_globalFrame, "Global Frame", "Global Frame");
            }
        }
    }

    void LanesService::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_listTracksService = ros2Node->create_service<lane_provider_msgs::srv::ListTracks>(
            "get_lanes_and_paths",
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

        if (requestedLaneName == "")
        {
            for (const auto entityId : m_lanesEntities)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                LaneComponent* laneComponent = entity->FindComponent<LaneComponent>();
                auto laneName = laneComponent->GetLaneName();
                response->names.push_back(laneName.c_str());
                response->lane_paths.push_back(laneComponent->GetLanePathMsgs());
            }
        }
        else
        {
            for (const auto entityId : m_lanesEntities)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                LaneComponent* laneComponent = entity->FindComponent<LaneComponent>();
                auto laneName = laneComponent->GetLaneName();
                if (requestedLaneName == laneName)
                {
                    response->names.push_back(laneName.c_str());
                    response->lane_paths.push_back(laneComponent->GetLanePathMsgs());
                    return;
                }
            }
        }
    }

} // namespace ROS2::Demo
