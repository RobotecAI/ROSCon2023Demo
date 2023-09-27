/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#include <HumanNpc/NpcNavigatorBus.h>
#include <HumanNpc/WaypointSelectorComponent.h>

namespace ROS2::Demo
{
    void WaypointSelectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WaypointSelectorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Seed", &WaypointSelectorComponent::m_seed)
                ->Field("Human Npcs", &WaypointSelectorComponent::m_humanNpcs)
                ->Field("Waypoint Entities", &WaypointSelectorComponent::m_waypoints);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext
                    ->Class<WaypointSelectorComponent>(
                        "Waypoint Selector",
                        "Component used to construct a randomized path from a provided waypoint set for selected Npc Navigators.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &WaypointSelectorComponent::m_seed, "Seed", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointSelectorComponent::m_humanNpcs,
                        "HumanNpcs",
                        "Entities with the NpcNavigator components.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointSelectorComponent::m_waypoints,
                        "Waypoints",
                        "Entities with the Waypoint components.");
                // clang-format on
            }
        }
    }

    void WaypointSelectorComponent::Activate()
    {
        m_mersenneTwister.seed(m_seed);
        for (AZ::EntityId humanNpcEntityId : m_humanNpcs)
        {
            AZ::EntityBus::MultiHandler::BusConnect(humanNpcEntityId);
        }
    }

    void WaypointSelectorComponent::Deactivate()
    {
        for (size_t i = m_humanNpcs.size() - 1; i <= 0; --i)
        {
            AZ::EntityBus::MultiHandler::BusDisconnect(m_humanNpcs[i]);
        }
    }

    void WaypointSelectorComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        NpcNavigatorRequestBus::Event(entityId, &NpcNavigatorRequests::ClearWaypoints);
        const AZStd::vector<AZ::EntityId> Waypoints = SelectWaypointPah();
        for (AZ::EntityId waypointEntityId : Waypoints)
        {
            NpcNavigatorRequestBus::Event(entityId, &NpcNavigatorRequests::AddWaypoint, waypointEntityId);
        }
    }

    AZStd::vector<AZ::EntityId> WaypointSelectorComponent::SelectWaypointPah()
    {
        if (m_waypoints.empty())
        {
            AZ_Printf(__func__, "No waypoint entities to select from.") return {};
        }

        std::uniform_int_distribution<size_t> uniformDistribution(0, m_waypoints.size());
        AZStd::vector<AZ::EntityId> waypointPath;
        while (waypointPath.size() != m_waypoints.size())
        {
            size_t index = uniformDistribution(m_mersenneTwister);
            while (!waypointPath.empty() && (m_waypoints[index] == waypointPath.back())) // Disallow duplicate waypoint sequences.
            {
                index = uniformDistribution(m_mersenneTwister);
            }

            waypointPath.push_back(m_waypoints[index]);
        }

        return waypointPath;
    }
} // namespace ROS2::Demo