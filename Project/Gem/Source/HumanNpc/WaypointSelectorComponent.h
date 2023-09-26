/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/containers/vector.h>
#include <random>

namespace ROS2::Demo
{
    //! Component used to construct a randomized path
    //! from a provided waypoint set for selected Npc Navigators.
    class WaypointSelectorComponent
        : public AZ::Component
        , private AZ::EntityBus::MultiHandler
    {
    public:
        AZ_COMPONENT(WaypointSelectorComponent, "{56293984-7751-4556-9aa7-a85906385992}", AZ::Component);

        WaypointSelectorComponent() = default;
        ~WaypointSelectorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        // clang-format off
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required) {}
        // clang-format on

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::EntityBus overrides
        void OnEntityActivated(const AZ::EntityId&) override;

        AZStd::vector<AZ::EntityId> SelectWaypointPah();

        size_t m_seed{ 0 };
        std::mt19937 m_mersenneTwister{ m_seed };
        AZStd::vector<AZ::EntityId> m_humanNpcs, m_waypoints;
    };
} // namespace ROS2::Demo
