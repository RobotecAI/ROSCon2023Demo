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
#include <AzCore/Component/TickBus.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>

namespace ROS2::Demo
{
    using Tag = AZ::Crc32;

    class PayloadDespawnerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        static constexpr AZ::Crc32 BoxTag = AZ_CRC_CE("Box");
        static constexpr AZ::Crc32 AMRTag = AZ_CRC_CE("AMR");

        AZ_COMPONENT(PayloadDespawnerComponent, "{2ea1e599-014e-4a13-9ed9-d9653710756d}");
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerEnterHandler;

        bool IsObjectBox(const AZ::EntityId entityId) const;
        bool IsObjectAMR(const AZ::EntityId entityId) const;
    };
} // namespace ROS2::Demo
