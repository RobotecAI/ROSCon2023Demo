/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/EntityId.h"
#include "AzCore/std/containers/vector.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>

namespace ROS2::Demo
{
    using Tag = AZ::Crc32;

    //! The PayloadDespawner, when triggered by the collider attached to an entity, performs the following actions:
    //! - It despawns the payload on the AMR.
    //! - It notifies OTTO deliberation about the change.
    //! It despawns all entities that have been spawned with ScriptSpawnSystemBus and have the specified tag.
    //! After `m_despawnerDelay`, the boxes are despawned and the component notifies via topic that tha cargo status has changed.

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
        float m_despawnerDelay = 15.0;
        float m_timer = 0.0f;

        bool IsObjectBox(const AZ::EntityId entityId) const;
        bool IsObjectAMR(const AZ::EntityId entityId) const;
        AZStd::vector<AZ::EntityId> m_collidingBoxes;
        AZ::EntityId m_collidingAmr{ AZ::EntityId::InvalidEntityId };
    };
} // namespace ROS2::Demo
