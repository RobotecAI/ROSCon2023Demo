/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "ScriptSpawnSytemBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>

namespace ROS2::Demo
{
   class FoilWrapperConfig
   {
   public:
       AZ_TYPE_INFO(FoilWrapperConfig, "{a806a1af-5fd4-4859-8974-67116e844fcc}");

       static void Reflect(AZ::ReflectContext* context);
       AZ::EntityId m_foilWrapperEntityId; //!< EntityId of the foil wrapper entity
       AZ::EntityId m_collisionTrigger; //!< EntityId of the foil entity
       AZ::Data::Asset<AzFramework::Spawnable> m_spawnablePayloadFoiled; //!< SpawnablePayloadFoiled asset
   };

   //! FoilWrapper wraps a foil around a box, it is scripted object that makes multiple things:
   //! It has some configuration, @see FoilWrapperConfig
   //! The component awaits for enter the `m_collisionTrigger`. It waits for boxes and pallets (found by Tags).
   //! When a box or pallet enters the `m_collisionTrigger`, it starts wrapping the box with the foil.
   //! After some time, boxes are despawned.
   //! And foiled pallets is spawned in the place of the despawned box.

   class FoilWrapper
       : public AZ::Component
       , public AZ::TickBus::Handler
   {
       enum class FoilWrapperState
       {
           Idle,
           Starting,
           Wrapping,
           Despawning,
       };
   public:
       static constexpr AZ::Crc32 BoxTag = AZ_CRC_CE("Box");
       static constexpr AZ::Crc32 PalletTag = AZ_CRC_CE("Pallet");

       AZ_COMPONENT(FoilWrapper, "{77a07015-bd4c-42c0-ad30-f0ead67e1c2e}");

       FoilWrapper() = default;

       ~FoilWrapper() = default;

       // AZ::Component overrides...
       void Activate() override;

       void Deactivate() override;

       static void Reflect(AZ::ReflectContext* context);

   private:
       FoilWrapperConfig m_configuration;
       FoilWrapperState m_state = FoilWrapperState::Idle;
       float m_timer = 0.0f;

       // AZ::TickBus::Handler overrides...
       void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

       bool isObjectBox(const AZ::EntityId entityId) const;
       bool isObjectPallet(const AZ::EntityId entityId) const;

       AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerEnterHandler;
       AZStd::vector<AZ::EntityId> m_collidingEntities; //!< List of boxes that are in foil wrapper
       AZ::EntityId m_pallet; //!< A pallet that is in foil wrapper
       int m_wrappedPallets {0}; //!< Number of wrapped pallets
   };
} // namespace ROS2::Demo
