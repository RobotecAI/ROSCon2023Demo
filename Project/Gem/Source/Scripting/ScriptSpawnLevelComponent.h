/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "ScriptSpawnSytemBus.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>


namespace ROS2::Demo
{
   //! Central singleton-like for spawning and despawing things in the demo
   class ScriptSpawnLevelComponent
       : public AZ::Component
       , public ScriptSpawnSystemRequestBus::Handler
   {
   public:
       AZ_COMPONENT(ScriptSpawnLevelComponent, "{da4b5a9c-2eed-403e-86e8-fb8dbcffa2c9}");

       static void Reflect(AZ::ReflectContext* context);
       static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
       static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

       ScriptSpawnLevelComponent() = default;
       virtual ~ScriptSpawnLevelComponent() = default;


   protected:
       // AZ::Component override  ...
       void Init() override {};
       void Activate() override;
       void Deactivate() override;

        // ScriptSpawnSystemRequestBus::Handler interface implementation ...
        void DespawnBox(const AZStd::string& spawnableName) override;
        AZ::EntityId GetSpawnedEntityId(const AZStd::string& spawnableName) override;
        void SpawnAssetAndSetParent(const AZ::Data::Asset<AzFramework::Spawnable>& spawnable, const AZ::Transform& transform, const AZStd::string& spawnableName, const AZ::EntityId parent) override;

   private:
       AZStd::unordered_map<AZStd::string, AZ::EntityId> m_spawnedEntitiesToNames;
       AZStd::unordered_map<AZStd::string, AzFramework::EntitySpawnTicket> m_spawnedTickets;

   };
} // namespace ROS2
