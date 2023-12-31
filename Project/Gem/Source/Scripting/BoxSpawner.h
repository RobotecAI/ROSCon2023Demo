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
#include <ImGuiBus.h>

namespace ROS2::Demo
{
    class BoxSpawnerConfiguration
    {
    public:
        AZ_TYPE_INFO(BoxSpawnerConfiguration, "{bec0955f-4498-421e-9da7-b0130d63bb4e}");

        static void Reflect(AZ::ReflectContext* context);

        AZ::Data::Asset<AzFramework::Spawnable> m_spawnable;
        unsigned int m_maxBoxCount{ 10 }; //!< Maximum number of boxes in barrier;
        AZ::EntityId m_barrierRegionEntityId; //!< EntityId of the barrier entity
        AZ::EntityId m_despawnRegionEntityId; //!< EntityId of the despawn region entity
        float m_boxSpawnInterval{ 1.0f };
        bool m_manualMode{ false };
    };

    class BoxSpawner
        : public AZ::Component
        , protected AZ::TickBus::Handler
        , protected ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        AZ_COMPONENT(BoxSpawner, "{4c85c12b-996a-42ff-874b-4cb3902784bf}");

        BoxSpawner() = default;

        ~BoxSpawner() = default;

        void OnImGuiUpdate() override;

        // AZ::Component overrides...
        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        static constexpr size_t FrustumPointCount = 5;

        // OnTickBus overrides...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Spawn a single box in the location of this entity with given name, if previous box has been spawned successfully.
        //! @param spawnableName The name of the spawnable to spawn.
        void SpawnBox(const AZStd::string& spawnableName);

        //! Despawn all boxex that are in the despawn region.
        void DespawnBoxes();

        //! Count number of boxes in the barrier region.
        //! @return Number of boxes in the barrier region.
        size_t CountBoxesInBarrierRegion();

        BoxSpawnerConfiguration m_configuration;
        AZStd::vector<AZStd::string> m_spawnableNames;
        float m_timeSinceLastSpawn{ 0.0f };
        bool m_shouldSpawnManual = false;
    };
} // namespace ROS2::Demo
