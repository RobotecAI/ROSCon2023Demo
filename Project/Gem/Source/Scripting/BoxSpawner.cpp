/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "BoxSpawner.h"
#include "ScriptSpawnSytemBus.h"

#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/SystemBus.h>
#include <ImGuiBus.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <imgui/imgui.h>

namespace ROS2::Demo
{

    void BoxSpawnerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<BoxSpawnerConfiguration>()
                ->Version(1)
                ->Field("Spawnable", &BoxSpawnerConfiguration::m_spawnable)
                ->Field("MaxBoxCount", &BoxSpawnerConfiguration::m_maxBoxCount)
                ->Field("BoxSpawnInterval", &BoxSpawnerConfiguration::m_boxSpawnInterval)
                ->Field("BarrierRegionEntityId", &BoxSpawnerConfiguration::m_barrierRegionEntityId)
                ->Field("DespawnRegionEntityId", &BoxSpawnerConfiguration::m_despawnRegionEntityId)
                ->Field("Manual Mode", &BoxSpawnerConfiguration::m_manualMode);
            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<BoxSpawnerConfiguration>("BoxSpawnerConfiguration", "BoxSpawnerConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2 Utilities")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BoxSpawnerConfiguration::m_spawnable, "m_spawnable", "m_spawnable")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BoxSpawnerConfiguration::m_maxBoxCount, "MaxBoxCount", "MaxBoxCount")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &BoxSpawnerConfiguration::m_boxSpawnInterval, "BoxSpawnInterval", "BoxSpawnInterval")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &BoxSpawnerConfiguration::m_barrierRegionEntityId,
                        "BarrierRegionEntityId",
                        "BarrierRegionEntityId with box shape. It will stop spawning boxes when the number of boxes are in this region.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &BoxSpawnerConfiguration::m_despawnRegionEntityId,
                        "DespawnRegionEntityId",
                        "DespawnRegionEntityId with box shape.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &BoxSpawnerConfiguration::m_manualMode,
                        "Manual mode",
                        "Switch for manual mode to be used with IMGui only");
            }
        }
    }

    void BoxSpawner::OnImGuiUpdate()
    {
        if (m_configuration.m_manualMode)
        {
            ImGui::Begin("Box spawner manual");
            if (ImGui::Button("Spawn"))
            {
                m_shouldSpawnManual = true;
            }
            ImGui::End();
        }
    }

    void BoxSpawner::Reflect(AZ::ReflectContext* context)
    {
        BoxSpawnerConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<BoxSpawner>()->Version(1)->Field("configuration", &BoxSpawner::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<BoxSpawner>("BoxSpawner", "BoxSpawner.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "BoxSpawner")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BoxSpawner::m_configuration, "configuration", "configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void BoxSpawner::Activate()
    {
        m_timeSinceLastSpawn = 0.f;
        AZ_Warning(
            "BoxSpawner",
            m_configuration.m_despawnRegionEntityId.IsValid(),
            "Note that the despawn region is not set, so boxes will never be despawned.");
        AZ_Warning(
            "BoxSpawner",
            m_configuration.m_barrierRegionEntityId.IsValid(),
            "Note that the barrier region is not set, so boxes will be spawned with constant interval.");
        AZ::TickBus::Handler::BusConnect();
        if (m_configuration.m_manualMode)
        {
            ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        }
    }

    void BoxSpawner::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        if (m_configuration.m_manualMode)
        {
            ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        }
    }

    bool CheckIfEntityInsideBox(
        const AZ::Vector3& boxLocation, const LmbrCentral::BoxShapeConfig& regionBoxConfig, AZ::Transform& entityTransform)
    {
        const AZ::Vector3 localPos = entityTransform.GetInverse().TransformPoint(boxLocation) - regionBoxConfig.m_translationOffset;
        const AZ::Vector3 boxMin = regionBoxConfig.GetDimensions() * -0.5f;
        const AZ::Vector3 boxMax = regionBoxConfig.GetDimensions() * 0.5f;
        if (localPos.IsGreaterThan(boxMin) && localPos.IsLessThan(boxMax))
        {
            return true;
        }
        return false;
    }

    void BoxSpawner::SpawnBox(const AZStd::string& spawnableName)
    {
        AZ::Transform thisTransform;
        AZ::TransformBus::EventResult(thisTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        ScriptSpawnSystemRequestBus::Broadcast(
            &ScriptSpawnSystemRequestBus::Events::SpawnAssetAndSetParent,
            m_configuration.m_spawnable,
            thisTransform,
            spawnableName,
            AZ::EntityId());
        m_spawnableNames.push_back(spawnableName);
    }

    void BoxSpawner::DespawnBoxes()
    {
        AZ::Transform despawnRegionTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(
            despawnRegionTransform, m_configuration.m_despawnRegionEntityId, &AZ::TransformBus::Events::GetWorldTM);
        LmbrCentral::BoxShapeConfig despawnRegionBoxConfig;
        LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
            despawnRegionBoxConfig, m_configuration.m_despawnRegionEntityId, &LmbrCentral::BoxShapeComponentRequests::GetBoxConfiguration);

        AZStd::erase_if(
            m_spawnableNames,
            [&despawnRegionTransform, &despawnRegionBoxConfig](const auto& name)
            {
                AZ::EntityId boxEntityId = AZ::EntityId();
                ScriptSpawnSystemRequestBus::BroadcastResult(boxEntityId, &ScriptSpawnSystemRequestBus::Events::GetSpawnedEntityId, name);
                AZ::Vector3 boxLocation{ AZ::Vector3::CreateZero() };
                AZ::TransformBus::EventResult(boxLocation, boxEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
                bool isInside = CheckIfEntityInsideBox(boxLocation, despawnRegionBoxConfig, despawnRegionTransform);
                if (isInside)
                {
                    ScriptSpawnSystemRequestBus::Broadcast(&ScriptSpawnSystemRequestBus::Events::DespawnBox,name);
                }
                return isInside;
            });
    }

    size_t BoxSpawner::CountBoxesInBarrierRegion()
    {
        AZ::Transform barrierRegionTransform;
        AZ::TransformBus::EventResult(
            barrierRegionTransform, m_configuration.m_barrierRegionEntityId, &AZ::TransformBus::Events::GetWorldTM);
        LmbrCentral::BoxShapeConfig barrierRegionBoxConfig;
        LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
            barrierRegionBoxConfig, m_configuration.m_barrierRegionEntityId, &LmbrCentral::BoxShapeComponentRequests::GetBoxConfiguration);
        int count = AZStd::count_if(
            m_spawnableNames.begin(),
            m_spawnableNames.end(),
            [&barrierRegionTransform, &barrierRegionBoxConfig](const auto& name)
            {
                AZ::EntityId boxEntityId = AZ::EntityId();
                ScriptSpawnSystemRequestBus::BroadcastResult(boxEntityId, &ScriptSpawnSystemRequestBus::Events::GetSpawnedEntityId, name);
                AZ::Vector3 boxLocation{ AZ::Vector3::CreateZero() };
                AZ::TransformBus::EventResult(boxLocation, boxEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
                return CheckIfEntityInsideBox(boxLocation, barrierRegionBoxConfig, barrierRegionTransform);
            });
        return count;
    }

    void BoxSpawner::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_timeSinceLastSpawn += deltaTime;
        DespawnBoxes();
        if (m_timeSinceLastSpawn > m_configuration.m_boxSpawnInterval && !m_configuration.m_manualMode)
        {
            const size_t count = m_configuration.m_barrierRegionEntityId.IsValid() ? CountBoxesInBarrierRegion() : 0;
            if (count < m_configuration.m_maxBoxCount)
            {
                AZ::Crc32 checksum = AZ::Crc32(m_entity->GetId().ToString());
                checksum.Add(time.ToString());
                const AZStd::string spawnableName = AZStd::string::format("Box_%u", AZ::u32(checksum));
                SpawnBox(spawnableName);
            }
            m_timeSinceLastSpawn = 0.f;
        }
        else
        {
            if (m_shouldSpawnManual)
            {
                AZ::Crc32 checksum = AZ::Crc32(m_entity->GetId().ToString());
                checksum.Add(time.ToString());
                const AZStd::string spawnableName = AZStd::string::format("Box_%u", AZ::u32(checksum));
                SpawnBox(spawnableName);
                m_shouldSpawnManual = false;
            }
        }
    }


} // namespace ROS2::Demo
