/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "BoxSpawner.h"
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/SystemBus.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>

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
                ->Field("DespawnRegionEntityId", &BoxSpawnerConfiguration::m_despawnRegionEntityId);
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
                        "DespawnRegionEntityId with box shape.");
            }
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
    }

    void BoxSpawner::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
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
        {
            AZStd::unique_lock<AZStd::mutex> lock(m_ticketMutex);
            if (m_ticket.IsValid())
            {
                return;
            };
        }

        AZ::Transform thisTransform;
        AZ::TransformBus::EventResult(thisTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition");
        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
        AzFramework::EntitySpawnTicket ticket(m_configuration.m_spawnable);

        // Set the pre-spawn callback to set the name of the root entity to the name of the spawnable
        optionalArgs.m_preInsertionCallback = [thisTransform, spawnableName](auto id, auto view)
        {
            if (view.empty())
            {
                return;
            }
            AZ::Entity* root = *view.begin();
            root->SetName(spawnableName.c_str());
            auto childPtrPtr =view.begin() + 1;
            // update the name of the first child entity
            if (childPtrPtr != view.end())
            {
                (**childPtrPtr).SetName(spawnableName.c_str());
            }
            auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
            transformInterface->SetWorldTM(thisTransform);
        };

        // set the post-spawn callback to add the entity to the list of spawned entities
        optionalArgs.m_completionCallback = [this, spawnableName](auto ticket, auto result)
        {
            if (!result.empty() && result.size() > 1)
            {
                AZStd::unique_lock<AZStd::mutex> lock(m_ticketMutex);

                // get first entity that is not container entity
                const AZ::Entity* firstChild = *(result.begin() + 1);

                m_spawnedEntities[firstChild->GetId()] = m_ticket;
                m_ticket = AzFramework::EntitySpawnTicket();
            }
        };

        optionalArgs.m_priority = AzFramework::SpawnablePriority_Lowest;
        spawner->SpawnAllEntities(ticket, optionalArgs);
        {
            AZStd::unique_lock<AZStd::mutex> lock(m_ticketMutex);
            m_ticket = ticket;
        }
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
            m_spawnedEntities,
            [&despawnRegionTransform, &despawnRegionBoxConfig](const auto& pair)
            {
                AZ::EntityId boxEntityId = pair.first;
                AZ::Vector3 boxLocation{ AZ::Vector3::CreateZero() };
                AZ::TransformBus::EventResult(boxLocation, boxEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
                return CheckIfEntityInsideBox(boxLocation, despawnRegionBoxConfig, despawnRegionTransform);
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
            m_spawnedEntities.begin(),
            m_spawnedEntities.end(),
            [&barrierRegionTransform, &barrierRegionBoxConfig](const auto& pair)
            {
                AZ::EntityId boxEntityId = pair.first;
                AZ::Vector3 boxLocation{ AZ::Vector3::CreateZero() };
                AZ::TransformBus::EventResult(boxLocation, boxEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
                return CheckIfEntityInsideBox(boxLocation, barrierRegionBoxConfig, barrierRegionTransform);
            });
        AZ_Printf("BoxSpawner", "CountBoxesInBarrierRegion: %d\n", count);
        return count;
    }

    void BoxSpawner::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_timeSinceLastSpawn += deltaTime;
        DespawnBoxes();
        if (m_timeSinceLastSpawn > m_configuration.m_boxSpawnInterval)
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
    }
} // namespace ROS2::Demo
