/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ScriptSpawnLevelComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2::Demo
{
    void ScriptSpawnLevelComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ScriptSpawnLevelComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ScriptSpawnLevelComponent>(
                      "ScriptSpawnLevelComponent", "Use this component to import robot definition from supported formats")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void ScriptSpawnLevelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ScriptSpawnLevelComponentService"));
    }

    void ScriptSpawnLevelComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ScriptSpawnLevelComponentService"));
    }

    void ScriptSpawnLevelComponent::Activate()
    {
        AZ_Printf("ScriptSpawnLevelComponent", "Activate");
        m_spawnedTickets.clear();
        m_spawnedEntitiesToNames.clear();
        ScriptSpawnSystemRequestBus::Handler::BusConnect();
    }

    void ScriptSpawnLevelComponent::Deactivate()
    {
        AZ_Printf("ScriptSpawnLevelComponent", "Deactivate");
        m_spawnedTickets.clear();
        m_spawnedEntitiesToNames.clear();
        ScriptSpawnSystemRequestBus::Handler::BusDisconnect();
    }

    void ScriptSpawnLevelComponent::DespawnBox(const AZStd::string& spawnableName)
    {
        auto it = m_spawnedEntitiesToNames.find(spawnableName);
        if (it != m_spawnedEntitiesToNames.end())
        {
            m_spawnedEntitiesToNames.erase(it);
        }
        auto it2 = m_spawnedTickets.find(spawnableName);
        if (it2 != m_spawnedTickets.end())
        {
            m_spawnedTickets.erase(it2);
        }
    }

    AZ::EntityId ScriptSpawnLevelComponent::GetSpawnedEntityId(const AZStd::string& spawnableName)
    {
        auto it = m_spawnedEntitiesToNames.find(spawnableName);
        if (it != m_spawnedEntitiesToNames.end())
        {
            return it->second;
        }
        return AZ::EntityId();
    }

    void ScriptSpawnLevelComponent::SpawnAssetAndSetParent(
        const AZ::Data::Asset<AzFramework::Spawnable>& spawnable,
        const AZ::Transform& transform,
        const AZStd::string& spawnableName,
        const AZ::EntityId parent)
    {
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "Unable to get spawnable entities definition");
        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
        AzFramework::EntitySpawnTicket ticket(spawnable);
        // Set the pre-spawn callback to set the name of the root entity to the name of the spawnable
        optionalArgs.m_preInsertionCallback = [transform, spawnableName, parent](auto id, auto view)
        {
            if (view.empty())
            {
                return;
            }
            AZ::Entity* root = *view.begin();
            root->SetName(spawnableName.c_str());
            auto childPtrPtr = view.begin() + 1;
            // update the name of the first child entity
            for (childPtrPtr; childPtrPtr != view.end(); ++childPtrPtr)
            {
                (**childPtrPtr).SetName(spawnableName.c_str());
            }

            auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
            transformInterface->SetWorldTM(transform);
            if (parent.IsValid())
            {
                transformInterface->SetParent(parent);
            }
        };

        optionalArgs.m_completionCallback = [this, spawnableName](auto ticket, auto result)
        {
            if (!result.empty() && result.size() > 1)
            {
                // get first entity that is not container entity
                const AZ::Entity* firstChild = *(result.begin() + 1);
                m_spawnedEntitiesToNames[spawnableName] = firstChild->GetId();
            }
        };

        optionalArgs.m_priority = AzFramework::SpawnablePriority_Lowest;
        spawner->SpawnAllEntities(ticket, optionalArgs);
        m_spawnedTickets.emplace(spawnableName, AZStd::move(ticket));
    }

} // namespace ROS2::Demo
