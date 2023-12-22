/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PayloadDespawnerComponent.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <Scripting/ScriptSpawnSytemBus.h>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ROS2::Demo
{
    void PayloadDespawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PayloadDespawnerComponent>()->Version(1)
                ->Field("DespawnDelay", &PayloadDespawnerComponent::m_despawnerDelay);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<PayloadDespawnerComponent>("PayloadDespawnerComponent", "PayloadDespawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PayloadDespawnerComponent::m_despawnerDelay, "Despawn Delay", "Despawn Delay");
            }
        }
    }

    void PayloadDespawnerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsColliderService"));
    }

    void PayloadDespawnerComponent::Activate()
    {
        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::TriggerEvent& event)
            {
                const auto entityId = event.m_otherBody->GetEntityId();

                if (IsObjectBox(entityId))
                {
                    m_collidingBoxes.push_back(entityId);
                }
                else if (IsObjectAMR(entityId))
                {
                    if (!m_collidingAmr.IsValid())
                    {
                        m_collidingAmr = entityId;
                        m_timer = 0;
                    }
                }
            });

        AZ::TickBus::Handler::BusConnect();
    }

    bool PayloadDespawnerComponent::IsObjectBox(const AZ::EntityId entityId) const
    {
        bool isBox = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isBox, entityId, &LmbrCentral::TagComponentRequests::HasTag, BoxTag);
        return isBox;
    }

    bool PayloadDespawnerComponent::IsObjectAMR(const AZ::EntityId entityId) const
    {
        bool isAMR = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isAMR, entityId, &LmbrCentral::TagComponentRequests::HasTag, AMRTag);
        return isAMR;
    }

    void PayloadDespawnerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void PayloadDespawnerComponent::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface.");

        [[maybe_unused]] AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle.");

        // Connects the collision handlers if not already connected
        if (!m_onTriggerEnterHandler.IsConnected())
        {
            AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                physicsSystem->FindAttachedBodyHandleFromEntityId(GetEntityId());
            AZ_Warning("PayloadDespawner", foundBody.first != AzPhysics::InvalidSceneHandle, "No body found for m_collisionTrigger.");
            if (foundBody.first != AzPhysics::InvalidSceneHandle)
            {
                AzPhysics::SimulatedBodyEvents::RegisterOnTriggerEnterHandler(foundBody.first, foundBody.second, m_onTriggerEnterHandler);
            }
        }

        if (m_collidingAmr.IsValid())
        {
            m_timer += delta;
            if (m_timer > m_despawnerDelay)
            {
                for (auto& entityId : m_collidingBoxes)
                {
                    AZ::Entity* box{};
                    AZ::ComponentApplicationBus::BroadcastResult(box, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                    if (box)
                    {
                        const auto& boxName = box->GetName();
                        ScriptSpawnSystemRequestBus::Broadcast(&ScriptSpawnSystemRequestBus::Events::DespawnBox, boxName);
                    }
                }

                AZ::Entity* collidingAmr{};
                AZ::ComponentApplicationBus::BroadcastResult(collidingAmr, &AZ::ComponentApplicationRequests::FindEntity, m_collidingAmr);
                AZ_Assert(collidingAmr, "No entity found for colliding AMR")
                    auto frame = collidingAmr->FindComponent<ROS2FrameComponent>();
                AZ_Assert(frame, "AMR base_link does not have ROS2 Frame Component");
                AZStd::string amr_namespace = frame->GetNamespace();
                AZStd::string fullNamespace = amr_namespace + "/cargo_status";

                auto ros2Node = ROS2Interface::Get()->GetNode();
                auto deliberationPublisher = ros2Node->create_publisher<std_msgs::msg::Bool>(fullNamespace.data(), rclcpp::ParametersQoS());

                AZ_Printf("PayloadDespawnerComponent", "Despawned %d entities from robot %s.", m_collidingBoxes.size(), amr_namespace.c_str());
                for (const auto& m_collidingBox : m_collidingBoxes)
                {
                    AZ::Entity* box{};
                    AZ::ComponentApplicationBus::BroadcastResult(box, &AZ::ComponentApplicationRequests::FindEntity, m_collidingBox);
                    if (box)
                    {
                        const auto& boxName = box->GetName();
                        AZ_Printf("PayloadDespawnerComponent", "Despawned box %s.", boxName.c_str());
                    }
                }

                std_msgs::msg::Bool message;
                message.data = false;
                deliberationPublisher->publish(message);

                m_collidingAmr = AZ::EntityId();// reset colliding AMR handle
            }


        }
    }
} // namespace ROS2::Demo
