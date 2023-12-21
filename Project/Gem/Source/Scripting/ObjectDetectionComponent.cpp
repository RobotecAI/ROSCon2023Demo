/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ObjectDetectionComponent.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Collision/CollisionEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <std_msgs/msg/detail/bool__struct.hpp>

namespace ROS2::Demo
{
    void ObjectDetectionConfig::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ObjectDetectionConfig>()
                ->Version(1)
                ->Field("CollisionTrigger", &ObjectDetectionConfig::m_collisionTrigger)
                ->Field("DetectableObjects", &ObjectDetectionConfig::m_detectableObjectsTags)
                ->Field("TopicConfiguration", &ObjectDetectionConfig::m_topicConfiguration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ObjectDetectionConfig>("ObjectDetectionConfig", "ObjectDetectionConfig")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ObjectDetectionConfig::m_collisionTrigger, "CollisionTrigger", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ObjectDetectionConfig::m_detectableObjectsTags, "Detectable objects tags", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ObjectDetectionConfig::m_topicConfiguration, "Topic configuration", "");
            }
        }
    }

    void ObjectDetectionComponent::Reflect(AZ::ReflectContext* context)
    {
        ObjectDetectionConfig::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ObjectDetectionComponent, AZ::Component>()->Version(1)->Field(
                "ObjectDetectionConfig", &ObjectDetectionComponent::m_configuration);
            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ObjectDetectionComponent>("ObjectDetector", "ObjectDetector")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ObjectDetectionComponent::m_configuration, "m_configuration", "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void ObjectDetectionComponent::Activate()
    {
        auto ROS2Interface = ROS2Interface::Get();
        AZ_Assert(ROS2Interface, "ROS2Interface not available");
        auto ROS2Node = ROS2Interface->GetNode();
        AZ_Assert(ROS2Node, "ROS2Node not available");
        auto ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        AZ_Assert(ros2Frame, "Missing ROS2FrameComponent");
        AZStd::string topicName = ros2Frame->GetNamespace() + m_configuration.m_topicConfiguration.m_topic;
        m_topicPublisher =
            ROS2Node->create_publisher<std_msgs::msg::Bool>(topicName.c_str(), m_configuration.m_topicConfiguration.GetQoS());

        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                AZ::EntityId otherBodyId = event.m_otherBody->GetEntityId();
                if (DoesObjectContainTag(otherBodyId))
                {
                    m_collidingEntities.insert(otherBodyId);
                    std_msgs::msg::Bool trueMsg;
                    trueMsg.data = true;
                    m_topicPublisher->publish(trueMsg);
                }
            });

        m_onTriggerExitHandler = AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::TriggerEvent& event)
            {
                AZ::EntityId otherBodyId = event.m_otherBody->GetEntityId();
                if (DoesObjectContainTag(otherBodyId))
                {
                    m_collidingEntities.erase(otherBodyId);
                    if (m_collidingEntities.empty())
                    {
                        std_msgs::msg::Bool falseMsg;
                        falseMsg.data = false;
                        m_topicPublisher->publish(falseMsg);
                    }
                }
            });

        AZ::TickBus::Handler::BusConnect();
    }

    void ObjectDetectionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    bool ObjectDetectionComponent::DoesObjectContainTag(AZ::EntityId entityId)
    {
        for (AZStd::string tag : m_configuration.m_detectableObjectsTags)
        {
            bool hasTag = false;
            LmbrCentral::TagComponentRequestBus::EventResult(hasTag, entityId, &LmbrCentral::TagComponentRequests::HasTag, AZ_CRC(tag));
            if (hasTag)
            {
                return true;
            }
        }

        return false;
    }

    void ObjectDetectionComponent::Deactivate()
    {
        m_topicPublisher.reset();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ObjectDetectionComponent::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        // Connects the collision handlers if not already connected
        if (!m_onTriggerEnterHandler.IsConnected() || !m_onTriggerExitHandler.IsConnected())
        {
            AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
            AZ_Assert(physicsSystem, "No physics system.");

            AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            AZ_Assert(sceneInterface, "No scene interface.");

            [[maybe_unused]] AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
            AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle.");

            AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                physicsSystem->FindAttachedBodyHandleFromEntityId(m_configuration.m_collisionTrigger);
            AZ_Warning("ObjectDetection", foundBody.first != AzPhysics::InvalidSceneHandle, "No body found for m_collisionTrigger.");
            if (foundBody.first != AzPhysics::InvalidSceneHandle)
            {
                AzPhysics::SimulatedBodyEvents::RegisterOnTriggerEnterHandler(foundBody.first, foundBody.second, m_onTriggerEnterHandler);
                AzPhysics::SimulatedBodyEvents::RegisterOnTriggerExitHandler(foundBody.first, foundBody.second, m_onTriggerExitHandler);
            }
        }
    }

} // namespace ROS2::Demo
