#include "PayloadDespawnerComponent.h"
#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Debug/Trace.h"
#include "AzCore/Serialization/EditContext.h"
#include "AzCore/std/string/string.h"
#include "AzFramework/Physics/PhysicsSystem.h"
#include "LmbrCentral/Scripting/TagComponentBus.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "Scripting/ScriptSpawnSytemBus.h"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp/publisher.hpp>

namespace ROS2::Demo
{
    void PayloadDespawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PayloadDespawnerComponent>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<PayloadDespawnerComponent>("PayloadDespawnerComponent", "PayloadDespawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
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
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const auto entityId = event.m_otherBody->GetEntityId();
                AZ::Entity* entity{};
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);

                if (IsObjectBox(entityId))
                {
                    AZ::Entity* entity{};
                    AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                    if (entity)
                    {
                        const auto& entityName = entity->GetName();
                        ScriptSpawnSystemRequestBus::Broadcast(&ScriptSpawnSystemRequestBus::Events::DespawnBox, entityName);
                    }
                }
                else if (IsObjectAMR(entityId))
                {
                    AZ::Entity* entity{};
                    AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                    auto frame = entity->FindComponent<ROS2FrameComponent>();
                    AZ_Assert(frame, "AMR base_link does not have ROS2 Frame Component");
                    AZStd::string amr_namespace = frame->GetNamespace();
                    AZStd::string fullNamespace = amr_namespace + "/cargo_status";
                    
                    auto ros2Node = ROS2Interface::Get()->GetNode();
                    auto deliberationPublisher =
                        ros2Node->create_publisher<std_msgs::msg::Bool>(fullNamespace.data(), rclcpp::SensorDataQoS());

                    std_msgs::msg::Bool message;
                    message.data = false;
                    deliberationPublisher->publish(message);
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

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
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
    }
} // namespace ROS2::Demo
