/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <HumanNpc/AnimGraphInputProviderComponent.h>
#include <Integration/AnimGraphComponentBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2::Demo
{
    void AnimGraphInputProviderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AnimGraphInputProviderComponent, AZ::Component>()
                ->Version(0)
                ->Field("Rigid Body", &AnimGraphInputProviderComponent::m_rigidBodyEntityId)
                ->Field("Linear Speed Multiplier", &AnimGraphInputProviderComponent::m_linearSpeedMultiplier);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<AnimGraphInputProviderComponent>("AnimGraph Input Provider", "Component that feeds the anim graph input.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AnimGraphInputProviderComponent::m_rigidBodyEntityId, "Rigid Body", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AnimGraphInputProviderComponent::m_linearSpeedMultiplier,
                        "Linear Speed Multiplier",
                        "");
            }
        }
    }

    void AnimGraphInputProviderComponent::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(m_rigidBodyEntityId);
    }

    void AnimGraphInputProviderComponent::Deactivate()
    {
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void AnimGraphInputProviderComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ::Quaternion worldRotation = AZ::Quaternion::CreateZero();
        AZ::TransformBus::EventResult(worldRotation, m_rigidBodyEntityId, &AZ::TransformInterface::GetWorldRotationQuaternion);
        const AZ::Quaternion InverseWorldRotation = worldRotation.GetInverseFull();

        AZ::Vector3 linearVelocity = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(linearVelocity, m_rigidBodyEntityId, &Physics::RigidBodyRequests::GetLinearVelocity);

        linearVelocity = InverseWorldRotation.TransformVector(linearVelocity);

        SetAnimGraphParameters(linearVelocity.GetX());
    }

    void AnimGraphInputProviderComponent::OnEntityActivated(const AZ::EntityId&)
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void AnimGraphInputProviderComponent::SetAnimGraphParameters(float linearSpeed)
    {
        EMotionFX::Integration::AnimGraphComponentRequestBus::Event(
            GetEntityId(),
            &EMotionFX::Integration::AnimGraphComponentRequests::SetNamedParameterFloat,
            "Speed",
            linearSpeed * m_linearSpeedMultiplier);
    }
} // namespace ROS2::Demo
