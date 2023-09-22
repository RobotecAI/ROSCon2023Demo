/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LifterController.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/MotorizedJoints/PidMotorControllerBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <std_msgs/msg/detail/bool__struct.hpp>

namespace ROS2::Demo
{
    void LifterControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<LifterControllerComponent, AZ::Component>()
                ->Version(1)
                ->Field("TopicName", &LifterControllerComponent::m_topic)
                ->Field("MotorSetpoint", &LifterControllerComponent::m_setpoint);
            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<LifterControllerComponent>("LifterControllerComponent", "LifterControllerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LifterControllerComponent::m_topic, "Topic name", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LifterControllerComponent::m_setpoint, "Extended motor setpoint", "")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.);
            }
        }
    }

    void LifterControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void LifterControllerComponent::Activate()
    {
        auto* ros2Interface = ROS2Interface::Get();
        AZ_Assert(ros2Interface, "ROS2 interface not available");
        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        AZ_Assert(ros2Frame, "Missing ROS2FrameComponent");
        AZStd::string topic = ros2Frame->GetNamespace() + "/" + m_topic;
        m_lifterTopicSubscriber = ros2Interface->GetNode()->create_subscription<std_msgs::msg::Bool>(
            topic.c_str(),
            10,
            [&](std_msgs::msg::Bool msg)
            {
                float setpoint = msg.data ? m_setpoint : 0.;
                PidMotorControllerRequestBus::Event(GetEntityId(), &PidMotorControllerRequestBus::Events::SetSetpoint, setpoint);
            });
    }

    void LifterControllerComponent::Deactivate()
    {
        m_lifterTopicSubscriber.reset();
    }
} // namespace ROS2::Demo
