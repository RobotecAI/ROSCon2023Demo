/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "LightController.h"
#include <AtomLyIntegration/CommonFeatures/CoreLights/AreaLightBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Utilities/ROS2Names.h>
namespace ROS2::Demo
{

    void LightControllerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LightControllerConfiguration>()
                ->Version(1)
                ->Field("LightEntityIdList", &LightControllerConfiguration::m_lightEntityIdList)
                ->Field("TopicConfiguration", &LightControllerConfiguration::m_topicConfiguration);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<LightControllerConfiguration>("LightControllerConfiguration", "LightControllerConfiguration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2 Utilities")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LightControllerConfiguration::m_topicConfiguration, "Topic configuration", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LightControllerConfiguration::m_lightEntityIdList, "LightEntiesToModify", "");
            }
        }
    }

    void LightController::Reflect(AZ::ReflectContext* context)
    {
        LightControllerConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LightController>()->Version(1)->Field("configuration", &LightController::m_config);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LightController>("LightController", "LightController.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "{")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LightController::m_config, "configuration", "configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void LightController::SetColor(const AZ::Color& color) const
    {
        for (const auto& lightEntityId : m_config.m_lightEntityIdList)
        {
            AZ::Render::AreaLightRequestBus::Event(lightEntityId, &AZ::Render::AreaLightRequests::SetColor, color);
        }
    }

    void LightController::ResetColor() const
    {
        for (const auto& lightEntityId : m_config.m_lightEntityIdList)
        {
            AZ_Assert(m_orignalColor.contains(lightEntityId), "Light entity id not found in original color map");
            AZ::Render::AreaLightRequestBus::Event(lightEntityId, &AZ::Render::AreaLightRequests::SetColor, m_orignalColor.at(lightEntityId));
        }
    }

    void LightController::Activate()
    {
        auto* ros2Interface = ROS2Interface::Get();
        AZ_Assert(ros2Interface, "ROS2 interface not available");
        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        AZ_Assert(ros2Frame, "Missing ROS2FrameComponent");
        AZStd::string topic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_config.m_topicConfiguration.m_topic);

        m_colorSubcriber = ros2Interface->GetNode()->create_subscription<std_msgs::msg::String>(
            topic.c_str(),
            m_config.m_topicConfiguration.GetQoS(),
            [&](std_msgs::msg::String msg)
            {
                AZStd::string message{ msg.data.c_str() };
                AZ_Printf("LightController", "Received message: %s", message.c_str());

                if (message.empty() || message == "default") // default is a reserved keyword for "reset to original color
                {
                    ResetColor();
                    return;
                }
                if (ColorMap.contains(message))
                {
                    SetColor(ColorMap[message]);
                    return;
                }
                AZ_Warning("LightController", false, "Unknown color: %s", message.c_str());
            });

        AZ::TickBus::Handler::BusConnect();
    }

    void LightController::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void LightController::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        m_orignalColor.clear();
        for (const auto& entityId : m_config.m_lightEntityIdList)
        {
            AZ::Color color;
            AZ::Render::AreaLightRequestBus::EventResult(color, entityId, &AZ::Render::AreaLightRequests::GetColor);
            m_orignalColor[entityId] = color;
        }
        AZ::TickBus::Handler::BusDisconnect();
    }

} // namespace ROS2::Demo
