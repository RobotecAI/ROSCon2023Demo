/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Color.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>
namespace ROS2::Demo
{
    class LightControllerConfiguration
    {
    public:
        AZ_TYPE_INFO(LightControllerConfiguration, "{a8073143-53f0-4981-bd2f-991e6a537167}");

        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<AZ::EntityId> m_lightEntityIdList;
        ROS2::TopicConfiguration m_topicConfiguration;
    };

    class LightController
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    private:
        AZStd::map<AZStd::string, AZ::Color> ColorMap{
            { "WHITE", AZ::Color{ 1.0f, 1.0f, 1.0f, 1.0f } },   { "RED", AZ::Color{ 1.0f, 0.0f, 0.0f, 1.0f } },
            { "GREEN", AZ::Color{ 0.0f, 1.0f, 0.0f, 1.0f } },   { "BLUE", AZ::Color{ 0.0f, 0.0f, 1.0f, 1.0f } },
            { "YELLOW", AZ::Color{ 1.0f, 1.0f, 0.0f, 1.0f } },  { "CYAN", AZ::Color{ 0.0f, 1.0f, 1.0f, 1.0f } },
            { "MAGENTA", AZ::Color{ 1.0f, 0.0f, 1.0f, 1.0f } }, { "ORANGE", AZ::Color{ 1.0f, 0.5f, 0.0f, 1.0f } },
            { "PURPLE", AZ::Color{ 0.5f, 0.0f, 0.5f, 1.0f } },  { "PINK", AZ::Color{ 1.0f, 0.5f, 0.8f, 1.0f } }
        };

    public:
        AZ_COMPONENT(LightController, "{1432d725-e626-4f76-87b5-123e98ce99be}");

        LightController() = default;

        ~LightController() = default;

        // AZ::Component overrides...
        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::TickBus::Handler overrides...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void SetColor(const AZ::Color& color) const;
        void ResetColor() const;

        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_colorSubcriber;
        LightControllerConfiguration m_config;
        AZStd::unordered_map<AZ::EntityId, AZ::Color> m_orignalColor;
    };
} // namespace ROS2::Demo
