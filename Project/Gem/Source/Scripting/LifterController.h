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
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>

namespace ROS2::Demo
{
    class LifterControllerComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(LifterControllerComponent, { "80a906af-eacd-4044-94a7-f7f02100fc2a" });

        LifterControllerComponent() = default;
        ~LifterControllerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_lifterTopicSubscriber;

        ROS2::TopicConfiguration m_topicConfiguration;
        float m_setpoint;
    };
} // namespace ROS2::Demo
