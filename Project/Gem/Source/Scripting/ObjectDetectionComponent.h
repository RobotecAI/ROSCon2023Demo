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
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ROS2::Demo
{
    class ObjectDetectionConfig
    {
    public:
        AZ_TYPE_INFO(ObjectDetectionConfig, "{3e2d7fce-adb9-4bce-91eb-a221fdda4179}");

        static void Reflect(AZ::ReflectContext* context);
        AZ::EntityId m_collisionTrigger;
        AZStd::vector<AZStd::string> m_detectableObjectsTags;
        ROS2::TopicConfiguration m_topicConfiguration;
    };

    class ObjectDetectionComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ObjectDetectionComponent, "{5ad991a7-fb4a-40af-b3b2-07880faf5372}");

        ObjectDetectionComponent() = default;

        ~ObjectDetectionComponent() = default;

        // AZ::Component overrides...
        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    private:
        bool DoesObjectContainTag(AZ::EntityId entityId);

        ObjectDetectionConfig m_configuration;

        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerEnterHandler;
        AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler m_onTriggerExitHandler;
        AZStd::unordered_set<AZ::EntityId> m_collidingEntities;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_topicPublisher;
    };
} // namespace ROS2::Demo
