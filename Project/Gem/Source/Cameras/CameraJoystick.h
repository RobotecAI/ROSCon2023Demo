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
#include <sensor_msgs/msg/joy.hpp>
namespace ROS2::Demo
{

    class CameraJoystick
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
        const unsigned int AXIS_ALEIRON  = 0;
        const unsigned int AXIS_PITCH    = 1;
        const unsigned int AXIS_THROTTLE = 2;
        const unsigned int AXIS_CAM1     = 3;
        const unsigned int AXIS_CAM2     = 4;
        const unsigned int AXIS_HIDE     = 6;


    public:
        AZ_COMPONENT(CameraJoystick, "2caa4273-b9ba-4516-b33a-15f1b39f84e1");

        CameraJoystick();

        ~CameraJoystick() = default;

        // AZ::Component overrides...
        void Activate() override;

        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        ROS2::TopicConfiguration m_joystickTopicConfiguration;
        AZ::EntityId m_cameraRigidBody {AZ::EntityId::InvalidEntityId};
        AZ::EntityId m_cameraGimbalEntityId {AZ::EntityId::InvalidEntityId};
        AZStd::vector<AZ::EntityId> m_hideEntities;

        float m_cameraSpeed = 10.0f;
        float m_gimbalSpeed = 0.1f;

        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> m_joystickSubscription;


        // AZ::TickBus::Handler overrides...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        AZ::Vector3 m_cameraLinearVelocity= AZ::Vector3::CreateZero();
        AZ::Vector3 m_cameraAngularVelocity = AZ::Vector3::CreateZero();
        float m_gimbalAxis1 = 0.0f;
        float m_gimbalAxis2 = 0.0f;
        bool m_hide = false;
        bool m_hideOld = false;

    };
} // namespace ROS2::Demo
