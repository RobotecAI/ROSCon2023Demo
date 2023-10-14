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

#include "CameraJoystick.h"
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>

namespace ROS2::Demo
{

   CameraJoystick::CameraJoystick()
   {
       m_joystickTopicConfiguration.m_topic = "joy";
   }

   void CameraJoystick::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<CameraJoystick>()
               ->Version(1)
               ->Field("joystickTopicConfiguration", &CameraJoystick::m_joystickTopicConfiguration)
               ->Field("cameraRigidBody", &CameraJoystick::m_cameraRigidBody)
               ->Field("cameraGimbalEntityId", &CameraJoystick::m_cameraGimbalEntityId)
               ->Field("cameraSpeed", &CameraJoystick::m_cameraSpeed)
               ->Field("gimbalSpeed", &CameraJoystick::m_gimbalSpeed)
               ->Field("hideEntities", &CameraJoystick::m_hideEntities);

           if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<CameraJoystick>("CameraJoystick", "CameraJoystick.")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "CameraJoystick")
                   ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoystick::m_joystickTopicConfiguration, "topic", "topic")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoystick::m_cameraRigidBody, "cameraRigidBody", "cameraRigidBody")
                   ->DataElement(
                       AZ::Edit::UIHandlers::Default,
                       &CameraJoystick::m_cameraGimbalEntityId,
                       "cameraGimbalEntityId",
                       "cameraGimbalEntityId")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoystick::m_cameraSpeed, "cameraSpeed", "cameraSpeed")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoystick::m_gimbalSpeed, "gimbalSpeed", "gimbalSpeed")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoystick::m_hideEntities, "hideEntities", "hideEntities");

           }
       }
   }

   float GetDeadzoned(float v, float d = 0.005f)
   {
       return AZStd::abs(v) < d ? 0.0f : v;
   }
   float Expo(float value, float exponent)
   {
       return value * (1.0f - exponent) + value * value * value * exponent;
   }
   void CameraJoystick::Activate()
   {
       auto* ros2Interface = ROS2Interface::Get();
       AZ_Assert(ros2Interface, "ROS2 interface not available");
       m_joystickSubscription = ros2Interface->GetNode()->create_subscription<sensor_msgs::msg::Joy>(
           m_joystickTopicConfiguration.m_topic.c_str(),
           m_joystickTopicConfiguration.GetQoS(),
           [&](sensor_msgs::msg::Joy msg)
           {
               const float expo = 0.0f;
               m_gimbalAxis1 = Expo(-GetDeadzoned(msg.axes[AXIS_CAM1]),expo);
               m_gimbalAxis2 = Expo(-GetDeadzoned(msg.axes[AXIS_CAM2]),expo);
               float pitchSpeed = Expo(GetDeadzoned(msg.axes[AXIS_PITCH]), expo);
               float aileronSpeed = Expo(-GetDeadzoned(msg.axes[AXIS_ALEIRON]), expo);
               float throttle = Expo(GetDeadzoned(msg.axes[AXIS_THROTTLE], 0.05),expo);
               m_cameraAngularVelocity = { 0.0f, 0.0f, -m_gimbalAxis1 };
               m_cameraLinearVelocity = { -pitchSpeed, -aileronSpeed, throttle };

               float hideAxis = msg.axes[AXIS_HIDE];
               if (hideAxis>0)
               {
                   m_hide = true;
               }
               else if (hideAxis<0)
               {
                   m_hide = false;
               }
           });

       AZ::TickBus::Handler::BusConnect();
   }

   void CameraJoystick::Deactivate()
   {
       AZ::TickBus::Handler::BusDisconnect();
   }

   void CameraJoystick::OnTick(float deltaTime, AZ::ScriptTimePoint time)
   {
       AZ::Transform transformDrone = AZ::Transform::Identity();
       AZ::TransformBus::EventResult(transformDrone, m_cameraRigidBody, &AZ::TransformBus::Events::GetWorldTM);

       Physics::RigidBodyRequestBus::Event(
           m_cameraRigidBody, &Physics::RigidBodyRequests::SetAngularVelocity, m_cameraSpeed * m_cameraAngularVelocity);
       Physics::RigidBodyRequestBus::Event(
           m_cameraRigidBody,
           &Physics::RigidBodyRequests::SetLinearVelocity,
           transformDrone.GetRotation().TransformVector(m_cameraSpeed * m_cameraLinearVelocity));

       AZ::Transform transform = AZ::Transform::Identity();
       AZ::TransformBus::EventResult(transform, m_cameraGimbalEntityId, &AZ::TransformBus::Events::GetWorldTM);
       AZ::Quaternion updateLocal =
           AZ::Quaternion::CreateFromScaledAxisAngle(deltaTime * m_gimbalSpeed * m_gimbalAxis2 * AZ::Vector3::CreateAxisX());

       transform.SetRotation(transform.GetRotation() * updateLocal);
       AZ::TransformBus::Event(m_cameraGimbalEntityId, &AZ::TransformBus::Events::SetWorldTM, transform);

       if (m_hide!=m_hideOld)
       {
           for (auto& entityId : m_hideEntities)
           {
               AZ::Render::MeshComponentRequestBus::Event(entityId, &AZ::Render::MeshComponentRequests::SetVisibility, !m_hide);

           }
       }
       m_hideOld = m_hide;
   }
} // namespace ROS2::Demo
