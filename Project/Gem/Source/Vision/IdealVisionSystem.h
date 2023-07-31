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
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/publisher.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace ROS2::Demo
{

    class IdealVisionSystem : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(IdealVisionSystem, "{4f983ee4-33e7-447e-b668-970166e24519}", ROS2SensorComponent);
        IdealVisionSystem();
        ~IdealVisionSystem() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        // ROS::SensorComponent overrides...
        void Visualise() override;
        void FrequencyTick();

        AZ::Matrix3x3 m_cameraMatrix = AZ::Matrix3x3::CreateIdentity();
        float m_cameraWidth{ 0 };
        float m_cameraHeight{ 0 };

        AZStd::vector<AZ::Vector3> m_frustrumPoints;

        //! Configuration stores the convex geometry for the cylinder and shape scale.
        AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> m_cookedMesh;

        AZStd::unordered_set<AZ::EntityId> m_excludeEntities;

        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> m_detection2DPublisher;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>> m_detection3DPublisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> m_detectionArrayPublisher;
    };
} // namespace ROS2::Demo
