/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "IdealVisionSystemConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/publisher.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace ROS2::Demo
{
    class IdealVisionSystem
        : public ROS2SensorComponentBase<TickBasedSource>
        , protected AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(IdealVisionSystem, "{4f983ee4-33e7-447e-b668-970166e24519}", SensorBaseType);
        IdealVisionSystem();
        IdealVisionSystem(const IdealVisionSystemConfiguration& configuration);
        ~IdealVisionSystem() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        static constexpr size_t FrustumPointCount = 5;

        // ROS::SensorComponent overrides...
        void FrequencyTick();

        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        //! Create a frustum of points that represent the camera's view.
        //! @param cameraMatrix The camera's matrix.
        //! @param cameraWidth The width of the camera.
        //! @param cameraHeight The height of the camera.
        //! @param detectionRange Thhe range of the camera.
        //! @return The frustum points in the camera's local coordinate system.
        AZStd::array<AZ::Vector3, FrustumPointCount> CreateFrustumPoints(
            const AZ::Matrix3x3& cameraMatrix, float cameraWidth, float cameraHeight, float detectionRange) const;

        //! Create a PhysX triangle mesh shape configuration from a set of points.
        //! @param frustumPoints The points to create the mesh from.
        AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> CookFrustumMesh(
            const AZStd::array<AZ::Vector3, FrustumPointCount>& frustumPoints) const;

        //! Get the lines that represent the frustum as lines for visualization (in camera's local coordinate system).
        //! @param frustumPoints The points to create the lines from, oder is important, and should be the same as the @ref
        //! CreateFrustumPoints.
        //! @return The lines that represent the frustum.
        AZStd::vector<AZ::Vector3> CreateFrustumLines(const AZStd::array<AZ::Vector3, FrustumPointCount>& frustumPoints) const;

        IdealVisionSystemConfiguration m_configuration; //!< The configuration for the ideal vision system.

        AZ::Matrix3x3 m_cameraMatrix = AZ::Matrix3x3::CreateIdentity(); //!< The camera's matrix from ROS2 Camera Sensor
        float m_cameraWidth{ 0 }; //!< The width of the camera.
        float m_cameraHeight{ 0 }; //!< The height of the camera.

        AZStd::array<AZ::Vector3, FrustumPointCount> m_frustumPoints; //!< Cache of the frustum points in camera's local coordinate system.
        AZStd::vector<AZ::Vector3> m_frustumLines; //!< Cache of the frustum lines for visualization.
        AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> m_cookedMesh; //!< The cooked mesh shape configuration for the frustum.

        //! ROS2 publishers
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> m_detection2DPublisher;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>> m_detection3DPublisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> m_detectionArrayPublisher;
    };
} // namespace ROS2::Demo
