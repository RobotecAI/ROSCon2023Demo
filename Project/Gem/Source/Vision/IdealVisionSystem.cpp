/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "IdealVisionSystem.h"
#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/SystemBus.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2::Demo
{

    namespace Internal
    {
        //! Get the 3D point from a 2D point and a depth value
        //! @param point2D point's coordinates in camera image (in pixels)
        //! @param depth depth value (in meters)
        //! @param cameraMatrix camera matrix
        //! @return 3D point in camera's local coordinate system
        AZ::Vector3 GetPointIn3D(const AZ::Vector2& point2D, float depth, AZ::Matrix3x3 cameraMatrix)
        {
            cameraMatrix = cameraMatrix.GetInverseFull();
            AZ::Vector3 point3D;
            point3D.SetX(point2D.GetX());
            point3D.SetY(point2D.GetY());
            point3D.SetZ(1.0f);
            point3D = depth * cameraMatrix * point3D;
            return point3D;
        }

        //! Get the 2D point from a 3D point
        //! @param point3D point's coordinates in camera's local coordinate system
        //! @param cameraMatrix camera matrix
        //! @return 2D point in camera image (in pixels)
        AZ::Vector2 GetPointIn2D(const AZ::Vector3& point3D, const AZ::Matrix3x3& cameraMatrix)
        {
            AZ::Vector3 point2D;
            point2D = cameraMatrix * point3D;
            point2D = point2D / point2D.GetZ();
            return AZ::Vector2(aznumeric_cast<float>(point2D.GetX()), aznumeric_cast<float>(point2D.GetY()));
        }
    } // namespace Internal

    constexpr char Detection2DType[] = "vision_msgs::msg::Detection2D";
    constexpr char Detection3DType[] = "vision_msgs::msg::Detection3D";
    constexpr char DetectionPoseArrayType[] = "geometry_msgs::msg::PoseArray";

    IdealVisionSystem::IdealVisionSystem()
    {
        TopicConfiguration tc;

        tc.m_type = DetectionPoseArrayType;
        tc.m_topic = "detectionsPoseArray";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(tc.m_type, tc));

        tc.m_type = Detection2DType;
        tc.m_topic = "detections2D";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(tc.m_type, tc));

        tc.m_type = Detection3DType;
        tc.m_topic = "detections3D";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(tc.m_type, tc));

        m_sensorConfiguration.m_frequency = 10;
    }

    IdealVisionSystem::IdealVisionSystem(const IdealVisionSystemConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    AZStd::array<AZ::Vector3, IdealVisionSystem::FrustumPointCount> IdealVisionSystem::CreateFrustumPoints(
        const AZ::Matrix3x3& cameraMatrix, float cameraWidth, float cameraHeight, float detectionRange) const
    {
        AZStd::array<AZ::Vector3, IdealVisionSystem::FrustumPointCount> frustumPoints;
        frustumPoints[0] = AZ::Vector3::CreateZero();
        frustumPoints[1] = Internal::GetPointIn3D(AZ::Vector2(0.0f, 0.0f), detectionRange, cameraMatrix);
        frustumPoints[2] = Internal::GetPointIn3D(AZ::Vector2(0.0f, cameraHeight), detectionRange, cameraMatrix);
        frustumPoints[3] = Internal::GetPointIn3D(AZ::Vector2(cameraWidth, cameraHeight), detectionRange, cameraMatrix);
        frustumPoints[4] = Internal::GetPointIn3D(AZ::Vector2(cameraWidth, 0.0f), detectionRange, cameraMatrix);
        return frustumPoints;
    }
    AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> IdealVisionSystem::CookFrustumMesh(
        const AZStd::array<AZ::Vector3, IdealVisionSystem::FrustumPointCount>& frustumPoints) const
    {
        AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> cookedMesh;
        // cook PhysX mesh
        AZStd::vector<AZ::u8> cookedData;
        bool cookingResult = false;
        Physics::SystemRequestBus::BroadcastResult(
            cookingResult,
            &Physics::SystemRequests::CookConvexMeshToMemory,
            m_frustumPoints.data(),
            aznumeric_cast<AZ::u32>(m_frustumPoints.size()),
            cookedData);
        if (cookingResult)
        {
            AZ_Printf("IdealVisionSystem", "Cooked PhysX mesh size: %d", cookedData.size());
            cookedMesh = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
            cookedMesh->SetCookedMeshData(cookedData.data(), cookedData.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);
            return cookedMesh;
        }
        return cookedMesh;
    }

    void IdealVisionSystem::Activate()
    {
        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraMatrix, GetEntityId(), &ROS2::CameraCalibrationRequest::GetCameraMatrix);
        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraWidth, GetEntityId(), &ROS2::CameraCalibrationRequest::GetWidth);
        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraHeight, GetEntityId(), &ROS2::CameraCalibrationRequest::GetHeight);

        m_frustumPoints = CreateFrustumPoints(m_cameraMatrix, m_cameraWidth, m_cameraHeight, m_configuration.m_maximumDetectionRange);
        m_cookedMesh = CookFrustumMesh(m_frustumPoints);
        AZ_Assert(m_cookedMesh, "Failed to cook PhysX mesh for IdealVisionSystem sensor");
        if (!m_cookedMesh)
        {
            AZ_Error("IdealVisionSystem", false, "Failed to cook PhysX mesh, IdealVisionSystem sensor will not be activated");
            return;
        }

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(
            m_sensorConfiguration.m_publishersConfigurations.size() == 3,
            "Invalid configuration of publishers for IdealVisionSystem sensor");
        const TopicConfiguration& publisherConfig2DDetection = m_sensorConfiguration.m_publishersConfigurations[Detection2DType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig2DDetection.m_topic);
        m_detection2DPublisher =
            ros2Node->create_publisher<vision_msgs::msg::Detection2DArray>(fullTopic.data(), publisherConfig2DDetection.GetQoS());

        const TopicConfiguration& publisherConfig3DDetection = m_sensorConfiguration.m_publishersConfigurations[Detection3DType];
        fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig3DDetection.m_topic);
        m_detection3DPublisher =
            ros2Node->create_publisher<vision_msgs::msg::Detection3DArray>(fullTopic.data(), publisherConfig3DDetection.GetQoS());

        const TopicConfiguration& publisherConfigPoseArray = m_sensorConfiguration.m_publishersConfigurations[DetectionPoseArrayType];
        fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfigPoseArray.m_topic);
        m_detectionArrayPublisher =
            ros2Node->create_publisher<geometry_msgs::msg::PoseArray>(fullTopic.data(), publisherConfigPoseArray.GetQoS());

        if (m_sensorConfiguration.m_visualize)
        {
            m_frustumLines = CreateFrustumLines(m_frustumPoints);
            AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(this->GetEntityId());
        }

        ROS2SensorComponent::Activate();
    }

    void IdealVisionSystem::Deactivate()
    {
        m_detection3DPublisher.reset();
        m_detection2DPublisher.reset();
        m_detectionArrayPublisher.reset();
        if (m_sensorConfiguration.m_visualize)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        }
        ROS2SensorComponent::Deactivate();

    }

    void IdealVisionSystem::Reflect(AZ::ReflectContext* context)
    {
        IdealVisionSystemConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<IdealVisionSystem, ROS2SensorComponent>()->Version(1)->Field(
                "configuration", &IdealVisionSystem::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<IdealVisionSystem>("IdealVisionSystem", "IdealVisionSystem.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "IdealVisionSystem")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &IdealVisionSystem::m_configuration, "configuration", "configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void IdealVisionSystem::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2CameraSensor"));
    }

    AZStd::vector<AZ::Vector3> IdealVisionSystem::CreateFrustumLines(
        const AZStd::array<AZ::Vector3, IdealVisionSystem::FrustumPointCount>& frustumPoints) const
    {
        constexpr AZStd::array<AZStd::pair<int, int>, 8> LinesIndices{
            { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 1 } }
        };
        AZStd::vector<AZ::Vector3> linePoints;
        linePoints.reserve(LinesIndices.size());

        for (const auto& [i, j] : LinesIndices)
        {
            linePoints.push_back(frustumPoints[i]);
            linePoints.push_back(frustumPoints[j]);
        }
        return linePoints;
    }

    void IdealVisionSystem::FrequencyTick()
    {
        AZ::Transform sensorTransform;
        AZ::TransformBus::EventResult(sensorTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        const AZ::Transform sensorTransformInv = sensorTransform.GetInverse();

        geometry_msgs::msg::PoseArray poseArray;
        vision_msgs::msg::Detection2DArray detection2DArray;
        vision_msgs::msg::Detection3DArray detection3DArray;

        std_msgs::msg::Header header;
        header.frame_id = GetFrameID().c_str();
        header.stamp = ROS2Interface::Get()->GetROSTimestamp();

        poseArray.header = header;
        detection3DArray.header = header;

        // Overlap query with frustum
        if (m_cookedMesh && sceneInterface && defaultSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            AzPhysics::OverlapRequest request;
            request.m_shapeConfiguration = m_cookedMesh;
            request.m_pose = sensorTransform;
            request.m_filterCallback = nullptr;
            AzPhysics::SceneQueryHits results = sceneInterface->QueryScene(defaultSceneHandle, &request);

            for (const auto& result : results.m_hits)
            {
                if (!m_configuration.m_excludeEntities.contains(result.m_entityId))
                {
                    const AZ::Aabb aabb = result.m_shape->GetAabbLocal();

                    AZ::Transform resutlTransform;
                    AZ::TransformBus::EventResult(resutlTransform, result.m_entityId, &AZ::TransformBus::Events::GetWorldTM);
                    const AZ::Transform resultTransformLocal = sensorTransformInv * resutlTransform;

                    LmbrCentral::Tags tags;
                    LmbrCentral::TagComponentRequestBus::EventResult(tags, result.m_entityId, &LmbrCentral::TagComponentRequests::GetTags);
                    int tagId = tags.empty() ? 0 : static_cast<int>(*tags.begin());

                    AZStd::string targetName;
                    AZ::ComponentApplicationBus::BroadcastResult(
                        targetName, &AZ::ComponentApplicationBus::Events::GetEntityName, result.m_entityId);

                    AZ::Entity* targetEntity = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(
                        targetEntity, &AZ::ComponentApplicationBus::Events::FindEntity, result.m_entityId);

                    AZ_Assert(targetEntity, "IdealVisionSystem result entity not found");

                    auto targetROS2FrameComponent = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(targetEntity);
                    AZStd::string frameNamespace = "";
                    if (targetROS2FrameComponent)
                    {
                        frameNamespace = targetROS2FrameComponent->GetNamespace();
                    }

                    AZStd::string targetId = frameNamespace + "/" + targetName;

                    geometry_msgs::msg::Pose pose;
                    pose.orientation = ROS2::ROS2Conversions::ToROS2Quaternion(resultTransformLocal.GetRotation());
                    pose.position = ROS2::ROS2Conversions::ToROS2Point(resultTransformLocal.GetTranslation());
                    poseArray.poses.push_back(pose);

                    // construct 3D detection
                    vision_msgs::msg::Detection3D detection3D;
                    detection3D.header = header;
                    detection3D.id = targetId.c_str();

                    detection3D.bbox.center.position = pose.position;
                    detection3D.bbox.size.x = aabb.GetXExtent() * 2.0f;
                    detection3D.bbox.size.y = aabb.GetYExtent() * 2.0f;
                    detection3D.bbox.size.z = aabb.GetZExtent() * 2.0f;

                    detection3D.results.resize(1);
                    detection3D.results.front().pose.pose = pose;
                    detection3D.results.front().hypothesis.class_id = tagId;
                    detection3D.results.front().hypothesis.score = 1.0f;

                    detection3DArray.detections.push_back(detection3D);

                    // construct 2D detection
                    vision_msgs::msg::Detection2D detection2D;
                    detection2D.header = header;
                    detection2D.id = targetId.c_str();

                    AZ::Vector2 center2D = Internal::GetPointIn2D(resultTransformLocal.GetTranslation(), m_cameraMatrix);

                    detection2D.results.resize(1);
                    detection2D.results.front().pose.pose.position.x = center2D.GetX();
                    detection2D.results.front().pose.pose.position.y = center2D.GetY();
                    detection2D.results.front().hypothesis.class_id = tagId;
                    detection2D.results.front().hypothesis.score = 1.0f;

                    detection2DArray.detections.push_back(detection2D);
                }
            }
        }
        m_detectionArrayPublisher->publish(poseArray);
        m_detection2DPublisher->publish(detection2DArray);
        m_detection3DPublisher->publish(detection3DArray);
    }

    void IdealVisionSystem::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        debugDisplay.PushMatrix(transform);
        debugDisplay.DrawLines(m_frustumLines, AZ::Colors::Green);
        debugDisplay.PopMatrix();
    }

} // namespace ROS2::Demo
