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
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2::Demo
{
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

    AZ::Vector2 GetPointIn2D(const AZ::Vector3& point3D, const AZ::Matrix3x3& cameraMatrix)
    {
        AZ::Vector3 point2D;
        point2D = cameraMatrix * point3D;
        point2D = point2D / point2D.GetZ();
        return AZ::Vector2(aznumeric_cast<float>(point2D.GetX()), aznumeric_cast<float>(point2D.GetY()));
    }

    void IdealVisionSystem::Activate()
    {
        constexpr float MaximalDetection = 2.0f;

        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraMatrix, GetEntityId(), &ROS2::CameraCalibrationRequest::GetCameraMatrix);
        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraWidth, GetEntityId(), &ROS2::CameraCalibrationRequest::GetWidth);
        ROS2::CameraCalibrationRequestBus::EventResult(m_cameraHeight, GetEntityId(), &ROS2::CameraCalibrationRequest::GetHeight);

        m_frustrumPoints.clear();
        m_frustrumPoints.push_back(AZ::Vector3(0.0f, 0.0f, 0.0f));
        m_frustrumPoints.push_back(GetPointIn3D(AZ::Vector2(0.0f, 0.0f), MaximalDetection, m_cameraMatrix));
        m_frustrumPoints.push_back(GetPointIn3D(AZ::Vector2(0.0f, m_cameraHeight), MaximalDetection, m_cameraMatrix));
        m_frustrumPoints.push_back(GetPointIn3D(AZ::Vector2(m_cameraWidth, m_cameraHeight), MaximalDetection, m_cameraMatrix));
        m_frustrumPoints.push_back(GetPointIn3D(AZ::Vector2(m_cameraWidth, 0.0f), MaximalDetection, m_cameraMatrix));

        // cook PhysX mesh
        AZStd::vector<AZ::u8> cookedData;
        bool cookingResult = false;
        Physics::SystemRequestBus::BroadcastResult(
            cookingResult,
            &Physics::SystemRequests::CookConvexMeshToMemory,
            m_frustrumPoints.data(),
            aznumeric_cast<AZ::u32>(m_frustrumPoints.size()),
            cookedData);
        if (cookingResult)
        {
            AZ_Printf("IdealVisionSystem", "Cooked PhysX mesh size: %d", cookedData.size());
            m_cookedMesh = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
            m_cookedMesh->SetCookedMeshData(cookedData.data(), cookedData.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

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

            ROS2SensorComponent::Activate();
        }
        else
        {
            AZ_Error("IdealVisionSystem", false, "Failed to cook PhysX mesh");
        }
    }

    void IdealVisionSystem::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_detection3DPublisher.reset();
        m_detection2DPublisher.reset();
        m_detectionArrayPublisher.reset();
    }

    void IdealVisionSystem::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<IdealVisionSystem, ROS2SensorComponent>()->Version(1)->Field(
                "ExcludeEntities", &IdealVisionSystem::m_excludeEntities);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<IdealVisionSystem>("IdealVisionSystem", "IdealVisionSystem.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "IdealVisionSystem")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &IdealVisionSystem::m_excludeEntities, "ExcludeEntities", "ExcludeEntities");
            }
        }
    }

    void IdealVisionSystem::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2CameraSensor"));
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
            AZ_Printf("IdealVisionSystem", "Overlap with %d objects", results.m_hits.size());

            for (const auto& result : results.m_hits)
            {
                if (!m_excludeEntities.contains(result.m_entityId))
                {
                    const AZ::Aabb aabb = result.m_shape->GetAabbLocal();

                    AZ::Transform resutlTransform;
                    AZ::TransformBus::EventResult(resutlTransform, result.m_entityId, &AZ::TransformBus::Events::GetWorldTM);
                    const AZ::Transform resultTransformLocal = sensorTransformInv * resutlTransform;

                    AZStd::string targetName;
                    AZ::ComponentApplicationBus::BroadcastResult(
                        targetName, &AZ::ComponentApplicationBus::Events::GetEntityName, result.m_entityId);

                    geometry_msgs::msg::Pose pose;
                    pose.orientation = ROS2::ROS2Conversions::ToROS2Quaternion(resultTransformLocal.GetRotation());
                    pose.position = ROS2::ROS2Conversions::ToROS2Point(resultTransformLocal.GetTranslation());
                    poseArray.poses.push_back(pose);

                    // construct 3D detection
                    vision_msgs::msg::Detection3D detection3D;
                    detection3D.header = header;
                    detection3D.id = targetName.c_str();

                    detection3D.bbox.center.position = pose.position;
                    detection3D.bbox.size.x = aabb.GetXExtent() * 2.0f;
                    detection3D.bbox.size.y = aabb.GetYExtent() * 2.0f;
                    detection3D.bbox.size.z = aabb.GetZExtent() * 2.0f;

                    detection3D.results.resize(1);
                    detection3D.results.front().pose.pose = pose;
                    detection3D.results.front().hypothesis.class_id = -1;
                    detection3D.results.front().hypothesis.score = 1.0f;

                    detection3DArray.detections.push_back(detection3D);

                    // construct 2D detection
                    vision_msgs::msg::Detection2D detection2D;
                    detection2D.header = header;
                    detection2D.id = targetName.c_str();

                    AZ::Vector2 center2D = GetPointIn2D(resultTransformLocal.GetTranslation(), m_cameraMatrix);

                    detection2D.results.resize(1);
                    detection2D.results.front().pose.pose.position.x = center2D.GetX();
                    detection2D.results.front().pose.pose.position.y = center2D.GetY();
                    detection2D.results.front().hypothesis.class_id = -1;
                    detection2D.results.front().hypothesis.score = 1.0f;

                    detection2DArray.detections.push_back(detection2D);
                }
            }
        }

        m_detectionArrayPublisher->publish(poseArray);
        m_detection2DPublisher->publish(detection2DArray);
        m_detection3DPublisher->publish(detection3DArray);
    }
    void IdealVisionSystem::Visualise()
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        // Draw frustum
        auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        if (entityScene)
        {
            auto drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);

            if (drawQueue)
            {
                AZStd::vector<AZ::Vector3> linePoints;

                AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;

                AZ_Assert(m_frustrumPoints.size() > 5, "Frustrum points are not 5");

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints.front()));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[1]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints.front()));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[2]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints.front()));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[3]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints.front()));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[4]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[1]));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[2]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[2]));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[3]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[3]));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[4]));

                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[4]));
                linePoints.push_back(transform.TransformPoint(m_frustrumPoints[1]));

                const uint8_t pixelSize = 5;
                drawArgs.m_size = pixelSize;
                drawArgs.m_colors = &AZ::Colors::Green;
                drawArgs.m_verts = linePoints.data();
                drawArgs.m_vertCount = linePoints.size();

                drawArgs.m_colorCount = 1;
                drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
                drawArgs.m_size = pixelSize;
                drawQueue->DrawLines(drawArgs);
            }
        }
    }

} // namespace ROS2::Demo
