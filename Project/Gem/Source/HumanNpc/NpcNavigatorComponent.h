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
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/queue.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/utils.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <HumanNpc/NpcNavigatorBus.h>
#include <HumanNpc/WaypointComponent.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2::Demo
{
    //! Component used for navigating an npc along a selected waypoint path.
    class NpcNavigatorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private RecastNavigation::RecastNavigationMeshNotificationBus::Handler
        , private NpcNavigatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT(NpcNavigatorComponent, "{2b71bda6-b986-4627-8e68-15821565f503}", AZ::Component);

        NpcNavigatorComponent() = default;
        ~NpcNavigatorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        // clang-format off
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required) {}
        // clang-format on

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        using PublisherPtr = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>;
        enum class NavigationState
        {
            Idle,
            Navigate,
            Rotate,
        };

        struct Speed
        {
            float m_linear{ 0.0f }, m_angular{ 0.0f };
        };

        struct GoalPose
        {
            AZ::Vector3 m_position{};
            AZ::Vector3 m_direction{};
        };

        static constexpr float AcceptableDistanceError = 0.5f;
        static constexpr float AcceptableAngleError = 0.1f;
        // clang-format off
        static bool IsClose(AZ::Vector3 vector1, AZ::Vector3 vector2) { return vector1.GetDistance(vector2) < AcceptableDistanceError; }
        // clang-format on

        static AZ::Transform GetEntityTransform(AZ::EntityId entityId);
        // Assumes that the argument vectors lie in the XY plane.
        static float GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2);
        static AZ::EntityId GetNavigationMeshEntityId(AZ::EntityId detourNavigationEntity);
        static NpcNavigatorComponent::Speed CalculateSpeedForGoal(
            const AZ::Transform& currentTransform, GoalPose goal, Speed maxSpeed, float crossTrackFactor);
        static WaypointConfiguration FetchWaypointConfiguration(AZ::EntityId waypointEntityId);
        static NpcNavigatorComponent::PublisherPtr CreatePublisher(
            ROS2FrameComponent* frame, const ROS2::TopicConfiguration& topicConfiguration);

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // EntityDebugDisplayEventBus overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        // RecastNavigationMeshNotificationBus overrides
        void OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity) override;
        void OnNavigationMeshBeganRecalculating(AZ::EntityId navigationMeshEntity) override
        {
        }

        // NpcNavigatorRequestBus overrides
        void ClearWaypoints() override;
        void AddWaypoint(AZ::EntityId waypointEntityId) override;

        [[nodiscard]] AZ::Transform GetCurrentTransform() const;

        AZStd::vector<GoalPose> TryFindGoalPath();
        [[nodiscard]] AZStd::vector<NpcNavigatorComponent::GoalPose> ConstructGoalPath(
            const AZStd::vector<AZ::Vector3>& positionPath) const;
        AZStd::vector<AZ::Vector3> FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition);
        NpcNavigatorComponent::Speed CalculateSpeed(float deltaTime);

        void Publish(Speed speed);
        void RecalculateCurrentGoalPath();

        bool m_debugMode{ false }, m_restartOnTraversed{ true };

        NavigationState m_state{ NavigationState::Navigate };
        AZ::EntityId m_navigationEntity;
        WaypointConfiguration m_waypointConfiguration;
        size_t m_waypointIndex{ 0LU }, m_goalIndex{ 0LU };

        float m_linearSpeed{ 1.5f };
        float m_angularSpeed{ 1.0f };
        float m_crossTrackFactor{ 0.1f };

        AZStd::vector<AZ::EntityId> m_waypointEntities;
        AZStd::vector<GoalPose> m_goalPath;

        TopicConfiguration m_topicConfiguration;
        PublisherPtr m_publisher;
    };
} // namespace ROS2::Demo
