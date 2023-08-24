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
#include <ROS2/Communication/TopicConfiguration.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2::Demo
{
    class NpcNavigatorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private RecastNavigation::RecastNavigationMeshNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(NpcNavigatorComponent, "{2b71bda6-b986-4627-8e68-15821565f503}", AZ::Component);

        NpcNavigatorComponent() = default;
        ~NpcNavigatorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        // clang-format off
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required) {}
        // clang-format on

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        using PublisherPtr = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>;

        struct Speed
        {
            float m_linear{ 0.0f };
            float m_angular{ 0.0f };
        };

        static constexpr float AcceptableError = 1.0f;
        // clang-format off
        static bool IsClose(AZ::Vector3 vector1, AZ::Vector3 vector2) { return vector1.GetDistance(vector2) < AcceptableError; }
        // clang-format on

        // Assumes that the argument vectors lie in the XY plane.
        static float GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2);

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // EntityDebugDisplayEventBus overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        // RecastNavigationMeshNotificationBus overrides
        void OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity) override;
        void OnNavigationMeshBeganRecalculating(AZ::EntityId navigationMeshEntity) override {}

        PublisherPtr CreatePublisher(const TopicConfiguration& topicConfiguration);

        static AZ::Transform GetEntityTransform(AZ::EntityId entityId);

        AZ::Transform GetCurrentTransform();
        AZ::EntityId GetNavigationMeshEntityId();
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetGoal();
        NpcNavigatorComponent::Speed CalculateSpeed(
            AZ::Transform currentTransform, AZ::Vector3 previousGoalPosition, AZ::Vector3 currentGoalPosition);
        AZStd::vector<AZ::Vector3> FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition);

        void Publish(Speed speed);
        void RecalculateCurrentGoalPath();

        // DEBUG
        AZ::EntityId m_goalEntityId;
        // DEBUG
        bool m_debugMode{ false };

        AZStd::queue<AZ::Vector3> m_goalQueue;
        AZStd::vector<AZ::Vector3> m_path;
        size_t m_currentPathIndex{ 0LU };
        AZ::EntityId m_navigationEntity;

        float m_linearSpeed{ 1.5f };
        float m_angularSpeed{ 1.0f };
        float m_crossTrackFactor{ 0.1f };

        TopicConfiguration m_topicConfiguration;
        PublisherPtr m_publisher;
    };
} // namespace ROS2::Demo
