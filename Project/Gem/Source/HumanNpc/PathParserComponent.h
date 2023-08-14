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
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2::Demo
{
    class PathParserComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(PathParserComponent, "{2b71bda6-b986-4627-8e68-15821565f503}", AZ::Component);

        PathParserComponent() = default;
        ~PathParserComponent() = default;

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

        // AzFramework::EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZ::Transform GetCurrentTransform();
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetGoal();
        PathParserComponent::Speed CalculateSpeed(
            AZ::Transform currentTransform, AZ::Vector3 previousGoalPosition, AZ::Vector3 currentGoalPosition);
        AZStd::vector<AZ::Vector3> FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition);
        void EnsureRecastMeshInitialized(AZ::EntityId navigationMeshEntityId);
        void Publish(Speed speed);

        // DEBUG
        AZ::Vector3 m_goal;
        // DEBUG
        bool m_debugMode{ false };

        AZStd::queue<AZ::Vector3> m_goalQueue;
        AZStd::vector<AZ::Vector3> m_path;
        size_t m_currentPathIndex{ 0LU };
        AZ::EntityId m_navigationEntity;

        float m_linearSpeed{ 1.0f };
        float m_angularSpeed{ 1.0f };
        float m_crossTrackFactor{ 1.0f };

        TopicConfiguration m_topicConfiguration;
        PublisherPtr m_publisher;
    };
} // namespace ROS2::Demo
