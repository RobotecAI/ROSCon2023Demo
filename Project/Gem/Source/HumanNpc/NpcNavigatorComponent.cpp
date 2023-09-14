#include <HumanNpc/NpcNavigatorComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <HumanNpc/WaypointBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <RecastNavigation/DetourNavigationBus.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>

namespace ROS2::Demo
{
    void NpcNavigatorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NpcNavigatorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Debug Mode", &NpcNavigatorComponent::m_debugMode)
                ->Field("Waypoints", &NpcNavigatorComponent::m_waypointEntities)
                ->Field("Detour Navigation Entity", &NpcNavigatorComponent::m_navigationEntity)
                ->Field("Topic Configuration", &NpcNavigatorComponent::m_topicConfiguration)
                ->Field("Linear Speed", &NpcNavigatorComponent::m_linearSpeed)
                ->Field("Angular Speed", &NpcNavigatorComponent::m_angularSpeed)
                ->Field("Cross Track Factor", &NpcNavigatorComponent::m_crossTrackFactor);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext->Class<NpcNavigatorComponent>("Npc Navigator", "Component that processes paths and publishes twist messages")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_debugMode, "Debug Mode", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_waypointEntities, "Waypoints", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NpcNavigatorComponent::m_navigationEntity,
                        "Detour Navigation Entity",
                        "Entity with the Detour Navigation Component")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_topicConfiguration, "Topic Configuration", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_linearSpeed, "Linear Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_angularSpeed, "Angular Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_crossTrackFactor, "Cross Track Factor", "");
                // clang-format on
            }
        }
    }

    void NpcNavigatorComponent::Activate()
    {
        m_publisher = CreatePublisher(ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity()), m_topicConfiguration);

        AZ::TickBus::Handler::BusConnect();
        RecastNavigation::RecastNavigationMeshNotificationBus::Handler::BusConnect(GetNavigationMeshEntityId(m_navigationEntity));
        if (m_debugMode)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(m_entity->GetId());
        }
    }

    void NpcNavigatorComponent::Deactivate()
    {
        if (m_debugMode)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect(m_entity->GetId());
        }
        RecastNavigation::RecastNavigationMeshNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZ::Transform NpcNavigatorComponent::GetEntityTransform(AZ::EntityId entityId)
    {
        AZ::Transform currentTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(currentTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        return currentTransform;
    }

    float NpcNavigatorComponent::GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2)
    {
        return AZ::Atan2(unitVector1.Cross(unitVector2).Dot(AZ::Vector3::CreateAxisZ()), unitVector1.Dot(unitVector2));
    }

    AZ::EntityId NpcNavigatorComponent::GetNavigationMeshEntityId(AZ::EntityId detourNavigationEntity)
    {
        AZ::EntityId navigationMeshEntityId;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            navigationMeshEntityId, detourNavigationEntity, &RecastNavigation::DetourNavigationRequests::GetNavigationMeshEntity);
        return navigationMeshEntityId;
    }

    NpcNavigatorComponent::Speed NpcNavigatorComponent::CalculateSpeedForGoal(
        const AZ::Transform& currentTransform, GoalPose goal, Speed maxSpeed, float crossTrackFactor)
    {
        const AZ::Vector3 RobotPosition = currentTransform.GetTranslation();
        const AZ::Vector3 RobotDirection = currentTransform.GetBasisX().GetNormalized();

        float bearingError = 0.0f;
        if (goal.m_position != RobotPosition)
        {
            const AZ::Vector3 GoalDirection = (goal.m_position - RobotPosition).GetNormalized();
            bearingError = GetSignedAngleBetweenUnitVectors(RobotDirection, GoalDirection);
        }

        const AZ::Vector3 GoalToRobot = RobotPosition - goal.m_position;
        const float CrossTrackError = goal.m_direction.Cross(GoalToRobot).GetLength();

        return Speed{ .m_linear = maxSpeed.m_linear, .m_angular = bearingError * maxSpeed.m_angular - CrossTrackError * crossTrackFactor };
    }

    WaypointConfiguration NpcNavigatorComponent::FetchWaypointConfiguration(AZ::EntityId waypointEntityId)
    {
        WaypointConfiguration waypointConfiguration;
        WaypointRequestBus::EventResult(waypointConfiguration, waypointEntityId, &WaypointRequests::GetConfiguration);
        return waypointConfiguration;
    }

    NpcNavigatorComponent::PublisherPtr NpcNavigatorComponent::CreatePublisher(
        ROS2FrameComponent* frame, const ROS2::TopicConfiguration& topicConfiguration)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        const auto& topicName = ROS2::ROS2Names::GetNamespacedName(frame->GetNamespace(), topicConfiguration.m_topic);
        const auto& qos = topicConfiguration.GetQoS();
        return ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.data(), qos);
    }

    void NpcNavigatorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_goalPath.empty() && (m_goalPath = TryFindGoalPath()).empty())
        {
            return;
        }

        Publish(CalculateSpeed(deltaTime));
    }

    void NpcNavigatorComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        for (size_t pointIndex = 0; pointIndex < m_goalPath.size(); ++pointIndex)
        {
            if (pointIndex == m_goalIndex)
            {
                debugDisplay.SetColor(AZ::Colors::YellowGreen);
                debugDisplay.DrawBall(m_goalPath[pointIndex].m_position, 0.1f);
            }
            else if (pointIndex == (m_goalPath.size() - 1))
            {
                debugDisplay.SetColor(AZ::Colors::Green);
                debugDisplay.DrawBall(m_goalPath[pointIndex].m_position, 0.2f);
            }

            if (pointIndex != 0)
            {
                debugDisplay.SetColor(AZ::Colors::Yellow);
                debugDisplay.DrawArrow(m_goalPath[pointIndex - 1].m_position, m_goalPath[pointIndex].m_position, 0.2f);
            }

            debugDisplay.SetColor(AZ::Colors::Red);
            debugDisplay.DrawArrow(
                m_goalPath[pointIndex].m_position, m_goalPath[pointIndex].m_position + m_goalPath[pointIndex].m_direction, 0.2f);
        }
    }

    void NpcNavigatorComponent::OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity)
    {
        RecalculateCurrentGoalPath();
    }

    AZ::Transform NpcNavigatorComponent::GetCurrentTransform() const
    {
        return GetEntityTransform(GetEntityId());
    }

    AZStd::vector<NpcNavigatorComponent::GoalPose> NpcNavigatorComponent::TryFindGoalPath()
    {
        if (m_waypointIndex >= m_waypointEntities.size())
        {
            return {};
        }

        const AZ::EntityId WaypointEntity = m_waypointEntities[m_waypointIndex];
        const AZStd::vector<AZ::Vector3> PositionPath =
            FindPathBetweenPositions(GetCurrentTransform().GetTranslation(), GetEntityTransform(WaypointEntity).GetTranslation());
        if (PositionPath.empty())
        {
            return {};
        }

        m_waypointConfiguration = FetchWaypointConfiguration(WaypointEntity);
        return ConstructGoalPath(PositionPath);
    }

    AZStd::vector<NpcNavigatorComponent::GoalPose> NpcNavigatorComponent::ConstructGoalPath(
        const AZStd::vector<AZ::Vector3>& positionPath) const
    {
        AZStd::vector<GoalPose> goalPath;
        for (size_t i = 0; i < positionPath.size(); ++i)
        {
            AZ::Vector3 direction = AZ::Vector3::CreateZero();
            if (i == positionPath.size() - 1 && m_waypointConfiguration.m_orientationCaptured)
            {
                direction =
                    GetEntityTransform(m_waypointEntities[m_waypointIndex]).GetRotation().TransformVector(AZ::Vector3::CreateAxisX());
            }
            else
            {
                direction = (positionPath[i + 1] - positionPath[i]).GetNormalized();
            }
            goalPath.push_back({ .m_position = positionPath[i], .m_direction = direction });
        }
        return goalPath;
    }

    AZStd::vector<AZ::Vector3> NpcNavigatorComponent::FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition)
    {
        if (!GetNavigationMeshEntityId(m_navigationEntity).IsValid())
        {
            AZ_Error(__func__, false, "Unable to query the Detour Navigation Request Bus.");
            return {};
        }

        AZStd::vector<AZ::Vector3> path;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            path, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::FindPathBetweenPositions, currentPosition, goalPosition);

        return path;
    }

    void NpcNavigatorComponent::Publish(Speed speed)
    {
        geometry_msgs::msg::Twist cmdVelMessage;
        cmdVelMessage.linear.x = speed.m_linear;
        cmdVelMessage.angular.z = speed.m_angular;

        m_publisher->publish(cmdVelMessage);
    }

    NpcNavigatorComponent::Speed NpcNavigatorComponent::CalculateSpeed(float deltaTime)
    {
        switch (m_state)
        {
        case NavigationState::IDLE:
            if ((m_waypointConfiguration.m_idleTime -= deltaTime) <= 0.0f)
            {
                m_goalIndex = 0;
                ++m_waypointIndex;
                if (m_waypointIndex >= m_waypointEntities.size())
                {
                    m_waypointIndex = 0;
                }
                m_goalPath.clear();
                m_waypointConfiguration = FetchWaypointConfiguration(m_waypointEntities[m_waypointIndex]);
                m_state = NavigationState::NAVIGATE;
            }
            return {};
        case NavigationState::ROTATE:
            {
                AZ_Assert(m_goalIndex == m_goalPath.size(), "The Npc Navigator component is in an invalid state due to programmer's error.");
                const float BearingError = GetSignedAngleBetweenUnitVectors(
                    GetCurrentTransform().GetRotation().TransformVector(AZ::Vector3::CreateAxisX()),
                    m_goalPath[m_goalIndex - 1].m_direction);

                if (std::abs(BearingError) < AcceptableAngleError)
                {
                    m_state = NavigationState::IDLE;
                    return {};
                }
                else
                {
                    return {
                        .m_linear = 0.0f,
                        .m_angular = m_angularSpeed * BearingError,
                    };
                }
            }
        case NavigationState::NAVIGATE:
            if (IsClose(m_goalPath[m_goalIndex].m_position, GetCurrentTransform().GetTranslation()))
            {
                if (++m_goalIndex == m_goalPath.size())
                {
                    m_state = m_waypointConfiguration.m_orientationCaptured ? NavigationState::ROTATE : NavigationState::IDLE;
                    return {};
                }
            }

            return CalculateSpeedForGoal(
                GetCurrentTransform(),
                m_goalPath[m_goalIndex],
                { .m_linear = m_linearSpeed, .m_angular = m_angularSpeed },
                m_crossTrackFactor);
        }
    }

    void NpcNavigatorComponent::RecalculateCurrentGoalPath()
    {
        if (m_waypointIndex == m_waypointEntities.size())
        {
            return;
        }

        m_goalIndex = 0;
        m_goalPath = TryFindGoalPath();
    }
} // namespace ROS2::Demo