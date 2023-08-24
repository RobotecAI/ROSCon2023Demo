#include <HumanNpc/NpcNavigatorComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
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
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NpcNavigatorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Debug Mode", &NpcNavigatorComponent::m_debugMode)
                ->Field("Goal Entity", &NpcNavigatorComponent::m_goalEntityId)
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_goalEntityId, "Goal Entity", "")
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
        m_publisher = CreatePublisher(m_topicConfiguration);

        m_goalQueue.push(GetEntityTransform(m_goalEntityId).GetTranslation());

        AZ::TickBus::Handler::BusConnect();
        // TODO: What if it is not yet initialized?
        RecastNavigation::RecastNavigationMeshNotificationBus::Handler::BusConnect(GetNavigationMeshEntityId());
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

    float NpcNavigatorComponent::GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2)
    {
        return AZ::Atan2(unitVector1.Cross(unitVector2).Dot(AZ::Vector3::CreateAxisZ()), unitVector1.Dot(unitVector2));
    }

    void NpcNavigatorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        const AZ::Transform CurrentTransform = GetCurrentTransform();
        if (m_path.empty())
        {
            if (m_goalQueue.empty())
            {
                AZ_Printf(__func__, "Nothing to do.");
                return;
            }

            m_path = FindPathBetweenPositions(GetCurrentTransform().GetTranslation(), m_goalQueue.front());
            if (m_path.empty())
            {
                AZ_Printf(__func__, "The generated path for current goal was empty.");
                return;
            }
            else
            {
                AZ_Printf(
                    __func__,
                    "Goal found -> [%f, %f, %f]. Path has %ul positions.",
                    m_goalQueue.front().GetX(),
                    m_goalQueue.front().GetY(),
                    m_goalQueue.front().GetZ(),
                    m_path.size());
            }
        }

        auto&& [previousGoal, currentGoal] = GetGoal();
        Publish(CalculateSpeed(CurrentTransform, previousGoal, currentGoal));
        if (IsClose(CurrentTransform.GetTranslation(), currentGoal) && (++m_currentPathIndex == m_path.size()))
        {
            AZ_Printf(__func__, "Finished iteration over path.");
            m_goalQueue.pop(); // Can get rid of the goal.
            m_currentPathIndex = 0;
            m_path.clear();
            Publish({});
        }
    }

    void NpcNavigatorComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        for (size_t pointIndex = 0; pointIndex < m_path.size(); ++pointIndex)
        {
            if (pointIndex == m_currentPathIndex)
            {
                debugDisplay.SetColor(AZ::Colors::YellowGreen);
                debugDisplay.DrawBall(m_path[pointIndex], 0.1f);
            }
            else if (pointIndex == (m_path.size() + 1))
            {
                debugDisplay.SetColor(AZ::Colors::Green);
                debugDisplay.DrawBall(m_path[pointIndex], 0.5f);
            }

            if (pointIndex != 0)
            {
                debugDisplay.SetColor(AZ::Colors::Yellow);
                debugDisplay.DrawArrow(m_path[pointIndex - 1], m_path[pointIndex], 0.1f);
            }
        }
    }

    void NpcNavigatorComponent::OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity)
    {
        RecalculateCurrentGoalPath();
    }

    NpcNavigatorComponent::PublisherPtr NpcNavigatorComponent::CreatePublisher(const ROS2::TopicConfiguration& topicConfiguration)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();

        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        const auto& topicName = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), topicConfiguration.m_topic);
        const auto& qos = topicConfiguration.GetQoS();
        return ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.data(), qos);
    }

    AZ::Transform NpcNavigatorComponent::GetEntityTransform(AZ::EntityId entityId)
    {
        AZ::Transform currentTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(currentTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        return currentTransform;
    }

    AZ::Transform NpcNavigatorComponent::GetCurrentTransform()
    {
        return GetEntityTransform(GetEntityId());
    }

    AZ::EntityId NpcNavigatorComponent::GetNavigationMeshEntityId()
    {
        AZ::EntityId navigationMeshEntityId;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            navigationMeshEntityId, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::GetNavigationMeshEntity);
        return navigationMeshEntityId;
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> NpcNavigatorComponent::GetGoal()
    {
        return AZStd::make_pair(
            m_currentPathIndex != 0 ? m_path[m_currentPathIndex - 1] : GetCurrentTransform().GetTranslation(), m_path[m_currentPathIndex]);
    }

    NpcNavigatorComponent::Speed NpcNavigatorComponent::CalculateSpeed(
        AZ::Transform currentTransform, AZ::Vector3 previousGoalPosition, AZ::Vector3 currentGoalPosition)
    {
        const AZ::Vector3 RobotPosition = currentTransform.GetTranslation();
        const AZ::Vector3 RobotDirection = currentTransform.GetBasisX().GetNormalized();

        const AZ::Vector3 GoalDirection = (currentGoalPosition - RobotPosition).GetNormalized();
        const float BearingError = GetSignedAngleBetweenUnitVectors(RobotDirection, GoalDirection);

        const AZ::Vector3 RobotInPreviousGoalFrame = RobotPosition - previousGoalPosition;
        const AZ::Vector3 CurrentGoalInPreviousGoalFrame = currentGoalPosition - previousGoalPosition;
        const float CrossTrackError = RobotInPreviousGoalFrame.Cross(CurrentGoalInPreviousGoalFrame.GetNormalized()).GetLength();
d
        AZ_Printf(__func__, "BearingError=%f, CrossTrackError=%f, ", BearingError, CrossTrackError);
        return Speed{ .m_linear = m_linearSpeed, .m_angular = BearingError * m_angularSpeed - CrossTrackError * m_crossTrackFactor };
    }

    AZStd::vector<AZ::Vector3> NpcNavigatorComponent::FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition)
    {
        if (!GetNavigationMeshEntityId().IsValid())
        {
            AZ_Error(__func__, false, "Unable to query the Detour Navigation Request Bus.");
            return {};
        }

        AZStd::vector<AZ::Vector3> path;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            path, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::FindPathBetweenPositions, currentPosition, goalPosition);
        AZ_Printf(
            __func__,
            "Queried path between: start=(%f, %f, %f); goal=(%f, %f, %f).",
            currentPosition.GetX(),
            currentPosition.GetY(),
            currentPosition.GetZ(),
            goalPosition.GetX(),
            goalPosition.GetY(),
            goalPosition.GetZ());

        return path;
    }

    void NpcNavigatorComponent::Publish(Speed speed)
    {
        geometry_msgs::msg::Twist cmdVelMessage;
        cmdVelMessage.linear.x = speed.m_linear;
        cmdVelMessage.angular.z = speed.m_angular;

        m_publisher->publish(cmdVelMessage);
    }

    void NpcNavigatorComponent::RecalculateCurrentGoalPath()
    {
        if (m_goalQueue.empty())
        {
            AZ_Printf(__func__, "Nothing to recalculate...");
            return;
        }

        AZ_Printf(__func__, "Recalculating...");
        m_currentPathIndex = 0;
        m_path = FindPathBetweenPositions(GetCurrentTransform().GetTranslation(), m_goalQueue.front());
        if (m_path.empty())
        {
            AZ_Printf(__func__, "The recalculated path was empty.");
        }
    }
} // namespace ROS2::Demo