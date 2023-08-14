#include <HumanNpc/PathParserComponent.h>

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
    void PathParserComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PathParserComponent, AZ::Component>()
                ->Version(1)
                ->Field("Debug Mode", &PathParserComponent::m_debugMode)
                ->Field("Goal", &PathParserComponent::m_goal)
                ->Field("Navigation Entity", &PathParserComponent::m_navigationEntity)
                ->Field("Topic Configuration", &PathParserComponent::m_topicConfiguration)
                ->Field("Linear Speed", &PathParserComponent::m_linearSpeed)
                ->Field("Angular Speed", &PathParserComponent::m_angularSpeed)
                ->Field("Cross Track Factor", &PathParserComponent::m_crossTrackFactor);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext->Class<PathParserComponent>("Path Parser", "Component that processes paths and publishes twist messages")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_debugMode, "Debug Mode", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_goal, "Goal", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_navigationEntity, "Navigation Entity", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_topicConfiguration, "Topic Configuration", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_linearSpeed, "Linear Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_angularSpeed, "Angular Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathParserComponent::m_crossTrackFactor, "Cross Track Factor", "");
                // clang-format on
            }
        }
    }

    void PathParserComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();

        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        const auto& topicName = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_topicConfiguration.m_topic);
        const auto& qos = m_topicConfiguration.GetQoS();
        m_publisher = ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.data(), qos);

        m_goalQueue.push(m_goal);

        AZ::TickBus::Handler::BusConnect();
        if (m_debugMode)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(m_entity->GetId());
        }
    }

    void PathParserComponent::Deactivate()
    {
        if (m_debugMode)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect(m_entity->GetId());
        }
        AZ::TickBus::Handler::BusDisconnect();
    }

    float PathParserComponent::GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2)
    {
        return AZ::Atan2(unitVector1.Cross(unitVector2).Dot(AZ::Vector3::CreateAxisZ()), unitVector1.Dot(unitVector2));
    }

    void PathParserComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
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
            m_goalQueue.pop();
        }

        auto&& [previousGoal, currentGoal] = GetGoal();
        Publish(CalculateSpeed(CurrentTransform, previousGoal, currentGoal));
        if (IsClose(CurrentTransform.GetTranslation(), currentGoal) && (++m_currentPathIndex == m_path.size()))
        {
            AZ_Printf(__func__, "Finished iteration over path.");
            m_currentPathIndex = 0UL;
            m_path.clear();
            Publish({});
        }
    }

    void PathParserComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        for (AZ::Vector3 point : m_path)
        {
            debugDisplay.SetColor(AZ::Colors::Yellow);
            debugDisplay.DrawBall(point, 0.2f);
        }
    }

    AZ::Transform PathParserComponent::GetCurrentTransform()
    {
        AZ::Transform currentTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(currentTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        return currentTransform;
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> PathParserComponent::GetGoal()
    {
        return AZStd::make_pair(
            m_currentPathIndex != 0 ? m_path[m_currentPathIndex - 1] : GetCurrentTransform().GetTranslation(), m_path[m_currentPathIndex]);
    }

    PathParserComponent::Speed PathParserComponent::CalculateSpeed(
        AZ::Transform currentTransform, AZ::Vector3 previousGoalPosition, AZ::Vector3 currentGoalPosition)
    {
        const AZ::Vector3 RobotPosition = currentTransform.GetTranslation();
        const AZ::Vector3 RobotDirection = currentTransform.GetBasisX().GetNormalized();

        const AZ::Vector3 GoalDirection = (currentGoalPosition - RobotPosition).GetNormalized();
        const float BearingError = GetSignedAngleBetweenUnitVectors(RobotDirection, GoalDirection);

        const AZ::Vector3 RobotInPreviousGoalFrame = RobotPosition - previousGoalPosition;
        const AZ::Vector3 CurrentGoalInPreviousGoalFrame = currentGoalPosition - previousGoalPosition;
        const float CrossTrackError = RobotInPreviousGoalFrame.Cross(CurrentGoalInPreviousGoalFrame.GetNormalized()).GetLength();

        return Speed{ .m_linear = m_linearSpeed, .m_angular = BearingError * m_angularSpeed - CrossTrackError * m_crossTrackFactor };
    }

    AZStd::vector<AZ::Vector3> PathParserComponent::FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition)
    {
        AZ::EntityId navigationMeshEntityId;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            navigationMeshEntityId, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::GetNavigationMeshEntity);
        if (!navigationMeshEntityId.IsValid())
        {
            AZ_Error(__func__, false, "Unable to query the Detour Navigation Request Bus.");
            return {};
        }

        EnsureRecastMeshInitialized(navigationMeshEntityId);

        AZStd::vector<AZ::Vector3> path;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            path, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::FindPathBetweenPositions, currentPosition, goalPosition);
        return path;
    }

    void PathParserComponent::EnsureRecastMeshInitialized(AZ::EntityId navigationMeshEntityId)
    {
        navigationMeshEntityId.SetInvalid();
    }

    void PathParserComponent::Publish(Speed speed)
    {
        geometry_msgs::msg::Twist cmdVelMessage;
        cmdVelMessage.linear.x = speed.m_linear;
        cmdVelMessage.angular.z = speed.m_angular;

        m_publisher->publish(cmdVelMessage);
    }
} // namespace ROS2::Demo