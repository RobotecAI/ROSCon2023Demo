#include <HumanNpc/WaypointComponent.h>

namespace ROS2::Demo
{
    void WaypointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WaypointConfiguration>()
                ->Version(1)
                ->Field("Orientation captured", &WaypointConfiguration::m_orientationCaptured)
                ->Field("Idle time", &WaypointConfiguration::m_idleTime);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // clang-format off
                editContext->Class<WaypointConfiguration>("Waypoint Configuration", "Waypoint Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointConfiguration::m_orientationCaptured,
                        "Orientation captured",
                        "Should the waypoint orientation be captured?")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointConfiguration::m_idleTime,
                        "Idle time",
                        "Time spent at waypoint.");
                // clang-format on
            }
        }
    }

    void WaypointComponent::Reflect(AZ::ReflectContext* context)
    {
        WaypointConfiguration::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WaypointComponent, AZ::Component>()->Version(0)->Field(
                "Waypoint Configuration", &WaypointComponent::m_configuration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext
                    ->Class<WaypointComponent>("Waypoint", "Waypoint")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &WaypointComponent::m_configuration, "Waypoint Configuration", "Waypoint Configuration");
                // clang-format on
            }
        }
    }

    void WaypointComponent::Activate()
    {
        WaypointRequestBus::Handler::BusConnect(GetEntityId());
    }

    void WaypointComponent::Deactivate()
    {
        WaypointRequestBus::Handler::BusDisconnect();
    }

    WaypointConfiguration WaypointComponent::GetConfiguration()
    {
        return m_configuration;
    }
} // namespace ROS2::Demo