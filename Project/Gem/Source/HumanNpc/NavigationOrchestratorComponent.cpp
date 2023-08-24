#include <HumanNpc/NavigationOrchestratorComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>

namespace ROS2::Demo
{
    void NavigationOrchestratorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NavigationOrchestratorComponent, AZ::Component>()
                ->Version(0)
                ->Field("NavMesh Update Frequency", &NavigationOrchestratorComponent::m_updateFrequency)
                ->Field("Asynchronous NavMesh Update", &NavigationOrchestratorComponent::m_isAsync);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext
                    ->Class<NavigationOrchestratorComponent>("Navigation Orchestrator", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NavigationOrchestratorComponent::m_updateFrequency, "NavMesh Update Frequency", "")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NavigationOrchestratorComponent::m_isAsync, "Asynchronous NavMesh Update", "");
                // clang-format on
            }
        }
    }

    void NavigationOrchestratorComponent::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
    }

    void NavigationOrchestratorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void NavigationOrchestratorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if ((m_elapsedTime += deltaTime) * m_updateFrequency > 1.0f)
        {
            m_elapsedTime = 0.0f;
            UpdateNavigationMesh();
        }
    }

    void NavigationOrchestratorComponent::OnEntityActivated(const AZ::EntityId&)
    {
        UpdateNavigationMesh();
        AZ::TickBus::Handler::BusConnect();
    }

    void NavigationOrchestratorComponent::UpdateNavigationMesh()
    {
        bool success = false;
        RecastNavigation::RecastNavigationMeshRequestBus::EventResult(
            success,
            GetEntityId(),
            (m_isAsync ? &RecastNavigation::RecastNavigationMeshRequests::UpdateNavigationMeshAsync
                       : &RecastNavigation::RecastNavigationMeshRequests::UpdateNavigationMeshBlockUntilCompleted));

        AZ_Printf(__func__, ( success ? "Updated the Navigation Mesh." : "Unable to update the Navigation Mesh."));
    }
} // namespace ROS2::Demo