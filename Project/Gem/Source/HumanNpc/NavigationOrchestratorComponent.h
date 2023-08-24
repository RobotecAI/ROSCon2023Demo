#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2::Demo
{
    class NavigationOrchestratorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AZ::EntityBus::Handler
    {
    public:
        AZ_COMPONENT(NavigationOrchestratorComponent, "{fb2eabfd-cafb-4fe2-999d-9faec09fb9ba}", AZ::Component);

        NavigationOrchestratorComponent() = default;
        ~NavigationOrchestratorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("RecastNavigationMeshComponent"));
        }

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AZ::EntityBus overrides
        void OnEntityActivated(const AZ::EntityId&);

        void UpdateNavigationMesh();

        bool m_isAsync{ true };
        float m_elapsedTime{ 0.0f }; //!< In seconds.
        float m_updateFrequency{ 0.1f }; //!< In Hertz.
    };
} // namespace ROS2::Demo
