#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2::Demo
{
    class AnimGraphInputProviderComponent
        : public AZ::Component
        , private AZ::EntityBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(AnimGraphInputProviderComponent, "{eb8bede0-1802-4a53-94a3-52572c975820}", AZ::Component);
        AnimGraphInputProviderComponent() = default;
        ~AnimGraphInputProviderComponent() override = default;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("EMotionFXAnimGraphService"));
        }
        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // EntityBus overrides
        void OnEntityActivated(const AZ::EntityId&) override;

        void SetAnimGraphParameters(float linearSpeed);

        AZ::EntityId m_rigidBodyEntityId;
        float m_linearSpeedMultiplier{ 0.05f };
    };
} // namespace ROS2::Demo
