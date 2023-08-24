
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "ROSCon2023DemoSystemComponent.h"
#include "Vision/IdealVisionSystem.h"
#include <HumanNpc/AnimGraphInputProviderComponent.h>
#include <HumanNpc/NpcNavigatorComponent.h>
#include <HumanNpc/NavigationOrchestratorComponent.h>

namespace ROSCon2023Demo
{
    class ROSCon2023DemoModule : public AZ::Module
    {
    public:
        AZ_RTTI(ROSCon2023DemoModule, "{C1A10E6E-069C-4D2D-97AD-73198D467101}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROSCon2023DemoModule, AZ::SystemAllocator, 0);

        ROSCon2023DemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROSCon2023DemoSystemComponent::CreateDescriptor(),
                    ROS2::Demo::IdealVisionSystem::CreateDescriptor(),
                    ROS2::Demo::AnimGraphInputProviderComponent::CreateDescriptor(),
                    ROS2::Demo::NpcNavigatorComponent::CreateDescriptor(),
                    ROS2::Demo::NavigationOrchestratorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROSCon2023DemoSystemComponent>(),
            };
        }
    };
} // namespace ROSCon2023Demo

AZ_DECLARE_MODULE_CLASS(Gem_ROSCon2023Demo, ROSCon2023Demo::ROSCon2023DemoModule)
