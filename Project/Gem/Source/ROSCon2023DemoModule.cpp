
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "Navigation/LaneComponent.h"
#include "Navigation/LanesServiceComponent.h"
#include "Navigation/SplinePosesPublisher.h"
#include "ROSCon2023DemoSystemComponent.h"
#include "Scripting/BoxSpawner.h"
#include "Scripting/ObjectDetectionComponent.h"
#include "Scripting/PayloadDespawnerComponent.h"
#include "Scripting/ScriptSpawnLevelComponent.h"
#include "Vision/IdealVisionSystem.h"
#include <Scripting/FoilWrapperController.h>
#include "CameraPostprocess/GrayscaleCamera.h"
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
                    ROS2::Demo::BoxSpawner::CreateDescriptor(),
                    ROS2::Demo::SplinePosesPublisher::CreateDescriptor(),
                    ROS2::Demo::LaneComponent::CreateDescriptor(),
                    ROS2::Demo::LanesServiceComponent::CreateDescriptor(),
                    ROS2::Demo::FoilWrapper::CreateDescriptor(),
                    ROS2::Demo::ScriptSpawnLevelComponent::CreateDescriptor(),
                    ROS2::Demo::PayloadDespawnerComponent::CreateDescriptor(),
                    ROS2::Demo::ObjectDetectionComponent::CreateDescriptor(),
                    ROS2::Demo::GrayscaleCamera::CreateDescriptor(),
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
