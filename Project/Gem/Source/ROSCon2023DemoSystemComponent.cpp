
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "ROSCon2023DemoSystemComponent.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <ILevelSystem.h>
#include <ISystem.h>
#include <AzCore/Component/TickBus.h>
namespace ROSCon2023Demo
{
    void ROSCon2023DemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROSCon2023DemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROSCon2023DemoSystemComponent>("ROSCon2023Demo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }

        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ROSCon2023DemoRequestBus>("ROSCon2023DemoRequestBus")
                ->Event("ReloadLevel", &ROSCon2023DemoRequestBus::Events::ReloadLevel);
        }
    }

    void ROSCon2023DemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("ROSCon2023DemoService"));
    }

    void ROSCon2023DemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("ROSCon2023DemoService"));
    }

    void ROSCon2023DemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROSCon2023DemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROSCon2023DemoSystemComponent::ROSCon2023DemoSystemComponent()
    {
        if (ROSCon2023DemoInterface::Get() == nullptr)
        {
            ROSCon2023DemoInterface::Register(this);
        }
    }

    ROSCon2023DemoSystemComponent::~ROSCon2023DemoSystemComponent()
    {
        if (ROSCon2023DemoInterface::Get() == this)
        {
            ROSCon2023DemoInterface::Unregister(this);
        }
    }

    void ROSCon2023DemoSystemComponent::Init()
    {
    }

    void ROSCon2023DemoSystemComponent::Activate()
    {
        ROSCon2023DemoRequestBus::Handler::BusConnect();
    }

    void ROSCon2023DemoSystemComponent::Deactivate()
    {
        ROSCon2023DemoRequestBus::Handler::BusDisconnect();
    }

    void ROSCon2023DemoSystemComponent::ReloadLevel()
    {
        ISystem* systemInterface = nullptr;
        CrySystemRequestBus::BroadcastResult(systemInterface, &CrySystemRequests::GetCrySystem);
        if(systemInterface && systemInterface->GetILevelSystem())
        {
            ILevelSystem* levelSystem = systemInterface->GetILevelSystem();
            AZStd::string currentLevelName = levelSystem->GetCurrentLevelName();
            levelSystem->UnloadLevel();
            AZ::TickBus::QueueFunction([levelSystem, currentLevelName]() {
                                           levelSystem->LoadLevel(currentLevelName.c_str());
                                       });
        }
    }
}
