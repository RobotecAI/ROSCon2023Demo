
#pragma once

#include <AzCore/Component/Component.h>

#include <ROSCon2023Demo/ROSCon2023DemoBus.h>

namespace ROSCon2023Demo
{
    class ROSCon2023DemoSystemComponent
        : public AZ::Component
        , protected ROSCon2023DemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROSCon2023DemoSystemComponent, "{B8794DFF-877B-4FE3-B268-2C6090259157}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROSCon2023DemoSystemComponent();
        ~ROSCon2023DemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ROSCon2023DemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    private:
        // ROSCon2023DemoRequests overrides ...
        void ReloadLevel() override;

    };
}
