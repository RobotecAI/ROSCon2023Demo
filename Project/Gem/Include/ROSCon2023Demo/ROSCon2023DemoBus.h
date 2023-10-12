
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROSCon2023Demo
{
    class ROSCon2023DemoRequests
    {
    public:
        AZ_RTTI(ROSCon2023DemoRequests, "{F5A9E3A5-CE4C-45E5-ACE7-16D372336D14}");
        virtual ~ROSCon2023DemoRequests() = default;
        // Put your public methods here
        //! Reload the current level
        virtual void ReloadLevel() = 0;
    };

    class ROSCon2023DemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////

    };

    using ROSCon2023DemoRequestBus = AZ::EBus<ROSCon2023DemoRequests, ROSCon2023DemoBusTraits>;
    using ROSCon2023DemoInterface = AZ::Interface<ROSCon2023DemoRequests>;

} // namespace ROSCon2023Demo
