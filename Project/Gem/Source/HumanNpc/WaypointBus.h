#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Component/EntityId.h>

namespace ROS2::Demo
{
    struct WaypointConfiguration;

    class WaypointRequests
    {
    public:
        AZ_RTTI(WaypointRequests, "{0c56b5b6-2daf-4b8b-a1a3-7b43a04d3549}");
        virtual ~WaypointRequests() = default;

        virtual WaypointConfiguration GetConfiguration() = 0;
    };

    class WaypointRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////
    };

    using WaypointRequestBus = AZ::EBus<WaypointRequests, WaypointRequestBusTraits>;
} // namespace ROS2::Demo