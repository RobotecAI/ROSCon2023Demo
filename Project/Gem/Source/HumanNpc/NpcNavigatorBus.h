#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Component/EntityId.h>

namespace ROS2::Demo
{
    class NpcNavigatorRequests
    {
    public:
        AZ_RTTI(NpcNavigatorRequests, "{31d0a864-9d15-4ad7-a597-a4573937957d}");
        virtual ~NpcNavigatorRequests() = default;

        virtual void ClearWaypoints() = 0;
        virtual void AddWaypoint(AZ::EntityId waypointEntityId) = 0;
    };

    class NpcNavigatorRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////
    };

    using NpcNavigatorRequestBus = AZ::EBus<NpcNavigatorRequests, NpcNavigatorRequestBusTraits>;
} // namespace ROS2::Demo