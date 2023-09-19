/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>

namespace ROS2::Demo
{
    class IdealVisionSystemNotification : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        virtual void OnObjectsDetected(AZStd::vector<AZ::EntityId> detectedObjects) = 0;
    };

    using IdealVisionSystemNotificationBus = AZ::EBus<IdealVisionSystemNotification>;

} // namespace ROS2::Demo
