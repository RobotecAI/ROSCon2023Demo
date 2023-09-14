/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/std/string/string.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2::Demo
{
    class LaneComponent : public AZ::Component
    {
        //! Component that stores paths which are associated with one working lane (one robotic arm).
    public:
        AZ_COMPONENT(LaneComponent, "{8c11ead8-5a40-4bbc-8547-4386d0eb1dd2}");
        // LaneComponent() = default;
        // ~LaneComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        AZStd::string GetLaneName();
        AZStd::vector<AZ::EntityId> GetPaths();

    private:
        int m_laneNumber = 0;
        AZStd::vector<AZ::EntityId> m_paths;
        //get paths associated with one working lane
    };
} // namespace ROS2::Demo