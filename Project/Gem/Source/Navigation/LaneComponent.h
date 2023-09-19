/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>
#include <ROSCon2023Demo/Navigation/PathInfo.h>
#include <lane_provider_msgs/msg/detail/lane_paths__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>

namespace ROS2::Demo
{
    class LaneComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
        //! Component that stores paths associated with one working lane (one robotic arm).
    public:
        AZ_COMPONENT(LaneComponent, "{8c11ead8-5a40-4bbc-8547-4386d0eb1dd2}");

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        AZStd::string GetLaneName();
        lane_provider_msgs::msg::LanePaths GetLanePathMsgs();

    private:
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        AZStd::set<PathInfo, PathInfoComparator> m_paths;
        AZStd::map<size_t, nav_msgs::msg::Path> m_pathsMsgs;
        lane_provider_msgs::msg::LanePaths m_lanePathMsgs;
        AZStd::string m_globalFrame = "map";

        nav_msgs::msg::Path CalculatePoses(const PathInfo& pathEntityId);
    };

} // namespace ROS2::Demo
