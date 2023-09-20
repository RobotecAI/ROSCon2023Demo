/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <lane_provider_msgs/srv/list_tracks.hpp>
#include <rclcpp/rclcpp.hpp>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2::Demo
{
    using ListTracksRequest = std::shared_ptr<lane_provider_msgs::srv::ListTracks::Request>;
    using ListTracksResponse = std::shared_ptr<lane_provider_msgs::srv::ListTracks::Response>;

    class LanesServiceComponent : public AZ::Component
    {
        //! Component that stores lanes.
    public:
        AZ_COMPONENT(LanesServiceComponent, "{3c3a5a90-2395-4278-8251-54d819b0f6ac}");

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        AZStd::vector<AZ::EntityId> m_tracks;

        AZStd::string m_globalFrame = "map";
        rclcpp::Service<lane_provider_msgs::srv::ListTracks>::SharedPtr m_listTracksService;

        // If no name is provided, it returns all lanes.
        // If a valid lane name is provided, it returns that specific lane.
        // If an invalid lane name is provided, it returns an empty message.
        void ListTracks(const ListTracksRequest request, const ListTracksResponse response);
    };

} // namespace ROS2::Demo
