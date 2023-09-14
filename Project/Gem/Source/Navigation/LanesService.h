/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lane_provider_msgs/srv/list_tracks.hpp>


#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2::Demo
{
    using ListTracksRequest = std::shared_ptr<lane_provider_msgs::srv::ListTracks::Request>;
    using ListTracksResponse = std::shared_ptr<lane_provider_msgs::srv::ListTracks::Response>;

    class LanesService : public AZ::Component
    {
        //! Component that stores paths which are associated with one working lane (one robotic arm).
    public:
        AZ_COMPONENT(LanesService, "{8c11ead8-5a40-4bbc-8547-4386d0eb1dd2}");
        // LanesService() = default;
        // ~LanesService() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        AZStd::vector<AZ::EntityId> m_laneEntities;
        rclcpp::Service<lane_provider_msgs::srv::ListTracks>::SharedPtr m_listTracksService;
        AZStd::string m_globalFrame = "map";
        size_t m_poseCount = 10;

        void ListTracks(const ListTracksRequest request, const ListTracksResponse response);
 
    };
} // namespace ROS2::Demo