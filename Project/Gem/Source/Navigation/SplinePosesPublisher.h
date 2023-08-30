/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>

#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/deque.h>

#include <AzCore/std/string/string.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ROS2::Demo
{
    //! Component that publishes poses along spline.
    class SplinePosesPublisher : public AZ::Component
    {
    public:
        AZ_COMPONENT(SplinePosesPublisher, "{67eb2130-660f-4011-a50b-946b5b7dc216}");
        SplinePosesPublisher() = default;
        ~SplinePosesPublisher() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        nav_msgs::msg::Path CalculatePoses();

        bool m_reverseDirection = false;
        AZStd::string m_topicName = "poses";
        AZStd::string m_globalFrame = "map";
        size_t m_poseCount = 10;

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> m_pathPublisher;
        nav_msgs::msg::Path m_path;
    };
} // namespace ROS2::Demo
