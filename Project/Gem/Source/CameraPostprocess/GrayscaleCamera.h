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
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

namespace ROS2::Demo
{

    class GrayscaleCamera
        : public AZ::Component
        , private CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(GrayscaleCamera, "{0191e0df-a2df-748a-9ddf-6e1337d8cb73}");

        static void Reflect(AZ::ReflectContext* context);

    private:
        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        // CameraPostProcessingRequestBus::Handler overrides...
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override
        {
            return 0;
        }
    };
} // namespace ROS2::Demo
