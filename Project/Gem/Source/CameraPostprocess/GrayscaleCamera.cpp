/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GrayscaleCamera.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <opencv2/opencv.hpp>

namespace ROS2::Demo
{
    void GrayscaleCamera::Activate()
    {
        AZ_Printf("GrayscaleCamera", "GrayscaleCamera activated");
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void GrayscaleCamera::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void GrayscaleCamera::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GrayscaleCamera, AZ::Component>()->Version(1);
            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GrayscaleCamera>("GrayscaleCamera", "GrayscaleCamera")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }

    void GrayscaleCamera::ApplyPostProcessing(sensor_msgs::msg::Image& imageMessage)
    {
        AZ_Assert(imageMessage.encoding == "rgba8", "GrayscaleCamera only supports rga8 format");
        // Convert the image to grayscale
        cv::Mat rgb_image(cv::Size(imageMessage.width, imageMessage.height), CV_8UC4, (void*)imageMessage.data.data());
        // Convert the RGB image to grayscale
        cv::Mat mono_image;
        cv::cvtColor(rgb_image, mono_image, cv::COLOR_RGBA2GRAY);
        sensor_msgs::msg::Image imageMessageMono;
        imageMessageMono.encoding = "mono8";
        imageMessageMono.width = mono_image.cols;
        imageMessageMono.height = mono_image.rows;
        imageMessageMono.step = mono_image.cols * sizeof(uint8_t);
        imageMessageMono.data = std::vector<uint8_t>(mono_image.data, mono_image.data + mono_image.total() * mono_image.elemSize());
        imageMessageMono.header = imageMessage.header;
        std::swap(imageMessageMono, imageMessage);
    }
} // namespace ROS2::Demo
