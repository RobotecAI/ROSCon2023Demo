/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Demo
{
    class IdealVisionSystemConfiguration
    {
    public:
        AZ_TYPE_INFO(IdealVisionSystemConfiguration, "{2049573f-5041-47d9-82c6-0d8f64239014}");
        static void Reflect(AZ::ReflectContext* context);
        AZStd::unordered_set<AZ::EntityId> m_excludeEntities;
        float m_maximumDetectionRange{ 2.0f };
    };
} // namespace ROS2::Demo
