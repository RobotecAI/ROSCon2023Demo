/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace ROS2::Demo
{
    struct PathInfo
    {
    public:
        AZ_TYPE_INFO(PathInfo, "{f2677037-0a9b-4725-8f9c-6035f869d0ec}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_entityId{ AZ::EntityId::InvalidEntityId };
        size_t m_orderNumber;
        size_t m_poseCount{ 10 };
        bool m_reverseDirection{ false };
    };

    struct PathInfoComparator
    {
        AZ_TYPE_INFO(PathInfoComparator, "{630e3ae8-fd20-447a-8194-e9b211af7f40}");

        bool operator()(const PathInfo& lhs, const PathInfo& rhs) const
        {
            return lhs.m_orderNumber < rhs.m_orderNumber;
        }
    };

} // namespace ROS2::Demo
