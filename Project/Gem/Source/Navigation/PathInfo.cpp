/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROSCon2023Demo/Navigation/PathInfo.h>

namespace ROS2::Demo
{
    void PathInfo::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PathInfo>()
                ->Version(1)
                ->Field("EntityId", &PathInfo::m_entityId)
                ->Field("OrderNumber", &PathInfo::m_orderNumber)
                ->Field("PoseCount", &PathInfo::m_poseCount)
                ->Field("ReverseDirection", &PathInfo::m_reverseDirection);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PathInfo>("Information about the path in a lane", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathInfo::m_entityId, "EntityId", "ID of entity with path")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PathInfo::m_orderNumber, "Order number", "Order number of path in a lane")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &PathInfo::m_poseCount, "Pose count", "Number of poses to create and publish")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PathInfo::m_reverseDirection,
                        "Reverse direction",
                        "Reverse the robot movement direction");
            }
        }
    }

} // namespace ROS2::Demo
