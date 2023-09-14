/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LaneComponent.h"
#include "AzCore/Component/EntityId.h"
#include "AzCore/std/containers/vector.h"
#include "AzCore/std/string/string.h"
#include <AzCore/Serialization/EditContext.h>
#include <string>

namespace ROS2::Demo
{

    void LaneComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LaneComponent>()
                ->Version(1)
                ->Field("LaneNumber", &LaneComponent::m_laneNumber)
                ->Field("PathsInLane", &LaneComponent::m_paths);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LaneComponent>("LaneComponent", "LaneComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LaneComponent::m_laneNumber,
                        "Lane number",
                        "Number corresponding to the robotic arm")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LaneComponent::m_paths,
                        "Set of paths associated with one working lane",
                        "Set of paths associated with one working lane");
            }
        }
    }

    void LaneComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void LaneComponent::Activate()
    {
    }

    void LaneComponent::Deactivate()
    {
    }

    AZStd::string LaneComponent::GetLaneName()
    {
        return AZStd::string("lane_" + AZStd::to_string(m_laneNumber));
    }

    AZStd::vector<AZ::EntityId> LaneComponent::GetPaths()
    {
        return m_paths;
    }

} // namespace ROS2::Demo
