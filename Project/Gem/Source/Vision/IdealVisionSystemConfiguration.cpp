/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "IdealVisionSystemConfiguration.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::Demo
{
    void IdealVisionSystemConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<IdealVisionSystemConfiguration>()
                ->Version(0)
                ->Field("ExcludeEntities", &IdealVisionSystemConfiguration::m_excludeEntities)
                ->Field("MaximumDetectionRange", &IdealVisionSystemConfiguration::m_maximumDetectionRange);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<IdealVisionSystemConfiguration>("IdealVisionSystemConfiguration", "Configuration for the IdealVisionSystem")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2::Demo")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &IdealVisionSystemConfiguration::m_excludeEntities,
                        "Exclude Entities",
                        "Entities to exclude from the vision system")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &IdealVisionSystemConfiguration::m_maximumDetectionRange,
                        "Maximum Detection Range",
                        "Maximum distance at which entities can be detected");
            }
        }
    }
} // namespace ROS2::Demo
