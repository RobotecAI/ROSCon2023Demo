# This file is copied during engine registration. Edits to this file will be lost next
# time a registration happens.

set(FILES
    Include/ROSCon2023Demo/ROSCon2023DemoBus.h
    Source/Navigation/LaneComponent.cpp
    Source/Navigation/LaneComponent.h
    Source/Navigation/LanesService.cpp
    Source/Navigation/LanesService.h
    Source/Navigation/SplinePosesPublisher.cpp
    Source/Navigation/SplinePosesPublisher.h
    Source/HumanNpc/AnimGraphInputProviderComponent.cpp
    Source/HumanNpc/AnimGraphInputProviderComponent.h
    Source/HumanNpc/NavigationMeshOrchestratorComponent.cpp
    Source/HumanNpc/NavigationMeshOrchestratorComponent.h
    Source/HumanNpc/NpcNavigatorComponent.cpp
    Source/HumanNpc/NpcNavigatorComponent.h
    Source/HumanNpc/WaypointBus.h
    Source/HumanNpc/WaypointComponent.cpp
    Source/HumanNpc/WaypointComponent.h
    Source/ROSCon2023DemoSystemComponent.cpp
    Source/ROSCon2023DemoSystemComponent.h
    Source/Scripting/BoxSpawner.cpp
    Source/Scripting/BoxSpawner.h
    Source/Vision/IdealVisionSystem.cpp
    Source/Vision/IdealVisionSystem.h
    Source/Vision/IdealVisionSystemConfiguration.cpp
    Source/Vision/IdealVisionSystemConfiguration.h
    enabled_gems.cmake
)
