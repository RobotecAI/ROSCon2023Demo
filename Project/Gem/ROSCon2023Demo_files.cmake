# This file is copied during engine registration. Edits to this file will be lost next
# time a registration happens.

set(FILES
    Include/ROSCon2023Demo/ROSCon2023DemoBus.h
    Include/ROSCon2023Demo/Navigation/PathInfo.h
    Source/Navigation/LaneComponent.cpp
    Source/Navigation/LaneComponent.h
    Source/Navigation/LanesServiceComponent.cpp
    Source/Navigation/LanesServiceComponent.h
    Source/Navigation/PathInfo.cpp
    Source/Navigation/SplinePosesPublisher.cpp
    Source/Navigation/SplinePosesPublisher.h
    Source/ROSCon2023DemoSystemComponent.cpp
    Source/ROSCon2023DemoSystemComponent.h
    Source/Scripting/BoxSpawner.cpp
    Source/Scripting/BoxSpawner.h
    Source/Scripting/ScriptSpawnSytemBus.h
    Source/Scripting/FoilWrapperController.cpp
    Source/Scripting/FoilWrapperController.h
    Source/Scripting/PayloadDespawnerComponent.cpp
    Source/Scripting/PayloadDespawnerComponent.h
    Source/Scripting/ScriptSpawnLevelComponent.cpp
    Source/Scripting/ScriptSpawnLevelComponent.h
    Source/Scripting/ObjectDetectionComponent.cpp
    Source/Scripting/ObjectDetectionComponent.h
    Source/Vision/IdealVisionSystem.cpp
    Source/Vision/IdealVisionSystem.h
    Source/Vision/IdealVisionSystemConfiguration.cpp
    Source/Vision/IdealVisionSystemConfiguration.h
    enabled_gems.cmake
)
