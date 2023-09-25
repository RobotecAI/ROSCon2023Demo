#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzFramework/Spawnable/Spawnable.h>
namespace ROS2::Demo
{

    class ScriptSpawnSystemRequests
    {
    public:
        AZ_RTTI(ScriptSpawnSystemRequests, "{ff669933-433d-4a41-a168-e23e010b7434}");
        virtual ~ScriptSpawnSystemRequests() = default;

        //! Spawn an asset at the given transform with the given name and spawnable.
        //! The spawnable name is used to track the spawned entity and despawn it later.
        //! @param spawnable The spawnable to spawn.
        //! @param transform The transform to spawn the asset at.
        //! @param spawnableName The name of the spawnable to spawn.
        //! @param parent The parent entity id of the spawned entity, optional, set to InvalidEntityId if not used.
        virtual void SpawnAssetAndSetParent(
            const AZ::Data::Asset<AzFramework::Spawnable>& spawnable,
            const AZ::Transform& transform,
            const AZStd::string& spawnableName,
            const AZ::EntityId parent) = 0;

        //! Get the entity id of the spawned box with the given name.
        virtual AZ::EntityId GetSpawnedEntityId(const AZStd::string& spawnableName) = 0;

        //! Despawn the box with the given name.
        virtual void DespawnBox(const AZStd::string& spawnableName) = 0;
    };

    class ScriptSpawnSystemBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ScriptSpawnSystemRequestBus = AZ::EBus<ScriptSpawnSystemRequests, ScriptSpawnSystemBusTraits>;
} // namespace ROS2::Demo
