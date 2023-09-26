#include "FoilWrapperController.h"
#include "ScriptSpawnSytemBus.h"
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <Integration/SimpleMotionComponentBus.h>
#include <PhysX/Joint/Configuration/PhysXJointConfiguration.h>
namespace ROS2::Demo
{
    void FoilWrapperConfig::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<FoilWrapperConfig>()
                ->Version(1)
                ->Field("FoilWrapperEntityId", &FoilWrapperConfig::m_foilWrapperEntityId)
                ->Field("CollisionTrigger", &FoilWrapperConfig::m_collisionTrigger)
                ->Field("SpawnablePayloadFoiled", &FoilWrapperConfig::m_spawnablePayloadFoiled);
            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext->Class<FoilWrapperConfig>("FoilWrapperConfig", "FoilWrapperConfig")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FoilWrapperConfig::m_foilWrapperEntityId, "m_foilWrapperEntityId", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FoilWrapperConfig::m_collisionTrigger, "m_collisionTrigger", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FoilWrapperConfig::m_spawnablePayloadFoiled, "m_spawnablePayloadFoiled", "");
                // clang-format on
            }
        }
    }

    void FoilWrapper::Reflect(AZ::ReflectContext* context)
    {
        FoilWrapperConfig::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FoilWrapper, AZ::Component>()->Version(1)->Field("FoilWrapperConfig", &FoilWrapper::m_configuration);
            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext->Class<FoilWrapper>("FoilWrapper", "FoilWrapper")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Demo")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FoilWrapper::m_configuration, "m_configuration", "");
                // clang-format on
            }
        }
    }

    void FoilWrapper::Activate()
    {
        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const auto entityId = event.m_otherBody->GetEntityId();
                if (isObjectBox(entityId))
                {
                    m_collidingEntities.push_back(entityId);
                }
                if (isObjectPallet(entityId))
                {
                    m_pallet = entityId;
                    if (m_state == FoilWrapperState::Idle)
                    {
                        m_state = FoilWrapperState::Starting;
                    }
                }
            });

        AZ::TickBus::Handler::BusConnect();
    }

    void FoilWrapper::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    bool FoilWrapper::isObjectBox(const AZ::EntityId entityId) const
    {
        bool isBox = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isBox, entityId, &LmbrCentral::TagComponentRequests::HasTag, BoxTag);
        return isBox;
    }

    bool FoilWrapper::isObjectPallet(const AZ::EntityId entityId) const
    {
        bool isBox = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isBox, entityId, &LmbrCentral::TagComponentRequests::HasTag, PalletTag);
        return isBox;
    }

    void CreateFixedJoint(
        const AZ::EntityId parentId,
        const AZ::EntityId childId,
        AzPhysics::SystemInterface* physicsSystem,
        AzPhysics::SceneInterface* sceneInterface,
        const AzPhysics::SceneHandle defaultSceneHandle)
    {
        AZ::Transform childTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(childTransform, childId, &AZ::TransformBus::Events::GetWorldTM);

        AZ::Transform parentTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(parentTransform, parentId, &AZ::TransformBus::Events::GetWorldTM);

        // Find Transform of the child in parent's frame
        AZ::Transform childTransformParent = parentTransform.GetInverse() * childTransform;
        childTransformParent.Invert();

        // get handles
        AzPhysics::SimulatedBodyHandle palletRigidBodyHandle = physicsSystem->FindAttachedBodyHandleFromEntityId(parentId).second;
        AzPhysics::SimulatedBodyHandle boxBodyHandle = physicsSystem->FindAttachedBodyHandleFromEntityId(childId).second;

        // Configure new joint
        PhysX::FixedJointConfiguration jointConfig;
        jointConfig.m_debugName = "BoxPalletJoint";
        jointConfig.m_parentLocalRotation = AZ::Quaternion::CreateIdentity();
        jointConfig.m_parentLocalPosition = AZ::Vector3::CreateZero();
        jointConfig.m_childLocalRotation = childTransformParent.GetRotation();
        jointConfig.m_childLocalPosition = childTransformParent.GetTranslation();
        jointConfig.m_startSimulationEnabled = true;

        // Create new joint
        sceneInterface->AddJoint(defaultSceneHandle, &jointConfig, palletRigidBodyHandle, boxBodyHandle);
    }

    void FoilWrapper::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface.");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle.");

        // Connect the trigger handlers if not already connected, it is circumventing the issue GH-16188, the
        // RigidbodyNotificationBus should be used instead.
        if (!m_onTriggerEnterHandler.IsConnected())
        {
            AZ_Assert(m_configuration.m_foilWrapperEntityId.IsValid(), "Invalid foil wrapper entity id.");
            if (m_configuration.m_foilWrapperEntityId.IsValid())
            {
                AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                    physicsSystem->FindAttachedBodyHandleFromEntityId(m_configuration.m_collisionTrigger);
                AZ_Warning("FoilWrapper", foundBody.first != AzPhysics::InvalidSceneHandle, "No body found for m_collisionTrigger.");
                if (foundBody.first != AzPhysics::InvalidSceneHandle)
                {
                    AzPhysics::SimulatedBodyEvents::RegisterOnTriggerEnterHandler(
                        foundBody.first, foundBody.second, m_onTriggerEnterHandler);
                }
            }
        }
        if (m_state == FoilWrapperState::Idle)
        {
            m_payloadSpawned = false;
            m_payloadFixed = false;
            m_payloadName = "";
            m_timer = 0;
        }
        if (m_state == FoilWrapperState::Starting)
        {
            m_timer += delta;
            if (m_timer > 2.0)
            {
                EMotionFX::Integration::SimpleMotionComponentRequestBus::Event(
                    m_configuration.m_foilWrapperEntityId,
                    &EMotionFX::Integration::SimpleMotionComponentRequestBus::Events::LoopMotion,
                    false);

                EMotionFX::Integration::SimpleMotionComponentRequestBus::Event(
                    m_configuration.m_foilWrapperEntityId, &EMotionFX::Integration::SimpleMotionComponentRequestBus::Events::PlayMotion);

                m_state = FoilWrapperState::Wrapping;
                m_timer = 0;
            }
        }
        if (m_state == FoilWrapperState::Wrapping)
        {
            m_timer += delta;

            if (m_timer > 2.0 && m_payloadSpawned == false)
            {
                m_payloadSpawned = true;
                AZ_TracePrintf("FoilWrapper", "Wrapping started");

                AZ::Transform palletTransform = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(palletTransform, m_pallet, &AZ::TransformBus::Events::GetWorldTM);

                // set foil as child to pallet
                AZ::Entity* palletEntity{};
                AZ::ComponentApplicationBus::BroadcastResult(palletEntity, &AZ::ComponentApplicationRequests::FindEntity, m_pallet);

                // connect box to pallet with fixed joints
                for (auto& boxesId : m_collidingEntities)
                {
                    CreateFixedJoint(m_pallet, boxesId, physicsSystem, sceneInterface, defaultSceneHandle);
                }

                // hide boxes
                for (auto& boxesId : m_collidingEntities)
                {
                    AZStd::vector<AZ::EntityId> descendants;
                    AZ::TransformBus::EventResult(descendants, boxesId, &AZ::TransformBus::Events::GetAllDescendants);
                    for (auto& descendant : descendants)
                    {
                        AZ::Render::MeshComponentRequestBus::Event(descendant, &AZ::Render::MeshComponentRequests::SetVisibility, false);
                    }
                }

                AZ_TracePrintf("FoilWrapper", "Number of boxes that were \"wrapped\": %d", m_collidingEntities.size());

                m_payloadName = AZStd::string::format("WrappedPallet%d", m_wrappedPallets++);
                ScriptSpawnSystemRequestBus::Broadcast(
                    &ScriptSpawnSystemRequestBus::Events::SpawnAssetAndSetParent,
                    m_configuration.m_spawnablePayloadFoiled,
                    palletTransform,
                    m_payloadName,
                    m_pallet);
            }

            float duration{ 0 };
            EMotionFX::Integration::SimpleMotionComponentRequestBus::EventResult(
                duration,
                m_configuration.m_foilWrapperEntityId,
                &EMotionFX::Integration::SimpleMotionComponentRequestBus::Events::GetDuration);

            AZ::EntityId wrappedPalletId(AZ::EntityId::InvalidEntityId);
            ScriptSpawnSystemRequestBus::BroadcastResult(
                wrappedPalletId, &ScriptSpawnSystemRequestBus::Events::GetSpawnedEntityId, m_payloadName);

            // create fixed joint
            if (wrappedPalletId.IsValid() && !m_payloadFixed)
            {
                CreateFixedJoint(m_pallet, wrappedPalletId, physicsSystem, sceneInterface, defaultSceneHandle);
                m_payloadFixed = true;
            }

            if (m_timer >= duration)
            {
                AZ_TracePrintf("FoilWrapper", "Wrapping done");
                m_timer = 0;
                m_state = FoilWrapperState::Idle;
            }
        }
    }
} // namespace ROS2::Demo
