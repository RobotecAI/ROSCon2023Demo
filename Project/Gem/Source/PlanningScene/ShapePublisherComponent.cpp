
#include "ShapePublisherComponent.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/Debug/Trace.h"
#include "AzCore/Math/Quaternion.h"
#include "AzCore/Math/Random.h"
#include "AzCore/Math/Transform.h"
#include "AzCore/Serialization/EditContextConstants.inl"
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Math/Obb.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/ShapeComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <rclcpp/qos.hpp>

namespace ROSCon2023Demo
{
    void ShapePublisherComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_publisher = ros2Node->create_publisher<moveit_msgs::msg::CollisionObject>("/collision_object", 10);

        AZ::TickBus::Handler::BusConnect();

        Publish();
    }

    void ShapePublisherComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZStd::string ShapePublisherComponent::GetFrameID() const
    {
        auto* ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameID();
    }

    moveit_msgs::msg::CollisionObject ShapePublisherComponent::CreateCollisionObject()
    {
        AZ::Crc32 shapeType;
        LmbrCentral::ShapeComponentRequestsBus::EventResult(shapeType, GetEntityId(), &LmbrCentral::ShapeComponentRequests::GetShapeType);

        if (shapeType == AZ_CRC("Box", 0x08a9483a))
        {
            return CreateBoxCollisionObject();
        }

        if (shapeType == AZ_CRC("Cylinder", 0x9b045bea))
        {
            return CreateCylinderCollisionObject();
        }

        AZ_Warning("ShapePublisherComponent", false, "Unsupported shape type: %d", (int)shapeType);
        return moveit_msgs::msg::CollisionObject();
    }

    namespace Internal
    {
        geometry_msgs::msg::Pose CreatePose(AZ::Transform tf)
        {
            auto translation = tf.GetTranslation();
            auto rotation = tf.GetRotation();
            geometry_msgs::msg::Pose box_pose;
            box_pose.position.x = translation.GetX();
            box_pose.position.y = translation.GetY();
            box_pose.position.z = translation.GetZ();
            box_pose.orientation.x = rotation.GetX();
            box_pose.orientation.y = rotation.GetY();
            box_pose.orientation.z = rotation.GetZ();
            box_pose.orientation.w = rotation.GetW();

            return box_pose;
        }
    } // namespace Internal

    moveit_msgs::msg::CollisionObject ShapePublisherComponent::CreateObject(
        shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose pose)
    {
        moveit_msgs::msg::CollisionObject collision_object;

        collision_object.header.frame_id = GetFrameID().c_str();
        collision_object.id = m_objectId.c_str();
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }

    moveit_msgs::msg::CollisionObject ShapePublisherComponent::CreateCylinderCollisionObject()
    {
        LmbrCentral::CylinderShapeConfig cylinderConfig = LmbrCentral::CylinderShapeConfig();
        LmbrCentral::CylinderShapeComponentRequestsBus::EventResult(
            cylinderConfig, GetEntityId(), &LmbrCentral::CylinderShapeComponentRequests::GetCylinderConfiguration);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = cylinderConfig.m_height;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = cylinderConfig.m_radius;

        auto pose = Internal::CreatePose(AZ::Transform::CreateIdentity());

        return CreateObject(primitive, pose);
    }

    moveit_msgs::msg::CollisionObject ShapePublisherComponent::CreateBoxCollisionObject()
    {
        LmbrCentral::BoxShapeConfig boxConfig = LmbrCentral::BoxShapeConfig();
        LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
            boxConfig, GetEntityId(), &LmbrCentral::BoxShapeComponentRequests::GetBoxConfiguration);

        auto boxDimensions = boxConfig.m_dimensions;
        auto offset = boxConfig.m_translationOffset;

        auto tm = AZ::Transform::CreateTranslation(offset);

        bool isAxisAligned = false;
        LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
            isAxisAligned, GetEntityId(), &LmbrCentral::BoxShapeComponentRequests::IsTypeAxisAligned);

        if (isAxisAligned)
        {
            tm.SetRotation(AZ::Quaternion::CreateIdentity());
        }

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.push_back(boxDimensions.GetX());
        primitive.dimensions.push_back(boxDimensions.GetY());
        primitive.dimensions.push_back(boxDimensions.GetZ());

        auto pose = Internal::CreatePose(tm);

        return CreateObject(primitive, pose);
    }

    void ShapePublisherComponent::Publish()
    {
        auto collisionObject = CreateCollisionObject();
        m_publisher->publish(collisionObject);
    }

    void ShapePublisherComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ShapePublisherComponent, AZ::Component>()
                ->Field("ObjectName", &ShapePublisherComponent::m_objectId)
                ->Version(1);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ShapePublisherComponent>("ShapePublisherComponent", "Publishes CollisionObject data about shape.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "ShapePublisher")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/Component_Placeholder.svg")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ShapePublisherComponent::m_objectId,
                        "Object Id",
                        "Id of the object, used as id in CollisionObject messages. Should be unique.");
            }
        }
    }

    void ShapePublisherComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ShapePublisherComponentService"));
    }

    void ShapePublisherComponent::GetIncompatibleServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
    }

    void ShapePublisherComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ShapeService", 0xe86aa5fe));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ShapePublisherComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void ShapePublisherComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AZ_Assert(m_frequency > 0.f, "ShapePublisher frequency must be greater than zero");
        auto frameTime = 1.f / m_frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
        {
            return;
        }

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }

        Publish();
    }

} // namespace ROSCon2023Demo
