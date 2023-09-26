
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROSCon2023Demo
{
    /* Component publishing shape as collision_object.
     * It serves as ground truth source for collision-aware motion planning with eg. MoveIt
     */
    class ShapePublisherComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ShapePublisherComponent, "{DFEE0399-B3EB-4D5F-98F4-D966C3EAFE90}", AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    protected:
        void Activate() override;
        void Deactivate() override;

    private:
        moveit_msgs::msg::CollisionObject CreateCollisionObject();
        moveit_msgs::msg::CollisionObject CreateBoxCollisionObject();
        moveit_msgs::msg::CollisionObject CreateCylinderCollisionObject();
        moveit_msgs::msg::CollisionObject CreateObject(shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose pose);

        AZStd::string GetFrameID() const;

        void Publish();

        rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr m_publisher;
        AZStd::string m_objectId = "unique_object_id";
        AZStd::string m_topicNamespace = "";
        AZStd::string m_topicName = "/collision_object";
        float m_frequency = 1.5f;
        float m_timeElapsedSinceLastTick = 0.0f;
    };
} // namespace ROSCon2023Demo
