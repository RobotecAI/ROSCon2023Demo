#include "utils.h"

namespace Utils
{

    Eigen::Quaterniond fromMsgQuaternion(const geometry_msgs::msg::Quaternion& msg)
    {
        Eigen::Quaterniond q;
        q.x() = msg.x;
        q.y() = msg.y;
        q.z() = msg.z;
        q.w() = msg.w;
        return q;
    }

    Eigen::Vector3d fromMsgPosition(const geometry_msgs::msg::Point msg)
    {
        Eigen::Vector3d v{ msg.x, msg.y, msg.z };
        return v;
    }
    Eigen::Vector3d fromMsgPosition(const geometry_msgs::msg::Vector3 msg)
    {
        Eigen::Vector3d v{ msg.x, msg.y, msg.z };
        return v;
    }

    const geometry_msgs::msg::Point toMsgPoint(Eigen::Vector3d v)
    {
        geometry_msgs::msg::Point msg;
        msg.x = v.x();
        msg.y = v.y();
        msg.z = v.z();
        return msg;
    }

    const geometry_msgs::msg::Quaternion toMsgQuaternion(const Eigen::Quaterniond& q)
    {
        geometry_msgs::msg::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();
        return msg;
    }

    moveit_msgs::msg::CollisionObject CreateBoxCollision(
        const std::string& name,
        const Eigen::Vector3d dimension,
        const Eigen::Vector3d location,
        const Eigen::Quaterniond& rot,
        std::string ns)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = ns + "/world";
        collision_object.id = name;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.push_back(dimension.x());
        primitive.dimensions.push_back(dimension.y());
        primitive.dimensions.push_back(dimension.z());
        collision_object.primitives.push_back(primitive);
        geometry_msgs::msg::Pose box_pose;
        box_pose.position = toMsgPoint(location);
        box_pose.orientation = toMsgQuaternion(rot);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
    }

    geometry_msgs::msg::Pose getBoxTargetPose(const Eigen::Vector3f& adress, const geometry_msgs::msg::Pose& palletPose,const Eigen::Vector3d& boxDimension, float separation)
    {
        geometry_msgs::msg::Pose pose;

        pose.position = palletPose.position;
        const Eigen::Quaterniond palletOrientation = fromMsgQuaternion(palletPose.orientation);
        Eigen::Vector3d localPalletDirX = palletOrientation * Eigen::Vector3d::UnitX();
        Eigen::Vector3d localPalletDirY = palletOrientation * Eigen::Vector3d::UnitY();
        Eigen::Vector3d localPalletDirZ = palletOrientation * Eigen::Vector3d::UnitZ();

        Eigen::Vector3d p = fromMsgPosition(palletPose.position);
        Eigen::Quaterniond q = fromMsgQuaternion(palletPose.orientation);

        // change local coordinate system for UR10 gipper targe pose in the way to have X+ pointing sky
        // Used blender to find this matrix.
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        rotation << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
        p += adress.x() * localPalletDirX * separation * boxDimension.x() + adress.y() * localPalletDirY * separation * boxDimension.y() +
            adress.z() * localPalletDirZ * separation * boxDimension.z();

        pose.position = toMsgPoint(p);
        pose.orientation = toMsg(Eigen::Quaterniond(q.toRotationMatrix() * rotation));
        return pose;
    }
} // namespace Utils
